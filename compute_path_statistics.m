function compute_path_statistics(varargin)
% compute_path_statistics  4-way CFRC path statistics + stress alignment
%
% 输入 (从当前目录或本 m 所在目录查找):
%   - all_layers_paths_only_*.mat
%   - stress field .mat  (含 theta_xoy/theta_xoz, 度)
%
% 计算三类指标:
%   A. 长度类:   total_length, smooth_run (连续 <30° 折角)
%   B. 路径-应力: 路径切向 vs 该位置应力方向夹角
%   C. 曲面-应力: 切片面法向 vs 应力方向夹角 (90° = 应力在面内)
%
% 输出 → C:\temp\cfrc_fea\results\:
%   <config>_path_stats.csv, all_configs_path_summary.csv
%   fig_path_length.png, fig_smooth_run_distribution.png
%   fig_angle_path_stress.png, fig_angle_surface_stress.png
%
% 用法:
%   compute_path_statistics
%   compute_path_statistics('stress_file', 'my_stress.mat', 'smooth_turn_deg', 30)

%% Config
p = inputParser;
addParameter(p, 'path_dir', '');
addParameter(p, 'stress_file', '');
% Default output_dir = subfolder next to this script file
addParameter(p, 'output_dir', fullfile(fileparts(mfilename('fullpath')), 'path_stats_output'));
addParameter(p, 'smooth_turn_deg', 30);
addParameter(p, 'voxel_size', 1.0);
addParameter(p, 'box_y_max', -1);   % auto: max(Q3)*1.4 over configs
addParameter(p, 'box_y_min', -8);   % default y_min for box plot
parse(p, varargin{:});
opt = p.Results;

if ~exist(opt.output_dir, 'dir'), mkdir(opt.output_dir); end

paths_files = {
    'mine_stream',   {'all_layers_paths_only_v3.mat', ...
                      'all_layers_paths_only_mine_stream.mat', ...
                      'paths_only_mine_stream.mat'},   'curved';
    'mine_offset',   {'all_layers_paths_only_mine_offset.mat', ...
                      'paths_only_mine_offset.mat', ...
                      'all_layers_paths_only_v3_offset.mat'},      'curved';
    'planar_stream', {'all_layers_paths_only_planar_stream.mat', ...
                      'paths_only_planar_stream.mat', ...
                      'all_layers_paths_only_planar_v3.mat'},      'planar';
    'planar_offset', {'all_layers_paths_only_planar_offset.mat', ...
                      'paths_only_planar_offset.mat', ...
                      'all_layers_paths_only_planar_offset_v3.mat'}, 'planar';
};

fprintf('\n=== compute_path_statistics: 4-way analysis ===\n\n');

search_dirs = {pwd, fileparts(mfilename('fullpath'))};
if ~isempty(opt.path_dir), search_dirs = [{opt.path_dir}, search_dirs]; end

% Also include all MATLAB path directories (so files in addpath'd dirs are found)
path_dirs = strsplit(path, pathsep);
% Filter out toolbox/system dirs (only keep dirs that don't have "toolbox" or
% the matlab install dir; this prevents searching thousands of system dirs)
matlabroot_dir = matlabroot;
user_path_dirs = {};
for k = 1:numel(path_dirs)
    if ~isempty(path_dirs{k}) && ~startsWith(path_dirs{k}, matlabroot_dir)
        user_path_dirs{end+1} = path_dirs{k}; %#ok<AGROW>
    end
end
search_dirs = [search_dirs, user_path_dirs];
fprintf('  [search] %d directories to search\n', numel(search_dirs));

for k = 1:size(paths_files, 1)
    cands = paths_files{k, 2};
    found = '';
    for ic = 1:numel(cands)
        candidate_path = find_in_dirs(cands{ic}, search_dirs);
        if ~isempty(candidate_path)
            found = candidate_path;
            break;
        end
    end
    paths_files{k, 4} = found;
end

stress = load_stress_field(opt.stress_file, search_dirs);
if isempty(stress)
    fprintf('\n[WARN] stress field NOT found. Only length stats will be computed.\n');
    fprintf('       Searched %d dirs (see [stress] log above).\n\n', numel(search_dirs));
else
    fprintf('\n');
end

%% Process each config
n_cfg = size(paths_files, 1);
config_summary = struct();
all_path_stress_angles = cell(n_cfg, 1);
all_surface_stress_angles = cell(n_cfg, 1);

for c = 1:n_cfg
    cfg_name = paths_files{c, 1};
    mat_file = paths_files{c, 4};
    slice_type = paths_files{c, 3};
    
    fprintf('---------- %s (%s) ----------\n', cfg_name, slice_type);
    if isempty(mat_file) || ~exist(mat_file, 'file')
        fprintf('  [SKIP] no path mat found. Tried:\n');
        cands_tried = paths_files{c, 2};
        for ic = 1:numel(cands_tried)
            fprintf('         %s\n', cands_tried{ic});
        end
        fprintf('\n');
        continue;
    end
    fprintf('  loading: %s\n', mat_file);
    
    S = load(mat_file);
    paths = extract_paths_robust(S, mat_file);
    if isempty(paths)
        fprintf('  [SKIP] could not extract paths from %s\n\n', mat_file);
        continue;
    end
    
    n_paths = numel(paths);
    fprintf('  paths: %d\n', n_paths);
    
    pp_n_points    = nan(n_paths, 1);
    pp_length      = nan(n_paths, 1);
    pp_smooth_run  = nan(n_paths, 1);
    pp_mean_seg    = nan(n_paths, 1);
    pp_layer_idx   = nan(n_paths, 1);
    pp_mean_angle_stress   = nan(n_paths, 1);
    pp_median_angle_stress = nan(n_paths, 1);
    
    angles_this_cfg = [];
    sa_angles = [];
    
    for ip = 1:n_paths
        pts = paths{ip};
        if isempty(pts) || size(pts, 1) < 2, continue; end
        if size(pts, 2) > 3, pts = pts(:, 1:3); end
        
        pp_n_points(ip) = size(pts, 1);
        pp_layer_idx(ip) = round(median(pts(:, 3)));
        
        % Length
        seg = diff(pts, 1, 1);
        seg_len = sqrt(sum(seg.^2, 2));
        pp_length(ip) = sum(seg_len);
        pp_mean_seg(ip) = mean(seg_len);
        
        % Smooth run
        if size(seg, 1) >= 2
            seg_unit = seg ./ max(seg_len, eps);
            cos_turn = sum(seg_unit(1:end-1, :) .* seg_unit(2:end, :), 2);
            cos_turn = max(min(cos_turn, 1), -1);
            turn_deg = acosd(cos_turn);
            break_idx = find(turn_deg > opt.smooth_turn_deg);
            run_starts = [1; break_idx + 1];
            run_ends   = [break_idx; numel(seg_len)];
            run_lens   = arrayfun(@(s, e) sum(seg_len(s:e)), run_starts, run_ends);
            pp_smooth_run(ip) = max(run_lens);
        else
            pp_smooth_run(ip) = pp_length(ip);
        end
        
        % Path-stress angles
        if ~isempty(stress)
            seg_unit = seg ./ max(seg_len, eps);
            seg_angles = nan(size(seg, 1), 1);
            for is = 1:size(seg, 1)
                mid = 0.5 * (pts(is, :) + pts(is+1, :));
                sv = lookup_stress_at_point(stress, mid, opt.voxel_size);
                if all(isnan(sv)) || norm(sv) < 1e-9, continue; end
                cos_a = abs(dot(seg_unit(is, :), sv) / norm(sv));
                cos_a = max(min(cos_a, 1), -1);
                seg_angles(is) = acosd(cos_a);
            end
            valid = ~isnan(seg_angles);
            if any(valid)
                pp_mean_angle_stress(ip)   = mean(seg_angles(valid));
                pp_median_angle_stress(ip) = median(seg_angles(valid));
                angles_this_cfg = [angles_this_cfg; seg_angles(valid)];
            end
            
            % Surface-stress (per path: collect)
            for is = 1:size(pts,1)-1
                mid = 0.5 * (pts(is,:) + pts(is+1,:));
                sv = lookup_stress_at_point(stress, mid, opt.voxel_size);
                if all(isnan(sv)) || norm(sv) < 1e-9, continue; end
                if strcmp(slice_type, 'planar')
                    n_vec = [0 0 1];
                else
                    if size(pts,1) >= 3 && is < size(pts,1)-1
                        v1 = pts(is+1,:) - pts(is,:);
                        v2 = pts(is+2,:) - pts(is+1,:);
                        n_vec = cross(v1, v2);
                        if norm(n_vec) < 1e-6, n_vec = [0 0 1]; end
                        n_vec = n_vec / norm(n_vec);
                        if n_vec(3) < 0, n_vec = -n_vec; end
                    else
                        n_vec = [0 0 1];
                    end
                end
                cos_a = abs(dot(n_vec, sv) / norm(sv));
                cos_a = max(min(cos_a, 1), -1);
                sa_angles(end+1, 1) = acosd(cos_a);
            end
        end
    end
    
    % Per-config CSV
    T = table((1:n_paths)', pp_layer_idx, pp_n_points, pp_length, pp_smooth_run, ...
              pp_mean_seg, pp_mean_angle_stress, pp_median_angle_stress, ...
              'VariableNames', {'path_idx', 'layer_idx', 'n_points', ...
                                'total_length_mm', 'smooth_run_mm', 'mean_seg_len_mm', ...
                                'mean_angle_stress_deg', 'median_angle_stress_deg'});
    csv_path = fullfile(opt.output_dir, sprintf('%s_path_stats.csv', cfg_name));
    writetable(T, csv_path);
    fprintf('  wrote CSV: %s\n', csv_path);
    
    config_summary.(cfg_name).slice_type = slice_type;
    config_summary.(cfg_name).n_paths = sum(~isnan(pp_length));
    config_summary.(cfg_name).total_length_mm = nansum(pp_length);
    config_summary.(cfg_name).mean_length_mm = nanmean(pp_length);
    config_summary.(cfg_name).sum_smooth_run_mm = nansum(pp_smooth_run);
    config_summary.(cfg_name).mean_smooth_run_mm = nanmean(pp_smooth_run);
    mvals = pp_smooth_run(~isnan(pp_smooth_run));
    if ~isempty(mvals)
        config_summary.(cfg_name).max_smooth_run_mm = max(mvals);
    else
        config_summary.(cfg_name).max_smooth_run_mm = NaN;
    end
    config_summary.(cfg_name).smooth_ratio = ...
        config_summary.(cfg_name).sum_smooth_run_mm / ...
        max(config_summary.(cfg_name).total_length_mm, eps);
    
    if ~isempty(angles_this_cfg)
        config_summary.(cfg_name).path_stress_mean = mean(angles_this_cfg);
        config_summary.(cfg_name).path_stress_median = median(angles_this_cfg);
        config_summary.(cfg_name).path_stress_p25 = prctile(angles_this_cfg, 25);
        config_summary.(cfg_name).path_stress_p75 = prctile(angles_this_cfg, 75);
        all_path_stress_angles{c} = angles_this_cfg;
    end
    if ~isempty(sa_angles)
        config_summary.(cfg_name).surf_stress_mean = mean(sa_angles);
        config_summary.(cfg_name).in_plane_ratio_mean = mean(sind(sa_angles));
        all_surface_stress_angles{c} = sa_angles;
    end
    
    fprintf('  total length:        %.2f mm\n', config_summary.(cfg_name).total_length_mm);
    fprintf('  smooth_run sum:      %.2f mm  (%.1f%% of total)\n', ...
        config_summary.(cfg_name).sum_smooth_run_mm, 100*config_summary.(cfg_name).smooth_ratio);
    fprintf('  max smooth_run:      %.2f mm\n', config_summary.(cfg_name).max_smooth_run_mm);
    if isfield(config_summary.(cfg_name), 'path_stress_mean')
        fprintf('  path-stress mean:    %.1f deg  (lower=better aligned)\n', ...
            config_summary.(cfg_name).path_stress_mean);
        fprintf('  path-stress median:  %.1f deg\n', config_summary.(cfg_name).path_stress_median);
    end
    if isfield(config_summary.(cfg_name), 'surf_stress_mean')
        fprintf('  surf-stress mean:    %.1f deg  (90=stress fully in plane)\n', ...
            config_summary.(cfg_name).surf_stress_mean);
        fprintf('  in-plane ratio:      %.3f      (1.0 = ideal)\n', ...
            config_summary.(cfg_name).in_plane_ratio_mean);
    end
    fprintf('\n');
end

%% Global summary CSV
cfg_names = fieldnames(config_summary);
n_rows = numel(cfg_names);
if n_rows == 0
    fprintf('[ERROR] No config produced data\n');
    return;
end

vars_list = {'slice_type', 'n_paths', 'total_length_mm', 'mean_length_mm', ...
             'sum_smooth_run_mm', 'mean_smooth_run_mm', 'max_smooth_run_mm', ...
             'smooth_ratio', 'path_stress_mean', 'path_stress_median', ...
             'path_stress_p25', 'path_stress_p75', ...
             'surf_stress_mean', 'in_plane_ratio_mean'};

T_sum = table();
T_sum.config = cfg_names;
for v = 1:numel(vars_list)
    fld = vars_list{v};
    col_num = nan(n_rows, 1);
    col_str = strings(n_rows, 1);
    is_str = strcmp(fld, 'slice_type');
    for k = 1:n_rows
        if isfield(config_summary.(cfg_names{k}), fld)
            val = config_summary.(cfg_names{k}).(fld);
            if is_str
                col_str(k) = string(val);
            else
                col_num(k) = val;
            end
        end
    end
    if is_str, T_sum.(fld) = col_str; else, T_sum.(fld) = col_num; end
end

summary_csv = fullfile(opt.output_dir, 'all_configs_path_summary.csv');
writetable(T_sum, summary_csv);
fprintf('Wrote summary: %s\n\n', summary_csv);

%% Plots
COLORS = [0.86 0.20 0.18;     % mine_stream     - red
          0.95 0.55 0.15;     % mine_offset     - orange
          0.17 0.40 0.74;     % planar_stream   - blue
          0.45 0.45 0.48];    % planar_offset   - gray (baseline)
LABELS_FULL = {'MINE+Stream', 'MINE+Offset', 'Planar+Stream', 'Planar+Offset'};
idx_map = zeros(n_rows, 1);
for k = 1:n_rows
    switch cfg_names{k}
        case 'mine_stream',   idx_map(k) = 1;
        case 'mine_offset',   idx_map(k) = 2;
        case 'planar_stream', idx_map(k) = 3;
        case 'planar_offset', idx_map(k) = 4;
    end
end

% Common style settings
FONT_NAME = 'Helvetica';
if ispc, FONT_NAME = 'Arial'; end
TXT_COLOR = [0.20 0.20 0.20];
GRID_COLOR = [0.85 0.85 0.85];

% --- Fig 1: Length stats (3 bar charts) ---
fig1 = figure('Name', 'Path Length Stats', 'Position', [100 100 1300 480], ...
              'Color', 'w');

subplot_titles = {'Total Path Length', ...
                  sprintf('Sum of Smooth Runs (turn<%d°)', opt.smooth_turn_deg), ...
                  'Smooth Ratio (smooth / total)'};
subplot_ylabels = {'Total Length (mm)', 'Smooth Run Length (mm)', 'Smooth Ratio (%)'};
subplot_metric = {'total_length_mm', 'sum_smooth_run_mm', 'smooth_ratio'};
subplot_multiplier = [1, 1, 100];
subplot_fmt = {'%.0f', '%.0f', '%.1f%%'};

for sp = 1:3
    ax = subplot(1, 3, sp);
    set(ax, 'FontSize', 11, 'FontName', FONT_NAME, ...
            'TickDir', 'out', 'Box', 'off', 'LineWidth', 1.1, ...
            'XColor', TXT_COLOR, 'YColor', TXT_COLOR);
    hold(ax, 'on');
    
    vals = arrayfun(@(k) get_field(config_summary, cfg_names{k}, ...
                                    subplot_metric{sp}, NaN), 1:n_rows);
    vals_plot = vals * subplot_multiplier(sp);
    
    b = bar(ax, vals_plot, 'FaceColor', 'flat', 'EdgeColor', 'none', ...
            'BarWidth', 0.65);
    for k = 1:n_rows, b.CData(k,:) = COLORS(idx_map(k), :); end
    
    % Value labels on top
    for k = 1:n_rows
        if ~isnan(vals_plot(k))
            yl = ylim(ax);
            text(ax, k, vals_plot(k) + max(yl)*0.02, ...
                 sprintf(subplot_fmt{sp}, vals_plot(k)), ...
                 'HorizontalAlignment', 'center', 'FontSize', 10, ...
                 'FontWeight', 'bold', 'Color', TXT_COLOR);
        end
    end
    
    set(ax, 'XTick', 1:n_rows, 'XTickLabel', LABELS_FULL(idx_map), ...
            'XTickLabelRotation', 18);
    ylabel(ax, subplot_ylabels{sp}, 'FontSize', 12, 'FontName', FONT_NAME);
    title(ax, subplot_titles{sp}, 'FontSize', 12.5, 'FontWeight', 'bold');
    
    grid(ax, 'on');
    ax.GridColor = GRID_COLOR;
    ax.GridLineStyle = '--';
    ax.GridAlpha = 0.6;
    ax.Layer = 'top';
    
    if sp == 3, ylim(ax, [0 105]); end
    yl = ylim(ax);
    ylim(ax, [0, yl(2) * 1.08]);
end
save_with_and_without_text(fig1, fullfile(opt.output_dir, 'fig_path_length.png'));

% --- Fig 2: Path-Stress Angle Histogram ---
have_any = false;
for k = 1:numel(all_path_stress_angles)
    if ~isempty(all_path_stress_angles{k}), have_any = true; break; end
end
if have_any
    fig2 = make_angle_figure(all_path_stress_angles, idx_map, COLORS, LABELS_FULL, ...
        n_rows, 0, true, ...   % ideal=0deg (path aligned w/ stress), shade [0..med]
        'Path-Stress Alignment', ...
        'Angle: path tangent vs stress direction (\circ)', ...
        'Fraction of segments (%)', ...
        '0\circ = path perfectly aligned with stress  \rightarrow  smaller = better', ...
        FONT_NAME, TXT_COLOR, GRID_COLOR);
    save_with_and_without_text(fig2, fullfile(opt.output_dir, 'fig_angle_path_stress.png'));
end

% --- Fig 3: Surface-Stress Angle Histogram (with 50% shading) ---
have_any = false;
for k = 1:numel(all_surface_stress_angles)
    if ~isempty(all_surface_stress_angles{k}), have_any = true; break; end
end
if have_any
    fig3 = make_angle_figure(all_surface_stress_angles, idx_map, COLORS, LABELS_FULL, ...
        n_rows, 90, true, ...  % ideal=90deg (stress in slicing plane), shade [med..90]
        'Surface-Stress Angle', ...
        'Angle: surface normal vs stress direction (\circ)', ...
        'Fraction of points (%)', ...
        '90\circ = stress fully in slicing plane  \rightarrow  larger = better. Shaded: top 50% region.', ...
        FONT_NAME, TXT_COLOR, GRID_COLOR);
    save_with_and_without_text(fig3, fullfile(opt.output_dir, 'fig_angle_surface_stress.png'));
end

% --- Fig 4: Smooth-Run Distribution (violin + box overlay, y-truncated) ---
fig4 = figure('Name', 'Smooth Run Distribution', 'Position', [100 100 1100 620], ...
              'Color', 'w');
ax = axes('Parent', fig4);
set(ax, 'FontSize', 11.5, 'FontName', FONT_NAME, ...
        'TickDir', 'out', 'Box', 'off', 'LineWidth', 1.1, ...
        'XColor', TXT_COLOR, 'YColor', TXT_COLOR);
hold(ax, 'on');
grid(ax, 'on');
ax.GridColor = GRID_COLOR;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.5;
ax.Layer = 'top';

% Collect data per config (canonical order)
data_per_cfg = cell(n_rows, 1);
labels_used = cell(n_rows, 1);
colors_used = zeros(n_rows, 3);
for k = 1:n_rows
    csv_p = fullfile(opt.output_dir, sprintf('%s_path_stats.csv', cfg_names{k}));
    if exist(csv_p, 'file')
        T = readtable(csv_p);
        data_per_cfg{k} = T.smooth_run_mm(~isnan(T.smooth_run_mm));
        labels_used{k} = LABELS_FULL{idx_map(k)};
        colors_used(k, :) = COLORS(idx_map(k), :);
    end
end

% Compute Q1/Q2/Q3 per config for y-axis cap
Q3_arr = nan(n_rows, 1);
maxvals = nan(n_rows, 1);
for k = 1:n_rows
    if ~isempty(data_per_cfg{k})
        Q3_arr(k) = quantile(data_per_cfg{k}, 0.75);
        maxvals(k) = max(data_per_cfg{k});
    end
end

% Determine y-axis limits
if opt.box_y_max > 0
    y_top = opt.box_y_max;
else
    y_top = nanmax(Q3_arr) * 1.6;   % a bit more headroom for violin tail
end
y_bot = opt.box_y_min;

positions = 1:n_rows;
violin_half_width = 0.38;

for k = 1:n_rows
    vals = data_per_cfg{k};
    if isempty(vals) || numel(vals) < 5, continue; end
    
    xc = positions(k);
    c = colors_used(k, :);
    
    % --- 1) Violin via KDE (entire distribution, then clip to y_top for display) ---
    bw_kde = max(0.5, std(vals) * 0.15);
    [f, xi] = ksdensity(vals, 'Support', [-eps, max(vals)*1.05], ...
                        'BoundaryCorrection', 'reflection', ...
                        'NumPoints', 300, 'Bandwidth', bw_kde);
    
    % Clip the KDE arrays to within visible y range (so violin ends cleanly at y_top)
    keep = (xi >= 0) & (xi <= y_top);
    if ~any(keep), continue; end
    xi_d = xi(keep);
    f_d = f(keep);
    
    % Force violin endpoints to 0 width so polygon closes nicely
    if xi_d(1) > 0
        xi_d = [0, xi_d]; f_d = [0, f_d];
    end
    if xi_d(end) < y_top
        xi_d = [xi_d, xi_d(end)]; f_d = [f_d, 0];
    else
        % Pinch top so it doesn't go past y_top
        f_d(end) = min(f_d(end), f_d(max(end-1,1)) * 0.5);
    end
    
    % Normalize width using ORIGINAL max f (not clipped) to preserve density meaning
    f_max_orig = max(f);
    if f_max_orig > 0
        f_d = f_d / f_max_orig * violin_half_width;
    end
    
    % Draw violin (mirrored KDE)
    fill(ax, [xc + f_d, fliplr(xc - f_d)], [xi_d, fliplr(xi_d)], c, ...
         'FaceAlpha', 0.32, 'EdgeColor', c, 'LineWidth', 1.4);
    
    % Dotted vertical line showing min->min(max, y_top) (visible range whisker)
    plot(ax, [xc, xc], [min(vals), min(maxvals(k), y_top)], ':', ...
         'Color', c*0.5, 'LineWidth', 0.8);
    
    % --- 2) Internal box (IQR + median) ---
    q = quantile(vals, [0.25, 0.5, 0.75]);
    box_half = 0.07;
    
    % White IQR box (high contrast vs violin)
    fill(ax, [xc-box_half, xc+box_half, xc+box_half, xc-box_half], ...
         [q(1), q(1), q(3), q(3)], 'w', ...
         'FaceAlpha', 0.88, 'EdgeColor', c*0.45, 'LineWidth', 1.6);
    
    % Median bar (thick, dark)
    plot(ax, [xc-box_half*1.4, xc+box_half*1.4], [q(2), q(2)], '-', ...
         'Color', c*0.3, 'LineWidth', 3.0);
    
    % --- 3) Labels ---
    text(ax, xc + box_half + 0.03, q(2), sprintf(' med=%.1f', q(2)), ...
         'FontSize', 10.5, 'FontWeight', 'bold', 'Color', c*0.25, ...
         'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    
    text(ax, xc, y_bot * 0.55, sprintf('N=%d', numel(vals)), ...
         'FontSize', 9, 'Color', TXT_COLOR*1.6, 'FontAngle', 'italic', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    
    % If true max exceeds visible range, annotate it
    if maxvals(k) > y_top
        text(ax, xc, y_top * 0.94, sprintf('max=%.0f', maxvals(k)), ...
             'FontSize', 9, 'Color', c*0.5, 'FontAngle', 'italic', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    end
end

% Zero line for visual anchor
plot(ax, [0.4, n_rows+0.6], [0, 0], '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.7);

set(ax, 'XTick', positions, 'XTickLabel', labels_used);
xlim(ax, [0.4, n_rows+0.6]);
ylim(ax, [y_bot, y_top]);
ylabel(ax, 'Smooth-run length per path (mm)', 'FontSize', 13, 'FontName', FONT_NAME);
title(ax, sprintf('Distribution of Longest Smooth-Run per Path (turn<%d\\circ)', ...
                  opt.smooth_turn_deg), ...
      'FontSize', 14, 'FontWeight', 'bold', 'Color', TXT_COLOR);
explain_str = sprintf(['Violin: KDE of smooth-run lengths   |   ' ...
                       'White box: IQR (25%%-75%%)   |   bar: median   |   ' ...
                       'y-axis capped at %.0f mm'], y_top);
text(ax, mean(xlim(ax)), y_top * 1.04, explain_str, ...
     'HorizontalAlignment', 'center', 'FontSize', 9.5, ...
     'Color', TXT_COLOR*1.4, 'FontAngle', 'italic');

save_with_and_without_text(fig4, fullfile(opt.output_dir, 'fig_smooth_run_distribution.png'));

fprintf('\n=== compute_path_statistics DONE ===\n');
fprintf('All outputs in: %s\n\n', opt.output_dir);

end


function fig = make_angle_figure(angles_per_cfg, idx_map, COLORS, LABELS_FULL, ...
                                  n_rows, ideal_angle, do_shade, ...
                                  title_main, xlabel_str, ylabel_str, subtitle_str, ...
                                  FONT_NAME, TXT_COLOR, GRID_COLOR)
% Beautified angle histogram with optional 50% shading toward ideal angle.
%
% angles_per_cfg : cell array of column-vectors of angles (degrees)
% ideal_angle    : 0 or 90 (where the desirable angle is)
% do_shade       : true to shade the 50% region nearest to ideal_angle

fig = figure('Name', title_main, 'Position', [100 100 1100 620], 'Color', 'w');
ax = axes('Parent', fig);
set(ax, 'FontSize', 11.5, 'FontName', FONT_NAME, ...
        'TickDir', 'out', 'Box', 'off', 'LineWidth', 1.1, ...
        'XColor', TXT_COLOR, 'YColor', TXT_COLOR);
hold(ax, 'on');
grid(ax, 'on');
ax.GridColor = GRID_COLOR;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.5;
ax.Layer = 'top';

bins = 0:2:90;             % 2-degree bins (finer)
bin_centers = bins(1:end-1) + 1;

% Precompute
n = numel(angles_per_cfg);
hist_smoothed = cell(n, 1);
thresholds_50 = nan(n, 1);
for k = 1:n
    ang = angles_per_cfg{k};
    if isempty(ang), continue; end
    h = histcounts(ang, bins, 'Normalization', 'probability') * 100;
    % Light gaussian-ish smoothing
    h_smooth = movmean(h, 3, 'Endpoints', 'shrink');
    hist_smoothed{k} = h_smooth;
    thresholds_50(k) = median(ang);
end

% Determine y-range first
y_max = 0;
for k = 1:n
    if ~isempty(hist_smoothed{k})
        y_max = max(y_max, max(hist_smoothed{k}));
    end
end
y_top = y_max * 1.18;

% (1) Draw "ideal" vertical line first (background)
plot(ax, [ideal_angle, ideal_angle], [0, y_top], '-', ...
     'Color', [0.55 0.55 0.55], 'LineWidth', 1.4);
if ideal_angle == 0
    text(ax, 1.5, y_top * 0.97, 'Ideal', 'FontSize', 11, ...
         'Color', [0.45 0.45 0.45], 'FontAngle', 'italic', ...
         'HorizontalAlignment', 'left');
else
    text(ax, 88.5, y_top * 0.97, 'Ideal', 'FontSize', 11, ...
         'Color', [0.45 0.45 0.45], 'FontAngle', 'italic', ...
         'HorizontalAlignment', 'right');
end

% (2) Draw shaded 50% regions BEFORE the curves
if do_shade
    for k = 1:n
        h = hist_smoothed{k};
        if isempty(h), continue; end
        c = COLORS(idx_map(k), :);
        thr = thresholds_50(k);
        
        if ideal_angle == 90
            mask = bin_centers >= thr;
            if ~any(mask), continue; end
            sx = [bin_centers(mask), 90, thr];
            sy = [h(mask), 0, 0];
        else
            mask = bin_centers <= thr;
            if ~any(mask), continue; end
            sx = [bin_centers(mask), thr, 0];
            sy = [h(mask), 0, 0];
        end
        fill(ax, sx, sy, c, 'FaceAlpha', 0.16, 'EdgeColor', 'none');
    end
end

% (3) Draw histogram curves on top
hps = gobjects(n, 1);
legend_labels = {};
hps_valid = [];
for k = 1:n
    h = hist_smoothed{k};
    if isempty(h), continue; end
    c = COLORS(idx_map(k), :);
    hp = plot(ax, bin_centers, h, '-', 'Color', c, 'LineWidth', 2.6);
    hps(k) = hp;
    hps_valid(end+1) = hp;
    if do_shade
        legend_labels{end+1} = sprintf('%s  (median: %.1f\\circ)', ...
            LABELS_FULL{idx_map(k)}, thresholds_50(k));
    else
        legend_labels{end+1} = LABELS_FULL{idx_map(k)};
    end
end

% (4) Threshold markers (small triangles at each config's median on x-axis)
if do_shade
    for k = 1:n
        if isnan(thresholds_50(k)), continue; end
        c = COLORS(idx_map(k), :);
        plot(ax, thresholds_50(k), 0, 'v', ...
             'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 9, ...
             'LineWidth', 1.3);
    end
end

xlabel(ax, xlabel_str, 'FontSize', 13, 'FontName', FONT_NAME);
ylabel(ax, ylabel_str, 'FontSize', 13, 'FontName', FONT_NAME);
title(ax, title_main, 'FontSize', 14.5, 'FontWeight', 'bold', 'Color', TXT_COLOR);
text(ax, 45, y_top * 1.04, subtitle_str, ...
     'HorizontalAlignment', 'center', 'FontSize', 10.5, ...
     'Color', TXT_COLOR*1.4, 'FontAngle', 'italic', 'Parent', ax);

legend(ax, hps_valid, legend_labels, ...
       'Location', 'best', 'FontSize', 10.5, 'Box', 'off');

xlim(ax, [0 90]);
ylim(ax, [0 y_top]);
set(ax, 'XTick', 0:10:90);
end


%% ============================================================
%% Helper functions
%% ============================================================
function p = find_in_dirs(filename, dirs)
p = '';
for k = 1:numel(dirs)
    if isempty(dirs{k}), continue; end
    candidate = fullfile(dirs{k}, filename);
    if exist(candidate, 'file')
        p = candidate; return;
    end
end
if exist(filename, 'file')
    pp = which(filename);
    if ~isempty(pp), p = pp; else, p = filename; end
end
end


function stress = load_stress_field(explicit_file, search_dirs)
% Loads stress field. Supports two formats:
%
%   Format A: voxel_refined_latest.mat
%     refined_data.grid_data.{x, y, z, xPhys, t_xoy, t_xoz, uu, vv, ww, is_valid}
%     -- per-voxel flat arrays; we reshape into 3D for fast lookup
%
%   Format B: topo_stress_result.mat
%     top-level {t_xoy, t_xoz, mask, nelx, nely, nelz, voxel_size, origin_xyz}
%     -- already 3D

candidates = {};
if ~isempty(explicit_file), candidates{end+1} = explicit_file; end
candidates = [candidates, {
    'voxel_refined_latest.mat', ...
    'voxel_refined_latest_PLANAR.mat', ...
    'topo_stress_result.mat', ...
    'voxel_stress_field.mat', ...
    'stress_field.mat', ...
    'Densities_with_stress.mat', ...
    'topology_stress.mat', ...
    'stress_angles.mat', ...
    'stress_orientation.mat', ...
}];

stress = [];
fprintf('  [stress] looking for stress field...\n');
for c = 1:numel(candidates)
    fp = find_in_dirs(candidates{c}, search_dirs);
    if isempty(fp), continue; end
    fprintf('  [stress] found candidate: %s\n', fp);
    
    try, S = load(fp); catch ME
        fprintf('  [stress] load failed: %s\n', ME.message);
        continue;
    end
    
    %% Format A: refined_data nested struct
    if isfield(S, 'refined_data') && isstruct(S.refined_data) && ...
       isfield(S.refined_data, 'grid_data') && isstruct(S.refined_data.grid_data)
        gd = S.refined_data.grid_data;
        
        % required fields
        req = {'x', 'y', 'z', 'uu', 'vv', 'ww', 'is_valid'};
        ok = true;
        for r = 1:numel(req)
            if ~isfield(gd, req{r})
                fprintf('  [stress] grid_data missing field: %s\n', req{r});
                ok = false; break;
            end
        end
        if ~ok, continue; end
        
        % Extract fields. gd might be:
        %   - scalar struct (1x1) with array-valued fields:  gd.x is Nx1
        %   - struct array (Nx1) with scalar-valued fields:  gd.x must use [gd.x]
        if numel(gd) > 1
            fprintf('  [stress] grid_data is a struct array of %d elements\n', numel(gd));
            x_arr = double([gd.x]).';
            y_arr = double([gd.y]).';
            z_arr = double([gd.z]).';
            uu_arr = double([gd.uu]).';
            vv_arr = double([gd.vv]).';
            ww_arr = double([gd.ww]).';
            valid_arr = double([gd.is_valid]).' > 0.5;
        else
            fprintf('  [stress] grid_data is a scalar struct with array fields\n');
            x_arr = double(gd.x(:));
            y_arr = double(gd.y(:));
            z_arr = double(gd.z(:));
            uu_arr = double(gd.uu(:));
            vv_arr = double(gd.vv(:));
            ww_arr = double(gd.ww(:));
            valid_arr = double(gd.is_valid(:)) > 0.5;
        end
        
        % Sanity check: all arrays same length
        N = numel(x_arr);
        if numel(y_arr) ~= N || numel(z_arr) ~= N || ...
           numel(uu_arr) ~= N || numel(vv_arr) ~= N || numel(ww_arr) ~= N
            fprintf('  [stress] field length mismatch (x=%d y=%d z=%d uu=%d vv=%d ww=%d)\n', ...
                N, numel(y_arr), numel(z_arr), numel(uu_arr), numel(vv_arr), numel(ww_arr));
            stress = [];
            continue;
        end
        fprintf('  [stress] extracted %d grid points\n', N);
        
        % Determine grid: spacing = min positive diff of unique values
        ux = unique(x_arr); uy = unique(y_arr); uz = unique(z_arr);
        if numel(ux) < 2 || numel(uy) < 2 || numel(uz) < 2
            fprintf('  [stress] grid too small (nx=%d ny=%d nz=%d)\n', ...
                numel(ux), numel(uy), numel(uz));
            continue;
        end
        dx = median(diff(ux));
        dy = median(diff(uy));
        dz = median(diff(uz));
        x_min = min(ux); y_min = min(uy); z_min = min(uz);
        x_max = max(ux); y_max = max(uy); z_max = max(uz);
        
        nelx = round((x_max - x_min) / dx) + 1;
        nely = round((y_max - y_min) / dy) + 1;
        nelz = round((z_max - z_min) / dz) + 1;
        
        % Build 3D arrays
        uu_3d = nan(nely, nelx, nelz);
        vv_3d = nan(nely, nelx, nelz);
        ww_3d = nan(nely, nelx, nelz);
        valid_3d = false(nely, nelx, nelz);
        
        ii = round((x_arr - x_min) / dx) + 1;
        jj = round((y_arr - y_min) / dy) + 1;
        kk = round((z_arr - z_min) / dz) + 1;
        in = ii >= 1 & ii <= nelx & jj >= 1 & jj <= nely & kk >= 1 & kk <= nelz;
        
        linidx = sub2ind([nely nelx nelz], jj(in), ii(in), kk(in));
        uu_3d(linidx) = uu_arr(in);
        vv_3d(linidx) = vv_arr(in);
        ww_3d(linidx) = ww_arr(in);
        valid_3d(linidx) = valid_arr(in);
        
        stress.format = 'refined_data';
        stress.uu_3d = uu_3d;
        stress.vv_3d = vv_3d;
        stress.ww_3d = ww_3d;
        stress.mask = valid_3d;
        stress.nelx = nelx;
        stress.nely = nely;
        stress.nelz = nelz;
        stress.voxel_size = mean([dx dy dz]);
        % voxel centers go: x_min, x_min+dx, ..., x_min+(nelx-1)*dx
        % so origin of voxel(1,1,1) "corner" = (x_min - dx/2, ...)
        stress.origin_xyz = [x_min - dx/2, y_min - dy/2, z_min - dz/2];
        stress.x_min = x_min; stress.dx = dx;
        stress.y_min = y_min; stress.dy = dy;
        stress.z_min = z_min; stress.dz = dz;
        
        fprintf('  [stress] FORMAT A (refined_data) loaded from: %s\n', fp);
        fprintf('     dims [nely nelx nelz] = [%d %d %d]\n', nely, nelx, nelz);
        fprintf('     spacings dx=%.3f dy=%.3f dz=%.3f mm\n', dx, dy, dz);
        fprintf('     voxel centers x in [%.2f, %.2f]\n', x_min, x_max);
        fprintf('                    y in [%.2f, %.2f]\n', y_min, y_max);
        fprintf('                    z in [%.2f, %.2f]\n', z_min, z_max);
        fprintf('     valid voxels: %d / %d (%.1f%%)\n', ...
            nnz(valid_3d), numel(valid_3d), 100*nnz(valid_3d)/numel(valid_3d));
        return;
    end
    
    %% Format B: flat top-level (topo_stress_result.mat)
    have_angles = false;
    if isfield(S, 't_xoy') && isfield(S, 't_xoz')
        stress.theta_xoy = S.t_xoy;
        stress.theta_xoz = S.t_xoz;
        have_angles = true;
    elseif isfield(S, 'theta_xoy') && isfield(S, 'theta_xoz')
        stress.theta_xoy = S.theta_xoy;
        stress.theta_xoz = S.theta_xoz;
        have_angles = true;
    end
    
    if ~have_angles
        fprintf('  [stress] file does not have known format, skipping\n');
        stress = [];
        continue;
    end
    
    if isfield(S, 'mask'), stress.mask = S.mask; end
    [stress.nely, stress.nelx, stress.nelz] = size(stress.theta_xoy);
    if isfield(S, 'voxel_size'), stress.voxel_size = double(S.voxel_size);
    else, stress.voxel_size = 1.0; end
    if isfield(S, 'origin_xyz'), stress.origin_xyz = double(S.origin_xyz(:)');
    else, stress.origin_xyz = [0 0 0]; end
    stress.format = 'topo_stress_result';
    
    fprintf('  [stress] FORMAT B (topo_stress_result) loaded from: %s\n', fp);
    fprintf('     dims [nely nelx nelz] = [%d %d %d]\n', stress.nely, stress.nelx, stress.nelz);
    return;
end

fprintf('  [stress] NONE of the candidates were found in any search dir\n');
fprintf('  [stress] candidates tried:\n');
for c = 1:numel(candidates)
    fprintf('           %s\n', candidates{c});
end
end


function v = lookup_stress_at_point(stress, point, voxel_size_unused)
% Look up stress direction vector at a 3D point in WORLD coordinates (mm).
% Returns [NaN NaN NaN] if outside grid or outside mask.
v = [NaN NaN NaN];

%% Format A: refined_data (uu/vv/ww already computed per voxel)
if isfield(stress, 'format') && strcmp(stress.format, 'refined_data')
    i = round((point(1) - stress.x_min) / stress.dx) + 1;
    j = round((point(2) - stress.y_min) / stress.dy) + 1;
    k = round((point(3) - stress.z_min) / stress.dz) + 1;
    if i < 1 || i > stress.nelx, return; end
    if j < 1 || j > stress.nely, return; end
    if k < 1 || k > stress.nelz, return; end
    if ~stress.mask(j, i, k), return; end
    
    u = stress.uu_3d(j, i, k);
    vv = stress.vv_3d(j, i, k);
    w = stress.ww_3d(j, i, k);
    if isnan(u) || isnan(vv) || isnan(w), return; end
    if u == 0 && vv == 0 && w == 0, return; end
    v = [u, vv, w];
    return;
end

%% Format B: topo_stress_result (angles in 3D arrays)
vs = stress.voxel_size;
ox = stress.origin_xyz(1);
oy = stress.origin_xyz(2);
oz = stress.origin_xyz(3);

i = round((point(1) - ox) / vs + 0.5);
j = round((point(2) - oy) / vs + 0.5);
k = round((point(3) - oz) / vs + 0.5);

if i < 1 || i > stress.nelx, return; end
if j < 1 || j > stress.nely, return; end
if k < 1 || k > stress.nelz, return; end

if isfield(stress, 'mask')
    if stress.mask(j, i, k) <= 0.5, return; end
end

if isfield(stress, 'theta_xoy')
    t_xoy = stress.theta_xoy(j, i, k);
    t_xoz = stress.theta_xoz(j, i, k);
    if isnan(t_xoy) || isnan(t_xoz), return; end
    if ~isfield(stress, 'mask') && t_xoy == 0 && t_xoz == 0, return; end
    u  = cosd(t_xoz) * cosd(t_xoy);
    vv = cosd(t_xoz) * sind(t_xoy);
    w  = sind(t_xoz);
    v = [u, vv, w];
end
end


function out = get_field(s, cfg, fld, default)
if isfield(s, cfg) && isfield(s.(cfg), fld)
    out = s.(cfg).(fld);
    if isempty(out), out = default; end
else
    out = default;
end
end


function save_with_and_without_text(fig, output_path)
% Save the figure twice:
%   (1) output_path           - normal version with all text
%   (2) output_path_notext    - same figure with all text stripped
% The notext version is for paper/poster composition where you want to add
% your own labels in InDesign/PowerPoint.

% First: save the normal version
saveas(fig, output_path);
[~, fn, ~] = fileparts(output_path);
fprintf('Saved: %s.png\n', fn);

% Then: strip ALL text from the figure and save again
[p, n, e] = fileparts(output_path);
notext_path = fullfile(p, [n '_notext' e]);

% Get all axes (subplots have multiple)
ax_list = findall(fig, 'Type', 'axes');
for k = 1:numel(ax_list)
    ax = ax_list(k);
    title(ax, '');
    xlabel(ax, '');
    ylabel(ax, '');
    set(ax, 'XTickLabel', {});
    set(ax, 'YTickLabel', {});
end

% Delete all standalone text annotations
text_objs = findall(fig, 'Type', 'text');
delete(text_objs);

% Delete legend
lgd_objs = findall(fig, 'Type', 'Legend');
delete(lgd_objs);

saveas(fig, notext_path);
fprintf('Also saved (text-free): %s_notext.png\n', n);
end


function paths = extract_paths_robust(S, mat_file_for_msg)
% Try multiple data-structure conventions to flatten paths into a cell array
% of Nx3 numeric arrays.
%
% Supported:
%   1. S.paths_only  is cell of Nx3                     (simple cell)
%   2. S.paths_only  is cell of cell of Nx3              (per-layer cell of paths)
%   3. S.paths_only  is struct(1,K) with .points/.xyz/.coords    (struct array)
%   4. S.paths_only  is struct(1,1) with sub-fields containing cells
%   5. Top-level S has multiple fields like S.layer1, S.layer2 each with cells
%   6. S.all_paths_only (alternate name)

paths = {};

% Step 1: identify the top-level field
field_candidates = {'paths_only', 'all_paths_only', 'paths', 'all_paths', ...
                    'stream_paths', 'beam_paths', 'offset_paths'};
field_used = '';
val = [];
for k = 1:numel(field_candidates)
    fn = field_candidates{k};
    if isfield(S, fn)
        val = S.(fn);
        field_used = fn;
        break;
    end
end

if isempty(val)
    % Maybe paths are stored at top-level (one cell per layer, no wrapper)
    f = fieldnames(S);
    fprintf('  [diag] no standard path field; top-level fields: %s\n', ...
        strjoin(f, ', '));
    return;
end

fprintf('  [diag] using field "%s" (class=%s, size=%s)\n', ...
    field_used, class(val), mat2str(size(val)));

paths = flatten_any(val);

if isempty(paths)
    fprintf('  [diag] extractor returned nothing -- dumping structure:\n');
    dump_struct(val, '    ', 3);
    fprintf('  [diag] Please tell Claude what format this is.\n');
    fprintf('  [diag] Specifically: open in MATLAB and inspect %s.(%s)\n', ...
        mat_file_for_msg, field_used);
else
    n_pts_total = sum(cellfun(@(p) size(p, 1), paths));
    fprintf('  [diag] extracted %d paths, total %d points\n', numel(paths), n_pts_total);
end
end


function paths = flatten_any(val)
% Recursively flatten any structure into a flat cell array of Nx3 matrices.
paths = {};

if isnumeric(val)
    if size(val, 2) >= 3 && size(val, 1) >= 2
        % It's already a single Nx3 path
        paths = {val(:, 1:3)};
    elseif size(val, 1) >= 3 && size(val, 2) >= 2
        % Maybe transposed (3xN)
        paths = {val(1:3, :)'};
    end
    return;
end

if iscell(val)
    for k = 1:numel(val)
        sub = flatten_any(val{k});
        paths = [paths; sub(:)];
    end
    return;
end

if isstruct(val)
    if numel(val) > 1
        % Struct array
        for k = 1:numel(val)
            sub = flatten_struct_one(val(k));
            paths = [paths; sub(:)];
        end
    else
        % Single struct -- look at fields
        paths = flatten_struct_one(val);
    end
    return;
end

if istable(val)
    % Table -- look for xyz columns
    cols = lower(val.Properties.VariableNames);
    if all(ismember({'x','y','z'}, cols))
        paths = {[val.x, val.y, val.z]};
    end
    return;
end
end


function paths = flatten_struct_one(s)
% Given a single struct, find which field holds the path coordinates.
paths = {};
preferred_subfields = {'points', 'pts', 'xyz', 'coords', 'path', 'data', 'P', 'XYZ'};

% First try preferred fields
for k = 1:numel(preferred_subfields)
    sk = preferred_subfields{k};
    if isfield(s, sk)
        sub = flatten_any(s.(sk));
        if ~isempty(sub)
            paths = [paths; sub(:)];
            return;
        end
    end
end

% Fall back: recurse into every field of the struct
f = fieldnames(s);
for k = 1:numel(f)
    val = s.(f{k});
    if isnumeric(val) || iscell(val) || isstruct(val)
        sub = flatten_any(val);
        paths = [paths; sub(:)];
    end
end
end


function dump_struct(val, indent, max_depth)
% Print structure recursively for diagnostic
if max_depth <= 0
    fprintf('%s  ... (max depth)\n', indent); return;
end
if isstruct(val)
    if numel(val) > 1
        fprintf('%sstruct array, numel=%d\n', indent, numel(val));
        if numel(val) >= 1
            fprintf('%s  element(1):\n', indent);
            dump_struct(val(1), [indent '    '], max_depth - 1);
        end
    else
        f = fieldnames(val);
        fprintf('%sstruct with %d fields:\n', indent, numel(f));
        for k = 1:min(numel(f), 8)
            sub = val.(f{k});
            fprintf('%s  %s: %s, size=%s\n', indent, f{k}, class(sub), mat2str(size(sub)));
            if (isstruct(sub) || iscell(sub)) && max_depth > 1
                dump_struct(sub, [indent '    '], max_depth - 1);
            end
        end
        if numel(f) > 8, fprintf('%s  ... (%d more fields)\n', indent, numel(f)-8); end
    end
elseif iscell(val)
    fprintf('%scell, size=%s\n', indent, mat2str(size(val)));
    if numel(val) >= 1
        fprintf('%s  element{1}: %s, size=%s\n', indent, class(val{1}), mat2str(size(val{1})));
        if numel(val) >= 1 && (isstruct(val{1}) || iscell(val{1})) && max_depth > 1
            dump_struct(val{1}, [indent '    '], max_depth - 1);
        end
    end
else
    fprintf('%s%s, size=%s\n', indent, class(val), mat2str(size(val)));
end
end