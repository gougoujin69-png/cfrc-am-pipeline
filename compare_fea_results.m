%% ==========================================================================
%% compare_fea_results.m  (4-way version, 2026-05-20)
%% --------------------------------------------------------------------------
%% 4-way FEA Stiffness Comparison Visualization
%%
%% 输入：  extract_fea_results.py 生成的
%%         C:/temp/cfrc_fea/results/<config>_time_history.csv
%%         C:/temp/cfrc_fea/results/summary.txt
%%
%% 输出：  - 4 条 F-U 曲线叠加图
%%         - K (刚度) 柱状对比图
%%         - |U3|_max、max Mises 柱状对比图
%%         - 相对 planar_offset baseline 的改进百分比
%% --------------------------------------------------------------------------

clear; close all; clc;

%% ========== 配置 ==========
RESULTS_DIR = 'C:\temp\cfrc_fea\results';
CONFIGS = {'mine_stream', 'mine_offset', 'planar_stream', 'planar_offset'};
CONFIG_LABELS = {'MINE + Stream', 'MINE + Offset', 'Planar + Stream', 'Planar + Offset'};
COLORS = [
    0.90 0.15 0.15;   % mine_stream     红
    0.95 0.55 0.15;   % mine_offset     橙
    0.15 0.55 0.90;   % planar_stream   蓝
    0.55 0.55 0.55    % planar_offset   灰 (baseline)
];
LINE_STYLES = {'-', '--', '-', ':'};
BASELINE = 'planar_offset';  % 以此为 100% baseline 计算相对改进

%% ========== 加载数据 ==========
fprintf('Loading FEA results from: %s\n', RESULTS_DIR);
if ~exist(RESULTS_DIR, 'dir')
    error('Results directory not found: %s\n  -> 先运行 abaqus cae noGUI=extract_fea_results.py', ...
          RESULTS_DIR);
end

data = struct();
for i = 1:length(CONFIGS)
    cname = CONFIGS{i};
    csv_file = fullfile(RESULTS_DIR, [cname '_time_history.csv']);
    
    if ~exist(csv_file, 'file')
        fprintf('  [SKIP] %s: no CSV\n', cname);
        data.(cname).has_data = false;
        continue;
    end
    
    T = readtable(csv_file);
    data.(cname).has_data = true;
    data.(cname).time       = T.time;
    data.(cname).F_applied  = T.F_applied_N;
    data.(cname).u_loadpt   = T.u_loadpt_mm;
    data.(cname).U3         = T.U3_mm;
    data.(cname).RF3        = T.RF3_N;
    data.(cname).CF3        = T.CF3_N;
    
    fprintf('  [OK]  %s: %d increments\n', cname, height(T));
end

% 加载 summary.txt
summary_file = fullfile(RESULTS_DIR, 'summary.txt');
if ~exist(summary_file, 'file')
    warning('summary.txt 不存在，将从 CSV 重新计算统计量');
    summary = [];
else
    summary = read_summary(summary_file);
end

%% ========== 统计 K、max_U3、max_Mises ==========
fprintf('\n========== Results Summary ==========\n');
fprintf('%-18s %-12s %-12s %-14s %-12s\n', 'Config', 'K(N/mm)', '|U3|_max', 'Mises_max', 'Beams');
fprintf('%s\n', repmat('-', 1, 75));

stiffness_K = zeros(1, length(CONFIGS));
max_u3      = zeros(1, length(CONFIGS));
max_mises   = zeros(1, length(CONFIGS));
max_sf1     = zeros(1, length(CONFIGS));
num_beams   = zeros(1, length(CONFIGS));

for i = 1:length(CONFIGS)
    cname = CONFIGS{i};
    if ~data.(cname).has_data
        continue;
    end
    
    % 从 summary 或 CSV 计算 K
    if ~isempty(summary) && isfield(summary, cname)
        stiffness_K(i) = summary.(cname).K_linear_Nmm;
        max_u3(i)      = summary.(cname).max_u3_abs_mm;
        max_mises(i)   = summary.(cname).max_mises_MPa;
        max_sf1(i)     = summary.(cname).max_sf1_N;
        num_beams(i)   = summary.(cname).num_beam_elems;
    else
        % 从 CSV 重算
        F = abs(data.(cname).F_applied);
        u = abs(data.(cname).u_loadpt);
        valid = u > 1e-10;
        if any(valid)
            stiffness_K(i) = sum(F(valid).*u(valid)) / sum(u(valid).^2);
        end
        max_u3(i) = max(abs(data.(cname).U3));
    end
    
    fprintf('%-18s %-12.4f %-12.4e %-14.2f %-12d\n', ...
            cname, stiffness_K(i), max_u3(i), max_mises(i), num_beams(i));
end

%% ========== 图 1：F-U 对比曲线 ==========
fig1 = figure('Name', 'F-U Comparison', 'Position', [100 100 1000 650]);
hold on; box on; grid on;

legend_labels = {};
for i = 1:length(CONFIGS)
    cname = CONFIGS{i};
    if ~data.(cname).has_data, continue; end
    u = abs(data.(cname).u_loadpt);
    F = abs(data.(cname).F_applied);
    plot(u, F, LINE_STYLES{i}, 'Color', COLORS(i,:), 'LineWidth', 2.5, ...
         'Marker', 'o', 'MarkerSize', 5, 'MarkerFaceColor', COLORS(i,:));
    legend_labels{end+1} = sprintf('%s  (K=%.2f N/mm)', CONFIG_LABELS{i}, stiffness_K(i));
end
xlabel('|u| at Load Point (mm)', 'FontSize', 12);
ylabel('|F| Applied (N)', 'FontSize', 12);
title('CFRC 4-way Comparison: Force-Displacement Curves', 'FontSize', 13);
legend(legend_labels, 'Location', 'northwest', 'FontSize', 10);
set(gca, 'FontSize', 11);

saveas(fig1, fullfile(RESULTS_DIR, 'fig1_FU_curves.png'));
fprintf('\nSaved: fig1_FU_curves.png\n');

%% ========== 图 2：刚度 K 柱状图 ==========
fig2 = figure('Name', 'Stiffness Comparison', 'Position', [100 100 900 600]);

subplot(1,2,1);
b = bar(stiffness_K, 'FaceColor', 'flat');
for i = 1:length(CONFIGS), b.CData(i,:) = COLORS(i,:); end
set(gca, 'XTickLabel', CONFIG_LABELS, 'XTickLabelRotation', 20, 'FontSize', 10);
ylabel('K (N/mm)', 'FontSize', 12);
title('Linear-fit Stiffness', 'FontSize', 12);
grid on; box on;
for i = 1:length(stiffness_K)
    if stiffness_K(i) > 0
        text(i, stiffness_K(i)*1.02, sprintf('%.2f', stiffness_K(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    end
end

% 相对 baseline 的改进率
baseline_idx = find(strcmp(CONFIGS, BASELINE), 1);
if ~isempty(baseline_idx) && stiffness_K(baseline_idx) > 0
    rel_K = 100 * (stiffness_K - stiffness_K(baseline_idx)) / stiffness_K(baseline_idx);
    subplot(1,2,2);
    b2 = bar(rel_K, 'FaceColor', 'flat');
    for i = 1:length(CONFIGS), b2.CData(i,:) = COLORS(i,:); end
    set(gca, 'XTickLabel', CONFIG_LABELS, 'XTickLabelRotation', 20, 'FontSize', 10);
    ylabel(sprintf('\\Delta K / K_{%s} (%%)', strrep(BASELINE, '_', '\_')), 'FontSize', 12);
    title('Relative Improvement vs Baseline', 'FontSize', 12);
    grid on; box on;
    yline(0, 'k--');
    for i = 1:length(rel_K)
        text(i, rel_K(i)+sign(rel_K(i))*2, sprintf('%+.1f%%', rel_K(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    end
end

sgtitle('Structural Stiffness K = \Sigma(F u) / \Sigma(u^2)', 'FontSize', 13);
saveas(fig2, fullfile(RESULTS_DIR, 'fig2_K_comparison.png'));
fprintf('Saved: fig2_K_comparison.png\n');

%% ========== 图 3：其他对比量 ==========
fig3 = figure('Name', 'Secondary Metrics', 'Position', [100 100 1200 400]);

subplot(1,3,1);
b = bar(max_u3*1e3, 'FaceColor', 'flat');
for i = 1:length(CONFIGS), b.CData(i,:) = COLORS(i,:); end
set(gca, 'XTickLabel', CONFIG_LABELS, 'XTickLabelRotation', 20, 'FontSize', 9);
ylabel('|U3|_{max} (\mum)', 'FontSize', 11);
title('Max Deflection at Load Point', 'FontSize', 11);
grid on; box on;

subplot(1,3,2);
b = bar(max_mises, 'FaceColor', 'flat');
for i = 1:length(CONFIGS), b.CData(i,:) = COLORS(i,:); end
set(gca, 'XTickLabel', CONFIG_LABELS, 'XTickLabelRotation', 20, 'FontSize', 9);
ylabel('Mises_{max} (MPa)', 'FontSize', 11);
title('Max Mises Stress in Matrix', 'FontSize', 11);
grid on; box on;

subplot(1,3,3);
b = bar(num_beams, 'FaceColor', 'flat');
for i = 1:length(CONFIGS), b.CData(i,:) = COLORS(i,:); end
set(gca, 'XTickLabel', CONFIG_LABELS, 'XTickLabelRotation', 20, 'FontSize', 9);
ylabel('# Beam Elements', 'FontSize', 11);
title('Fiber Reinforcement Count', 'FontSize', 11);
grid on; box on;

saveas(fig3, fullfile(RESULTS_DIR, 'fig3_secondary_metrics.png'));
fprintf('Saved: fig3_secondary_metrics.png\n');

%% ========== 图 4：综合对比雷达图 ==========
if all(stiffness_K > 0) && all(max_u3 > 0)
    fig4 = figure('Name', 'Radar Comparison', 'Position', [100 100 700 700]);
    
    % 归一化（越大越好的量）
    norm_K      = stiffness_K / max(stiffness_K);
    norm_stiff  = (1./max_u3) / max(1./max_u3);
    norm_stress = (1./max(max_mises, eps)) / max(1./max(max_mises, eps));
    
    metrics = [norm_K; norm_stiff; norm_stress];
    metric_labels = {'Stiffness K', '1/Deflection', '1/Peak Stress'};
    
    angles = linspace(0, 2*pi, size(metrics,1)+1);
    
    polar_ax = polaraxes();
    hold(polar_ax, 'on');
    for i = 1:length(CONFIGS)
        vals = [metrics(:,i); metrics(1,i)];
        polarplot(polar_ax, angles, vals, '-o', ...
                  'Color', COLORS(i,:), 'LineWidth', 2, ...
                  'MarkerFaceColor', COLORS(i,:), 'MarkerSize', 6);
    end
    polar_ax.ThetaTick = rad2deg(angles(1:end-1));
    polar_ax.ThetaTickLabel = metric_labels;
    polar_ax.RLim = [0 1.1];
    polar_ax.RTick = 0:0.2:1;
    title('Normalized Performance Comparison (outer = better)', 'FontSize', 13);
    legend(CONFIG_LABELS, 'Location', 'southoutside', 'FontSize', 10);
    
    saveas(fig4, fullfile(RESULTS_DIR, 'fig4_radar.png'));
    fprintf('Saved: fig4_radar.png\n');
end

%% ========== 输出 markdown 对比表 ==========
md_file = fullfile(RESULTS_DIR, 'comparison_table.md');
fid = fopen(md_file, 'w');
fprintf(fid, '# CFRC 4-way FEA Comparison\n\n');
fprintf(fid, '| Config | K (N/mm) | \\|U3\\|_max (mm) | Mises_max (MPa) | Max SF1 (N) | # Beams |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|\n');
for i = 1:length(CONFIGS)
    fprintf(fid, '| %s | %.4f | %.4e | %.2f | %.3f | %d |\n', ...
            CONFIG_LABELS{i}, stiffness_K(i), max_u3(i), ...
            max_mises(i), max_sf1(i), num_beams(i));
end
if ~isempty(baseline_idx) && stiffness_K(baseline_idx) > 0
    fprintf(fid, '\n## Relative Improvement vs %s\n\n', CONFIG_LABELS{baseline_idx});
    rel_K = 100 * (stiffness_K - stiffness_K(baseline_idx)) / stiffness_K(baseline_idx);
    for i = 1:length(CONFIGS)
        fprintf(fid, '- **%s**: %+.2f%%\n', CONFIG_LABELS{i}, rel_K(i));
    end
end
fclose(fid);
fprintf('\nSaved markdown table: %s\n', md_file);

fprintf('\n=== 完成 ===\n');
fprintf('所有图片和表格已保存到：%s\n', RESULTS_DIR);


%% ==========================================================================
%% Local functions
%% ==========================================================================
function s = read_summary(path)
% 解析 summary.txt 的 CSV-like 数据行
    s = struct();
    fid = fopen(path, 'r');
    headers = {};
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line), continue; end
        if startsWith(line, '#')
            tmp = strtrim(line(2:end));
            parts = split(tmp, ',');
            parts = strtrim(parts);
            if length(parts) > 3 && contains(tmp, 'K_linear')
                headers = parts;
            end
            continue;
        end
        parts = split(line, ',');
        parts = strtrim(parts);
        if length(parts) < 3, continue; end
        cname = char(parts{1});
        if strcmpi(strtrim(parts{2}), 'OK')
            if isempty(headers)
                headers = {'config', 'status', 'K_linear_Nmm', 'K_end_Nmm', ...
                           'max_u3_abs_mm', 'max_u_mag_mm', 'max_mises_MPa', ...
                           'max_sf1_N', 'num_beam_elems', 'num_host_elems', 'num_load_nodes'};
            end
            for k = 3:length(parts)
                if k > length(headers), break; end
                field = matlab.lang.makeValidName(headers{k});
                s.(cname).(field) = str2double(parts{k});
            end
        end
    end
    fclose(fid);
end
