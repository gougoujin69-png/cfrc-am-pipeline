%% debug_layer_coverage.m
%% ======================================================================
%% 逐层诊断可视化：曲面 + 激活网格 + 轮廓 + 路径
%%
%% 目的：定位路径未覆盖区域的具体原因
%%   - 红色方块 = 该层激活了但路径没覆盖的网格 (gap!)
%%   - 绿色方块 = 该层激活且被路径覆盖的网格
%%   - 灰色方块 = 全局有效但该层未激活的网格
%%   - 蓝线 = 外轮廓, 红线 = 内轮廓, 黑线 = 路径
%%
%% Usage:
%%   1. 修改 slice_mat / path_mat / data_dir 路径
%%   2. 直接运行
%% ======================================================================

clear; clc; close all;

fprintf('====================================================\n');
fprintf('  Layer-by-Layer Coverage Diagnostic\n');
fprintf('====================================================\n\n');

%% ========== 配置 ==========
% --- 文件路径 ---
slice_mat = 'slice_results_refined_latest.mat';   % 切片结果
path_mat  = 'all_layers_path_results_v3.mat';     % 路径结果
data_dir  = 'C:\temp\abaqus_topo';                % 密度场

% --- 可视化参数 ---
COVERAGE_TOL = 0.8;   % 路径点到网格中心的距离阈值(判断是否覆盖)
VOXEL_SIZE   = 0.3;   % 网格方块半宽(仅用于2D可视化)

%% ========== 加载切片数据 ==========
fprintf('[1] Loading slice results...\n');
tmp = load(slice_mat);
slice_results = tmp.slice_results;
surface_layers = slice_results.surface_layers;
grid_data      = slice_results.grid_data;
valid_mask     = slice_results.valid_grid_mask;
global_act     = slice_results.global_activated;
nelx = slice_results.grid_size.nelx;
nely = slice_results.grid_size.nely;
nelz = slice_results.grid_size.nelz;
num_layers_slice = slice_results.statistics.num_layers;
fprintf('  Grid: %d x %d x %d, Layers: %d\n', nelx, nely, nelz, num_layers_slice);
fprintf('  Valid grids: %d, Global activated: %d (%.1f%%)\n', ...
    sum(valid_mask(:)), sum(global_act(:)), ...
    100*sum(global_act(:))/max(sum(valid_mask(:)),1));

%% ========== 加载路径数据 ==========
fprintf('[2] Loading path results...\n');
tmp = load(path_mat);
results = tmp.results;
all_layers_data = results.all_layers_data;
num_layers_path = results.num_layers;
fprintf('  Path layers: %d\n', num_layers_path);

num_layers = min(num_layers_slice, num_layers_path);

%% ========== 加载密度场(可选) ==========
fprintf('[3] Loading density field (optional)...\n');
density_file = fullfile(data_dir, 'xPhys_full.txt');
has_density = false;
if exist(density_file, 'file')
    params_file = fullfile(data_dir, 'mesh_params.txt');
    fid = fopen(params_file, 'r');
    mp = struct();
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || line(1) == '#', continue; end
        parts = strsplit(line);
        if length(parts) >= 2
            mp.(parts{1}) = str2double(parts{2});
        end
    end
    fclose(fid);
    
    fid = fopen(density_file, 'r');
    fgetl(fid);
    xPhys_vec = fscanf(fid, '%f');
    fclose(fid);
    xPhys = reshape(xPhys_vec, [mp.nely, mp.nelx, mp.nelz]);
    has_density = true;
    fprintf('  Density field loaded: [%.4f, %.4f]\n', min(xPhys(:)), max(xPhys(:)));
else
    fprintf('  Density file not found, skipping.\n');
end

%% ========== 统计：每层激活 vs 路径覆盖 ==========
fprintf('\n[4] Computing per-layer coverage statistics...\n');

layer_stats = struct();

for li = 1:num_layers
    % --- 该层激活的网格 ---
    layer_data_slice = surface_layers{li};
    act_grids = layer_data_slice.final_grids;
    
    num_act = length(act_grids);
    act_xy = zeros(num_act, 2);   % 2D投影坐标 (x, y)
    act_xyz = zeros(num_act, 3);  % 3D坐标
    act_idx = zeros(num_act, 3);  % 网格索引
    
    for g = 1:num_act
        act_xy(g,:)  = [act_grids(g).x, act_grids(g).y];
        act_xyz(g,:) = [act_grids(g).x, act_grids(g).y, act_grids(g).z];
        act_idx(g,:) = act_grids(g).grid_index;
    end
    
    % --- 该层的路径点(2D) ---
    ld = all_layers_data(li);
    path_pts_2d = [];
    if ld.success && ~isempty(ld.paths_2d)
        for pi = 1:length(ld.paths_2d)
            pts = ld.paths_2d{pi};
            if ~isempty(pts)
                % paths_2d 存储格式可能是 [col, row] 即 [y_img, x_img]
                % 但在3D转换中 pts_3d = [x, y, z] 其中 x=pts_2d(:,2), y=pts_2d(:,1)
                % 所以 paths_2d 的实际物理坐标为 (pts(:,2), pts(:,1))
                path_pts_2d = [path_pts_2d; pts(:,2), pts(:,1)];
            end
        end
    end
    
    % --- 也收集3D路径点 ---
    path_pts_3d = [];
    if ld.success && ~isempty(ld.paths_3d)
        for pi = 1:length(ld.paths_3d)
            pts = ld.paths_3d{pi};
            if ~isempty(pts) && size(pts,1) >= 2
                path_pts_3d = [path_pts_3d; pts];
            end
        end
    end
    
    % --- 判断每个激活网格是否被路径覆盖 ---
    covered = false(num_act, 1);
    if ~isempty(path_pts_2d) && num_act > 0
        for g = 1:num_act
            dists = sqrt((path_pts_2d(:,1) - act_xy(g,1)).^2 + ...
                         (path_pts_2d(:,2) - act_xy(g,2)).^2);
            if min(dists) <= COVERAGE_TOL
                covered(g) = true;
            end
        end
    end
    
    % --- 也检查轮廓覆盖 ---
    in_contour = false(num_act, 1);
    if ld.success && ~isempty(ld.outer_contours)
        for g = 1:num_act
            for ci = 1:length(ld.outer_contours)
                oc = ld.outer_contours{ci};
                if inpolygon(act_xy(g,1), act_xy(g,2), oc(:,1), oc(:,2))
                    in_contour(g) = true;
                    break;
                end
            end
            % 检查是否在内轮廓(孔洞)内
            if in_contour(g) && ~isempty(ld.inner_contours)
                for ci = 1:length(ld.inner_contours)
                    ic = ld.inner_contours{ci};
                    if inpolygon(act_xy(g,1), act_xy(g,2), ic(:,1), ic(:,2))
                        in_contour(g) = false;
                        break;
                    end
                end
            end
        end
    end
    
    layer_stats(li).num_activated   = num_act;
    layer_stats(li).num_covered     = sum(covered);
    layer_stats(li).num_uncovered   = sum(~covered);
    layer_stats(li).num_in_contour  = sum(in_contour);
    layer_stats(li).num_outside_contour = sum(~in_contour);
    layer_stats(li).coverage_rate   = 100 * sum(covered) / max(num_act, 1);
    layer_stats(li).contour_rate    = 100 * sum(in_contour) / max(num_act, 1);
    layer_stats(li).offset          = layer_data_slice.offset;
    layer_stats(li).act_xy          = act_xy;
    layer_stats(li).act_xyz         = act_xyz;
    layer_stats(li).act_idx         = act_idx;
    layer_stats(li).covered         = covered;
    layer_stats(li).in_contour      = in_contour;
    layer_stats(li).path_pts_2d     = path_pts_2d;
    layer_stats(li).path_pts_3d     = path_pts_3d;
    layer_stats(li).success         = ld.success;
end

%% ========== 打印统计表 ==========
fprintf('\n  %-6s %-8s %-8s %-8s %-10s %-10s %-10s %-10s\n', ...
    'Layer', 'Offset', 'Activ', 'InCont', 'OutCont', 'Covered', 'Uncov', 'CovRate');
fprintf('  %s\n', repmat('-', 1, 85));

total_act = 0; total_cov = 0; total_uncov = 0;
for li = 1:num_layers
    s = layer_stats(li);
    fprintf('  %-6d %+7.2f %-8d %-8d %-10d %-10d %-10d %6.1f%%\n', ...
        li, s.offset, s.num_activated, s.num_in_contour, ...
        s.num_outside_contour, s.num_covered, s.num_uncovered, s.coverage_rate);
    total_act = total_act + s.num_activated;
    total_cov = total_cov + s.num_covered;
    total_uncov = total_uncov + s.num_uncovered;
end
fprintf('  %s\n', repmat('-', 1, 85));
fprintf('  %-6s %-8s %-8d %-8s %-10s %-10d %-10d %6.1f%%\n', ...
    'Total', '', total_act, '', '', total_cov, total_uncov, ...
    100*total_cov/max(total_act,1));

%% ========== 诊断分类 ==========
fprintf('\n[5] Diagnosing gap sources...\n');

% 找出全局有效但从未被任何层激活的网格
never_activated = valid_mask & ~global_act;
num_never = sum(never_activated(:));
fprintf('  Valid but NEVER activated by any layer: %d\n', num_never);

% 找出被激活但所有层都没覆盖的网格
all_uncovered_idx = [];
for li = 1:num_layers
    s = layer_stats(li);
    uncov_mask = ~s.covered;
    if any(uncov_mask)
        all_uncovered_idx = [all_uncovered_idx; s.act_idx(uncov_mask, :)];
    end
end
if ~isempty(all_uncovered_idx)
    % 去重
    [~, ia] = unique(all_uncovered_idx, 'rows');
    unique_uncov = all_uncovered_idx(ia, :);
    fprintf('  Activated but NEVER covered by paths: %d unique grids\n', size(unique_uncov, 1));
else
    unique_uncov = [];
    fprintf('  All activated grids covered.\n');
end

% 进一步分类未覆盖原因
num_outside_contour_total = 0;
num_inside_contour_no_path = 0;
for li = 1:num_layers
    s = layer_stats(li);
    num_outside_contour_total = num_outside_contour_total + sum(~s.in_contour);
    num_inside_contour_no_path = num_inside_contour_no_path + sum(s.in_contour & ~s.covered);
end
fprintf('\n  Gap breakdown (across all layers):\n');
fprintf('    Activated but OUTSIDE contour:           %d\n', num_outside_contour_total);
fprintf('    Inside contour but NOT covered by path:  %d\n', num_inside_contour_no_path);

% [NEW] 密度验证：未覆盖网格是否真的有材料(密度>0.5)？
if has_density && ~isempty(unique_uncov)
    fprintf('\n  [Density Validation] Checking uncovered grids...\n');
    
    % 检查网格尺寸是否匹配
    if mp.nelx == nelx && mp.nely == nely && mp.nelz == nelz
        n_dense = 0;  % 密度>0.5
        n_sparse = 0; % 密度<=0.5
        density_vals = zeros(size(unique_uncov, 1), 1);
        
        for g = 1:size(unique_uncov, 1)
            gi = unique_uncov(g, :);  % [i, j, k]
            % xPhys indexing: (nely, nelx, nelz) = (j, i, k)
            d = xPhys(gi(2), gi(1), gi(3));
            density_vals(g) = d;
            if d > 0.5
                n_dense = n_dense + 1;
            else
                n_sparse = n_sparse + 1;
            end
        end
        
        fprintf('    Uncovered grids with density > 0.5:  %d (%.1f%%) -- NEED coverage\n', ...
            n_dense, 100*n_dense/max(size(unique_uncov,1),1));
        fprintf('    Uncovered grids with density <= 0.5: %d (%.1f%%) -- low priority\n', ...
            n_sparse, 100*n_sparse/max(size(unique_uncov,1),1));
        fprintf('    Density stats: min=%.3f, median=%.3f, max=%.3f\n', ...
            min(density_vals), median(density_vals), max(density_vals));
        
        % 按层统计密度验证
        fprintf('\n    Per-layer density breakdown:\n');
        fprintf('    %-6s %-10s %-10s %-10s %-10s\n', ...
            'Layer', 'Uncov', 'Dense', 'Sparse', 'DenseRate');
        for li = 1:num_layers
            s = layer_stats(li);
            uncov_mask = ~s.covered;
            if ~any(uncov_mask), continue; end
            uncov_idx = s.act_idx(uncov_mask, :);
            n_d = 0; n_s = 0;
            for g = 1:size(uncov_idx, 1)
                gi = uncov_idx(g, :);
                if gi(1)>=1 && gi(1)<=nelx && gi(2)>=1 && gi(2)<=nely && gi(3)>=1 && gi(3)<=nelz
                    d = xPhys(gi(2), gi(1), gi(3));
                    if d > 0.5, n_d = n_d + 1; else, n_s = n_s + 1; end
                end
            end
            fprintf('    %-6d %-10d %-10d %-10d %6.1f%%\n', ...
                li, size(uncov_idx,1), n_d, n_s, 100*n_d/max(size(uncov_idx,1),1));
        end
    else
        fprintf('    [SKIP] Grid size mismatch: xPhys(%d,%d,%d) vs slice(%d,%d,%d)\n', ...
            mp.nelx, mp.nely, mp.nelz, nelx, nely, nelz);
    end
end

%% ========== 图1: 逐层2D诊断 ==========
fprintf('\n[6] Drawing per-layer 2D diagnostic...\n');

n_cols = min(4, num_layers);
n_rows = ceil(num_layers / n_cols);

fig1 = figure('Name', 'Per-Layer 2D Coverage Diagnostic', ...
    'NumberTitle', 'off', 'Position', [30, 30, 400*n_cols, 350*n_rows], 'Color', 'w');

for li = 1:num_layers
    subplot(n_rows, n_cols, li);
    hold on;
    
    s = layer_stats(li);
    ld = all_layers_data(li);
    
    % --- 绘制激活网格(用颜色区分覆盖状态) ---
    if s.num_activated > 0
        % 未覆盖的: 红色
        uncov = ~s.covered;
        if any(uncov)
            scatter(s.act_xy(uncov,1), s.act_xy(uncov,2), 30, 'r', 's', 'filled', ...
                'MarkerFaceAlpha', 0.6);
        end
        % 覆盖的: 绿色
        if any(s.covered)
            scatter(s.act_xy(s.covered,1), s.act_xy(s.covered,2), 20, 'g', 's', 'filled', ...
                'MarkerFaceAlpha', 0.4);
        end
        % 轮廓外的: 加x标记
        if any(~s.in_contour)
            scatter(s.act_xy(~s.in_contour,1), s.act_xy(~s.in_contour,2), ...
                15, 'k', 'x', 'LineWidth', 1);
        end
    end
    
    % --- 绘制轮廓 ---
    if ld.success
        for ci = 1:length(ld.outer_contours)
            oc = ld.outer_contours{ci};
            plot(oc(:,1), oc(:,2), 'b-', 'LineWidth', 2);
        end
        for ci = 1:length(ld.inner_contours)
            ic = ld.inner_contours{ci};
            plot(ic(:,1), ic(:,2), 'r-', 'LineWidth', 1.5);
        end
    end
    
    % --- 绘制路径 ---
    if ld.success && ~isempty(ld.paths_2d)
        for pi = 1:length(ld.paths_2d)
            pts = ld.paths_2d{pi};
            if ~isempty(pts) && size(pts,1) >= 2
                % paths_2d: (col, row) -> plot as (x=col, y=row)
                plot(pts(:,2), pts(:,1), 'k-', 'LineWidth', 0.5);
            end
        end
    end
    
    axis equal; axis tight;
    grid on;
    title(sprintf('L%d (off=%.1f) %d/%d=%.0f%%', ...
        li, s.offset, s.num_covered, s.num_activated, s.coverage_rate), ...
        'FontSize', 9);
    
    if li == 1
        % 只在第一个子图加图例
        h1 = scatter(NaN, NaN, 30, 'r', 's', 'filled');
        h2 = scatter(NaN, NaN, 20, 'g', 's', 'filled');
        h3 = scatter(NaN, NaN, 15, 'k', 'x');
        h4 = plot(NaN, NaN, 'b-', 'LineWidth', 2);
        h5 = plot(NaN, NaN, 'k-', 'LineWidth', 0.5);
        legend([h1 h2 h3 h4 h5], ...
            {'Uncovered', 'Covered', 'Outside contour', 'Outer contour', 'Path'}, ...
            'FontSize', 7, 'Location', 'best');
    end
    
    hold off;
end

sgtitle('Per-Layer 2D Coverage: Red=gap, Green=covered, x=outside contour', 'FontSize', 13);
saveas(fig1, 'debug_layer_coverage_2d.png');
fprintf('  Saved: debug_layer_coverage_2d.png\n');

%% ========== 图2: 3D总览(标注未覆盖网格) ==========
fprintf('\n[7] Drawing 3D overview with gap highlighting...\n');

fig2 = figure('Name', '3D Coverage Overview', 'NumberTitle', 'off', ...
    'Position', [50, 50, 1400, 1000], 'Color', 'w');
hold on;

% --- 绘制体素网格(半透明) ---
if has_density
    threshold = 0.5;
    cube_face = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    for k = 1:mp.nelz
        for i = 1:mp.nelx
            for j = 1:mp.nely
                if xPhys(j, i, k) > threshold
                    x = (i-1); y = (j-1); z = (k-1);
                    vert = [x,y,z; x+1,y,z; x+1,y,z+1; x,y,z+1;
                            x,y+1,z; x+1,y+1,z; x+1,y+1,z+1; x,y+1,z+1];
                    dv = vert;
                    dv(:,2) = vert(:,3);
                    dv(:,3) = vert(:,2);
                    patch('Faces', cube_face, 'Vertices', dv, ...
                        'FaceColor', [0.6 0.6 0.6], 'EdgeColor', [0.75 0.75 0.75], ...
                        'LineWidth', 0.2, 'FaceAlpha', 0.06);
                end
            end
        end
    end
end

% --- 从 display_mesh_with_paths 复用坐标变换逻辑 ---
% 收集所有路径点
all_pts = [];
for li = 1:num_layers
    if ~all_layers_data(li).success, continue; end
    for pi = 1:length(all_layers_data(li).paths_3d)
        pts = all_layers_data(li).paths_3d{pi};
        if ~isempty(pts) && size(pts,1) >= 2
            all_pts = [all_pts; pts];
        end
    end
end

if has_density && ~isempty(all_pts)
    % 坐标变换参数
    vox_x_range = [0, mp.nelx - 1];
    vox_y_range = [0, mp.nelz - 1];
    vox_z_range = [0, mp.nely - 1];
    
    path_x_min = min(all_pts(:,1)); path_x_max = max(all_pts(:,1));
    path_y_min = min(all_pts(:,2)); path_y_max = max(all_pts(:,2));
    path_z_min = min(all_pts(:,3)); path_z_max = max(all_pts(:,3));
    
    sx = diff(vox_x_range) / max(path_x_max - path_x_min, 1e-6);
    sy = diff(vox_z_range) / max(path_y_max - path_y_min, 1e-6);
    sz = diff(vox_y_range) / max(path_z_max - path_z_min, 1e-6);
    ox = vox_x_range(1) - path_x_min * sx;
    oy = vox_z_range(1) - path_y_min * sy;
    oz = vox_y_range(1) - path_z_min * sz;
    
    has_transform = true;
else
    has_transform = false;
    sx=1; sy=1; sz=1; ox=0; oy=0; oz=0;
end

% --- 绘制路径(彩色) ---
layer_colors = jet(num_layers);
for li = 1:num_layers
    ld = all_layers_data(li);
    if ~ld.success, continue; end
    c = layer_colors(li,:);
    for pi = 1:length(ld.paths_3d)
        pts = ld.paths_3d{pi};
        if isempty(pts) || size(pts,1) < 2, continue; end
        if has_transform
            Xd = pts(:,1)*sx + ox;
            Yd = pts(:,3)*sz + oz;
            Zd = pts(:,2)*sy + oy;
        else
            Xd = pts(:,1); Yd = pts(:,3); Zd = pts(:,2);
        end
        plot3(Xd, Yd, Zd, '-', 'Color', c, 'LineWidth', 0.8);
    end
end

% --- 标注未覆盖的激活网格(大红点) ---
for li = 1:num_layers
    s = layer_stats(li);
    uncov = ~s.covered;
    if any(uncov) && ~isempty(s.path_pts_3d)
        uncov_xyz = s.act_xyz(uncov, :);
        if has_transform
            Xd = uncov_xyz(:,1)*sx + ox;
            Yd = uncov_xyz(:,3)*sz + oz;
            Zd = uncov_xyz(:,2)*sy + oy;
        else
            Xd = uncov_xyz(:,1); Yd = uncov_xyz(:,3); Zd = uncov_xyz(:,2);
        end
        scatter3(Xd, Yd, Zd, 40, 'r', 'filled', 'MarkerFaceAlpha', 0.7);
    end
end

% --- 标注从未被激活的有效网格(黄色方块) ---
if num_never > 0
    never_pts = zeros(num_never, 3);
    cnt = 0;
    for i = 1:nelx
        for j = 1:nely
            for k = 1:nelz
                if never_activated(i,j,k)
                    cnt = cnt + 1;
                    never_pts(cnt,:) = [grid_data(i,j,k).x, ...
                                        grid_data(i,j,k).y, ...
                                        grid_data(i,j,k).z];
                end
            end
        end
    end
    if has_transform
        Xd = never_pts(:,1)*sx + ox;
        Yd = never_pts(:,3)*sz + oz;
        Zd = never_pts(:,2)*sy + oy;
    else
        Xd = never_pts(:,1); Yd = never_pts(:,3); Zd = never_pts(:,2);
    end
    scatter3(Xd, Yd, Zd, 50, [1 0.8 0], 'd', 'filled', 'MarkerFaceAlpha', 0.8);
end

axis equal; axis tight; box on; grid on;
view(3); rotate3d on;
xlabel('X (nelx)'); ylabel('Z (nelz) - Height'); zlabel('Y (nely) - Depth');
title(sprintf('Coverage Diagnostic: Red=uncovered, Yellow=never activated'), 'FontSize', 13);

light('Position', [-10, 30, 20], 'Style', 'local');
lighting gouraud; material dull;

colormap(jet);
c_bar = colorbar; c_bar.Label.String = 'Layer';
caxis([1 num_layers]);

hold off;
saveas(fig2, 'debug_3d_coverage.png');
saveas(fig2, 'debug_3d_coverage.fig');
fprintf('  Saved: debug_3d_coverage.png / .fig\n');

%% ========== 图3: 逐层曲面+激活网格3D视图(选取关键层) ==========
fprintf('\n[8] Drawing per-layer 3D surface + grid views...\n');

% 选取问题最严重的层(未覆盖率最高)
uncov_rates = zeros(num_layers, 1);
for li = 1:num_layers
    uncov_rates(li) = layer_stats(li).num_uncovered;
end
[~, worst_order] = sort(uncov_rates, 'descend');
show_layers = worst_order(1:min(6, num_layers));
show_layers = sort(show_layers);

fig3 = figure('Name', 'Per-Layer 3D Surface + Grids', 'NumberTitle', 'off', ...
    'Position', [80, 50, 1800, 1000], 'Color', 'w');

for idx = 1:length(show_layers)
    li = show_layers(idx);
    subplot(2, 3, idx);
    hold on;
    
    s = layer_stats(li);
    layer_data_slice = surface_layers{li};
    ld = all_layers_data(li);
    
    % --- 绘制曲面(半透明) ---
    X_s = layer_data_slice.X_surf;
    Y_s = layer_data_slice.Y_surf;
    Z_s = layer_data_slice.Z_surf;
    if ~isempty(X_s) && numel(X_s) > 10
        % 降采样显示
        step = max(1, round(size(X_s,1)/50));
        X_ds = X_s(1:step:end, 1:step:end);
        Y_ds = Y_s(1:step:end, 1:step:end);
        Z_ds = Z_s(1:step:end, 1:step:end);
        surf(X_ds, Y_ds, Z_ds, 'FaceColor', [0.7 0.85 1], ...
            'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
    
    % --- 绘制激活网格 ---
    if s.num_activated > 0
        % 覆盖的
        if any(s.covered)
            scatter3(s.act_xyz(s.covered,1), s.act_xyz(s.covered,2), ...
                s.act_xyz(s.covered,3), 25, 'g', 's', 'filled', 'MarkerFaceAlpha', 0.5);
        end
        % 未覆盖的
        uncov = ~s.covered;
        if any(uncov)
            scatter3(s.act_xyz(uncov,1), s.act_xyz(uncov,2), ...
                s.act_xyz(uncov,3), 40, 'r', 's', 'filled', 'MarkerFaceAlpha', 0.8);
        end
    end
    
    % --- 绘制路径 ---
    if ld.success
        for pi = 1:length(ld.paths_3d)
            pts = ld.paths_3d{pi};
            if ~isempty(pts) && size(pts,1) >= 2
                plot3(pts(:,1), pts(:,2), pts(:,3), 'k-', 'LineWidth', 0.8);
            end
        end
    end
    
    axis equal; axis tight;
    view(3); grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('L%d (off=%.1f): %d uncov / %d act', ...
        li, s.offset, s.num_uncovered, s.num_activated), 'FontSize', 10);
    hold off;
end

sgtitle('Worst Coverage Layers (3D): Red=uncovered grids, Green=covered', 'FontSize', 13);
saveas(fig3, 'debug_worst_layers_3d.png');
fprintf('  Saved: debug_worst_layers_3d.png\n');

%% ========== 图4: 汇总柱状图 ==========
fprintf('\n[9] Drawing summary bar chart...\n');

fig4 = figure('Name', 'Coverage Summary', 'NumberTitle', 'off', ...
    'Position', [100, 100, 900, 500], 'Color', 'w');

acts    = [layer_stats.num_activated];
covs    = [layer_stats.num_covered];
in_cont = [layer_stats.num_in_contour];
offsets = [layer_stats.offset];

subplot(1,2,1);
bar_data = [acts; in_cont; covs]';
b = bar(1:num_layers, bar_data, 'grouped');
b(1).FaceColor = [0.7 0.7 0.7];
b(2).FaceColor = [0.3 0.5 0.9];
b(3).FaceColor = [0.2 0.8 0.3];
legend('Activated', 'In contour', 'Path covered', 'Location', 'best');
xlabel('Layer'); ylabel('Grid count');
title('Per-Layer Coverage Breakdown');
set(gca, 'XTick', 1:num_layers);

subplot(1,2,2);
cov_rate = 100 * covs ./ max(acts, 1);
cont_rate = 100 * in_cont ./ max(acts, 1);
bar_pct = [cont_rate; cov_rate]';
b2 = bar(1:num_layers, bar_pct, 'grouped');
b2(1).FaceColor = [0.3 0.5 0.9];
b2(2).FaceColor = [0.2 0.8 0.3];
hold on;
yline(100, 'r--', '100%', 'LineWidth', 1.5);
hold off;
legend('Contour coverage %', 'Path coverage %', 'Location', 'best');
xlabel('Layer'); ylabel('Coverage (%)');
title('Per-Layer Coverage Rate');
set(gca, 'XTick', 1:num_layers);
ylim([0, 110]);

sgtitle(sprintf('Total: %d activated, %d covered (%.1f%%), %d gaps', ...
    total_act, total_cov, 100*total_cov/max(total_act,1), total_uncov), 'FontSize', 13);
saveas(fig4, 'debug_coverage_summary.png');
fprintf('  Saved: debug_coverage_summary.png\n');

%% ========== 完成 ==========
fprintf('\n====================================================\n');
fprintf('  Diagnostic complete!\n');
fprintf('  Key files:\n');
fprintf('    debug_layer_coverage_2d.png  - per-layer 2D\n');
fprintf('    debug_3d_coverage.png/fig    - 3D overview\n');
fprintf('    debug_worst_layers_3d.png    - worst layers 3D\n');
fprintf('    debug_coverage_summary.png   - bar charts\n');
fprintf('====================================================\n');