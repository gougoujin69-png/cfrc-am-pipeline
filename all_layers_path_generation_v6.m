function all_layers_path_generation_v6(slice_file, output_file, full_results_file)
%% ========================================
%% 全层路径生成 V6 — 结构遮罩裁剪 + parfor 并行
%% ========================================
% v6 vs v5b:
%   [NEW-1] 预计算有效结构 XY 投影遮罩 (alphaShape → polyshape)
%   [NEW-2] 区域分割后用 intersect 裁剪到结构范围内
%   [NEW-3] 3D 路径点级裁剪 (isinterior)
%   [NEW-4] parfor 多层并行 (cell 数组存储, 避免 struct 赋值限制)
%   保留 v5b 所有修复: polybuffer 外扩, 膨胀, xold>0 等
%
% 输入 (可选):
%   slice_file        - 切片 mat 路径 (默认 'slice_results_refined_latest.mat')
%                       可以是 _PLANAR 版本以跑 planar_stream 对照组
%   output_file       - paths_only 输出 mat (默认 'all_layers_paths_only_v3.mat')
%   full_results_file - 完整 results 输出 mat (默认 'all_layers_path_results_v3.mat')
%
% 用法:
%   all_layers_path_generation_v6();                                       % 默认
%   all_layers_path_generation_v6('slice_results_refined_latest_PLANAR.mat', ...
%                                 'all_layers_paths_only_planar_stream.mat');
%%

if nargin < 1 || isempty(slice_file)
    slice_file = 'slice_results_refined_latest.mat';
end
if nargin < 2 || isempty(output_file)
    output_file = 'all_layers_paths_only_v3.mat';
end
if nargin < 3 || isempty(full_results_file)
    full_results_file = 'all_layers_path_results_v3.mat';
end

warning('off', 'MATLAB:polyshape:repairedBySimplify');
warning('off', 'MATLAB:polyshape:boundary3Points');

fprintf('\n');
fprintf('================================================================\n');
fprintf('  全层路径生成 V6 — 结构遮罩 + parfor\n');
fprintf('  Slice input : %s\n', slice_file);
fprintf('  Paths output: %s\n', output_file);
fprintf('================================================================\n\n');

%% ========== 参数设置 ==========
params = struct();
params.offset_distance = 0.3;
params.max_iterations = 30;
params.min_path_length = 2;
params.volfrac = 0.5;
params.filter_radius = 5;
params.filter_iterations = 2;
params.min_region_area = 0.05;
params.min_contour_length_inner = 4;
params.contour_dilate_pixels = 3;
params.contour_expand_ratio = 0.6;

%% ========== Step 1: 检查工具箱 ==========
fprintf('[Step 1] Checking toolboxes...\n');
v = ver;
hasParallelToolbox = any(strcmp({v.Name}, 'Parallel Computing Toolbox'));
hasImageToolbox = any(strcmp({v.Name}, 'Image Processing Toolbox'));

if hasParallelToolbox
    fprintf('  Parallel Computing Toolbox: installed\n');
    p = gcp('nocreate');
    if isempty(p)
        try
            parpool('local');
            p = gcp('nocreate');
            fprintf('  Workers: %d\n', p.NumWorkers);
        catch
            fprintf('  Warning: parallel pool failed, using serial\n');
            hasParallelToolbox = false;
        end
    else
        fprintf('  Workers: %d\n', p.NumWorkers);
    end
else
    fprintf('  Parallel Computing Toolbox: not installed (serial mode)\n');
end

if hasImageToolbox
    fprintf('  Image Processing Toolbox: installed\n');
end

%% ========== Step 2: 加载切片数据 ==========
fprintf('\n[Step 2] Loading slice data...\n');

if ~exist(slice_file, 'file')
    error('%s not found', slice_file);
end

load(slice_file);
surface_layers = slice_results.surface_layers;
grid_data = slice_results.grid_data;
num_layers = slice_results.statistics.num_layers;
valid_grid_mask = slice_results.valid_grid_mask;

fprintf('  Total layers: %d\n', num_layers);

%% ========== Step 2b [NEW]: 加载 Z 高度场 + XY 遮罩 ==========
fprintf('\n[Step 2b] Loading structure validity data...\n');

% XY 投影遮罩
structure_mask_poly = build_structure_mask(grid_data, valid_grid_mask);
has_structure_mask = area(structure_mask_poly) > 0;
if has_structure_mask
    fprintf('  XY mask area: %.2f\n', area(structure_mask_poly));
end

% Z 高度场 (Z_min, Z_max at each XY)
F_zmin_global = [];
F_zmax_global = [];
has_z_field = false;

if isfield(slice_results, 'z_height_field')
    zhf = slice_results.z_height_field;
    try
        F_zmin_global = griddedInterpolant({zhf.yy, zhf.xx}, zhf.Z_min_map, 'linear', 'nearest');
        F_zmax_global = griddedInterpolant({zhf.yy, zhf.xx}, zhf.Z_max_map, 'linear', 'nearest');
        has_z_field = true;
        fprintf('  Z height field loaded: %dx%d\n', length(zhf.xx), length(zhf.yy));
    catch ME
        fprintf('  [WARN] Z height field failed: %s\n', ME.message);
    end
else
    fprintf('  [WARN] No z_height_field in slice_results (run latest slicing script)\n');
end

%% ========== Step 3: 准备并行数据 ==========
fprintf('\n[Step 3] Preparing parallel data...\n');

layer_data_cells = cell(num_layers, 1);
for li = 1:num_layers
    layer_data_cells{li} = surface_layers{li};
end

% Z高度场数据 (传 struct, parfor 内重建插值器)
if has_z_field
    zhf_data = slice_results.z_height_field;
else
    zhf_data = [];
end

fprintf('  %d layers prepared for processing\n', num_layers);

%% ========== Step 4: 逐层处理 (parfor 并行) ==========
fprintf('\n[Step 4] Processing all layers...\n\n');

total_start_time = tic;

all_results_cell = cell(num_layers, 1);

if hasParallelToolbox
    fprintf('  Using PARFOR (%d workers)...\n\n', p.NumWorkers);
    parfor layer_idx = 1:num_layers
        all_results_cell{layer_idx} = process_single_layer(...
            layer_idx, layer_data_cells{layer_idx}, grid_data, ...
            params, hasImageToolbox, ...
            has_structure_mask, structure_mask_poly, ...
            has_z_field, zhf_data);
    end
else
    fprintf('  Using serial FOR loop...\n\n');
    for layer_idx = 1:num_layers
        all_results_cell{layer_idx} = process_single_layer(...
            layer_idx, layer_data_cells{layer_idx}, grid_data, ...
            params, hasImageToolbox, ...
            has_structure_mask, structure_mask_poly, ...
            has_z_field, zhf_data);
    end
end

total_time = toc(total_start_time);

% 转回 struct 数组
all_layers_data = struct(...
    'layer_idx', cell(num_layers, 1), ...
    'offset', cell(num_layers, 1), ...
    'outer_contours', cell(num_layers, 1), ...
    'inner_contours', cell(num_layers, 1), ...
    'streamlines', cell(num_layers, 1), ...
    'medial_axis', cell(num_layers, 1), ...
    'regions', cell(num_layers, 1), ...
    'paths_2d', cell(num_layers, 1), ...
    'paths_3d', cell(num_layers, 1), ...
    'pointCloud_data', cell(num_layers, 1), ...
    'statistics', cell(num_layers, 1), ...
    'processing_time', cell(num_layers, 1), ...
    'success', cell(num_layers, 1), ...
    'error_message', cell(num_layers, 1)); 

for li = 1:num_layers
    all_layers_data(li) = all_results_cell{li};
end

%% ========== Step 5: 汇总统计 ==========
fprintf('\n[Step 5] Summary...\n');

total_outer = 0; total_inner = 0; total_streamlines = 0;
total_regions = 0; total_paths_2d = 0; total_paths_3d = 0;
success_count = 0; failed_layers = [];

for i = 1:num_layers
    if all_layers_data(i).success
        success_count = success_count + 1;
        total_outer = total_outer + all_layers_data(i).statistics.num_outer;
        total_inner = total_inner + all_layers_data(i).statistics.num_inner;
        total_streamlines = total_streamlines + all_layers_data(i).statistics.num_streamlines;
        total_regions = total_regions + all_layers_data(i).statistics.num_regions;
        total_paths_2d = total_paths_2d + all_layers_data(i).statistics.num_paths_2d;
        total_paths_3d = total_paths_3d + all_layers_data(i).statistics.num_paths_3d;
    else
        failed_layers = [failed_layers, i];
    end
end

fprintf('\n  %-6s %-8s %-8s %-8s %-10s %-10s %-10s %-10s %-8s\n', ...
    'Layer', 'Outer', 'Inner', 'Stream', 'Regions', '2D Path', '3D Path', 'Time(s)', 'Status');
fprintf('  %s\n', repmat('-', 1, 90));

for i = 1:num_layers
    if all_layers_data(i).success
        fprintf('  %-6d %-8d %-8d %-8d %-10d %-10d %-10d %-10.2f %-8s\n', ...
            i, all_layers_data(i).statistics.num_outer, ...
            all_layers_data(i).statistics.num_inner, ...
            all_layers_data(i).statistics.num_streamlines, ...
            all_layers_data(i).statistics.num_regions, ...
            all_layers_data(i).statistics.num_paths_2d, ...
            all_layers_data(i).statistics.num_paths_3d, ...
            all_layers_data(i).processing_time, 'OK');
    else
        fprintf('  %-6d %-8s %-8s %-8s %-10s %-10s %-10s %-10s %-8s\n', ...
            i, '-', '-', '-', '-', '-', '-', '-', 'FAIL');
    end
end

%% ========== Step 6: 可视化验证 ==========
fprintf('\n[Step 6] Visualization...\n');

vis_layers = round(linspace(1, num_layers, min(6, num_layers)));
figure('Name', 'Contour Verification V6', 'Position', [50, 50, 1800, 1000]);

for idx = 1:length(vis_layers)
    li = vis_layers(idx);
    ld = all_layers_data(li);
    
    subplot(2, 3, idx);
    hold on;
    
    % 画结构遮罩边界
    if has_structure_mask
        [mx, my] = boundary(structure_mask_poly);
        plot(mx, my, 'k--', 'LineWidth', 1.5);
    end
    
    if ld.success && ~isempty(ld.outer_contours)
        for i = 1:length(ld.outer_contours)
            fill(ld.outer_contours{i}(:,1), ld.outer_contours{i}(:,2), ...
                [0.85, 0.85, 0.85], 'EdgeColor', 'b', 'LineWidth', 2);
        end
        for i = 1:length(ld.inner_contours)
            fill(ld.inner_contours{i}(:,1), ld.inner_contours{i}(:,2), ...
                [1, 1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
        end
        for i = 1:length(ld.streamlines)
            sl = ld.streamlines{i};
            if ~isempty(sl), plot(sl(:,1), sl(:,2), 'g-', 'LineWidth', 1.5); end
        end
        
        ag = layer_data_cells{li}.final_grids;
        if ~isempty(ag)
            ag_x = arrayfun(@(g) g.x, ag);
            ag_y = arrayfun(@(g) g.y, ag);
            scatter(ag_x, ag_y, 5, [0.8 0.2 0.2], '.', 'MarkerFaceAlpha', 0.3);
        end
        
        title(sprintf('L%d: O%d I%d S%d R%d P%d', ...
            li, length(ld.outer_contours), length(ld.inner_contours), ...
            length(ld.streamlines), length(ld.regions), length(ld.paths_3d)), 'FontSize', 10);
    else
        title(sprintf('Layer %d (FAILED)', li), 'FontSize', 10);
    end
    
    axis equal tight; hold off;
end

sgtitle('V6: black dashed=structure mask, blue=outer, red=inner', 'FontSize', 13);
saveas(gcf, 'all_layers_contour_v6.png');
fprintf('  Saved: all_layers_contour_v6.png\n');

%% ========== Step 7: 三维可视化 ==========
fprintf('\n[Step 7] 3D visualization...\n');

figure('Name', 'All Layers 3D V6', 'Position', [100, 100, 1200, 900]);
hold on;
layer_colors = jet(num_layers);

for layer_idx = 1:num_layers
    ld = all_layers_data(layer_idx);
    if ~ld.success, continue; end
    for i = 1:length(ld.paths_3d)
        pts = ld.paths_3d{i};
        if ~isempty(pts) && size(pts, 1) >= 2
            plot3(pts(:,1), pts(:,2), pts(:,3), ...
                'Color', layer_colors(layer_idx,:), 'LineWidth', 1);
        end
    end
end

hold off; axis equal; axis off; view(-37.5, 30); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('All Layers 3D Paths V6', 'FontSize', 14);
colormap(jet); c = colorbar; c.Label.String = 'Layer'; caxis([1, num_layers]);
saveas(gcf, 'all_layers_3d_v6.png');
fprintf('  Saved: all_layers_3d_v6.png\n');

%% ========== Step 8: 保存结果 ==========
fprintf('\n[Step 8] Saving results...\n');

results = struct();
results.params = params;
results.num_layers = num_layers;
results.all_layers_data = all_layers_data;
results.structure_mask_poly = structure_mask_poly;
results.total_statistics = struct(...
    'total_outer', total_outer, 'total_inner', total_inner, ...
    'total_streamlines', total_streamlines, 'total_regions', total_regions, ...
    'total_paths_2d', total_paths_2d, 'total_paths_3d', total_paths_3d, ...
    'success_count', success_count, 'failed_count', length(failed_layers), ...
    'failed_layers', failed_layers, 'total_time', total_time);
results.version = 'v6_structure_mask_parfor';

save(full_results_file, 'results', '-v7.3');
fprintf('  Saved: %s\n', full_results_file);

paths_only = struct();
paths_only.num_layers = num_layers;
paths_only.layer_paths_2d = cell(num_layers, 1);
paths_only.layer_paths_3d = cell(num_layers, 1);
paths_only.layer_offsets = zeros(num_layers, 1);
paths_only.layer_outer_contours = cell(num_layers, 1);
paths_only.layer_inner_contours = cell(num_layers, 1);

for i = 1:num_layers
    if all_layers_data(i).success
        paths_only.layer_paths_2d{i} = all_layers_data(i).paths_2d;
        paths_only.layer_paths_3d{i} = all_layers_data(i).paths_3d;
        paths_only.layer_offsets(i) = all_layers_data(i).offset;
        paths_only.layer_outer_contours{i} = all_layers_data(i).outer_contours;
        paths_only.layer_inner_contours{i} = all_layers_data(i).inner_contours;
    else
        paths_only.layer_paths_2d{i} = {};
        paths_only.layer_paths_3d{i} = {};
        paths_only.layer_offsets(i) = NaN;
        paths_only.layer_outer_contours{i} = {};
        paths_only.layer_inner_contours{i} = {};
    end
end

save(output_file, 'paths_only', '-v7.3');
fprintf('  Saved: %s\n', output_file);

%% ========== 最终统计 ==========
fprintf('\n================================================================\n');
fprintf('                    Processing Complete (V6)\n');
fprintf('================================================================\n');
fprintf('  Layers: %d (OK: %d, Fail: %d)\n', num_layers, success_count, length(failed_layers));
fprintf('  Outer: %d, Inner: %d, Streams: %d, Regions: %d\n', ...
    total_outer, total_inner, total_streamlines, total_regions);
fprintf('  2D paths: %d, 3D paths: %d\n', total_paths_2d, total_paths_3d);
fprintf('  Time: %.1f sec (%.1f sec/layer)\n', total_time, total_time/num_layers);
if has_structure_mask
    fprintf('  Structure mask: ON (area=%.2f)\n', area(structure_mask_poly));
end
fprintf('================================================================\n');

if ~isempty(failed_layers)
    fprintf('  Failed layers: %s\n', mat2str(failed_layers));
end

warning('on', 'MATLAB:polyshape:repairedBySimplify');
warning('on', 'MATLAB:polyshape:boundary3Points');
fprintf('\n  Done!\n');

end % function all_layers_path_generation_v6


%% ================================================================
%% 核心处理函数 — 处理单层 (parfor 兼容)
%% ================================================================
function layer_result = process_single_layer(...
    layer_idx, layer_data, grid_data, ...
    params, hasImageToolbox, ...
    has_structure_mask, structure_mask_poly, ...
    has_z_field, zhf_data)
% 处理一层的完整管线 (parfor 兼容)
% v6 新增: 3D 有效性检查 → 生成 "墙" 内轮廓

    layer_start_time = tic;
    fprintf('  ======== Layer %d ========\n', layer_idx);
    
    % 重建 Z 高度场插值器 (parfor 内不能共享句柄)
    F_zmin_local = [];
    F_zmax_local = [];
    if has_z_field && ~isempty(zhf_data)
        try
            F_zmin_local = griddedInterpolant({zhf_data.yy, zhf_data.xx}, ...
                zhf_data.Z_min_map, 'linear', 'nearest');
            F_zmax_local = griddedInterpolant({zhf_data.yy, zhf_data.xx}, ...
                zhf_data.Z_max_map, 'linear', 'nearest');
        catch
        end
    end
    
    try
        %% === 步骤1: 获取当前层数据 ===
        activated_grids = layer_data.final_grids;
        X_surf = layer_data.X_surf;
        Y_surf = layer_data.Y_surf;
        Z_surf = layer_data.Z_surf;
        
        if isempty(activated_grids) || length(activated_grids) < 3
            fprintf('    Skipped: insufficient grids (%d)\n', length(activated_grids));
            layer_result = make_empty_layer_result(layer_idx, layer_data.offset, ...
                'insufficient grids', toc(layer_start_time));
            return;
        end
        
        %% === 步骤2: 创建点云和插值器 ===
        pointCloud_data = create_pointcloud_from_surface(X_surf, Y_surf, Z_surf);
        
        X_flat = X_surf(:);
        Y_flat = Y_surf(:);
        Z_flat = Z_surf(:);
        valid_pts = all(isfinite([X_flat, Y_flat, Z_flat]), 2);
        
        if sum(valid_pts) >= 3
            F_z = scatteredInterpolant(X_flat(valid_pts), Y_flat(valid_pts), ...
                Z_flat(valid_pts), 'linear', 'nearest');
        else
            F_z = [];
        end
        
        %% === 步骤3: 提取二维投影 ===
        [xold, t, nelx, nely, ~] = extract_layer_2d_projection(activated_grids, grid_data);
        
        %% === 步骤3b: 预计算坐标映射 (提前到二值图之前) ===
        num_grids = length(activated_grids);
        actual_x = zeros(num_grids, 1);
        actual_y = zeros(num_grids, 1);
        actual_ix = zeros(num_grids, 1);
        actual_iy = zeros(num_grids, 1);
        for i = 1:num_grids
            g = activated_grids(i);
            actual_x(i) = g.x;
            actual_y(i) = g.y;
            actual_ix(i) = g.grid_index(1);
            actual_iy(i) = g.grid_index(2);
        end
        
        actual_x_min = min(actual_x);
        actual_x_max = max(actual_x);
        actual_y_min = min(actual_y);
        actual_y_max = max(actual_y);
        actual_range = max(actual_x_max - actual_x_min, actual_y_max - actual_y_min);
        adaptive_min_outer = max(actual_range * 0.05, 1.5);

        % --- [FIX] pixel<->physical 映射必须用真实的物理 cell 尺寸 ---
        % 旧版用 scale_x = (actual_x_max - actual_x_min) / (nelx - 1), 这只有在
        % 激活 voxel 在 X 方向跨满全网格时才正确. 当结构在 X 中段窄 (拓扑优化
        % 常见) 时, 这个 scale 比真实 dx_phys 小, 路径会被 X 方向压缩.
        % 修复: 从同层任意两个 ix 不同的 activated grids 反推 dx_phys 和原点.
        [ix_min_val, p_ix_min] = min(actual_ix);
        [ix_max_val, p_ix_max] = max(actual_ix);
        if ix_max_val > ix_min_val
            dx_phys = (actual_x(p_ix_max) - actual_x(p_ix_min)) / ...
                      (ix_max_val - ix_min_val);
            grid_x_origin = actual_x(p_ix_min) - (ix_min_val - 1) * dx_phys;
        else
            dx_phys = 1.0;
            grid_x_origin = actual_x_min;  % 退化: 只有一个 ix
        end
        [iy_min_val, p_iy_min] = min(actual_iy);
        [iy_max_val, p_iy_max] = max(actual_iy);
        if iy_max_val > iy_min_val
            dy_phys = (actual_y(p_iy_max) - actual_y(p_iy_min)) / ...
                      (iy_max_val - iy_min_val);
            grid_y_origin = actual_y(p_iy_min) - (iy_min_val - 1) * dy_phys;
        else
            dy_phys = 1.0;
            grid_y_origin = actual_y_min;
        end
        scale_x = dx_phys;
        scale_y = dy_phys;
        expand_dist = max(scale_x, scale_y) * params.contour_expand_ratio;
        
        %% === 步骤4: 方向场滤波 ===
        t_filtered = filter_orientation_simple(t, xold, params.filter_radius, params.filter_iterations);
        
        %% === 步骤5: 二值图像 + 3D有效性烧入 ===
        x_filter = zeros(nely, nelx);
        x_filter(xold > 0) = 1;
        
        if hasImageToolbox && params.contour_dilate_pixels > 0
            se = strel('disk', params.contour_dilate_pixels);
            x_filter = imdilate(x_filter, se);
        end
        
        % === [核心] 直接在二值图上烧掉Z越界和XY越界的像素 ===
        % 这样 bwboundaries 天然只产生有效区域的轮廓
        n_burned_z = 0;
        n_burned_xy = 0;
        
        if ~isempty(F_zmin_local) && ~isempty(F_zmax_local) && ~isempty(F_z)
            z_margin = max(scale_x, scale_y) * 0.5;
            
            for row = 1:nely
                for col = 1:nelx
                    if x_filter(row, col) == 0, continue; end
                    
                    % 像素 → 实际坐标
                    px = grid_x_origin + (col - 1) * scale_x;
                    py = grid_y_origin + (row - 1) * scale_y;
                    
                    % XY 越界检查
                    if has_structure_mask
                        if ~isinterior(structure_mask_poly, px, py)
                            x_filter(row, col) = 0;
                            n_burned_xy = n_burned_xy + 1;
                            continue;
                        end
                    end
                    
                    % Z 越界检查: 曲面Z vs 结构Z范围
                    pz = F_z(px, py);
                    if isnan(pz)
                        x_filter(row, col) = 0;
                        n_burned_xy = n_burned_xy + 1;
                        continue;
                    end
                    
                    z_lo = F_zmin_local(py, px);
                    z_hi = F_zmax_local(py, px);
                    
                    if pz < z_lo - z_margin || pz > z_hi + z_margin
                        x_filter(row, col) = 0;
                        n_burned_z = n_burned_z + 1;
                    end
                end
            end
            
            fprintf('    [BURN] XY越界:%d, Z越界:%d 像素已清除\n', n_burned_xy, n_burned_z);
        end
        
        % 烧入后再侵蚀1像素 (清理烧入边缘的锯齿)
        if hasImageToolbox && (n_burned_z + n_burned_xy) > 0
            x_filter = imerode(x_filter, ones(3));
            x_filter = imdilate(x_filter, ones(3));  % 开运算: 去小噪点
        end
        
        [B, L, N, A] = bwboundaries(x_filter, 'holes');
        
        outer_boundaries_raw = {};
        inner_boundaries_raw = {};
        for k = 1:length(B)
            if k <= N
                outer_boundaries_raw{end+1} = B{k};
            else
                inner_boundaries_raw{end+1} = B{k};
            end
        end
        
        %% === 步骤6: 轮廓坐标转换 + polybuffer 外扩 ===
        % (坐标映射已在步骤3b计算)
        
        % 外轮廓
        outer_contours = {};
        for k = 1:length(outer_boundaries_raw)
            bnd = outer_boundaries_raw{k};
            if isempty(bnd) || size(bnd, 1) < 3, continue; end
            if size(bnd, 1) > 200
                step = ceil(size(bnd, 1) / 200);
                bnd = bnd(1:step:end, :);
            end
            if ~isequal(bnd(1,:), bnd(end,:))
                bnd(end+1,:) = bnd(1,:);
            end
            if size(bnd, 1) > 6
                bnd(:,1) = smooth(bnd(:,1), 3);
                bnd(:,2) = smooth(bnd(:,2), 3);
            end
            x_actual = grid_x_origin + (bnd(:, 2) - 1) * scale_x;
            y_actual = grid_y_origin + (bnd(:, 1) - 1) * scale_y;
            contour_pts = [x_actual(:), y_actual(:)];
            contour_pts = expand_contour(contour_pts, expand_dist);
            contour_length = sum(sqrt(sum(diff(contour_pts).^2, 2)));
            if contour_length >= adaptive_min_outer
                outer_contours{end+1} = contour_pts;
            end
        end
        
        % 内轮廓 (包括烧入产生的洞)
        inner_contours = {};
        for k = 1:length(inner_boundaries_raw)
            bnd = inner_boundaries_raw{k};
            if isempty(bnd) || size(bnd, 1) < 3, continue; end
            if size(bnd, 1) > 200
                step = ceil(size(bnd, 1) / 200);
                bnd = bnd(1:step:end, :);
            end
            if ~isequal(bnd(1,:), bnd(end,:))
                bnd(end+1,:) = bnd(1,:);
            end
            if size(bnd, 1) > 6
                bnd(:,1) = smooth(bnd(:,1), 3);
                bnd(:,2) = smooth(bnd(:,2), 3);
            end
            x_actual = grid_x_origin + (bnd(:, 2) - 1) * scale_x;
            y_actual = grid_y_origin + (bnd(:, 1) - 1) * scale_y;
            contour_pts = [x_actual(:), y_actual(:)];
            contour_length = sum(sqrt(sum(diff(contour_pts).^2, 2)));
            if contour_length >= params.min_contour_length_inner
                inner_contours{end+1} = contour_pts;
            end
        end
        
        %% === 步骤7: 中轴线 ===
        skeleton = bwmorph(x_filter, 'skel', Inf);
        skeleton_clean = bwmorph(skeleton, 'spur', 3);
        [y_skel, x_skel] = find(skeleton_clean);
        
        if ~isempty(x_skel)
            x_skel_actual = grid_x_origin + (x_skel - 1) * scale_x;
            y_skel_actual = grid_y_origin + (y_skel - 1) * scale_y;
            medial_axis = [x_skel_actual(:), y_skel_actual(:)];
        else
            medial_axis = [];
        end
        
        %% === 步骤8: 流线 ===
        [~, ~, streamlines_raw, ~, ~, ~] = ...
            plotTopologyWithMedialAxis(xold, t_filtered, nelx, nely, params.volfrac);
        
        streamlines = {};
        for k = 1:length(streamlines_raw)
            sl = streamlines_raw{k};
            if isempty(sl) || size(sl, 1) < 3, continue; end
            x_actual = grid_x_origin + (sl(:, 1) - 1) * scale_x;
            y_actual = grid_y_origin + (sl(:, 2) - 1) * scale_y;
            streamlines{end+1} = [x_actual(:), y_actual(:)];
        end
        
        %% === 步骤9: 流线过滤和延长 ===
        if ~isempty(streamlines) && ~isempty(outer_contours)
            [filtered_streamlines, ~] = filterStreamlinesInsideContours(streamlines, outer_contours, inner_contours);
            filtered_streamlines = filtered_streamlines(~cellfun(@isempty, filtered_streamlines));
            
            if ~isempty(filtered_streamlines)
                extended_streamlines = extendStreamlinesToContour(filtered_streamlines, outer_contours, inner_contours);
                extended_streamlines = extended_streamlines(~cellfun(@isempty, extended_streamlines));
            else
                extended_streamlines = {};
            end
        else
            extended_streamlines = {};
        end
        
        %% === 步骤11: 区域分割 ===
        all_regions = {};
        
        for contour_idx = 1:length(outer_contours)
            current_outer = outer_contours(contour_idx);
            
            current_inner = {};
            try
                main_poly_temp = polyshape(current_outer{1}(:,1), current_outer{1}(:,2));
                for h = 1:length(inner_contours)
                    if isempty(inner_contours{h}), continue; end
                    inner_center = mean(inner_contours{h}, 1);
                    if isinterior(main_poly_temp, inner_center(1), inner_center(2))
                        current_inner{end+1} = inner_contours{h};
                    end
                end
            catch
                main_poly_temp = polyshape();
            end
            
            current_streamlines = {};
            for s = 1:length(extended_streamlines)
                sl = extended_streamlines{s};
                if isempty(sl), continue; end
                mid_pt = sl(round(size(sl,1)/2), :);
                in_outer = inpolygon(mid_pt(1), mid_pt(2), current_outer{1}(:,1), current_outer{1}(:,2));
                in_inner = false;
                for h = 1:length(current_inner)
                    if inpolygon(mid_pt(1), mid_pt(2), current_inner{h}(:,1), current_inner{h}(:,2))
                        in_inner = true; break;
                    end
                end
                if in_outer && ~in_inner
                    current_streamlines{end+1} = sl;
                end
            end
            
            try
                if ~isempty(current_streamlines)
                    [~, regions_poly] = split_region_points_improved(...
                        current_outer, current_inner, current_streamlines, ...
                        'point_tol', 1e-6, 'dist_tol', 1e-6, 'area_tol', 1e-12, 'debug', false);
                else
                    regions_poly = main_poly_temp;
                    for h = 1:length(current_inner)
                        hole = polyshape(current_inner{h}(:,1), current_inner{h}(:,2));
                        regions_poly = subtract(regions_poly, hole);
                    end
                end
            catch ME
                fprintf('    [WARN] Region split failed contour %d: %s\n', contour_idx, ME.message);
                regions_poly = polyshape.empty;
            end
            
            for i = 1:length(regions_poly)
                try
                    if length(regions_poly) == 1
                        temp = regions_poly;
                    else
                        temp = regions_poly(i);
                    end
                    for k = 1:length(current_inner)
                        poly = polyshape(current_inner{k}(:,1), current_inner{k}(:,2));
                        temp = subtract(temp, poly);
                    end
                    if area(temp) > params.min_region_area
                        all_regions{end+1} = temp;
                    end
                catch ME
                    fprintf('    [WARN] Region %d subtract: %s\n', i, ME.message);
                end
            end
        end
        
        %% === 步骤12 [NEW]: 结构遮罩裁剪 ===
        if has_structure_mask && ~isempty(all_regions)
            clipped_regions = {};
            for ri = 1:length(all_regions)
                try
                    clipped = intersect(all_regions{ri}, structure_mask_poly);
                    if area(clipped) > params.min_region_area
                        clipped_parts = regions(clipped);
                        for cp = 1:length(clipped_parts)
                            if area(clipped_parts(cp)) > params.min_region_area
                                clipped_regions{end+1} = clipped_parts(cp);
                            end
                        end
                    end
                catch
                    if area(all_regions{ri}) > params.min_region_area
                        clipped_regions{end+1} = all_regions{ri};
                    end
                end
            end
            n_before = length(all_regions);
            all_regions = clipped_regions;
            fprintf('    [CLIP] Regions: %d -> %d\n', n_before, length(all_regions));
        end
        
        % 裁剪外轮廓 (可视化用)
        if has_structure_mask
            outer_contours = clip_contours_to_mask(outer_contours, structure_mask_poly);
        end
        
        %% === 步骤13: 生成偏置路径 ===
        all_paths_2d = {};
        all_paths_3d = {};
        
        for i = 1:length(all_regions)
            region_result = all_regions{i};
            if area(region_result) < params.min_region_area, continue; end
            
            try
                [outer_cell, inner_cell] = polyshape_to_cell(region_result);
                if isempty(outer_cell), continue; end
                
                rings_results = generate_offset_path2(outer_cell, inner_cell, ...
                    params.offset_distance, params.max_iterations, params.min_path_length, pointCloud_data);
                
                for j = 1:length(rings_results)
                    rings_iter = rings_results{j};
                    if isempty(rings_iter), continue; end
                    
                    for k = 1:length(rings_iter)
                        pts_2d = rings_iter{k};
                        if isempty(pts_2d) || size(pts_2d, 1) < 2, continue; end
                        
                        all_paths_2d{end+1} = pts_2d;
                        
                        % 2D → 3D
                        pts_3d = zeros(size(pts_2d, 1), 3);
                        for pp = 1:size(pts_2d, 1)
                            x = pts_2d(pp, 2);
                            y = pts_2d(pp, 1);
                            
                            if ~isempty(F_z)
                                z = F_z(x, y);
                            else
                                dist = (pointCloud_data.X(:) - x).^2 + (pointCloud_data.Y(:) - y).^2;
                                [~, idx] = min(dist);
                                z = pointCloud_data.Z(idx);
                            end
                            
                            if isnan(z) || ~isfinite(z)
                                dist = (pointCloud_data.X(:) - x).^2 + (pointCloud_data.Y(:) - y).^2;
                                [~, idx] = min(dist);
                                z = pointCloud_data.Z(idx);
                            end
                            
                            pts_3d(pp, :) = [x, y, z];
                        end
                        
                        % [NEW-3] 3D 路径结构遮罩裁剪
                        if has_structure_mask
                            clipped_3d = clip_path_to_mask(pts_3d, structure_mask_poly);
                            for ss = 1:length(clipped_3d)
                                all_paths_3d{end+1} = clipped_3d{ss};
                            end
                        else
                            all_paths_3d{end+1} = pts_3d;
                        end
                    end
                end
            catch ME
                fprintf('    [WARN] Path gen region %d: %s\n', i, ME.message);
            end
        end
        
        %% === 存入结构 ===
        layer_result = struct();
        layer_result.layer_idx = layer_idx;
        layer_result.offset = layer_data.offset;
        layer_result.outer_contours = outer_contours;
        layer_result.inner_contours = inner_contours;
        layer_result.streamlines = extended_streamlines;
        layer_result.medial_axis = medial_axis;
        layer_result.regions = all_regions;
        layer_result.paths_2d = all_paths_2d;
        layer_result.paths_3d = all_paths_3d;
        layer_result.pointCloud_data = pointCloud_data;
        layer_result.statistics = struct(...
            'num_outer', length(outer_contours), ...
            'num_inner', length(inner_contours), ...
            'num_streamlines', length(extended_streamlines), ...
            'num_regions', length(all_regions), ...
            'num_paths_2d', length(all_paths_2d), ...
            'num_paths_3d', length(all_paths_3d));
        layer_result.processing_time = toc(layer_start_time);
        layer_result.success = true;
        layer_result.error_message = '';
        
        fprintf('    outer=%d inner=%d stream=%d region=%d 2d=%d 3d=%d (%.1fs)\n', ...
            length(outer_contours), length(inner_contours), ...
            length(extended_streamlines), length(all_regions), ...
            length(all_paths_2d), length(all_paths_3d), ...
            layer_result.processing_time);
        
    catch ME
        fprintf('    FAILED: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('    Stack: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
        end
        layer_result = make_empty_layer_result(layer_idx, layer_data.offset, ...
            ME.message, toc(layer_start_time));
    end
end


%% ================================================================
%% 辅助函数
%% ================================================================

function mask_poly = build_structure_mask(grid_data, valid_grid_mask)
% 从有效体素构建 XY 投影遮罩 (alphaShape → polyshape)
    [nelx_g, nely_g, nelz_g] = size(valid_grid_mask);
    
    valid_xy = [];
    for i = 1:nelx_g
        for j = 1:nely_g
            for k = 1:nelz_g
                if valid_grid_mask(i,j,k)
                    gc = grid_data(i,j,k);
                    valid_xy = [valid_xy; gc.x, gc.y];
                end
            end
        end
    end
    valid_xy = unique(valid_xy, 'rows');
    
    if size(valid_xy, 1) < 3
        mask_poly = polyshape();
        return;
    end
    
    % 体素间距
    dx_v = diff(sort(unique(valid_xy(:,1))));
    dx_v = dx_v(dx_v > 0.01);
    vox_sp = median(dx_v);
    if isempty(vox_sp) || isnan(vox_sp), vox_sp = 1.0; end
    
    try
        % alphaShape 捕获凹形边界
        alpha_r = vox_sp * 2.0;
        shp = alphaShape(valid_xy(:,1), valid_xy(:,2), alpha_r);
        
        % 提取边界, 排序成连续多边形
        [bf, bv] = boundaryFacets(shp);
        
        % 邻接表
        n_v = size(bv, 1);
        adj = cell(n_v, 1);
        for ei = 1:size(bf, 1)
            adj{bf(ei,1)}(end+1) = bf(ei,2);
            adj{bf(ei,2)}(end+1) = bf(ei,1);
        end
        
        % 遍历找最大边界环
        visited = false(n_v, 1);
        loops = {};
        for sv = 1:n_v
            if visited(sv) || isempty(adj{sv}), continue; end
            loop = sv; visited(sv) = true;
            cur = sv; prv = 0;
            while true
                nb = adj{cur};
                nxt = 0;
                for ni = 1:length(nb)
                    if nb(ni) ~= prv && ~visited(nb(ni))
                        nxt = nb(ni); break;
                    end
                end
                if nxt == 0, break; end
                visited(nxt) = true;
                loop(end+1) = nxt;
                prv = cur; cur = nxt;
            end
            if length(loop) >= 3
                loops{end+1} = loop;
            end
        end
        
        if ~isempty(loops)
            lsizes = cellfun(@length, loops);
            [~, mi] = max(lsizes);
            main_loop = loops{mi};
            bx = bv(main_loop, 1);
            by = bv(main_loop, 2);
            mask_poly = polyshape(bx, by);
        else
            kh = convhull(valid_xy(:,1), valid_xy(:,2));
            mask_poly = polyshape(valid_xy(kh,1), valid_xy(kh,2));
        end
    catch
        kh = convhull(valid_xy(:,1), valid_xy(:,2));
        mask_poly = polyshape(valid_xy(kh,1), valid_xy(kh,2));
    end
    
    % 外扩半个体素
    try
        mask_poly = polybuffer(mask_poly, vox_sp * 0.6);
    catch
    end
end


function contour_pts = expand_contour(contour_pts, expand_dist)
% polybuffer 外扩轮廓
    if expand_dist <= 0, return; end
    try
        ps = polyshape(contour_pts(:,1), contour_pts(:,2));
        ps_expanded = polybuffer(ps, expand_dist);
        if area(ps_expanded) > area(ps)
            [bx, by] = boundary(ps_expanded);
            if length(bx) >= 3
                contour_pts = [bx(:), by(:)];
                if ~isequal(contour_pts(1,:), contour_pts(end,:))
                    contour_pts(end+1,:) = contour_pts(1,:);
                end
            end
        end
    catch
        centroid = mean(contour_pts, 1);
        dirs = contour_pts - centroid;
        norms = sqrt(sum(dirs.^2, 2));
        norms(norms < 1e-10) = 1e-10;
        unit_dirs = dirs ./ norms;
        contour_pts = contour_pts + unit_dirs * expand_dist;
    end
end


function contours_out = clip_contours_to_mask(contours_in, mask_poly)
% 将多个轮廓裁剪到遮罩范围内
    contours_out = contours_in;
    for oi = 1:length(contours_in)
        try
            oc = contours_in{oi};
            oc_poly = polyshape(oc(:,1), oc(:,2));
            oc_clipped = intersect(oc_poly, mask_poly);
            if area(oc_clipped) > 0
                [bx_c, by_c] = boundary(oc_clipped);
                % 去掉 NaN 分隔, 取最大段
                nan_idx = find(isnan(bx_c));
                if isempty(nan_idx)
                    contours_out{oi} = [bx_c, by_c];
                else
                    segs = {};
                    prev = 1;
                    for si = [nan_idx(:)', length(bx_c)+1]
                        seg = [bx_c(prev:si-1), by_c(prev:si-1)];
                        if size(seg,1) >= 3, segs{end+1} = seg; end
                        prev = si + 1;
                    end
                    if ~isempty(segs)
                        seg_lens = cellfun(@(s) sum(sqrt(sum(diff(s).^2,2))), segs);
                        [~, best] = max(seg_lens);
                        contours_out{oi} = segs{best};
                    end
                end
            end
        catch
        end
    end
end


function clipped = clip_path_to_mask(pts_3d, mask_poly)
% 将3D路径裁剪到遮罩范围内, 返回 cell 数组 (可能分成多段)
    in_mask = isinterior(mask_poly, pts_3d(:,1), pts_3d(:,2));
    
    if all(in_mask)
        clipped = {pts_3d};
        return;
    end
    
    clipped = {};
    seg_start = 0;
    n_pts = size(pts_3d, 1);
    
    for pp = 1:n_pts
        if in_mask(pp) && seg_start == 0
            seg_start = pp;
        elseif ~in_mask(pp) && seg_start > 0
            if pp - seg_start >= 2
                clipped{end+1} = pts_3d(seg_start:pp-1, :);
            end
            seg_start = 0;
        end
    end
    if seg_start > 0 && n_pts - seg_start + 1 >= 2
        clipped{end+1} = pts_3d(seg_start:end, :);
    end
    
    % 如果全部被裁掉, 返回整条 (退化保护)
    if isempty(clipped) && size(pts_3d, 1) >= 2
        clipped = {pts_3d};
    end
end


function result = make_empty_layer_result(layer_idx, offset, err_msg, proc_time)
    result = struct();
    result.layer_idx = layer_idx;
    result.offset = offset;
    result.success = false;
    result.error_message = err_msg;
    result.outer_contours = {};
    result.inner_contours = {};
    result.streamlines = {};
    result.medial_axis = [];
    result.regions = {};
    result.paths_2d = {};
    result.paths_3d = {};
    result.pointCloud_data = [];
    result.statistics = struct('num_outer', 0, 'num_inner', 0, ...
        'num_streamlines', 0, 'num_regions', 0, 'num_paths_2d', 0, 'num_paths_3d', 0);
    result.processing_time = proc_time;
end