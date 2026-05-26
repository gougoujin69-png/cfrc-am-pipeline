%% ================================================================
%% 全层路径生成 - 纯偏置 (Offset-Only)
%% ================================================================
%
% 用途: 给 mine_offset / planar_offset 这两个对照组生成路径.
%       相对 all_layers_path_generation_v6.m, 砍掉了:
%         - 步骤 4: 方向场滤波 (用不到 t 字段)
%         - 步骤 7: 中轴线
%         - 步骤 8/9: 流线生成 + 延长
%         - 步骤 11: 用流线切区域
%       保留:
%         - 步骤 5: 二值图 + 3D 越界烧入 (XY mask + Z range)
%         - 步骤 6: 轮廓提取 + polybuffer 外扩
%         - 步骤 12: 结构遮罩裁剪
%         - 步骤 13: generate_offset_path2 + 2D->3D 投影
%
% 输入:
%   slice_file        - 'slice_results_refined_latest.mat' 或 '_PLANAR.mat'
%   output_file       - 输出 mat 名 (默认 'all_layers_paths_only_<cfg>.mat')
%
% 输出 (与 v6 输出的 paths_only 结构一致, 可直接喂给 export_paths_to_fea):
%   paths_only.num_layers
%   paths_only.layer_paths_2d{i}      - cell of Nx2 (XY)
%   paths_only.layer_paths_3d{i}      - cell of Nx3 (XYZ)
%   paths_only.layer_offsets(i)
%   paths_only.layer_outer_contours{i}
%   paths_only.layer_inner_contours{i}
%
% 用法:
%   path_generation_offset_only();                                  % 默认输入输出
%   path_generation_offset_only('slice_results_refined_latest_PLANAR.mat', ...
%                               'all_layers_paths_only_planar_offset.mat');
%

function path_generation_offset_only(slice_file, output_file)

if nargin < 1 || isempty(slice_file)
    slice_file = 'slice_results_refined_latest.mat';
end
if nargin < 2 || isempty(output_file)
    output_file = 'all_layers_paths_only_offset.mat';
end

warning('off', 'MATLAB:polyshape:repairedBySimplify');
warning('off', 'MATLAB:polyshape:boundary3Points');

fprintf('\n');
fprintf('================================================================\n');
fprintf('  全层路径生成 - 纯偏置 (Offset-Only)\n');
fprintf('  Input : %s\n', slice_file);
fprintf('  Output: %s\n', output_file);
fprintf('================================================================\n\n');

%% ========== 参数 ==========
params = struct();
params.offset_distance        = 0.3;
params.max_iterations         = 30;
params.min_path_length        = 2;
params.volfrac                = 0.5;
params.min_region_area        = 0.05;
params.min_contour_length_inner = 4;
params.contour_dilate_pixels  = 3;
params.contour_expand_ratio   = 0.6;

%% ========== Step 1: 工具箱检查 ==========
fprintf('[Step 1] Checking toolboxes...\n');
v = ver;
hasParallelToolbox = any(strcmp({v.Name}, 'Parallel Computing Toolbox'));
hasImageToolbox    = any(strcmp({v.Name}, 'Image Processing Toolbox'));
if hasParallelToolbox
    p = gcp('nocreate');
    if isempty(p)
        try
            parpool('local');
            p = gcp('nocreate');
        catch
            hasParallelToolbox = false;
        end
    end
    if hasParallelToolbox && ~isempty(p)
        fprintf('  Parallel ON (%d workers)\n', p.NumWorkers);
    end
end
fprintf('  Image Processing Toolbox: %s\n', mat2str(hasImageToolbox));

%% ========== Step 2: 加载切片 ==========
fprintf('\n[Step 2] Loading slice data...\n');
if ~exist(slice_file, 'file')
    error('%s not found', slice_file);
end
load(slice_file);
surface_layers   = slice_results.surface_layers;
grid_data        = slice_results.grid_data;
num_layers       = length(surface_layers);
valid_grid_mask  = slice_results.valid_grid_mask;
fprintf('  Total layers: %d\n', num_layers);

%% ========== Step 2b: XY 结构遮罩 + Z 场 ==========
structure_mask_poly = build_structure_mask(grid_data, valid_grid_mask);
has_structure_mask = area(structure_mask_poly) > 0;

if isfield(slice_results, 'z_height_field') && ~isempty(slice_results.z_height_field)
    zhf_data = slice_results.z_height_field;
    has_z_field = true;
else
    zhf_data = [];
    has_z_field = false;
end

%% ========== Step 3: 准备数据 ==========
layer_data_cells = cell(num_layers, 1);
for li = 1:num_layers
    layer_data_cells{li} = surface_layers{li};
end

%% ========== Step 4: 逐层处理 ==========
fprintf('\n[Step 4] Processing all layers (offset-only)...\n');
total_start_time = tic;
all_results_cell = cell(num_layers, 1);

if hasParallelToolbox
    parfor layer_idx = 1:num_layers
        all_results_cell{layer_idx} = process_layer_offset_only(...
            layer_idx, layer_data_cells{layer_idx}, params, ...
            has_structure_mask, structure_mask_poly, ...
            has_z_field, zhf_data, hasImageToolbox);
    end
else
    for layer_idx = 1:num_layers
        all_results_cell{layer_idx} = process_layer_offset_only(...
            layer_idx, layer_data_cells{layer_idx}, params, ...
            has_structure_mask, structure_mask_poly, ...
            has_z_field, zhf_data, hasImageToolbox);
    end
end

%% ========== Step 5: 汇总 ==========
all_layers_data(num_layers) = all_results_cell{end};
for li = 1:num_layers-1
    all_layers_data(li) = all_results_cell{li};
end

success_count = sum([all_layers_data.success]);
total_paths_3d = sum(arrayfun(@(x) length(x.paths_3d), all_layers_data));
total_paths_2d = sum(arrayfun(@(x) length(x.paths_2d), all_layers_data));
total_time = toc(total_start_time);
fprintf('  Done: %d/%d layers, 3D paths: %d, 2D paths: %d, time: %.1f s\n', ...
    success_count, num_layers, total_paths_3d, total_paths_2d, total_time);

%% ========== Step 6: 保存 paths_only 结构 ==========
paths_only = struct();
paths_only.num_layers              = num_layers;
paths_only.layer_paths_2d          = cell(num_layers, 1);
paths_only.layer_paths_3d          = cell(num_layers, 1);
paths_only.layer_offsets           = zeros(num_layers, 1);
paths_only.layer_outer_contours    = cell(num_layers, 1);
paths_only.layer_inner_contours    = cell(num_layers, 1);
paths_only.mode                    = 'offset_only';
paths_only.source_slice_file       = slice_file;

for i = 1:num_layers
    if all_layers_data(i).success
        paths_only.layer_paths_2d{i}        = all_layers_data(i).paths_2d;
        paths_only.layer_paths_3d{i}        = all_layers_data(i).paths_3d;
        paths_only.layer_offsets(i)         = all_layers_data(i).offset;
        paths_only.layer_outer_contours{i}  = all_layers_data(i).outer_contours;
        paths_only.layer_inner_contours{i}  = all_layers_data(i).inner_contours;
    else
        paths_only.layer_paths_2d{i}        = {};
        paths_only.layer_paths_3d{i}        = {};
        paths_only.layer_offsets(i)         = NaN;
        paths_only.layer_outer_contours{i}  = {};
        paths_only.layer_inner_contours{i}  = {};
    end
end

save(output_file, 'paths_only', '-v7.3');
fprintf('\n  Saved: %s\n', output_file);

fprintf('\n================================================================\n');
fprintf('  Offset-only path generation complete\n');
fprintf('================================================================\n\n');

end % function path_generation_offset_only


%% ========================================================================
%% 子函数: 单层处理 (纯偏置, 不用流线)
%% ========================================================================
function layer_result = process_layer_offset_only(...
        layer_idx, layer_data, params, ...
        has_structure_mask, structure_mask_poly, ...
        has_z_field, zhf_data, hasImageToolbox)

layer_start_time = tic;
fprintf('  Layer %d ', layer_idx);

% 重建 Z 场插值器 (parfor 内每个 worker 自己重建)
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
    activated_grids = layer_data.final_grids;
    X_surf = layer_data.X_surf;
    Y_surf = layer_data.Y_surf;
    Z_surf = layer_data.Z_surf;

    if isempty(activated_grids) || length(activated_grids) < 3
        layer_result = make_empty_layer_result(layer_idx, layer_data.offset, ...
            'insufficient grids', toc(layer_start_time));
        fprintf('SKIP (few grids)\n');
        return;
    end

    % 创建曲面插值器
    pointCloud_data = struct('X', X_surf, 'Y', Y_surf, 'Z', Z_surf);
    X_flat = X_surf(:); Y_flat = Y_surf(:); Z_flat = Z_surf(:);
    valid_pts = all(isfinite([X_flat, Y_flat, Z_flat]), 2);
    F_z = [];
    if sum(valid_pts) >= 3
        F_z = scatteredInterpolant(X_flat(valid_pts), Y_flat(valid_pts), ...
            Z_flat(valid_pts), 'linear', 'nearest');
    end

    % 2D 投影 (沿用 v6 的工具)
    [xold, ~, nelx, nely, ~] = extract_layer_2d_projection(activated_grids, ...
        get_grid_data_from_caller(layer_data));

    % 坐标范围
    num_grids = length(activated_grids);
    actual_x = arrayfun(@(g) g.x, activated_grids);
    actual_y = arrayfun(@(g) g.y, activated_grids);
    actual_ix = arrayfun(@(g) g.grid_index(1), activated_grids);
    actual_iy = arrayfun(@(g) g.grid_index(2), activated_grids);
    actual_x_min = min(actual_x); actual_x_max = max(actual_x);
    actual_y_min = min(actual_y); actual_y_max = max(actual_y);
    actual_range = max(actual_x_max - actual_x_min, actual_y_max - actual_y_min);
    adaptive_min_outer = max(actual_range * 0.05, 1.5);

    % --- [FIX] pixel<->physical 映射必须用真实的物理 cell 尺寸 ---
    % 旧版 scale_x = (actual_x_max - actual_x_min) / (nelx - 1) 只有在激活
    % voxel 跨满全网格时才正确; 拓扑优化结构在 X 中段窄会导致 scale_x 被
    % 低估, 路径在 X 方向被压缩到中段. 这里从同层 activated grids 反推真实
    % dx_phys 和原点 grid_x_origin.
    [ix_min_val, p_ix_min] = min(actual_ix);
    [ix_max_val, p_ix_max] = max(actual_ix);
    if ix_max_val > ix_min_val
        dx_phys = (actual_x(p_ix_max) - actual_x(p_ix_min)) / ...
                  (ix_max_val - ix_min_val);
        grid_x_origin = actual_x(p_ix_min) - (ix_min_val - 1) * dx_phys;
    else
        dx_phys = 1.0;
        grid_x_origin = actual_x_min;
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

    % --- 二值图像 ---
    x_filter = zeros(nely, nelx);
    x_filter(xold > 0) = 1;
    if hasImageToolbox && params.contour_dilate_pixels > 0
        se = strel('disk', params.contour_dilate_pixels);
        x_filter = imdilate(x_filter, se);
    end

    % --- 3D 烧入: 把 Z 越界 / XY 越界的像素清掉 ---
    if ~isempty(F_zmin_local) && ~isempty(F_zmax_local) && ~isempty(F_z)
        z_margin = max(scale_x, scale_y) * 0.5;
        for row = 1:nely
            for col = 1:nelx
                if x_filter(row, col) == 0, continue; end
                px = grid_x_origin + (col - 1) * scale_x;
                py = grid_y_origin + (row - 1) * scale_y;
                if has_structure_mask && ~isinterior(structure_mask_poly, px, py)
                    x_filter(row, col) = 0; continue;
                end
                pz = F_z(px, py);
                if isnan(pz), x_filter(row, col) = 0; continue; end
                z_lo = F_zmin_local(py, px);
                z_hi = F_zmax_local(py, px);
                if pz < z_lo - z_margin || pz > z_hi + z_margin
                    x_filter(row, col) = 0;
                end
            end
        end
        if hasImageToolbox
            x_filter = imerode(x_filter, ones(3));
            x_filter = imdilate(x_filter, ones(3));
        end
    end

    % --- 轮廓 ---
    [B, ~, N, ~] = bwboundaries(x_filter, 'holes');
    outer_contours = {};
    inner_contours = {};
    for k = 1:length(B)
        bnd = B{k};
        if isempty(bnd) || size(bnd, 1) < 3, continue; end
        if size(bnd, 1) > 200
            step = ceil(size(bnd, 1) / 200);
            bnd = bnd(1:step:end, :);
        end
        if ~isequal(bnd(1,:), bnd(end,:))
            bnd(end+1,:) = bnd(1,:); %#ok<AGROW>
        end
        if size(bnd, 1) > 6
            bnd(:,1) = smooth(bnd(:,1), 3);
            bnd(:,2) = smooth(bnd(:,2), 3);
        end
        x_actual = grid_x_origin + (bnd(:, 2) - 1) * scale_x;
        y_actual = grid_y_origin + (bnd(:, 1) - 1) * scale_y;
        contour_pts = [x_actual(:), y_actual(:)];

        if k <= N
            contour_pts = expand_contour(contour_pts, expand_dist);
            contour_length = sum(sqrt(sum(diff(contour_pts).^2, 2)));
            if contour_length >= adaptive_min_outer
                outer_contours{end+1} = contour_pts; %#ok<AGROW>
            end
        else
            contour_length = sum(sqrt(sum(diff(contour_pts).^2, 2)));
            if contour_length >= params.min_contour_length_inner
                inner_contours{end+1} = contour_pts; %#ok<AGROW>
            end
        end
    end

    % --- 直接构造区域 (不切流线) ---
    % 每个 outer 减去落在它内部的所有 inner = 一个区域
    all_regions = {};
    for ci = 1:length(outer_contours)
        try
            out_poly = polyshape(outer_contours{ci}(:,1), outer_contours{ci}(:,2));
        catch
            continue;
        end
        if area(out_poly) < params.min_region_area, continue; end

        % 找属于这个外轮廓的内孔
        my_inners = {};
        for h = 1:length(inner_contours)
            ic = inner_contours{h};
            inner_center = mean(ic, 1);
            if isinterior(out_poly, inner_center(1), inner_center(2))
                my_inners{end+1} = ic; %#ok<AGROW>
            end
        end

        % 减去内孔
        for h = 1:length(my_inners)
            try
                hole_poly = polyshape(my_inners{h}(:,1), my_inners{h}(:,2));
                out_poly = subtract(out_poly, hole_poly);
            catch
            end
        end

        if area(out_poly) > params.min_region_area
            all_regions{end+1} = out_poly; %#ok<AGROW>
        end
    end

    % 结构遮罩裁剪
    if has_structure_mask && ~isempty(all_regions)
        clipped = {};
        for ri = 1:length(all_regions)
            try
                c = intersect(all_regions{ri}, structure_mask_poly);
                parts = regions(c);
                for cp = 1:length(parts)
                    if area(parts(cp)) > params.min_region_area
                        clipped{end+1} = parts(cp); %#ok<AGROW>
                    end
                end
            catch
                clipped{end+1} = all_regions{ri}; %#ok<AGROW>
            end
        end
        all_regions = clipped;
    end

    % --- 生成偏置路径 ---
    all_paths_2d = {};
    all_paths_3d = {};
    for i = 1:length(all_regions)
        region_result = all_regions{i};
        if area(region_result) < params.min_region_area, continue; end
        try
            [outer_cell, inner_cell] = polyshape_to_cell(region_result);
            if isempty(outer_cell), continue; end
            rings_results = generate_offset_path2(outer_cell, inner_cell, ...
                params.offset_distance, params.max_iterations, ...
                params.min_path_length, pointCloud_data);

            for j = 1:length(rings_results)
                rings_iter = rings_results{j};
                if isempty(rings_iter), continue; end
                for k = 1:length(rings_iter)
                    pts_2d_poly = rings_iter{k};
                    if isempty(pts_2d_poly), continue; end
                    if isa(pts_2d_poly, 'polyshape')
                        [px, py] = boundary(pts_2d_poly);
                        if length(px) < 2, continue; end
                        pts_2d = [py(:), px(:)];   % v6 习惯: 列 = (row, col) -> 后面 pp(2),pp(1)
                    else
                        if size(pts_2d_poly, 1) < 2, continue; end
                        pts_2d = pts_2d_poly;
                    end
                    all_paths_2d{end+1} = pts_2d; %#ok<AGROW>

                    % 2D -> 3D
                    pts_3d = zeros(size(pts_2d, 1), 3);
                    for pp = 1:size(pts_2d, 1)
                        x = pts_2d(pp, 2);
                        y = pts_2d(pp, 1);
                        if ~isempty(F_z)
                            z = F_z(x, y);
                        else
                            z = mean(Z_surf(:), 'omitnan');
                        end
                        if isnan(z) || ~isfinite(z)
                            d2 = (pointCloud_data.X(:) - x).^2 + (pointCloud_data.Y(:) - y).^2;
                            [~, idx] = min(d2);
                            z = pointCloud_data.Z(idx);
                        end
                        pts_3d(pp, :) = [x, y, z];
                    end

                    if has_structure_mask
                        cl3 = clip_path_to_mask(pts_3d, structure_mask_poly);
                        for ss = 1:length(cl3)
                            all_paths_3d{end+1} = cl3{ss}; %#ok<AGROW>
                        end
                    else
                        all_paths_3d{end+1} = pts_3d; %#ok<AGROW>
                    end
                end
            end
        catch ME
            fprintf('[WARN region %d: %s] ', i, ME.message);
        end
    end

    layer_result = struct();
    layer_result.layer_idx        = layer_idx;
    layer_result.offset           = layer_data.offset;
    layer_result.outer_contours   = outer_contours;
    layer_result.inner_contours   = inner_contours;
    layer_result.streamlines      = {};
    layer_result.medial_axis      = [];
    layer_result.regions          = all_regions;
    layer_result.paths_2d         = all_paths_2d;
    layer_result.paths_3d         = all_paths_3d;
    layer_result.statistics       = struct(...
        'num_outer', length(outer_contours), ...
        'num_inner', length(inner_contours), ...
        'num_regions', length(all_regions), ...
        'num_paths_2d', length(all_paths_2d), ...
        'num_paths_3d', length(all_paths_3d));
    layer_result.processing_time = toc(layer_start_time);
    layer_result.success = true;
    layer_result.error_message = '';

    fprintf('outer=%d inner=%d region=%d 3d=%d (%.1fs)\n', ...
        length(outer_contours), length(inner_contours), ...
        length(all_regions), length(all_paths_3d), layer_result.processing_time);

catch ME
    fprintf('FAILED: %s\n', ME.message);
    layer_result = make_empty_layer_result(layer_idx, layer_data.offset, ...
        ME.message, toc(layer_start_time));
end

end % function process_layer_offset_only


function gd = get_grid_data_from_caller(layer_data)
% extract_layer_2d_projection 需要 grid_data 来推 [nelx, nely, nelz]
% 但我们其实只在乎尺寸. 这里用 layer_data.final_grids 的 grid_index 反推.
% 但更可靠的方式: 直接从 base workspace 取 (parfor 下不行), 因此读外部 mat:
persistent cached_grid_data
if ~isempty(cached_grid_data)
    gd = cached_grid_data;
    return;
end
% 第一次调用时从 mat 加载
try
    S = load('voxel_refined_latest.mat', 'refined_data');
    cached_grid_data = S.refined_data.grid_data;
    gd = cached_grid_data;
catch
    % 退化: 用 grid_index 推 size
    max_ijk = max(reshape([layer_data.final_grids.grid_index], 3, []), [], 2);
    gd = zeros(max_ijk(1), max_ijk(2), max_ijk(3));
    cached_grid_data = gd;
end
end


function lr = make_empty_layer_result(layer_idx, offset, msg, t)
lr = struct();
lr.layer_idx = layer_idx;
lr.offset = offset;
lr.outer_contours = {}; lr.inner_contours = {};
lr.streamlines = {}; lr.medial_axis = [];
lr.regions = {}; lr.paths_2d = {}; lr.paths_3d = {};
lr.statistics = struct('num_outer',0,'num_inner',0,'num_regions',0,'num_paths_2d',0,'num_paths_3d',0);
lr.processing_time = t;
lr.success = false;
lr.error_message = msg;
end


function mask_poly = build_structure_mask(grid_data, valid_grid_mask)
% 同 v6: 把所有有效体素投影到 XY 平面, 用 alphaShape 构造遮罩
[nelx, nely, nelz] = size(valid_grid_mask);
xy_pts = [];
for k = 1:nelz
    for j = 1:nely
        for i = 1:nelx
            if valid_grid_mask(i,j,k)
                xy_pts(end+1, :) = [grid_data(i,j,k).x, grid_data(i,j,k).y]; %#ok<AGROW>
            end
        end
    end
end
if isempty(xy_pts)
    mask_poly = polyshape();
    return;
end
xy_pts = unique(xy_pts, 'rows');
try
    shp = alphaShape(xy_pts(:,1), xy_pts(:,2), 2.0);
    [B, V] = boundaryFacets(shp);
    if isempty(B), mask_poly = polyshape(); return; end
    rings_x = {}; rings_y = {};
    cur_x = V(B(1,1), 1); cur_y = V(B(1,1), 2);
    rings_x{1} = cur_x; rings_y{1} = cur_y;
    for q = 1:size(B,1)
        rings_x{1}(end+1) = V(B(q,2), 1); %#ok<AGROW>
        rings_y{1}(end+1) = V(B(q,2), 2); %#ok<AGROW>
    end
    mask_poly = polyshape(rings_x{1}, rings_y{1});
    mask_poly = polybuffer(mask_poly, 0.5);
catch
    mask_poly = polyshape();
end
end


function contour_pts_out = expand_contour(contour_pts, expand_dist)
% 用 polybuffer 外扩
try
    poly = polyshape(contour_pts(:,1), contour_pts(:,2));
    poly_ex = polybuffer(poly, expand_dist);
    if poly_ex.NumRegions == 0
        contour_pts_out = contour_pts; return;
    end
    [bx, by] = boundary(poly_ex);
    contour_pts_out = [bx(:), by(:)];
catch
    contour_pts_out = contour_pts;
end
end


function out = clip_path_to_mask(pts_3d, mask_poly)
% 把 3D 路径按 XY 投影裁到 mask 内, 沿 XY 越界切断成多条
out = {};
if isempty(pts_3d) || size(pts_3d,1) < 2
    return;
end
in_flag = isinterior(mask_poly, pts_3d(:,1), pts_3d(:,2));
cur = [];
for q = 1:size(pts_3d,1)
    if in_flag(q)
        cur(end+1, :) = pts_3d(q, :); %#ok<AGROW>
    else
        if size(cur,1) >= 2, out{end+1} = cur; end %#ok<AGROW>
        cur = [];
    end
end
if size(cur,1) >= 2, out{end+1} = cur; end
end
