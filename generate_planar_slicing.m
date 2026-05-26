function generate_planar_slicing(voxel_mat, output_mat)
%% ========================================
%% 平面切片 (Planar Slicing) - 对照组用
%% ========================================
%
% 目的: 提供与 slice_refined_model_v6.m 完全一致的输出结构 (slice_results),
%       但每一层是恒 Z 的平面切片. 用作 planar_stream / planar_offset 这两个
%       对照组的输入. 下游的 all_layers_path_generation_v6.m /
%       path_generation_offset_only.m 不需要任何改动, 只要换输入 mat 即可.
%
% 输入 (可选):
%   voxel_mat    - 体素 mat 路径 (默认 'voxel_refined_latest.mat')
%                  须含 refined_data.grid_data + valid_grid_mask
%   output_mat   - 输出 mat 路径 (默认 'slice_results_refined_latest_PLANAR.mat')
%
% 用法:
%   generate_planar_slicing();                                   % 默认
%   generate_planar_slicing('voxel_refined_latest.mat', ...
%                           'slice_results_refined_latest_PLANAR.mat');
%
% 字段对照表 (与 v6 完全一致):
%   slice_results.surface_layers{i}.offset           - 层中心 Z 坐标 (vs 基准面)
%   slice_results.surface_layers{i}.direction        - 'planar'
%   slice_results.surface_layers{i}.X_surf, Y_surf, Z_surf  - 层网格 (Z 恒定)
%   slice_results.surface_layers{i}.final_grids      - 该层激活的体素列表
%   slice_results.surface_layers{i}.total_activated  - 数量
%   slice_results.grid_data                          - 复制自 refined_data
%   slice_results.valid_grid_mask                    - 同上
%   slice_results.grid_size                          - struct(nelx,nely,nelz)
%   slice_results.z_height_field.{xx,yy,Z_min_map,Z_max_map}
%   slice_results.statistics, .parameters, .metadata
%
% 注意: 这里"每层物理厚度"会被 refined_data.parameters.SCALE_FACTOR 缩放
%       (LAYER_THICKNESS_MM = LAYER_THICKNESS_ORIG * SCALE_FACTOR),
%       与 slice_refined_model_v6 中的 OFFSET_STEP = OFFSET_STEP_ORIG *
%       SCALE_FACTOR 完全对齐, 保证 planar 和 mine 的层数一致.
%       默认 LAYER_THICKNESS_ORIG = 1.0 mm (= 一个原始体素的 z 维).

if nargin < 1 || isempty(voxel_mat)
    voxel_mat = 'voxel_refined_latest.mat';
end
if nargin < 2 || isempty(output_mat)
    output_mat = 'slice_results_refined_latest_PLANAR.mat';
end

fprintf('\n');
fprintf('==============================================================\n');
fprintf('     Planar Slicing for CFRC 5-way Comparison              \n');
fprintf('  Input : %s\n', voxel_mat);
fprintf('  Output: %s\n', output_mat);
fprintf('==============================================================\n\n');

%% ========== Step 0: 参数 (与 v6 命名对齐) ==========
LAYER_THICKNESS_ORIG = 1.0;   % 原始物理层厚 (= 一个原始体素的 z 维, mm)
DENSITY_THRESHOLD    = 0.5;   % 与 v6 一致
MIN_GRIDS_PER_LAYER  = 3;     % 少于这个数的层丢弃
LAYER_XY_PADDING     = 0.5;   % X_surf/Y_surf 边界外扩 (mm)

%% ========== Step 1: 加载体素数据 ==========
fprintf('[1] Loading voxel data...\n');

if ~exist(voxel_mat, 'file')
    error('%s not found.', voxel_mat);
end
load(voxel_mat);

grid_data        = refined_data.grid_data;
valid_grid_mask  = refined_data.valid_grid_mask;
nelx             = refined_data.grid_size.nelx;
nely             = refined_data.grid_size.nely;
nelz             = refined_data.grid_size.nelz;

fprintf('  Grid: %d x %d x %d, valid voxels: %d\n', ...
    nelx, nely, nelz, sum(valid_grid_mask(:)));

%% ========== Step 1b: 应用 SCALE_FACTOR (与 v6 OFFSET_STEP 同步) ==========
% v6 里:  OFFSET_STEP = OFFSET_STEP_ORIG * SCALE_FACTOR;
% 这里也用完全相同的换算, 保证 planar 和 mine 的层数一致.
if ~isfield(refined_data, 'parameters') || ~isfield(refined_data.parameters, 'SCALE_FACTOR')
    warning(['refined_data.parameters.SCALE_FACTOR not found in %s; ' ...
             'falling back to SCALE_FACTOR = 1.0 (planar layer count may ' ...
             'NOT match curved).'], voxel_mat);
    SCALE_FACTOR = 1.0;
else
    SCALE_FACTOR = refined_data.parameters.SCALE_FACTOR;
end
LAYER_THICKNESS_MM = LAYER_THICKNESS_ORIG * SCALE_FACTOR;

fprintf('  SCALE_FACTOR         : %.4f\n', SCALE_FACTOR);
fprintf('  LAYER_THICKNESS_ORIG : %.4f mm\n', LAYER_THICKNESS_ORIG);
fprintf('  LAYER_THICKNESS_MM   : %.4f mm  (= ORIG * SCALE_FACTOR)\n', ...
    LAYER_THICKNESS_MM);

%% ========== Step 2: 全部有效体素的几何范围 ==========
fprintf('\n[2] Computing global bounds of valid voxels...\n');

% 把所有有效体素的坐标取出来
n_valid = sum(valid_grid_mask(:));
all_xc = zeros(n_valid, 1);
all_yc = zeros(n_valid, 1);
all_zc = zeros(n_valid, 1);
all_t_xoy = zeros(n_valid, 1);
all_t_xoz = zeros(n_valid, 1);
all_density = zeros(n_valid, 1);
all_ijk = zeros(n_valid, 3);

cnt = 0;
for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if valid_grid_mask(i,j,k)
                cnt = cnt + 1;
                g = grid_data(i,j,k);
                all_xc(cnt)      = g.x;
                all_yc(cnt)      = g.y;
                all_zc(cnt)      = g.z;
                all_t_xoy(cnt)   = g.t_xoy;
                all_t_xoz(cnt)   = g.t_xoz;
                all_density(cnt) = g.xPhys;
                all_ijk(cnt,:)   = [i, j, k];
            end
        end
    end
end

xmin = min(all_xc); xmax = max(all_xc);
ymin = min(all_yc); ymax = max(all_yc);
zmin = min(all_zc); zmax = max(all_zc);

fprintf('  X range: [%.3f, %.3f]\n', xmin, xmax);
fprintf('  Y range: [%.3f, %.3f]\n', ymin, ymax);
fprintf('  Z range: [%.3f, %.3f]\n', zmin, zmax);

%% ========== Step 3: 决定 Z 切层位置 ==========
fprintf('\n[3] Determining planar layer positions...\n');

% 切层中心位置: 从 zmin 起步, 间距 LAYER_THICKNESS_MM
% 用闭区间 [center - h/2, center + h/2) 把体素分到对应层
h = LAYER_THICKNESS_MM;
n_layers_max = ceil((zmax - zmin) / h) + 1;
layer_centers = zmin + (0:n_layers_max-1) * h;

% 截到 zmax 以下的最后一个
layer_centers = layer_centers(layer_centers <= zmax + h/2);
num_layers = length(layer_centers);

fprintf('  Layer thickness: %.3f mm\n', h);
fprintf('  Number of planar layers (initial): %d\n', num_layers);

%% ========== Step 4: 给每个体素分配 layer index ==========
fprintf('\n[4] Assigning voxels to layers...\n');

% 每个体素属于 floor((zc - zmin) / h) + 1 这一层
voxel_layer_idx = floor((all_zc - zmin) / h) + 1;
voxel_layer_idx(voxel_layer_idx < 1) = 1;
voxel_layer_idx(voxel_layer_idx > num_layers) = num_layers;

% 给每层准备 final_grids
layer_voxels = cell(num_layers, 1);
for L = 1:num_layers
    mask = (voxel_layer_idx == L);
    if sum(mask) < MIN_GRIDS_PER_LAYER
        layer_voxels{L} = [];
        continue;
    end
    % 构造和 v6 一致的 activated_grids struct array
    idx = find(mask);
    n_in_L = length(idx);
    ag = struct('x', cell(1, n_in_L), 'y', cell(1, n_in_L), ...
                'z', cell(1, n_in_L), 't_xoy', cell(1, n_in_L), ...
                't_xoz', cell(1, n_in_L), 'xPhys', cell(1, n_in_L), ...
                'grid_index', cell(1, n_in_L));
    for q = 1:n_in_L
        ii = idx(q);
        ag(q).x          = all_xc(ii);
        ag(q).y          = all_yc(ii);
        ag(q).z          = all_zc(ii);
        ag(q).t_xoy      = all_t_xoy(ii);
        ag(q).t_xoz      = all_t_xoz(ii);
        ag(q).xPhys      = all_density(ii);
        ag(q).grid_index = all_ijk(ii,:);
    end
    layer_voxels{L} = ag;
end

% 过滤掉空层 (重新编号)
keep_mask = ~cellfun(@isempty, layer_voxels);
layer_voxels   = layer_voxels(keep_mask);
layer_centers  = layer_centers(keep_mask);
num_layers     = length(layer_voxels);
fprintf('  Layers after filtering: %d (dropped %d empty)\n', ...
    num_layers, sum(~keep_mask));

%% ========== Step 5: 构造 X_surf/Y_surf/Z_surf (每层恒 Z 平面) ==========
fprintf('\n[5] Building per-layer planar surfaces...\n');

% 整个工件的 XY 覆盖网格 (共享给所有层)
surf_res = h * 0.5;  % 分辨率 = 半个 voxel, 便于路径采样
xx_g = (xmin - LAYER_XY_PADDING) : surf_res : (xmax + LAYER_XY_PADDING);
yy_g = (ymin - LAYER_XY_PADDING) : surf_res : (ymax + LAYER_XY_PADDING);
[X_grid, Y_grid] = meshgrid(xx_g, yy_g);
fprintf('  Surface XY grid: %d x %d (res=%.3f mm)\n', ...
    size(X_grid,1), size(X_grid,2), surf_res);

surface_layers = cell(num_layers, 1);
for L = 1:num_layers
    z_const = layer_centers(L);
    X_surf  = X_grid;
    Y_surf  = Y_grid;
    Z_surf  = z_const * ones(size(X_grid));

    sl = struct();
    sl.offset             = z_const - zmin;   % 相对最底层的偏置
    sl.direction          = 'planar';
    sl.final_grids        = layer_voxels{L};
    sl.final_threshold    = DENSITY_THRESHOLD;
    sl.total_activated    = length(layer_voxels{L});
    sl.total_newly_activated = length(layer_voxels{L});
    sl.X_surf             = X_surf;
    sl.Y_surf             = Y_surf;
    sl.Z_surf             = Z_surf;
    sl.attempts           = struct('threshold', DENSITY_THRESHOLD, ...
                                   'num_activated', length(layer_voxels{L}), ...
                                   'num_newly_activated', length(layer_voxels{L}));
    surface_layers{L} = sl;
end

%% ========== Step 6: Z 高度场 (Z_min_map / Z_max_map) ==========
% 把每个 (x,y) 处 valid 体素的 Z 范围插值到一张 2D 图上, 用于下游路径生成时
% 在 X_filter 烧入 Z 越界像素.
fprintf('\n[6] Building Z height field...\n');

% 把所有有效体素的 (x, y, z) 拿来; 对每个 (x,y) 取所有该 (x,y) 上方体素的
% z_min 和 z_max. 这里用稀疏插值, 不需要稠密重建.
zmap_xx = xx_g;
zmap_yy = yy_g;
Z_min_map = nan(length(yy_g), length(xx_g));
Z_max_map = nan(length(yy_g), length(xx_g));

% 对每个有效 (xc, yc) (XY 投影), 找属于该列的所有 zc
% 简化处理: 按 (xc, yc) 离散化分桶
[xy_uniq, ~, xy_ids] = unique([all_xc, all_yc], 'rows');
for u = 1:size(xy_uniq, 1)
    col_mask = (xy_ids == u);
    z_in_col = all_zc(col_mask);
    z_lo = min(z_in_col) - h/2;
    z_hi = max(z_in_col) + h/2;
    % 把这一列广播到 X_grid 上离 (xc, yc) 最近的几个 grid 点
    xc = xy_uniq(u, 1); yc = xy_uniq(u, 2);
    [~, ix_near] = min(abs(xx_g - xc));
    [~, iy_near] = min(abs(yy_g - yc));
    % 在邻域 (1.5 voxel 半径) 内填充, 避免阶梯
    r = ceil(1.5 * h / surf_res);
    for ddx = -r:r
        for ddy = -r:r
            cx = ix_near + ddx;
            cy = iy_near + ddy;
            if cx < 1 || cx > length(xx_g), continue; end
            if cy < 1 || cy > length(yy_g), continue; end
            % 距离判断
            d2 = (xx_g(cx) - xc)^2 + (yy_g(cy) - yc)^2;
            if d2 > (1.5*h)^2, continue; end
            if isnan(Z_min_map(cy, cx)) || z_lo < Z_min_map(cy, cx)
                Z_min_map(cy, cx) = z_lo;
            end
            if isnan(Z_max_map(cy, cx)) || z_hi > Z_max_map(cy, cx)
                Z_max_map(cy, cx) = z_hi;
            end
        end
    end
end

% 填充剩余 NaN: nearest 邻近
nan_mask = isnan(Z_min_map);
if any(nan_mask(:))
    valid_idx = ~nan_mask;
    F_lo = scatteredInterpolant(X_grid(valid_idx), Y_grid(valid_idx), ...
                                Z_min_map(valid_idx), 'nearest', 'nearest');
    F_hi = scatteredInterpolant(X_grid(valid_idx), Y_grid(valid_idx), ...
                                Z_max_map(valid_idx), 'nearest', 'nearest');
    Z_min_map(nan_mask) = F_lo(X_grid(nan_mask), Y_grid(nan_mask));
    Z_max_map(nan_mask) = F_hi(X_grid(nan_mask), Y_grid(nan_mask));
end

fprintf('  Z height field: %d x %d, Z_min in [%.2f, %.2f], Z_max in [%.2f, %.2f]\n', ...
    size(Z_min_map,1), size(Z_min_map,2), ...
    min(Z_min_map(:)), max(Z_min_map(:)), ...
    min(Z_max_map(:)), max(Z_max_map(:)));

%% ========== Step 7: 打包 slice_results 结构 ==========
fprintf('\n[7] Packing slice_results...\n');

slice_results = struct();
slice_results.surface_layers     = surface_layers;
slice_results.grid_data          = grid_data;
slice_results.valid_grid_mask    = valid_grid_mask;
slice_results.global_activated   = double(valid_grid_mask);  % 平面切片下全部体素都是激活过的
slice_results.above_surface_mask = [];
slice_results.below_surface_mask = [];
slice_results.grid_size = struct('nelx', nelx, 'nely', nely, 'nelz', nelz);
slice_results.statistics = struct(...
    'num_layers', num_layers, ...
    'total_activated_voxels', n_valid, ...
    'coverage_rate', 100.0);
slice_results.parameters = struct(...
    'LAYER_THICKNESS_ORIG', LAYER_THICKNESS_ORIG, ...
    'LAYER_THICKNESS_MM', LAYER_THICKNESS_MM, ...
    'SCALE_FACTOR', SCALE_FACTOR, ...
    'DENSITY_THRESHOLD', DENSITY_THRESHOLD, ...
    'mode', 'planar');
slice_results.surface_params = refined_data.surface_params;  % 透传 (路径生成不强依赖)
slice_results.metadata = struct(...
    'timestamp', datestr(now, 'yyyymmdd_HHMMSS'), ...
    'date', datestr(now), ...
    'version', 'planar_v2_scale_factor_aligned');
slice_results.z_height_field = struct(...
    'xx', zmap_xx, 'yy', zmap_yy, ...
    'Z_min_map', Z_min_map, 'Z_max_map', Z_max_map);

%% ========== Step 8: 保存 ==========
filename = output_mat;
save(filename, 'slice_results', '-v7.3');
fprintf('\n  Saved: %s\n', filename);
fi = dir(filename);
fprintf('  Size: %.2f MB\n', fi.bytes/1024/1024);

fprintf('\n==============================================================\n');
fprintf('  Planar slicing complete!\n');
fprintf('  Layers: %d, layer thickness: %.2f mm\n', num_layers, LAYER_THICKNESS_MM);
fprintf('  下一步: 在 path_generation_offset_only.m / all_layers_path_generation_v6.m\n');
fprintf('         里把 slice_file 改成 ''slice_results_refined_latest_PLANAR.mat''\n');
fprintf('==============================================================\n\n');

end % function generate_planar_slicing
