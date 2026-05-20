%% ========================================
%% 细化模型完整切片系统
%% ========================================
% 功能：在细化后的体素模型上运行完整的自适应切片算法
% 输入：voxel_refined_latest.mat（细化结果）
% 输出：slice_results_refined_*.mat（切片结果）
%%

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║        细化模型自适应切片系统                         ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% ========== 第1步：加载细化数据 ==========
fprintf('【步骤1】加载细化模型数据...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

if ~exist('voxel_refined_latest.mat', 'file')
    error('找不到细化模型文件！请先运行 voxel_refinement_from_test.m');
end

load('voxel_refined_latest.mat');
load('Pre_surface.mat');
% 提取数据
grid_data = refined_data.grid_data;
valid_grid_mask = refined_data.valid_grid_mask;
nelx = refined_data.grid_size.nelx;
nely = refined_data.grid_size.nely;
nelz = refined_data.grid_size.nelz;

fprintf('✓ 成功加载细化模型\n');
fprintf('  网格尺寸: %d × %d × %d\n', nelx, nely, nelz);
fprintf('  有效体素: %d\n', sum(valid_grid_mask(:)));

%% ========== 第2步：应用缩放后的切片参数 ==========
fprintf('\n【步骤2】配置切片参数（自动缩放）...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 从细化参数中获取缩放因子
SCALE_FACTOR = refined_data.parameters.SCALE_FACTOR;

% 原始参数（从原切片系统）
OFFSET_STEP_ORIG = 1.0;
%OFFSET_STEP_ORIG = 0.5;%实际                                                            层高应该是0.125，缩放层
INITIAL_THRESHOLD_ORIG = 0.70;
THRESHOLD_INCREMENT_ORIG = 0.5;
SURFACE_RESOLUTION_ORIG = 0.04;

% 缩放后的参数
OFFSET_STEP = OFFSET_STEP_ORIG * SCALE_FACTOR;
INITIAL_THRESHOLD = INITIAL_THRESHOLD_ORIG * SCALE_FACTOR;
THRESHOLD_INCREMENT = THRESHOLD_INCREMENT_ORIG * SCALE_FACTOR;
SURFACE_RESOLUTION = SURFACE_RESOLUTION_ORIG / SCALE_FACTOR;
DENSITY_THRESHOLD = 0.5;
NEW_ACTIVATION_THRESHOLD = 5;
MAX_OFFSET = 120 / SCALE_FACTOR;

fprintf('切片参数:\n');
fprintf('  缩放因子: %.4f\n', SCALE_FACTOR);
fprintf('  OFFSET_STEP: %.4f (原始: %.2f)\n', OFFSET_STEP, OFFSET_STEP_ORIG);
fprintf('  INITIAL_THRESHOLD: %.4f (原始: %.2f)\n', INITIAL_THRESHOLD, INITIAL_THRESHOLD_ORIG);
fprintf('  THRESHOLD_INCREMENT: %.4f (原始: %.2f)\n', THRESHOLD_INCREMENT, THRESHOLD_INCREMENT_ORIG);
fprintf('  SURFACE_RESOLUTION: %.5f (原始: %.3f)\n', SURFACE_RESOLUTION, SURFACE_RESOLUTION_ORIG);
fprintf('  MAX_OFFSET: %.2f\n', MAX_OFFSET);

%% ========== 第3步：加载曲面参数 ==========
fprintf('\n【步骤3】加载曲面方程参数...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

if isfield(refined_data, 'surface_params')
    sp = refined_data.surface_params;
    a = sp.a; b = sp.b; c = sp.c; d = sp.d;
    e = sp.e; f = sp.f; g = sp.g; h = sp.h;
    X0 = sp.X0; Y0 = sp.Y0; Para_me = sp.Para_me;
    fprintf('✓ 从细化数据中加载曲面参数\n');
else
    error('细化数据中缺少曲面参数！');
end

fprintf('  曲面中心: (%.3f, %.3f)\n', X0, Y0);
fprintf('  偏移参数: %.2f\n', Para_me);

%% ========== 第4步：统计网格边界 ==========
fprintf('\n【步骤4】计算有效区域边界...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

total_valid_grids = sum(valid_grid_mask(:));

min_valid_x = inf; max_valid_x = -inf;
min_valid_y = inf; max_valid_y = -inf;
min_valid_z = inf; max_valid_z = -inf;

for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if valid_grid_mask(i,j,k)
                min_valid_x = min(min_valid_x, grid_data(i,j,k).x);
                max_valid_x = max(max_valid_x, grid_data(i,j,k).x);
                min_valid_y = min(min_valid_y, grid_data(i,j,k).y);
                max_valid_y = max(max_valid_y, grid_data(i,j,k).y);
                min_valid_z = min(min_valid_z, grid_data(i,j,k).z);
                max_valid_z = max(max_valid_z, grid_data(i,j,k).z);
            end
        end
    end
end

fprintf('有效区域:\n');
fprintf('  X: [%.3f, %.3f]\n', min_valid_x, max_valid_x);
fprintf('  Y: [%.3f, %.3f]\n', min_valid_y, max_valid_y);
fprintf('  Z: [%.3f, %.3f]\n', min_valid_z, max_valid_z);
fprintf('  总有效网格: %d\n', total_valid_grids);

%% ========== 第5步：定义曲面方程 ==========
%compute_surface_z = @(x, y) a.*(x-X0).^3 + b.*(y-Y0).^3 + ...
%                            c.*(x-X0).*(y-Y0) + d.*(x-X0).^2 + ...
%                            e.*(y-Y0).^2 + f.*(x-X0) + g.*(y-Y0) + ...
%                            h + Para_me;

%compute_normal = @(x, y) deal(...
%    -(3*a*(x-X0).^2 + c*(y-Y0) + 2*d*(x-X0) + f), ...
%    -(3*b*(y-Y0).^2 + c*(x-X0) + 2*e*(y-Y0) + g), ...
%    1);
Xs=Pre_surface{1};
Ys=Pre_surface{2};
Zs=Pre_surface{3};
% 1. 创建插值对象 (线性插值 'linear' 或 平滑插值 'natural')
% 注意：如果是矩阵形式，需要转为列向量 [:]
F_z = scatteredInterpolant(Xs(:), Ys(:), Zs(:), 'linear', 'none');
% 2. 构造计算 Z 的匿名函数
compute_surface_z = @(x, y) F_z(x, y);
% 3. 构造法向量函数 (基于数值梯度)
% 注意：法向量通常需要对 Zs 提前计算梯度
%[dZdx, dZdy] = gradient(Zs, Xs(1,2)-Xs(1,1), Ys(2,1)-Ys(1,1)); 
%F_nx = scatteredInterpolant(Xs(:), Ys(:), dZdx(:), 'linear');
%F_ny = scatteredInterpolant(Xs(:), Ys(:), dZdy(:), 'linear');
%compute_normal = @(x, y) normalize_vector(-F_nx(x, y), -F_ny(x, y), 1);
%% --- 核心计算部分 ---
P = [Xs(:), Ys(:), Zs(:)];
N = size(P, 1);
normals = zeros(N, 3);
k = 15; % 邻域点数

% 1. 寻找最近邻
if exist('createns', 'file')
    ns = createns(P, 'NSMethod', 'kdtree');
    idx = knnsearch(ns, P, 'K', k);
else
    D = pdist2(P, P);
    [~, idx] = sort(D, 2);
    idx = idx(:, 1:k);
end

% 2. PCA 计算法向量
for i = 1:N
    neighbors = P(idx(i, :), :);
    centroid = mean(neighbors, 1);
    de_neighbors = neighbors - centroid;
    [~, ~, V] = svd(de_neighbors, 0);
    n = V(:, 3)'; 
    if n(3) < 0, n = -n; end % 确保朝上
    normals(i, :) = n;
end

% 3. 构造插值对象
F_nx = scatteredInterpolant(Xs(:), Ys(:), normals(:,1), 'linear');
F_ny = scatteredInterpolant(Xs(:), Ys(:), normals(:,2), 'linear');
F_nz = scatteredInterpolant(Xs(:), Ys(:), normals(:,3), 'linear');

% --- 关键修正：修改匿名函数，使其返回 3 个结果 ---
compute_normal = @(x, y) normalize_vector(F_nx(x, y), F_ny(x, y), F_nz(x, y));
%% ========== 第6步：分类网格（上方/下方）==========
fprintf('\n【步骤5】分类网格（曲面上方/下方）...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

above_surface_mask = zeros(nelx, nely, nelz);
below_surface_mask = zeros(nelx, nely, nelz);

fprintf('扫描所有体素... ');
tic;
for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if valid_grid_mask(i,j,k)
                gc_x = grid_data(i,j,k).x;
                gc_y = grid_data(i,j,k).y;
                gc_z = grid_data(i,j,k).z;
                
                surf_z = compute_surface_z(gc_x, gc_y);
                
                if gc_z > surf_z
                    above_surface_mask(i,j,k) = 1;
                else
                    below_surface_mask(i,j,k) = 1;
                end
            end
        end
    end
end
fprintf('完成 (%.2fs)\n', toc);

num_above = sum(above_surface_mask(:));
num_below = sum(below_surface_mask(:));

fprintf('分类结果:\n');
fprintf('  曲面上方: %d (%.1f%%)\n', num_above, 100*num_above/total_valid_grids);
fprintf('  曲面下方: %d (%.1f%%)\n', num_below, 100*num_below/total_valid_grids);

%% ========== 初始化切片系统 ==========
surface_layers = {};
num_layers = 0;
global_activated = zeros(nelx, nely, nelz);

%% ========== 第7步：处理原始曲面 ==========
fprintf('\n【步骤6】处理原始曲面（offset=0）...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

num_layers = num_layers + 1;
current_threshold = INITIAL_THRESHOLD;

fprintf('  检测激活（阈值=%.3f）...\n', current_threshold);
[activated_grids, X_offset, Y_offset, Z_offset] = detect_activation_refined(...
    0, min_valid_x, max_valid_x, min_valid_y, max_valid_y, SURFACE_RESOLUTION, ...
    compute_surface_z, compute_normal, grid_data, valid_grid_mask, current_threshold);

surface_layers{num_layers} = struct(...
    'offset', 0, ...
    'direction', 'original', ...
    'attempts', struct('threshold', current_threshold, ...
                      'num_activated', length(activated_grids), ...
                      'num_newly_activated', length(activated_grids)), ...
    'final_grids', activated_grids, ...
    'final_threshold', current_threshold, ...
    'total_activated', length(activated_grids), ...
    'total_newly_activated', length(activated_grids), ...
    'X_surf', X_offset, ...
    'Y_surf', Y_offset, ...
    'Z_surf', Z_offset);

% 更新全局激活记录
for g = 1:length(activated_grids)
    idx = activated_grids(g).grid_index;
    global_activated(idx(1), idx(2), idx(3)) = 1;
end

fprintf('  ✓ 原始曲面: 激活 %d 个 (覆盖率: %.1f%%)\n', ...
        length(activated_grids), 100*sum(global_activated(:))/total_valid_grids);

%% ========== 第8步：向上偏移 ==========
fprintf('\n【步骤7】向上偏移（处理上方网格）...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('目标: 激活 %d 个上方网格\n\n', num_above);

current_offset = OFFSET_STEP;
current_threshold = INITIAL_THRESHOLD;
threshold_increase_count = 0;

while current_offset <= MAX_OFFSET
    fprintf('偏移 %+.3f (阈值 %.3f)... ', current_offset, current_threshold);
    
    [activated_grids, X_offset, Y_offset, Z_offset] = detect_activation_refined(...
        current_offset, min_valid_x, max_valid_x, min_valid_y, max_valid_y, SURFACE_RESOLUTION, ...
        compute_surface_z, compute_normal, grid_data, valid_grid_mask, current_threshold);
    
    % 计算新激活的网格
    newly_activated = [];
    for g = 1:length(activated_grids)
        idx = activated_grids(g).grid_index;
        if global_activated(idx(1), idx(2), idx(3)) == 0
            newly_activated = [newly_activated, activated_grids(g)];
            global_activated(idx(1), idx(2), idx(3)) = 1;
        end
    end
    
    num_newly = length(newly_activated);
    above_activated = global_activated & above_surface_mask;
    remaining_above = num_above - sum(above_activated(:));
    
    fprintf('新激活 %d (累计: %.1f%%, 上方剩余: %d)\n', ...
            num_newly, 100*sum(global_activated(:))/total_valid_grids, remaining_above);
    
    % 记录层信息
    if threshold_increase_count == 0
        num_layers = num_layers + 1;
        surface_layers{num_layers} = struct(...
            'offset', current_offset, ...
            'direction', 'up', ...
            'attempts', struct('threshold', current_threshold, ...
                              'num_activated', length(activated_grids), ...
                              'num_newly_activated', num_newly), ...
            'final_grids', activated_grids, ...
            'final_threshold', current_threshold, ...
            'total_activated', length(activated_grids), ...
            'total_newly_activated', num_newly, ...
            'X_surf', X_offset, ...
            'Y_surf', Y_offset, ...
            'Z_surf', Z_offset);
    else
        new_attempt = struct('threshold', current_threshold, ...
                            'num_activated', length(activated_grids), ...
                            'num_newly_activated', num_newly);
        surface_layers{num_layers}.attempts(end+1) = new_attempt;
        surface_layers{num_layers}.final_grids = activated_grids;
        surface_layers{num_layers}.final_threshold = current_threshold;
        surface_layers{num_layers}.total_activated = length(activated_grids);
        surface_layers{num_layers}.total_newly_activated = ...
            surface_layers{num_layers}.total_newly_activated + num_newly;
        surface_layers{num_layers}.X_surf = X_offset;
        surface_layers{num_layers}.Y_surf = Y_offset;
        surface_layers{num_layers}.Z_surf = Z_offset;
    end
    
    % 自适应策略
    if num_newly < NEW_ACTIVATION_THRESHOLD && remaining_above > 0
        current_threshold = current_threshold + THRESHOLD_INCREMENT;
        threshold_increase_count = threshold_increase_count + 1;
        fprintf('  → 新激活太少，增大阈值到 %.3f\n', current_threshold);
    else
        if num_newly >= NEW_ACTIVATION_THRESHOLD || threshold_increase_count == 0
            current_offset = current_offset + OFFSET_STEP;
            current_threshold = INITIAL_THRESHOLD;
            threshold_increase_count = 0;
        end
    end
    
    if remaining_above == 0
        fprintf('✓ 上方所有网格已激活！\n');
        break;
    end
    
    if current_threshold > 20
        fprintf('⚠️  阈值超过20，停止向上偏移\n');
        break;
    end
end

%% ========== 第9步：向下偏移 ==========
fprintf('\n【步骤8】向下偏移（处理下方网格）...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('目标: 激活 %d 个下方网格\n\n', num_below);

current_offset = -OFFSET_STEP;
current_threshold = INITIAL_THRESHOLD;
threshold_increase_count = 0;

while abs(current_offset) <= MAX_OFFSET
    fprintf('偏移 %+.3f (阈值 %.3f)... ', current_offset, current_threshold);
    
    [activated_grids, X_offset, Y_offset, Z_offset] = detect_activation_refined(...
        current_offset, min_valid_x, max_valid_x, min_valid_y, max_valid_y, SURFACE_RESOLUTION, ...
        compute_surface_z, compute_normal, grid_data, valid_grid_mask, current_threshold);
    
    newly_activated = [];
    for g = 1:length(activated_grids)
        idx = activated_grids(g).grid_index;
        if global_activated(idx(1), idx(2), idx(3)) == 0
            newly_activated = [newly_activated, activated_grids(g)];
            global_activated(idx(1), idx(2), idx(3)) = 1;
        end
    end
    
    num_newly = length(newly_activated);
    below_activated = global_activated & below_surface_mask;
    remaining_below = num_below - sum(below_activated(:));
    
    fprintf('新激活 %d (累计: %.1f%%, 下方剩余: %d)\n', ...
            num_newly, 100*sum(global_activated(:))/total_valid_grids, remaining_below);
    
    if threshold_increase_count == 0
        num_layers = num_layers + 1;
        surface_layers{num_layers} = struct(...
            'offset', current_offset, ...
            'direction', 'down', ...
            'attempts', struct('threshold', current_threshold, ...
                              'num_activated', length(activated_grids), ...
                              'num_newly_activated', num_newly), ...
            'final_grids', activated_grids, ...
            'final_threshold', current_threshold, ...
            'total_activated', length(activated_grids), ...
            'total_newly_activated', num_newly, ...
            'X_surf', X_offset, ...
            'Y_surf', Y_offset, ...
            'Z_surf', Z_offset);
    else
        new_attempt = struct('threshold', current_threshold, ...
                            'num_activated', length(activated_grids), ...
                            'num_newly_activated', num_newly);
        surface_layers{num_layers}.attempts(end+1) = new_attempt;
        surface_layers{num_layers}.final_grids = activated_grids;
        surface_layers{num_layers}.final_threshold = current_threshold;
        surface_layers{num_layers}.total_activated = length(activated_grids);
        surface_layers{num_layers}.total_newly_activated = ...
            surface_layers{num_layers}.total_newly_activated + num_newly;
        surface_layers{num_layers}.X_surf = X_offset;
        surface_layers{num_layers}.Y_surf = Y_offset;
        surface_layers{num_layers}.Z_surf = Z_offset;
    end
    
    if num_newly < NEW_ACTIVATION_THRESHOLD && remaining_below > 0
        current_threshold = current_threshold + THRESHOLD_INCREMENT;
        threshold_increase_count = threshold_increase_count + 1;
        fprintf('  → 新激活太少，增大阈值到 %.3f\n', current_threshold);
    else
        if num_newly >= NEW_ACTIVATION_THRESHOLD || threshold_increase_count == 0
            current_offset = current_offset - OFFSET_STEP;
            current_threshold = INITIAL_THRESHOLD;
            threshold_increase_count = 0;
        end
    end
    
    if remaining_below == 0
        fprintf('✓ 下方所有网格已激活！\n');
        break;
    end
    
    if current_threshold > 20
        fprintf('⚠️  阈值超过20，停止向下偏移\n');
        break;
    end
end

%% ========== 第10步：排序和统计 ==========
fprintf('\n【步骤9】生成最终结果...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 按偏移量排序
offsets = zeros(num_layers, 1);
for i = 1:num_layers
    offsets(i) = surface_layers{i}.offset;
end
[~, sort_idx] = sort(offsets, 'ascend');

sorted_layers = {};
for new_id = 1:num_layers
    old_id = sort_idx(new_id);
    sorted_layers{new_id} = surface_layers{old_id};
    sorted_layers{new_id}.layer_id = new_id;
end
surface_layers = sorted_layers;

% 最终统计
final_coverage = sum(global_activated(:));
coverage_rate = 100 * final_coverage / total_valid_grids;

fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('最终统计\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('总层数: %d\n', num_layers);
fprintf('偏移范围: %.3f 到 %.3f\n', surface_layers{1}.offset, surface_layers{end}.offset);
fprintf('总有效网格: %d\n', total_valid_grids);
fprintf('已激活网格: %d\n', final_coverage);
fprintf('覆盖率: %.2f%%\n', coverage_rate);

if coverage_rate >= 99
    fprintf('\n✅ 优秀！接近完美覆盖\n');
elseif coverage_rate >= 95
    fprintf('\n✅ 很好！覆盖率超过95%%\n');
else
    fprintf('\n覆盖率: %.1f%%\n', coverage_rate);
end

if coverage_rate < 100
    fprintf('未覆盖: %d 个 (%.2f%%)\n', total_valid_grids - final_coverage, 100 - coverage_rate);
end

%% ========== 第11步：保存结果 ==========
fprintf('\n【步骤10】保存切片结果...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('slice_results_refined_%s.mat', timestamp);

slice_results = struct();
slice_results.surface_layers = surface_layers;
slice_results.grid_data = grid_data;
slice_results.valid_grid_mask = valid_grid_mask;
slice_results.global_activated = global_activated;
slice_results.above_surface_mask = above_surface_mask;
slice_results.below_surface_mask = below_surface_mask;
slice_results.grid_size = struct('nelx', nelx, 'nely', nely, 'nelz', nelz);
slice_results.statistics = struct(...
    'num_layers', num_layers, ...
    'total_valid_grids', total_valid_grids, ...
    'final_coverage', final_coverage, ...
    'coverage_rate', coverage_rate, ...
    'num_above', num_above, ...
    'num_below', num_below);
slice_results.parameters = struct(...
    'OFFSET_STEP', OFFSET_STEP, ...
    'INITIAL_THRESHOLD', INITIAL_THRESHOLD, ...
    'THRESHOLD_INCREMENT', THRESHOLD_INCREMENT, ...
    'SURFACE_RESOLUTION', SURFACE_RESOLUTION, ...
    'DENSITY_THRESHOLD', DENSITY_THRESHOLD, ...
    'NEW_ACTIVATION_THRESHOLD', NEW_ACTIVATION_THRESHOLD, ...
    'MAX_OFFSET', MAX_OFFSET, ...
    'SCALE_FACTOR', SCALE_FACTOR);
slice_results.surface_params = struct(...
    'a', a, 'b', b, 'c', c, 'd', d, ...
    'e', e, 'f', f, 'g', g, 'h', h, ...
    'X0', X0, 'Y0', Y0, 'Para_me', Para_me);
slice_results.metadata = struct('timestamp', timestamp, 'date', datestr(now));

fprintf('保存到文件... ');
save(filename, 'slice_results', '-v7.3');
copyfile(filename, 'slice_results_refined_latest.mat');

file_info = dir(filename);
fprintf('完成\n');
fprintf('  文件名: %s\n', filename);
fprintf('  文件大小: %.2f MB\n', file_info.bytes / 1024 / 1024);
fprintf('  副本: slice_results_refined_latest.mat\n');

%% ========== 完成 ==========
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║              ✅ 切片完成！                            ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('查看详细结果:\n');
fprintf('  >> visualize_slicing_results\n');
fprintf('\n');

%% ========================================
%% 子函数：激活检测（细化版本）
%% ========================================
function [activated_grids, X_offset, Y_offset, Z_offset] = detect_activation_refined(...
    offset, min_x, max_x, min_y, max_y, resolution, ...
    compute_surface_z, compute_normal, grid_data, valid_grid_mask, grid_threshold)

margin = 5;
xx = (min_x-margin):resolution:(max_x+margin);
yy = (min_y-margin):resolution:(max_y+margin);
[X_grid, Y_grid] = meshgrid(xx, yy);

Z_original = compute_surface_z(X_grid, Y_grid);
[X_offset, Y_offset, Z_offset] = deal(zeros(size(X_grid)));

% 等距偏移
for i = 1:size(X_grid,1)
    for j = 1:size(X_grid,2)
        x_pt = X_grid(i,j);
        y_pt = Y_grid(i,j);
        z_pt = Z_original(i,j);
        
        [nx, ny, nz] = compute_normal(x_pt, y_pt);
        norm_factor = sqrt(nx^2 + ny^2 + nz^2);
        nx = nx / norm_factor;
        ny = ny / norm_factor;
        nz = nz / norm_factor;
        
        X_offset(i,j) = x_pt + offset * nx;
        Y_offset(i,j) = y_pt + offset * ny;
        Z_offset(i,j) = z_pt + offset * nz;
    end
end

% 激活检测
activated_grids = [];
[nelx, nely, nelz] = size(valid_grid_mask);
expanded_margin = abs(offset) + grid_threshold + 1.5;

for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if ~valid_grid_mask(i,j,k)
                continue;
            end
            
            gc_x = grid_data(i,j,k).x;
            gc_y = grid_data(i,j,k).y;
            gc_z = grid_data(i,j,k).z;
            
            x_search_idx = find(xx >= (gc_x - expanded_margin) & ...
                               xx <= (gc_x + expanded_margin));
            y_search_idx = find(yy >= (gc_y - expanded_margin) & ...
                               yy <= (gc_y + expanded_margin));
            
            is_activated = false;
            min_dist = inf;
            
            for yi = y_search_idx
                for xi = x_search_idx
                    if yi > 0 && yi <= size(Z_offset,1) && xi > 0 && xi <= size(Z_offset,2)
                        surf_x = X_offset(yi, xi);
                        surf_y = Y_offset(yi, xi);
                        surf_z = Z_offset(yi, xi);
                        
                        dist = sqrt((gc_x - surf_x)^2 + (gc_y - surf_y)^2 + (gc_z - surf_z)^2);
                        min_dist = min(min_dist, dist);
                        
                        if dist <= grid_threshold
                            is_activated = true;
                        end
                    end
                end
            end
            
            if is_activated
                grid_info = grid_data(i,j,k);
                grid_info.grid_index = [i, j, k];
                grid_info.distance = min_dist;
                activated_grids = [activated_grids, grid_info];
            end
        end
    end
end

end
function [nx, ny, nz] = normalize_vector(dx, dy, dz)
    len = sqrt(dx.^2 + dy.^2 + dz.^2);
    nx = dx ./ len;
    ny = dy ./ len;
    nz = dz ./ len;
end