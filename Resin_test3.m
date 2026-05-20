%%%% 此代码用于调试生成树脂代码 - 并行计算优化版
%%%% 修复：解决了 Java Negative capacity 内存溢出报错
%%%% 修复：解决了 parpool 启动时的配置只读问题

%% ========== 1. 数据加载 ==========
load('test.mat');
load('slice_results_refined_latest.mat');
load('all_layers_path_results_v3.mat');

% 提取核心数据
all_layers_data = results.all_layers_data;
surface_layers = slice_results.surface_layers;
num_layers = slice_results.statistics.num_layers;
total_layers = length(all_layers_data);

% 算法参数
offset_distance = 1;
max_iter = 1;
min_length = 1;

%% ========== 第2步：初始化并行池 (修正版) ==========
fprintf('\n【步骤2】初始化并行计算...\n');

% 1. 检查当前是否已有并行池
currentPool = gcp('nocreate');

% 2. 如果已有池，必须先删除（因为我们要修改启动配置）
if ~isempty(currentPool)
    fprintf('  正在关闭旧的并行池以应用新配置...\n');
    delete(currentPool);
end

% 3. 启动新池，并在启动时直接禁用自动路径扫描
% 'AutoAddClientPath', false 是防止 Java 内存溢出的关键
fprintf('  正在启动新并行池 (禁用自动路径依赖)...\n');
try
    pool = parpool('local', 'AutoAddClientPath', false);
catch
    % 如果您的 MATLAB 版本较老不支持此参数，尝试普通启动
    warning('当前版本不支持 AutoAddClientPath 参数，尝试普通启动...');
    pool = parpool('local');
end

% 4. 手动添加路径 (因为禁用了自动扫描，必须手动告诉 Worker 代码在哪里)
current_folder = fileparts(mfilename('fullpath'));
if isempty(current_folder), current_folder = pwd; end

fprintf('  手动分发代码路径到各核心...\n');
% 将当前文件夹添加到所有 Worker 的搜索路径
parfevalOnAll(pool, @addpath, 0, current_folder);

% 如果您的代码依赖了子文件夹（例如 slice_results 里的数据），建议把子文件夹也加上
addpath(genpath(current_folder)); 
parfevalOnAll(pool, @addpath, 0, genpath(current_folder));

fprintf('  并行池准备就绪，工作进程数: %d\n', pool.NumWorkers);

%% ========== 3. 关键：数据切片化 (避免 Java 报错) ==========
% 将结构体中的大字段抽离成平铺的 Cell 数组
% 这一步是解决问题的关键：让 MATLAB 只按需传输数据
S_layers_cell = surface_layers; 
O_carbon_cell = {all_layers_data.outer_contours};
P_cloud_cell  = {all_layers_data.pointCloud_data};

% 预分配结果存储（parfor 只能赋值给切片变量）
paths_2dr_results = cell(total_layers, 1);
paths_3dr_results = cell(total_layers, 1);

% 清除原始大数据对象释放主线程内存，防止总内存超限
clear results slice_results; 

fprintf('开始并行处理 %d 层 (采用切片数据传输模式)...\n', total_layers);
tic;

%% ========== 4. 并行处理每一层 ==========
parfor layer_num = 1:total_layers
    % --- 从切片变量引用数据，MATLAB 只会把第 layer_num 个数据发给当前核心 ---
    layer_data = S_layers_cell{layer_num};
    outer_carbon_raw = O_carbon_cell{layer_num};
    pointCloud_data = P_cloud_cell{layer_num};
    
    % 曲面数据处理
    X_surf = layer_data.X_surf;
    Y_surf = layer_data.Y_surf;
    Z_surf = layer_data.Z_surf;
    
    % 创建三维插值器
    X_flat = X_surf(:); Y_flat = Y_surf(:); Z_flat = Z_surf(:);
    valid_pts = all(isfinite([X_flat, Y_flat, Z_flat]), 2);
    F_z = [];
    if sum(valid_pts) >= 3
        F_z = scatteredInterpolant(X_flat(valid_pts), Y_flat(valid_pts), ...
            Z_flat(valid_pts), 'linear', 'nearest');
    end
    
    % 处理外轮廓格式
    outer_carbon_new = cell(1, length(outer_carbon_raw));
    for i = 1:length(outer_carbon_raw)
        outer_carbon_new{i} = {outer_carbon_raw{i}};
    end
    
    % 生成内部路径 
    % 注意：请确保 outer_contours 在 workspace (test.mat) 中存在，否则这里会报错
    % 如果 outer_contours 是随层变化的，请确保它是被切片的变量
    all_rings_results = generate_offset_path2(outer_contours, outer_carbon_new, ...
        offset_distance, max_iter, min_length, pointCloud_data);
    
    % 转换路径数据并插值
    local_paths_2dr = {};
    local_paths_3dr = {};
    pc_X = pointCloud_data.X(:); pc_Y = pointCloud_data.Y(:); pc_Z = pointCloud_data.Z(:);
    
    for j = 1:length(all_rings_results)
        rings_iter = all_rings_results{j};
        if isempty(rings_iter), continue; end
        for kk = 1:length(rings_iter)
            pts_2d = rings_iter{kk};
            if isempty(pts_2d) || size(pts_2d, 1) < 2, continue; end
            
            % 保存 2D
            local_paths_2dr{end+1} = pts_2d;
            
            % 转换为 3D
            num_pts = size(pts_2d, 1);
            pts_3d = zeros(num_pts, 3);
            for p = 1:num_pts
                x = pts_2d(p, 2); y = pts_2d(p, 1);
                z = NaN;
                if ~isempty(F_z), z = F_z(x, y); end
                if isnan(z) || ~isfinite(z)
                    dist = (pc_X - x).^2 + (pc_Y - y).^2;
                    [~, min_idx] = min(dist);
                    z = pc_Z(min_idx);
                end
                pts_3d(p, :) = [x, y, z];
            end
            local_paths_3dr{end+1} = pts_3d;
        end
    end
    
    % 将当前核心的计算结果存入输出容器
    paths_2dr_results{layer_num} = local_paths_2dr;
    paths_3dr_results{layer_num} = local_paths_3dr;
end

%% ========== 5. 结果合并与保存 ==========
% 将计算结果回填至结构体
% 注意：因为之前 clear 了 results，这里需要确保 all_layers_data 还在
% 这里的 all_layers_data 是之前提取出来的结构体数组
for layer_num = 1:total_layers
    all_layers_data(layer_num).paths_2dr = paths_2dr_results{layer_num};
    all_layers_data(layer_num).paths_3dr = paths_3dr_results{layer_num};
end

elapsed_time = toc;
fprintf('处理完成，总耗时: %.2f 秒\n', elapsed_time);
save('all_layers_path_carbon_resin_fixed.mat', 'all_layers_data', '-v7.3');

fprintf('完整结果已保存。\n');