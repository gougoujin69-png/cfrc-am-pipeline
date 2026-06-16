%% Full Layer Path Assembly Script - Advanced Spiral Merging
% Logic:
%   1. Group loops into basic spirals (inner to outer).
%   2. Cluster adjacent spirals using Graph Theory (Connected Components).
%   3. For each cluster, find nearest points to redefine entry/exit points and merge.
%   4. Isolated spirals remain untouched.

clear; clc; close all;

%% ==================== 1. Parameters ====================
ADJACENT_THRESHOLD = 5.0;           % mm (同一螺旋内 loop 的间距)
SPIRAL_MERGE_THRESHOLD = 5.0;      % mm (螺旋与螺旋之间触发融合的距离)
input_file = 'Modified_printing_path.mat';
output_file = 'Modified_printing_path.mat';

%% ==================== 2. Load Data ====================
fprintf('Starting advanced path merging...\n');
if ~exist(input_file, 'file')
    error('Input file not found: %s', input_file);
end
load(input_file, 'all_layers_data');
num_layers = length(all_layers_data);

%% ==================== 3. Process All Layers ====================
for layer = 1:num_layers
    modified_paths = all_layers_data(layer).modified_paths;
    if isempty(modified_paths), continue; end
    
    % --- Step A: Original Loop to Spiral Assembly ---
    loops = modified_paths; % 假设已经是 cell array
    num_loops = length(loops);
    
    % 计算相邻 loop 间距并分组
    adj_distances = zeros(max(1, num_loops-1), 1);
    for i = 1:num_loops-1
        adj_distances(i) = min_endpoint_distance(loops{i}, loops{i+1});
    end
    
    spiral_groups = {};
    current_group = [1];
    for i = 1:num_loops-1
        if adj_distances(i) <= ADJACENT_THRESHOLD
            current_group = [current_group, i+1];
        else
            spiral_groups{end+1} = current_group;
            current_group = [i+1];
        end
    end
    spiral_groups{end+1} = current_group;
    
    % 初步组装单个螺旋
    assembled_spirals = {};
    for s = 1:length(spiral_groups)
        group = spiral_groups{s};
        group_rev = fliplr(group); % 内向外
        path = [];
        for idx = group_rev
            if isempty(path), path = loops{idx};
            else, path = [path; loops{idx}]; end % 简单连接，后续会细化
        end
        assembled_spirals{end+1} = path;
    end
    
    % --- Step B: Spiral Clustering (Graph Theory) ---
    num_spirals = length(assembled_spirals);
    if num_spirals > 1
        % 计算螺旋间的最小距离矩阵
        dist_matrix = inf(num_spirals, num_spirals);
        for i = 1:num_spirals
            for j = i+1:num_spirals
                d = find_nearest_points_distance(assembled_spirals{i}, assembled_spirals{j});
                dist_matrix(i,j) = d; dist_matrix(j,i) = d;
            end
        end
        
        % 构建邻接矩阵并寻找连通分量
        adj_matrix = dist_matrix <= SPIRAL_MERGE_THRESHOLD;
        G = graph(adj_matrix);
        bins = conncomp(G);
        
        final_output_paths = {};
        for cluster_id = 1:max(bins)
            member_idx = find(bins == cluster_id);
            if length(member_idx) == 1
                final_output_paths{end+1} = assembled_spirals{member_idx};
            else
                % 融合该簇内的所有螺旋
                cluster_paths = assembled_spirals(member_idx);
                final_output_paths{end+1} = merge_path_cluster(cluster_paths);
            end
        end
    else
        final_output_paths = assembled_spirals;
    end
    
    all_layers_data(layer).connected_paths = final_output_paths;
    fprintf('Layer %d processed: %d clusters formed.\n', layer, length(final_output_paths));
end

%% ==================== 4. Save & Visualize ====================
save(output_file, 'all_layers_data', '-v7.3');
fprintf('Success! Saved to %s\n', output_file);

% 简单可视化
figure; hold on; axis equal; grid on;
if ~isempty(all_layers_data(1).connected_paths)
    sample_paths = all_layers_data(1).connected_paths;
    colors = lines(length(sample_paths));
    for i = 1:length(sample_paths)
        p = sample_paths{i};
        plot(p(:,1), p(:,2), 'Color', colors(i,:), 'LineWidth', 1.5);
        plot(p(1,1), p(1,2), 'go', 'MarkerFaceColor', 'g'); % 起点
        plot(p(end,1), p(end,2), 'rs', 'MarkerFaceColor', 'r'); % 终点
    end
    title('Merged Spiral Paths (Layer 1)');
end

%% ==================== Helper Functions ====================

function d = min_endpoint_distance(l1, l2)
    pts1 = [l1(1,:); l1(end,:)]; pts2 = [l2(1,:); l2(end,:)];
    d = min(min(pdist2(pts1(:,1:2), pts2(:,1:2))));
end

function d = find_nearest_points_distance(p1, p2)
    % 采样以提高计算速度
    step1 = max(1, floor(size(p1,1)/50));
    step2 = max(1, floor(size(p2,1)/50));
    D = pdist2(p1(1:step1:end, 1:2), p2(1:step2:end, 1:2));
    d = min(D(:));
end

function [idx1, idx2] = find_exact_nearest_indices(p1, p2)
    D = pdist2(p1(:,1:2), p2(:,1:2));
    [min_val, row_idx] = min(D, [], 1);
    [~, idx2] = min(min_val);
    idx1 = row_idx(idx2);
end

function merged = merge_path_cluster(paths)
    num_p = length(paths);
    used = false(num_p, 1);
    merged = paths{1};
    used(1) = true;
    
    for k = 1:num_p-1
        min_d = inf;
        next_idx = -1;
        for j = 1:num_p
            if ~used(j)
                [idx_m, idx_j] = find_exact_nearest_indices(merged, paths{j});
                d = norm(merged(idx_m, 1:2) - paths{j}(idx_j, 1:2));
                if d < min_d
                    min_d = d; next_idx = j;
                    best_m = idx_m; best_j = idx_j;
                end
            end
        end
        if next_idx ~= -1
            % 在最近点切开并重新组装
            pathB = paths{next_idx};
            % 重新排列 pathB，使其从 best_j 开始绕一圈
            reordered_B = [pathB(best_j:end, :); pathB(1:best_j, :)];
            % 插入到 merged 的 best_m 位置
            merged = [merged(1:best_m, :); reordered_B; merged(best_m:end, :)];
            used(next_idx) = true;
        end
    end
end