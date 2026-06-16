%% ========== 路径重采样与点云对齐处理 (修正版) ==========
%%%该代码获得了过滤干净的连续纤维以及树脂路径
load('Modified_printing_path.mat');

target_spacing = 1.2; 
num_layers = length(all_layers_data);

fprintf('开始处理，总层数: %d\n', num_layers);

for layer_idx = 1:num_layers
    fprintf('正在处理第 %d 层...\n', layer_idx);
    pc = all_layers_data(layer_idx).pointCloud_data;
    
    path_fields = { 'connected_paths'};
    
    for f = 1:length(path_fields)
        current_field = path_fields{f};
        current_paths = all_layers_data(layer_idx).(current_field);
        processed_paths = cell(size(current_paths));
        
        for i = 1:length(current_paths)
            original_pts = current_paths{i};
            
            % --- 1. 数据清洗：去除重复点或距离过近的点 ---
            if isempty(original_pts) || size(original_pts, 1) < 2
                processed_paths{i} = original_pts;
                continue;
            end
            
            % 计算点与点之间的距离
            dists = sqrt(sum(diff(original_pts(:, 1:3)).^2, 2));
            keep_idx = [true; dists > 1e-6]; 
            pts_cleaned = original_pts(keep_idx, :);
            
            if size(pts_cleaned, 1) < 2
                processed_paths{i} = pts_cleaned;
                continue;
            end

            % --- 2. 路径重采样准备 ---
            diffs = diff(pts_cleaned(:, 1:3));
            dist_segments = sqrt(sum(diffs.^2, 2));
            cum_dist = [0; cumsum(dist_segments)];
            total_dist = cum_dist(end);
            
            [cum_dist_unique, unique_map] = unique(cum_dist);
            pts_for_interp = pts_cleaned(unique_map, :);
            
            num_new_pts = max(2, round(total_dist / target_spacing));
            new_query_dist = linspace(0, total_dist, num_new_pts);
            
            % X 和 Y 始终执行线性插值
            new_x = interp1(cum_dist_unique, pts_for_interp(:, 1), new_query_dist, 'linear')';
            new_y = interp1(cum_dist_unique, pts_for_interp(:, 2), new_query_dist, 'linear')';
            
            % --- 3. Z轴处理 (区分路径类型) ---
            if strcmp(current_field, 'connected_paths')
                % 情况 A: 只有 connected_paths 执行点云映射 (Z轴贴合)
                try
                    % 优先尝试 interp2 (针对网格点云)
                    new_z = interp2(pc.X, pc.Y, pc.Z, new_x, new_y, 'linear');
                    
                    % 如果有 NaN，使用最近邻补充
                    if any(isnan(new_z))
                        F_fallback = scatteredInterpolant(pc.X(:), pc.Y(:), pc.Z(:), 'nearest', 'nearest');
                        nan_mask = isnan(new_z);
                        new_z(nan_mask) = F_fallback(new_x(nan_mask), new_y(nan_mask));
                    end
                catch
                    % 如果点云不是标准网格，使用散点插值
                    F = scatteredInterpolant(pc.X(:), pc.Y(:), pc.Z(:), 'linear', 'nearest');
                    new_z = F(new_x, new_y);
                end
            else
                % 情况 B: resin_connected_paths 只对原始 Z 进行插值，不贴合点云
                new_z = interp1(cum_dist_unique, pts_for_interp(:, 3), new_query_dist, 'linear')';
            end
            
            processed_paths{i} = [new_x, new_y, new_z];
        end
        all_layers_data(layer_idx).(current_field) = processed_paths;
    end
end

fprintf('全部处理完成！\n');

%% ========== 可视化验证 (以第10层为例) ==========
testnum = 10;
if testnum <= length(all_layers_data)
    figure('Name', '处理后效果验证');
    hold on;

    % 绘制点云底图
    pc_test = all_layers_data(testnum).pointCloud_data;
    surf(pc_test.X, pc_test.Y, pc_test.Z, 'EdgeColor','none', 'FaceAlpha',0.3);

    % 绘制处理后的树脂路径 (蓝色) - 此时它应保持原始层高度
    res_paths = all_layers_data(testnum).resin_connected_paths;
    for i = 1:length(res_paths)
        p = res_paths{i};
        plot3(p(:,1), p(:,2), p(:,3), 'b-', 'LineWidth', 1);
    end

    % 绘制处理后的碳纤维路径 (红色点) - 此时它应紧贴点云表面
    con_paths = all_layers_data(testnum).connected_paths;
    for i = 1:length(con_paths)
        p = con_paths{i};
        plot3(p(:,1), p(:,2), p(:,3), 'r-o', 'MarkerSize', 2, 'LineWidth', 1);
    end

    title(['第 ' num2str(testnum) ' 层路径处理结果 (红线贴合点云，蓝线保持原始高度)']);
    axis equal; grid on; view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
end
