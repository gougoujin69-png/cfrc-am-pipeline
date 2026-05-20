%% ========================================
%% 单层路径生成 - 完整版本 V3
%% 使用正确的 N 值判断内外轮廓
%% ========================================
% 功能：
%   1. 正确识别内外轮廓（使用N值判断）
%   2. 生成流线并过滤延长
%   3. 区域分割
%   4. 生成偏置路径
%   5. 投影到三维曲面
%   6. 完整可视化
%%

clear; clc; close all;

warning('off', 'MATLAB:polyshape:repairedBySimplify');
warning('off', 'MATLAB:polyshape:boundary3Points');

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║    单层路径生成 V3 - 完整功能版本                              ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% ========== 参数设置 ==========
params = struct();
params.offset_distance = 0.3;          % 偏移距离
params.max_iterations = 30;            % 最大偏移迭代
params.min_path_length = 2;            % 最小路径长度
params.volfrac = 0.5;                  % 体积分数
params.filter_radius = 5;              % 方向场滤波半径
params.filter_iterations = 2;          % 方向场滤波迭代次数
params.min_region_area = 0.1;          % 最小区域面积
% params.min_contour_length_outer = 8;  % 外轮廓最小长度
params.min_contour_length_inner = 4;   % 内轮廓最小长度

% 选择要处理的层（可以修改）
layer_idx = 2;

%% ========== 第1步：加载数据 ==========
fprintf('【步骤1】加载数据...\n');

if ~exist('slice_results_refined_latest.mat', 'file')
    error('找不到 slice_results_refined_latest.mat');
end

load('slice_results_refined_latest.mat');
surface_layers = slice_results.surface_layers;
grid_data = slice_results.grid_data;
num_layers = slice_results.statistics.num_layers;

fprintf('  总层数: %d\n', num_layers);
fprintf('  测试层: %d\n', layer_idx);

layer_data = surface_layers{layer_idx};
activated_grids = layer_data.final_grids;
X_surf = layer_data.X_surf;
Y_surf = layer_data.Y_surf;
Z_surf = layer_data.Z_surf;

fprintf('  激活网格数: %d\n', length(activated_grids));

%% ========== 第2步：创建点云数据和三维插值器 ==========
fprintf('\n【步骤2】创建点云数据和三维插值器...\n');

pointCloud_data = create_pointcloud_from_surface(X_surf, Y_surf, Z_surf);

% 创建三维插值器
X_flat = X_surf(:);
Y_flat = Y_surf(:);
Z_flat = Z_surf(:);
valid_pts = all(isfinite([X_flat, Y_flat, Z_flat]), 2);

if sum(valid_pts) >= 3
    F_z = scatteredInterpolant(X_flat(valid_pts), Y_flat(valid_pts), ...
        Z_flat(valid_pts), 'linear', 'nearest');
    fprintf('  三维插值器创建成功，有效点数: %d\n', sum(valid_pts));
else
    F_z = [];
    fprintf('  警告: 有效点数不足，无法创建三维插值器\n');
end

%% ========== 第3步：提取二维数据 ==========
fprintf('\n【步骤3】提取二维数据...\n');

[xold, t, nelx, nely, ~] = extract_layer_2d_projection(activated_grids, grid_data);

fprintf('  网格尺寸: nelx=%d, nely=%d\n', nelx, nely);
fprintf('  密度场非零元素: %d\n', nnz(xold));

%% ========== 第4步：方向场滤波 ==========
fprintf('\n【步骤4】方向场滤波...\n');

t_filtered = filter_orientation_simple(t, xold, params.filter_radius, params.filter_iterations);

fprintf('  滤波参数: 半径=%d, 迭代=%d\n', params.filter_radius, params.filter_iterations);

%% ========== 第5步：创建二值图像并提取轮廓（使用N值判断） ==========
fprintf('\n【步骤5】提取内外轮廓（使用N值判断）...\n');

% 创建二值图像
x_filter = zeros(nely, nelx);
th = prctile(xold(:), (1 - params.volfrac) * 100);
th = th * 1.2;
x2_num = find(xold > th);
x_filter(x2_num) = 1;

fprintf('  阈值: %.4f\n', th);
fprintf('  材料像素数: %d\n', nnz(x_filter));

% 使用 bwboundaries 提取边界
[B, L, N, A] = bwboundaries(x_filter, 'holes');

fprintf('  总边界数: %d\n', length(B));
fprintf('  对象数 N: %d\n', N);

% *** 关键：使用 N 值正确区分内外轮廓 ***
outer_boundaries_raw = {};
inner_boundaries_raw = {};

for k = 1:length(B)
    if k <= N
        outer_boundaries_raw{end+1} = B{k};
    else
        inner_boundaries_raw{end+1} = B{k};
    end
end

fprintf('  外轮廓数（材料边界）: %d\n', length(outer_boundaries_raw));
fprintf('  内轮廓数（孔洞边界）: %d\n', length(inner_boundaries_raw));

%% ========== 第6步：坐标转换 ==========
fprintf('\n【步骤6】坐标转换（网格索引 -> 实际坐标）...\n');

num_grids = length(activated_grids);
actual_x = zeros(num_grids, 1);
actual_y = zeros(num_grids, 1);

for i = 1:num_grids
    g = activated_grids(i);
    actual_x(i) = g.x;
    actual_y(i) = g.y;
end

actual_x_min = min(actual_x);
actual_x_max = max(actual_x);
actual_y_min = min(actual_y);
actual_y_max = max(actual_y);
actual_range = max(actual_x_max - actual_x_min, actual_y_max - actual_y_min);
adaptive_min_outer = max(actual_range * 0.3, 3);  % 最小3，或范围的30%

fprintf('  自适应外轮廓最小长度: %.1f\n', adaptive_min_outer);
% 网格索引到实际坐标的缩放
scale_x = (actual_x_max - actual_x_min) / max(nelx - 1, 1);
scale_y = (actual_y_max - actual_y_min) / max(nely - 1, 1);

fprintf('  实际坐标范围: X=[%.1f, %.1f], Y=[%.1f, %.1f]\n', ...
    actual_x_min, actual_x_max, actual_y_min, actual_y_max);

% 转换外轮廓
outer_contours = {};
for k = 1:length(outer_boundaries_raw)
    boundary = outer_boundaries_raw{k};
    if isempty(boundary) || size(boundary, 1) < 3
        continue;
    end
    
    % 减少点数
    if size(boundary, 1) > 200
        step = ceil(size(boundary, 1) / 200);
        boundary = boundary(1:step:end, :);
    end
    
    % 确保闭合
    if ~isequal(boundary(1,:), boundary(end,:))
        boundary(end+1,:) = boundary(1,:);
    end
    
    % 平滑
    if size(boundary, 1) > 6
        boundary(:,1) = smooth(boundary(:,1), 3);
        boundary(:,2) = smooth(boundary(:,2), 3);
    end
    
    % 坐标转换：bwboundaries 返回 [row, col]
    x_actual = actual_x_min + (boundary(:, 2) - 1) * scale_x;
    y_actual = actual_y_min + (boundary(:, 1) - 1) * scale_y;
    
    contour_pts = [x_actual(:), y_actual(:)];
    contour_length = sum(sqrt(sum(diff(contour_pts).^2, 2)));
    
    if contour_length >= adaptive_min_outer
        outer_contours{end+1} = contour_pts;
    end
end

% 转换内轮廓
inner_contours = {};
for k = 1:length(inner_boundaries_raw)
    boundary = inner_boundaries_raw{k};
    if isempty(boundary) || size(boundary, 1) < 3
        continue;
    end
    
    if size(boundary, 1) > 200
        step = ceil(size(boundary, 1) / 200);
        boundary = boundary(1:step:end, :);
    end
    
    if ~isequal(boundary(1,:), boundary(end,:))
        boundary(end+1,:) = boundary(1,:);
    end
    
    if size(boundary, 1) > 6
        boundary(:,1) = smooth(boundary(:,1), 3);
        boundary(:,2) = smooth(boundary(:,2), 3);
    end
    
    x_actual = actual_x_min + (boundary(:, 2) - 1) * scale_x;
    y_actual = actual_y_min + (boundary(:, 1) - 1) * scale_y;
    
    contour_pts = [x_actual(:), y_actual(:)];
    contour_length = sum(sqrt(sum(diff(contour_pts).^2, 2)));
    
    if contour_length >= params.min_contour_length_inner
        inner_contours{end+1} = contour_pts;
    end
end

fprintf('  转换后外轮廓数: %d\n', length(outer_contours));
fprintf('  转换后内轮廓数: %d\n', length(inner_contours));

%% ========== 第7步：计算中轴线 ==========
fprintf('\n【步骤7】计算中轴线...\n');

skeleton = bwmorph(x_filter, 'skel', Inf);
skeleton_clean = bwmorph(skeleton, 'spur', 3);
[y_skel, x_skel] = find(skeleton_clean);

if ~isempty(x_skel)
    x_skel_actual = actual_x_min + (x_skel - 1) * scale_x;
    y_skel_actual = actual_y_min + (y_skel - 1) * scale_y;
    medial_axis = [x_skel_actual(:), y_skel_actual(:)];
    fprintf('  中轴线点数: %d\n', size(medial_axis, 1));
else
    medial_axis = [];
    fprintf('  中轴线点数: 0\n');
end

%% ========== 第8步：生成流线 ==========
fprintf('\n【步骤8】生成流线（plotTopologyWithMedialAxis）...\n');

[~, ~, streamlines_raw, ~, ~, ~] = ...
    plotTopologyWithMedialAxis(xold, t_filtered, nelx, nely, params.volfrac);

fprintf('  原始流线数: %d\n', length(streamlines_raw));

% 转换流线坐标
streamlines = {};
for k = 1:length(streamlines_raw)
    sl = streamlines_raw{k};
    if isempty(sl) || size(sl, 1) < 3
        continue;
    end
    x_actual = actual_x_min + (sl(:, 1) - 1) * scale_x;
    y_actual = actual_y_min + (sl(:, 2) - 1) * scale_y;
    streamlines{end+1} = [x_actual(:), y_actual(:)];
end

fprintf('  转换后流线数: %d\n', length(streamlines));

%% ========== 第9步：流线过滤和延长 ==========
fprintf('\n【步骤9】流线过滤和延长...\n');

if ~isempty(streamlines) && ~isempty(outer_contours)
    [filtered_streamlines, ~] = filterStreamlinesInsideContours(streamlines, outer_contours, inner_contours);
    filtered_streamlines = filtered_streamlines(~cellfun(@isempty, filtered_streamlines));
    fprintf('  过滤后流线数: %d\n', length(filtered_streamlines));
    
    if ~isempty(filtered_streamlines)
        extended_streamlines = extendStreamlinesToContour(filtered_streamlines, outer_contours, inner_contours);
        extended_streamlines = extended_streamlines(~cellfun(@isempty, extended_streamlines));
        fprintf('  延长后流线数: %d\n', length(extended_streamlines));
    else
        extended_streamlines = {};
    end
else
    extended_streamlines = {};
    fprintf('  流线数: 0\n');
end

%% ========== 第10步：可视化轮廓和流线 ==========
fprintf('\n【步骤10】可视化轮廓和流线...\n');

figure('Name', '轮廓和流线', 'Position', [50, 50, 1600, 600]);

% 子图1：密度场
subplot(1, 3, 1);
imagesc(xold');
axis xy equal tight;
colormap(gca, gray);
colorbar;
title('密度场', 'FontSize', 12);
xlabel('X'); ylabel('Y');

% 子图2：内外轮廓（填充显示）
subplot(1, 3, 2);
hold on;

% 填充外轮廓（灰色=材料）
for i = 1:length(outer_contours)
    fill(outer_contours{i}(:,1), outer_contours{i}(:,2), ...
        [0.85, 0.85, 0.85], 'EdgeColor', 'b', 'LineWidth', 2);
end

% 填充内轮廓（白色=孔洞）
for i = 1:length(inner_contours)
    fill(inner_contours{i}(:,1), inner_contours{i}(:,2), ...
        [1, 1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
end

axis equal tight;
title(sprintf('内外轮廓: 外%d(蓝) 内%d(红)', length(outer_contours), length(inner_contours)), 'FontSize', 12);
xlabel('X'); ylabel('Y');
hold off;

% 子图3：轮廓+流线
subplot(1, 3, 3);
hold on;

% 绘制外轮廓
for i = 1:length(outer_contours)
    plot(outer_contours{i}(:,1), outer_contours{i}(:,2), 'b-', 'LineWidth', 2);
end

% 绘制内轮廓
for i = 1:length(inner_contours)
    plot(inner_contours{i}(:,1), inner_contours{i}(:,2), 'r-', 'LineWidth', 2);
end

% 绘制流线
for i = 1:length(extended_streamlines)
    sl = extended_streamlines{i};
    if ~isempty(sl)
        plot(sl(:,1), sl(:,2), 'g-', 'LineWidth', 1.5);
    end
end

% 绘制中轴线
if ~isempty(medial_axis)
    plot(medial_axis(:,1), medial_axis(:,2), 'm.', 'MarkerSize', 2);
end

axis equal tight;
title(sprintf('轮廓+流线: 流线%d条', length(extended_streamlines)), 'FontSize', 12);
xlabel('X'); ylabel('Y');
legend({'外轮廓', '内轮廓', '流线', '中轴线'}, 'Location', 'best');
hold off;

sgtitle(sprintf('层 %d 轮廓和流线', layer_idx), 'FontSize', 14);
saveas(gcf, sprintf('layer%d_contours_streamlines.png', layer_idx));
fprintf('  轮廓流线图已保存\n');

%% ========== 第11步：区域分割 ==========
fprintf('\n【步骤11】区域分割...\n');

all_regions = {};

for contour_idx = 1:length(outer_contours)
    fprintf('  处理外轮廓 %d/%d...\n', contour_idx, length(outer_contours));
    
    current_outer = outer_contours(contour_idx);
    
    % 找到属于该外轮廓的内轮廓
    current_inner = {};
    try
        main_poly_temp = polyshape(current_outer{1}(:,1), current_outer{1}(:,2));
        
        for h = 1:length(inner_contours)
            if isempty(inner_contours{h})
                continue;
            end
            inner_center = mean(inner_contours{h}, 1);
            if isinterior(main_poly_temp, inner_center(1), inner_center(2))
                current_inner{end+1} = inner_contours{h};
            end
        end
    catch
        main_poly_temp = polyshape();
    end
    
    fprintf('    该轮廓内轮廓数: %d\n', length(current_inner));
    
    % 找到属于该外轮廓的流线
    current_streamlines = {};
    for s = 1:length(extended_streamlines)
        sl = extended_streamlines{s};
        if isempty(sl), continue; end
        
        mid_pt = sl(round(size(sl,1)/2), :);
        
        in_outer = inpolygon(mid_pt(1), mid_pt(2), current_outer{1}(:,1), current_outer{1}(:,2));
        in_inner = false;
        for h = 1:length(current_inner)
            if inpolygon(mid_pt(1), mid_pt(2), current_inner{h}(:,1), current_inner{h}(:,2))
                in_inner = true;
                break;
            end
        end
        
        if in_outer && ~in_inner
            current_streamlines{end+1} = sl;
        end
    end
    
    fprintf('    该轮廓流线数: %d\n', length(current_streamlines));
    
    % 区域分割
    try
        if ~isempty(current_streamlines)
            [~, regions_poly] = split_region_points_improved(...
                current_outer, current_inner, current_streamlines, ...
                'point_tol', 1e-6, 'dist_tol', 1e-6, 'area_tol', 1e-12, 'debug', false);
            fprintf('    分割区域数: %d\n', length(regions_poly));
        else
            % 没有流线，整个轮廓作为一个区域
            regions_poly = main_poly_temp;
            for h = 1:length(current_inner)
                hole = polyshape(current_inner{h}(:,1), current_inner{h}(:,2));
                regions_poly = subtract(regions_poly, hole);
            end
            fprintf('    无流线，保留整个区域\n');
        end
    catch ME
        fprintf('    区域分割失败: %s\n', ME.message);
        regions_poly = polyshape.empty;
    end
    
    % 处理每个区域
    for i = 1:length(regions_poly)
        try
            if length(regions_poly) == 1
                temp = regions_poly;
            else
                temp = regions_poly(i);
            end
            
            % 减去内轮廓
            for k = 1:length(current_inner)
                poly = polyshape(current_inner{k}(:,1), current_inner{k}(:,2));
                temp = subtract(temp, poly);
            end
            
            if area(temp) > params.min_region_area
                all_regions{end+1} = temp;
            end
        catch
        end
    end
end

fprintf('  总有效区域数: %d\n', length(all_regions));

%% ========== 第12步：可视化分割区域 ==========
fprintf('\n【步骤12】可视化分割区域...\n');

figure('Name', '区域分割结果', 'Position', [100, 100, 800, 600]);
hold on;

colors = jet(max(length(all_regions), 1));
for i = 1:length(all_regions)
    plot(all_regions{i}, 'FaceColor', colors(i,:), 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 1);
end

% 绘制流线
for i = 1:length(extended_streamlines)
    sl = extended_streamlines{i};
    if ~isempty(sl)
        plot(sl(:,1), sl(:,2), 'w-', 'LineWidth', 2);
    end
end

axis equal tight;
title(sprintf('层 %d 区域分割结果: %d 个区域', layer_idx, length(all_regions)), 'FontSize', 12);
xlabel('X'); ylabel('Y');
hold off;

saveas(gcf, sprintf('layer%d_regions.png', layer_idx));
fprintf('  区域分割图已保存\n');

%% ========== 第13步：生成偏置路径 ==========
fprintf('\n【步骤13】生成偏置路径...\n');

all_paths_2d = {};
all_paths_3d = {};

for i = 1:length(all_regions)
    region_result = all_regions{i};
    
    if area(region_result) < 0.5
        continue;
    end
    
    try
        [outer_cell, inner_cell] = polyshape_to_cell(region_result);
        
        if isempty(outer_cell)
            continue;
        end
        
        rings_results = generate_offset_path2(outer_cell, inner_cell, ...
            params.offset_distance, params.max_iterations, params.min_path_length, pointCloud_data);
        
        path_count = 0;
        for j = 1:length(rings_results)
            rings_iter = rings_results{j};
            if isempty(rings_iter), continue; end
            
            for k = 1:length(rings_iter)
                pts_2d = rings_iter{k};
                if isempty(pts_2d) || size(pts_2d, 1) < 2
                    continue;
                end
                
                all_paths_2d{end+1} = pts_2d;
                
                % 转换为三维路径
                pts_3d = zeros(size(pts_2d, 1), 3);
                for p = 1:size(pts_2d, 1)
                    x = pts_2d(p, 2);
                    y = pts_2d(p, 1);
                    
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
                    
                    pts_3d(p, :) = [x, y, z];
                end
                
                all_paths_3d{end+1} = pts_3d;
                path_count = path_count + 1;
            end
        end
        
        fprintf('  区域 %d: 生成 %d 条路径\n', i, path_count);
    catch ME
        fprintf('  区域 %d: 路径生成失败 - %s\n', i, ME.message);
    end
end

fprintf('  总二维路径数: %d\n', length(all_paths_2d));
fprintf('  总三维路径数: %d\n', length(all_paths_3d));

%% ========== 第14步：可视化二维路径 ==========
fprintf('\n【步骤14】可视化二维路径...\n');

figure('Name', '二维路径', 'Position', [100, 100, 800, 600]);
hold on;

% 绘制轮廓
for i = 1:length(outer_contours)
    plot(outer_contours{i}(:,1), outer_contours{i}(:,2), 'k-', 'LineWidth', 2);
end
for i = 1:length(inner_contours)
    plot(inner_contours{i}(:,1), inner_contours{i}(:,2), 'r-', 'LineWidth', 2);
end

% 绘制路径
for i = 1:length(all_paths_2d)
    pts = all_paths_2d{i};
    if ~isempty(pts) && size(pts, 1) >= 2
        plot(pts(:,2), pts(:,1), 'b-', 'LineWidth', 0.5);
    end
end

axis equal tight;
title(sprintf('层 %d 二维路径: %d 条', layer_idx, length(all_paths_2d)), 'FontSize', 12);
xlabel('X'); ylabel('Y');
hold off;

saveas(gcf, sprintf('layer%d_paths_2d.png', layer_idx));
fprintf('  二维路径图已保存\n');

%% ========== 第15步：可视化三维路径 ==========
fprintf('\n【步骤15】可视化三维路径...\n');

figure('Name', '三维路径', 'Position', [100, 100, 1000, 800]);
hold on;

% 绘制曲面
surf(X_surf, Y_surf, Z_surf, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8]);

% 绘制三维路径
for i = 1:length(all_paths_3d)
    pts = all_paths_3d{i};
    if ~isempty(pts) && size(pts, 1) >= 2
        plot3(pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 0.8);
    end
end

axis equal;
view(-37.5, 30);
grid on;
title(sprintf('层 %d 三维路径: %d 条', layer_idx, length(all_paths_3d)), 'FontSize', 12);
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off;

saveas(gcf, sprintf('layer%d_paths_3d.png', layer_idx));
fprintf('  三维路径图已保存\n');

%% ========== 第16步：综合可视化 ==========
fprintf('\n【步骤16】生成综合可视化...\n');

figure('Name', '综合视图', 'Position', [50, 50, 1800, 900]);

% 子图1：密度场
subplot(2, 3, 1);
imagesc(xold');
axis xy equal tight;
colormap(gca, gray);
colorbar;
title('1. 密度场', 'FontSize', 11);

% 子图2：方向场
subplot(2, 3, 2);
imagesc(t_filtered');
axis xy equal tight;
colormap(gca, hsv);
colorbar;
title('2. 方向场', 'FontSize', 11);

% 子图3：内外轮廓
subplot(2, 3, 3);
hold on;
for i = 1:length(outer_contours)
    fill(outer_contours{i}(:,1), outer_contours{i}(:,2), [0.85, 0.85, 0.85], 'EdgeColor', 'b', 'LineWidth', 2);
end
for i = 1:length(inner_contours)
    fill(inner_contours{i}(:,1), inner_contours{i}(:,2), [1, 1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
end
for i = 1:length(extended_streamlines)
    sl = extended_streamlines{i};
    if ~isempty(sl)
        plot(sl(:,1), sl(:,2), 'g-', 'LineWidth', 1.5);
    end
end
axis equal tight;
title(sprintf('3. 轮廓+流线 (外%d 内%d 流%d)', length(outer_contours), length(inner_contours), length(extended_streamlines)), 'FontSize', 11);
hold off;

% 子图4：分割区域
subplot(2, 3, 4);
hold on;
colors = jet(max(length(all_regions), 1));
for i = 1:length(all_regions)
    plot(all_regions{i}, 'FaceColor', colors(i,:), 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end
axis equal tight;
title(sprintf('4. 分割区域 (%d个)', length(all_regions)), 'FontSize', 11);
hold off;

% 子图5：二维路径
subplot(2, 3, 5);
hold on;
for i = 1:length(outer_contours)
    plot(outer_contours{i}(:,1), outer_contours{i}(:,2), 'k-', 'LineWidth', 1.5);
end
for i = 1:length(inner_contours)
    plot(inner_contours{i}(:,1), inner_contours{i}(:,2), 'r-', 'LineWidth', 1.5);
end
for i = 1:length(all_paths_2d)
    pts = all_paths_2d{i};
    if ~isempty(pts) && size(pts, 1) >= 2
        plot(pts(:,2), pts(:,1), 'b-', 'LineWidth', 0.3);
    end
end
axis equal tight;
title(sprintf('5. 二维路径 (%d条)', length(all_paths_2d)), 'FontSize', 11);
hold off;

% 子图6：三维路径
subplot(2, 3, 6);
hold on;
surf(X_surf, Y_surf, Z_surf, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8]);
for i = 1:length(all_paths_3d)
    pts = all_paths_3d{i};
    if ~isempty(pts) && size(pts, 1) >= 2
        plot3(pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 0.5);
    end
end
axis equal;
view(-37.5, 30);
grid on;
title(sprintf('6. 三维路径 (%d条)', length(all_paths_3d)), 'FontSize', 11);
hold off;

sgtitle(sprintf('层 %d 路径生成综合视图', layer_idx), 'FontSize', 14);
saveas(gcf, sprintf('layer%d_comprehensive.png', layer_idx));
fprintf('  综合视图已保存\n');

%% ========== 第17步：保存结果 ==========
fprintf('\n【步骤17】保存结果...\n');

results = struct();
results.layer_idx = layer_idx;
results.params = params;
results.outer_contours = outer_contours;
results.inner_contours = inner_contours;
results.streamlines = extended_streamlines;
results.medial_axis = medial_axis;
results.regions = all_regions;
results.paths_2d = all_paths_2d;
results.paths_3d = all_paths_3d;
results.pointCloud_data = pointCloud_data;
results.X_surf = X_surf;
results.Y_surf = Y_surf;
results.Z_surf = Z_surf;

save(sprintf('layer%d_results_v3.mat', layer_idx), 'results');
fprintf('  结果已保存: layer%d_results_v3.mat\n', layer_idx);

%% ========== 统计输出 ==========
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        处理完成                                ║\n');
fprintf('╠════════════════════════════════════════════════════════════════╣\n');
fprintf('║  层号: %-5d                                                   \n', layer_idx);
fprintf('║  外轮廓数: %-5d  (材料边界)                                   \n', length(outer_contours));
fprintf('║  内轮廓数: %-5d  (孔洞边界)                                   \n', length(inner_contours));
fprintf('║  流线数: %-5d                                                 \n', length(extended_streamlines));
fprintf('║  分割区域: %-5d                                               \n', length(all_regions));
fprintf('║  二维路径: %-5d                                               \n', length(all_paths_2d));
fprintf('║  三维路径: %-5d                                               \n', length(all_paths_3d));
fprintf('╚════════════════════════════════════════════════════════════════╝\n');

warning('on', 'MATLAB:polyshape:repairedBySimplify');
warning('on', 'MATLAB:polyshape:boundary3Points');

fprintf('\n✅ 单层处理完成！\n');
fprintf('\n生成的文件:\n');
fprintf('  - layer%d_contours_streamlines.png  (轮廓和流线)\n', layer_idx);
fprintf('  - layer%d_regions.png               (分割区域)\n', layer_idx);
fprintf('  - layer%d_paths_2d.png              (二维路径)\n', layer_idx);
fprintf('  - layer%d_paths_3d.png              (三维路径)\n', layer_idx);
fprintf('  - layer%d_comprehensive.png         (综合视图)\n', layer_idx);
fprintf('  - layer%d_results_v3.mat            (结果数据)\n', layer_idx);