function [outer_contours, inner_contours, streamlines, medial_axis,Original_streamline,material_mask] = plotTopologyWithMedialAxis(xold, t, nelx, nely, volfrac)
    % 参数说明:
    % xold - 密度场矩阵
    % t - 角度场矩阵
    % nelx, nely - 网格尺寸
    % volfrac - 体积分数
    % 输出:
    % outer_contours - 外轮廓线数据
    % inner_contours - 内轮廓线数据
    % streamlines - 代表性流线数据
    % medial_axis - 中轴线数据
    % 创建二值图像
    x_filter = zeros(nely, nelx);
    th = prctile(xold(:), (1-volfrac)*100);
    th=th*1.2;
    x2_num = find(xold > th);
    x_filter(x2_num) = 1;
    x_show = x_filter;
    
    % 1. 提取轮廓并区分内外轮廓
    [B, L, N, A] = bwboundaries(x_filter, 'holes');   
    % 区分内外轮廓
    outer_boundaries = {};
    inner_boundaries = {};    
    for k = 1:length(B)
        % 判断是外轮廓还是内轮廓
        if any(A(:, k)) % 如果有父轮廓，则是内轮廓
            inner_boundaries{end+1} = B{k};
        else % 否则是外轮廓
            outer_boundaries{end+1} = B{k};
        end
    end
    
    % 预处理轮廓数据（平滑和减少点数）
    outer_contours = preprocessContours(outer_boundaries);
    inner_contours = preprocessContours(inner_boundaries);
    % 先过滤外轮廓，只保留长度>=4的轮廓   
    valid_outer = {};
    for k = 1:length(outer_contours)
        contour_length = calculate_contour_length(outer_contours{k});
        if contour_length >= 10
            valid_outer{end+1} = outer_contours{k};
        end
    end
    outer_contours=valid_outer;
    % 先过滤内轮廓，只保留长度>=4的轮廓
    valid_inner = {};
    for k = 1:length(inner_contours)
        contour_length = calculate_contour_length(inner_contours{k});
        if contour_length >= 4
            valid_inner{end+1} = inner_contours{k};
        end
    end
    inner_contours=valid_inner;
    
    % 2. 计算中轴线（骨架）
    skeleton = bwmorph(x_filter, 'skel', Inf);
    skeleton_clean = bwmorph(skeleton, 'spur', 3);
    [y_skel, x_skel] = find(skeleton_clean);
    medial_axis = [x_skel, y_skel]; % 存储中轴线坐标
    
    % 3. 根据纤维方向生成流线
    % 创建网格坐标
    [X, Y] = meshgrid(1:nelx, 1:nely);
    
    % 重塑向量场为网格格式
    U = cos(t);
    V = sin(t);
    
    % 只在材料区域内生成流线
    material_mask = x_filter > 0;
    %生成用于后续作图的流线
    % 创建覆盖整个区域的种子点网格
    seed_density = 1.5; % 控制种子点密度
    startx = 1:seed_density:nelx; % X方向种子点
    starty = 1:seed_density:nely; % Y方向种子点
    [startX, startY] = meshgrid(startx, starty); % 生成网格状种子点
    startX = startX(:); % 转换为列向量
    startY = startY(:); % 转换为列向量
    %生成整体结构的初始流线
    % 使用stream2计算流线顶点
    verts = stream2(X, Y, U, V, startX, startY);
    Original_streamline=verts;
    % 裁剪流线，只保留材料区域内的部分
    clipped_verts = {};
    for i = 1:length(verts)
        if ~isempty(verts{i})
            % 获取当前流线的所有点
            points = verts{i};
            % 确保所有点都在有效范围内
            points(:, 1) = max(1, min(points(:, 1), nelx));
            points(:, 2) = max(1, min(points(:, 2), nely));
            % 使用interp2进行快速插值
            x_coords = points(:, 1);
            y_coords = points(:, 2);
            material_vals = interp2(X, Y, double(material_mask), x_coords, y_coords, 'linear');
            % 确定哪些点在材料区域内
            in_material = material_vals > 0.5;
            % 找到材料区域内的连续段
            segments = find_continuous_segments(in_material);
            % 提取每个连续段
            for seg = 1:size(segments, 1)
                start_idx = segments(seg, 1);
                end_idx = segments(seg, 2);
                if end_idx - start_idx >= 1  % 至少有两个点
                    segment_points = points(start_idx:end_idx, :);

                    % 计算段的长度
                    segment_length = calculate_segment_length(segment_points);

                    % 计算段的直线度（直线距离/实际路径长度）
                    straightness = calculate_straightness(segment_points);

                    % 只保留长度足够且直线度较高的段
                    if segment_length > 2 && straightness > 0.7
                        clipped_verts{end+1} = segment_points;
                    end
                end
            end
        end
    end
    
    % 4. 过滤弯曲流线
    filtered_verts = filter_curvy_streamlines(clipped_verts);
    %4-1次数删除过短的流线：长度小于5的剩余流线都可以认为没有参考价值
    % 假设 filtered_verts 是 cell 数组，每个 cell 是一条流线的点矩阵
    minLength = 5;   % 最小长度阈值
    new_filtered_verts = {};   % 保存长度大于阈值的流线
    count = 0;

    for k = 1:length(filtered_verts)
        pts = filtered_verts{k};
        % 计算流线长度
        diffs = diff(pts, 1, 1);        % 每段向量
        seg_lengths = sqrt(sum(diffs.^2, 2)); % 每段长度
        L = sum(seg_lengths);           % 流线总长度

        if L >= minLength
            count = count + 1;
            new_filtered_verts{count} = pts;
        end
    end
    % 替换原来的 filtered_verts
    filtered_verts = new_filtered_verts;

    % 5. 筛选流线：只保留那些覆盖中轴点且必要的流线
    streamlines = {};
    if ~isempty(y_skel) && ~isempty(filtered_verts)
        % 将中轴点组合成矩阵
        skel_points = [x_skel, y_skel];
        num_skel = size(skel_points, 1);
        num_streamlines = length(filtered_verts);
        
        % 初始化覆盖次数数组
        cov_count = zeros(num_skel, 1);
        % 初始化每条流线的中轴点集合
        streamline_skel_sets = cell(num_streamlines, 1);
        
        dist_thresh = 5; % 距离阈值1.0网格单位（假设1.0mm对应1.0网格单位）
        
        % 计算每条流线覆盖的中轴点
        for idx = 1:num_streamlines
            points = filtered_verts{idx};
            skel_indices = [];
            % 遍历流线上的每个点
            for p_idx = 1:size(points, 1)
                point = points(p_idx, :);
                % 计算到所有中轴点的距离
                dists = sqrt((skel_points(:,1) - point(1)).^2 + (skel_points(:,2) - point(2)).^2);
                near_indices = find(dists <= dist_thresh);
                skel_indices = union(skel_indices, near_indices);
            end
            streamline_skel_sets{idx} = skel_indices;
            % 更新覆盖次数
            cov_count(skel_indices) = cov_count(skel_indices) + 1;
        end
        
        % 计算流线长度（按点的数量）
        streamline_lengths = cellfun(@(x) size(x,1), filtered_verts);
        [~, sorted_indices] = sort(streamline_lengths); % 按长度排序流线索引
        
        keep = true(num_streamlines, 1); % 初始化保留标志
        
        % 遍历排序后的流线
        for i = 1:num_streamlines
            idx = sorted_indices(i);
            if ~keep(idx)
                continue;
            end
            skel_set = streamline_skel_sets{idx};
            if isempty(skel_set)
                keep(idx) = false;
                continue;
            end
            % 检查该流线的所有中轴点是否都被其他流线覆盖（覆盖次数>1）
            can_delete = true;
            for p = skel_set'
                if cov_count(p) <= 1
                    can_delete = false;
                    break;
                end
            end
            if can_delete
                keep(idx) = false;
                % 更新覆盖次数
                cov_count(skel_set) = cov_count(skel_set) - 1;
            end
        end
        
        % 存储保留的流线
        for idx = 1:num_streamlines
            if keep(idx)
                streamlines{end+1} = filtered_verts{idx};
            end
        end
    end
    
    % 如果不需要绘图，直接返回数据
    %if nargout > 0
     %   return;
    %end
% 创建网格坐标
    [X, Y] = meshgrid(1:nelx, 1:nely);
    % 以下为绘图部分（仅在无输出参数时执行）
    figure;
    set(gcf, 'Position', [100, 100, 1000, 800]);
    hold on;    
    
    % 绘制所有外轮廓
    for k = 1:length(outer_contours)
        contour = outer_contours{k};
        plot(contour(:, 1), contour(:, 2), 'k-', 'LineWidth', 2.5);
        fill(contour(:, 1), contour(:, 2), [0.8, 0.8, 0.8], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    end
    
    % 绘制所有内轮廓
    for k = 1:length(inner_contours)
        contour = inner_contours{k};
        plot(contour(:, 1), contour(:, 2), 'y-', 'LineWidth', 2.5);
        fill(contour(:, 1), contour(:, 2), [1, 1, 1], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    end
    
    % 绘制保留的流线
    for idx = 1:length(streamlines)
        points = streamlines{idx};
        plot(points(:, 1), points(:, 2), 'b-', 'LineWidth', 2);
    end
    
    % 绘制中轴线（红色点表示）
    plot(medial_axis(:, 1), medial_axis(:, 2), 'r.', 'MarkerSize', 10);
    %绘制纤维方向
    
   quiver(X(1:2:end, 1:2:end), Y(1:2:end, 1:2:end), ...
       U(1:2:end, 1:2:end), V(1:2:end, 1:2:end), ...
       0.5, 'b', 'LineWidth', 0.6);

    % 设置图形属性
    title('拓扑优化结构与纤维方向、流线及中轴线');
    axis equal;
    axis off;
    set(gca, 'YDir', 'normal');
    xlim([1, nelx]);
    ylim([1, nely]);
    
    % 添加图例
    legend('轮廓', '保留流线', '中轴线', 'Location', 'best');
    
    hold off;
end

% 预处理轮廓数据函数
function processed_contours = preprocessContours(contours)
    processed_contours = {};
    for k = 1:length(contours)
        boundary = contours{k};
        % 减少点数以使曲线更光滑但不改变形状
        if length(boundary) > 200
            step = ceil(length(boundary)/200);
            indices = 1:step:length(boundary);
            boundary = boundary(indices, :);
        end
        
        % 确保轮廓闭合
        if ~isequal(boundary(1, :), boundary(end, :))
            boundary(end+1, :) = boundary(1, :);
        end
        
        % 使用移动平均滤波平滑
        window_size = 4;
        if length(boundary) > window_size * 2
            x_points = smooth(boundary(:, 2), window_size);
            y_points = smooth(boundary(:, 1), window_size);
        else
            x_points = boundary(:, 2);
            y_points = boundary(:, 1);
        end
        
        % 转换为(x,y)格式并存储
        processed_contours{end+1} = [x_points, y_points];
    end
end

% 辅助函数 - 查找连续段
function segments = find_continuous_segments(logical_vec)
    segments = [];
    start_idx = find(diff([0; logical_vec; 0]) == 1);
    end_idx = find(diff([0; logical_vec; 0]) == -1) - 1;
    
    if ~isempty(start_idx)
        segments = [start_idx, end_idx];
    end
end

% 辅助函数 - 计算段长度
function length = calculate_segment_length(points)
    dx = diff(points(:, 1));
    dy = diff(points(:, 2));
    length = sum(sqrt(dx.^2 + dy.^2));
end

% 辅助函数 - 计算直线度
function straightness = calculate_straightness(points)
    start_point = points(1, :);
    end_point = points(end, :);
    straight_distance = sqrt(sum((end_point - start_point).^2));
    actual_distance = calculate_segment_length(points);
    straightness = straight_distance / actual_distance;
end

% 辅助函数 - 过滤弯曲流线
function filtered_verts = filter_curvy_streamlines(verts)
    filtered_verts = {};
    for i = 1:length(verts)
        points = verts{i};
        if size(points, 1) > 2
            % 计算平均曲率
            dx = gradient(points(:, 1));
            dy = gradient(points(:, 2));
            ddx = gradient(dx);
            ddy = gradient(dy);
            curvature = abs(ddx .* dy - dx .* ddy) ./ (dx.^2 + dy.^2).^(3/2);
            curvature(isnan(curvature)) = 0;
            
            % 如果平均曲率小于阈值，则保留
            if mean(curvature) < 0.5
                filtered_verts{end+1} = points;
            end
        end
    end
end
% 计算轮廓周长（几何长度）的函数
function perimeter = calculate_contour_length(contour)
    % 计算闭合轮廓的周长
    % 输入：contour - N×2矩阵，每行是一个点的(x,y)坐标
    % 输出：perimeter - 轮廓的周长
    
    if size(contour, 1) < 2
        perimeter = 0;
        return;
    end
    
    % 检查轮廓是否闭合（第一个点和最后一个点是否相同）
    is_closed = all(contour(1,:) == contour(end,:));
    
    if is_closed
        % 对于闭合轮廓，计算所有相邻点之间的距离（包括最后一个点到第一个点）
        dx = diff(contour(:,1));
        dy = diff(contour(:,2));
        distances = sqrt(dx.^2 + dy.^2);
        
        % 添加最后一个点到第一个点的距离
        last_dist = sqrt((contour(end,1)-contour(1,1))^2 + ...
                        (contour(end,2)-contour(1,2))^2);
        perimeter = sum(distances) + last_dist;
    else
        % 对于开放轮廓，只计算相邻点之间的距离
        dx = diff(contour(:,1));
        dy = diff(contour(:,2));
        distances = sqrt(dx.^2 + dy.^2);
        perimeter = sum(distances);
    end
end
