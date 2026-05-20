function [outer_contours, inner_contours, streamlines, medial_axis,Original_streamline,material_mask] = plotTopologyWithMedialAxis2(xold, t, nelx, nely, volfrac)
    % --- 1. 创建二值图像 (保持原样) ---
    x_filter = zeros(nely, nelx);
    th = prctile(xold(:), (1-volfrac)*100);
    th = th * 1.2;
    x2_num = find(xold > th);
    x_filter(x2_num) = 1;
    x_show = x_filter;
    
    % --- 2. 提取轮廓并区分内外轮廓 ---
    [B, L, N, A] = bwboundaries(x_filter, 'holes');   

    outer_boundaries = B(1:N);           % 外轮廓
    inner_boundaries = B(N+1:end);      % 内孔洞
    
    % --- 核心修改：针对内外轮廓分别进行平滑处理 ---
    outer_contours = processOuter(outer_boundaries); 
    inner_contours = processInner(inner_boundaries); % 这里实现了椭圆化和向内收缩

    % 先过滤外轮廓，只保留长度>=10的轮廓   
    valid_outer = {};
    for k = 1:length(outer_contours)
        contour_length = calculate_contour_length(outer_contours{k});
        if contour_length >= 10
            valid_outer{end+1} = outer_contours{k};
        end
    end
    outer_contours = valid_outer;

    % 先过滤内轮廓，只保留长度>=4的轮廓
    valid_inner = {};
    for k = 1:length(inner_contours)
        contour_length = calculate_contour_length(inner_contours{k});
        if contour_length >= 4
            valid_inner{end+1} = inner_contours{k};
        end
    end
    inner_contours = valid_inner;
    
    % --- 3. 计算中轴线 (保持原样) ---
    skeleton = bwmorph(x_filter, 'skel', Inf);
    skeleton_clean = bwmorph(skeleton, 'spur', 3);
    [y_skel, x_skel] = find(skeleton_clean);
    medial_axis = [x_skel, y_skel]; 
    
    % --- 4. 生成流线逻辑 (保持原样) ---
    [X, Y] = meshgrid(1:nelx, 1:nely);
    U = cos(t); V = sin(t);
    material_mask = x_filter > 0;
    seed_density = 1.5; 
    startx = 1:seed_density:nelx; 
    starty = 1:seed_density:nely; 
    [startX, startY] = meshgrid(startx, starty); 
    startX = startX(:); startY = startY(:); 
    
    verts = stream2(X, Y, U, V, startX, startY);
    Original_streamline = verts;
    
    clipped_verts = {};
    for i = 1:length(verts)
        if ~isempty(verts{i})
            points = verts{i};
            points(:, 1) = max(1, min(points(:, 1), nelx));
            points(:, 2) = max(1, min(points(:, 2), nely));
            material_vals = interp2(X, Y, double(material_mask), points(:, 1), points(:, 2), 'linear');
            in_material = material_vals > 0.5;
            segments = find_continuous_segments(in_material);
            for seg = 1:size(segments, 1)
                start_idx = segments(seg, 1); end_idx = segments(seg, 2);
                if end_idx - start_idx >= 1
                    segment_points = points(start_idx:end_idx, :);
                    segment_length = calculate_segment_length(segment_points);
                    straightness = calculate_straightness(segment_points);
                    if segment_length > 2 && straightness > 0.7
                        clipped_verts{end+1} = segment_points;
                    end
                end
            end
        end
    end
    
    % --- 5. 过滤弯曲流线与筛选 (保持原样) ---
    filtered_verts = filter_curvy_streamlines(clipped_verts);
    minLength = 15;   
    new_filtered_verts = {};  
    count = 0;
    for k = 1:length(filtered_verts)
        pts = filtered_verts{k};
        diffs = diff(pts, 1, 1);       
        seg_lengths = sqrt(sum(diffs.^2, 2)); 
        L_val = sum(seg_lengths);           
        if L_val >= minLength
            count = count + 1;
            new_filtered_verts{count} = pts;
        end
    end
    filtered_verts = new_filtered_verts;

    streamlines = {};
    if ~isempty(y_skel) && ~isempty(filtered_verts)
        skel_points = [x_skel, y_skel];
        num_skel = size(skel_points, 1);
        num_streamlines = length(filtered_verts);
        cov_count = zeros(num_skel, 1);
        streamline_skel_sets = cell(num_streamlines, 1);
        dist_thresh = 10; 
        
        for idx = 1:num_streamlines
            points = filtered_verts{idx};
            skel_indices = [];
            for p_idx = 1:size(points, 1)
                point = points(p_idx, :);
                dists = sqrt((skel_points(:,1) - point(1)).^2 + (skel_points(:,2) - point(2)).^2);
                near_indices = find(dists <= dist_thresh);
                skel_indices = union(skel_indices, near_indices);
            end
            streamline_skel_sets{idx} = skel_indices;
            cov_count(skel_indices) = cov_count(skel_indices) + 1;
        end
        
        streamline_lengths = cellfun(@(x) size(x,1), filtered_verts);
        [~, sorted_indices] = sort(streamline_lengths); 
        keep = true(num_streamlines, 1); 
        for i = 1:num_streamlines
            idx = sorted_indices(i);
            if ~keep(idx), continue; end
            skel_set = streamline_skel_sets{idx};
            if isempty(skel_set), keep(idx) = false; continue; end
            can_delete = true;
            for p = skel_set'
                if cov_count(p) <= 1, can_delete = false; break; end
            end
            if can_delete
                keep(idx) = false;
                cov_count(skel_set) = cov_count(skel_set) - 1;
            end
        end
        for idx = 1:num_streamlines
            if keep(idx), streamlines{end+1} = filtered_verts{idx}; end
        end
    end
    
    % --- 6. 绘图部分 (保持原样) ---
    figure; set(gcf, 'Position', [100, 100, 1000, 800]); hold on;    
    for k = 1:length(outer_contours)
        contour = outer_contours{k};
        plot(contour(:, 1), contour(:, 2), 'k-', 'LineWidth', 2.5);
        fill(contour(:, 1), contour(:, 2), [0.8, 0.8, 0.8], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    end
    for k = 1:length(inner_contours)
        contour = inner_contours{k};
        plot(contour(:, 1), contour(:, 2), 'y-', 'LineWidth', 2.5);
        fill(contour(:, 1), contour(:, 2), [1, 1, 1], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    end
    for idx = 1:length(streamlines)
        points = streamlines{idx};
        plot(points(:, 1), points(:, 2), 'b-', 'LineWidth', 2);
    end
    plot(medial_axis(:, 1), medial_axis(:, 2), 'r.', 'MarkerSize', 10);
    quiver(X(1:2:end, 1:2:end), Y(1:2:end, 1:2:end), U(1:2:end, 1:2:end), V(1:2:end, 1:2:end), 0.5, 'b', 'LineWidth', 0.6);
    title('拓扑优化结构与纤维方向、流线及中轴线');
    axis equal; axis off; set(gca, 'YDir', 'normal'); xlim([1, nelx]); ylim([1, nely]);
    legend('轮廓', '保留流线', '中轴线', 'Location', 'best'); hold off;
end

%% --- 新增：外轮廓处理 (追求极致平滑) ---
function refined = processOuter(boundaries)
    refined = {};
    for k = 1:length(boundaries)
        pts = [boundaries{k}(:,2), boundaries{k}(:,1)]; % 转换为 [x, y]
        if size(pts, 1) < 5, continue; end
        
        % 均匀重采样
        dists = [0; cumsum(sqrt(sum(diff(pts,1,1).^2, 2)))];
        % 设定合理点数，避免过密
        n_points = min(200, max(40, floor(dists(end)/1.2)));
        query_dists = linspace(0, dists(end), n_points);
        pts = interp1(dists, pts, query_dists, 'pchip'); % 使用pchip保持形状顺滑
        
        % 循环扩充平滑
        window = 4;
        padded = [pts(end-window+1:end, :); pts; pts(1:window, :)];
        sx = smooth(padded(:,1), window, 'moving');
        sy = smooth(padded(:,2), window, 'moving');
        res = [sx(window+1:end-window), sy(window+1:end-window)];
        res(end+1, :) = res(1, :); % 确保严格闭合
        refined{end+1} = res;
    end
end

%% --- 新增：内轮廓处理 (椭圆化 + 向内收缩) ---
function refined = processInner(boundaries)
    refined = {};
    for k = 1:length(boundaries)
        pts = [boundaries{k}(:,2), boundaries{k}(:,1)];
        if size(pts, 1) < 8, continue; end
        
        % 1. 计算中心
        centroid = mean(pts, 1);
        
        % 2. 保守收缩：向中心移动点 (shrink_factor 越小，孔越小，结构越强)
        shrink_factor = 0.925; 
        pts = centroid + (pts - centroid) * shrink_factor;
        
        % 3. 椭圆拟合与混合
        cov_mat = cov(pts);
        [V, D] = eig(cov_mat);
        a = sqrt(D(2,2)) * 2; b = sqrt(D(1,1)) * 2;
        angle = atan2(V(2,2), V(1,2));
        
        t = linspace(0, 2*pi, size(pts, 1))';
        ideal_ellipse = [a * cos(t), b * sin(t)];
        R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
        ideal_ellipse = (R * ideal_ellipse')' + centroid;
        
        % 混合原始意图与理想椭圆 (80%椭圆权重)
        pts = 0.8 * ideal_ellipse + 0.2 * pts;
        
        % 4. 均匀重采样与循环平滑
        dists = [0; cumsum(sqrt(sum(diff(pts,1,1).^2, 2)))];
        query_dists = linspace(0, dists(end), size(pts, 1));
        pts = interp1(dists, pts, query_dists, 'pchip');
        
        window = 7;
        padded = [pts(end-window+1:end, :); pts; pts(1:window, :)];
        sx = smooth(padded(:,1), window, 'loess'); % loess 更加圆润
        sy = smooth(padded(:,2), window, 'loess');
        res = [sx(window+1:end-window), sy(window+1:end-window)];
        res(end+1, :) = res(1, :);
        refined{end+1} = res;
    end
end

%% --- 原始辅助函数 (完全保留) ---
function perimeter = calculate_contour_length(contour)
    if size(contour, 1) < 2, perimeter = 0; return; end
    is_closed = all(contour(1,:) == contour(end,:));
    dx = diff(contour(:,1)); dy = diff(contour(:,2));
    distances = sqrt(dx.^2 + dy.^2);
    if is_closed
        last_dist = sqrt((contour(end,1)-contour(1,1))^2 + (contour(end,2)-contour(1,2))^2);
        perimeter = sum(distances) + last_dist;
    else
        perimeter = sum(distances);
    end
end

function segments = find_continuous_segments(logical_vec)
    segments = [];
    start_idx = find(diff([0; logical_vec; 0]) == 1);
    end_idx = find(diff([0; logical_vec; 0]) == -1) - 1;
    if ~isempty(start_idx), segments = [start_idx, end_idx]; end
end

function length_val = calculate_segment_length(points)
    dx = diff(points(:, 1)); dy = diff(points(:, 2));
    length_val = sum(sqrt(dx.^2 + dy.^2));
end

function straightness = calculate_straightness(points)
    start_point = points(1, :); end_point = points(end, :);
    straight_distance = sqrt(sum((end_point - start_point).^2));
    actual_distance = calculate_segment_length(points);
    straightness = straight_distance / actual_distance;
end

function filtered_verts = filter_curvy_streamlines(verts)
    filtered_verts = {};
    for i = 1:length(verts)
        points = verts{i};
        if size(points, 1) > 2
            dx = gradient(points(:, 1)); dy = gradient(points(:, 2));
            ddx = gradient(dx); ddy = gradient(dy);
            curvature = abs(ddx .* dy - dx .* ddy) ./ (dx.^2 + dy.^2).^(3/2);
            curvature(isnan(curvature)) = 0;
            if mean(curvature) < 0.5, filtered_verts{end+1} = points; end
        end
    end
end