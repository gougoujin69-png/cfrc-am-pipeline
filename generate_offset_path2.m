function all_rings_results=generate_offset_path2(outer_path,inner_path,offset_distanceall,max_iter,min_length,pointCloud_data)
% 保存所有轮廓集合（cell，每一轮存一个 polyshape cell）
all_results = {}; 
outer_polys_old = cellfun(@(c) polyshape(c(:,2), c(:,1)), outer_path, 'UniformOutput', false);
outer_polys_old=outer_polys_old{1};
% --------------------- 迭代 ---------------------
for iter = 1:max_iter
    offset_distance=offset_distanceall;
    if iter==1
        offset_distance=0.5*offset_distanceall;
    end
    fprintf('迭代 %d/%d ...\n', iter, max_iter);
    % --------------------- 初始polyshape ---------------------
    outer_polys = cellfun(@(c) polyshape(c(:,2), c(:,1)), outer_path, 'UniformOutput', false);    
    inner_polys={}; %首先建立空集，储存面域
    %%%此处更新新的outer_polys_old
    
    for i=1:numel(inner_path)
        inner_poly=cellfun(@(c) polyshape(c(:,2), c(:,1)), inner_path{i}, 'UniformOutput', false);
        inner_polys{end+1}=inner_poly{1};
    end    
    %inner_polys = cellfun(@(c) polyshape(c(:,2), c(:,1)), inner_path, 'UniformOutput', false);
    outer_offset =outer_polys;
    inner_offset=inner_polys;
    final_results = {};   % ✅ 用 cell 保存    
    for i = 1:numel(outer_offset)
        numel(outer_offset)
        pOuter = outer_offset{i};
        diffPoly = pOuter;      
        for j = 1:numel(inner_offset)
            pInner = inner_offset{j};
            interPoly = intersect(pOuter, pInner);
            
            if ~isempty(interPoly.Vertices)
                diffPoly = subtract(diffPoly, pInner);
            %else
            %    final_results{end+1} = pInner; %#ok<AGROW>
            end
        end       
        if ~isempty(diffPoly.Vertices)
            regionsDiff = regions(diffPoly);
            for k = 1:numel(regionsDiff)
                final_results{end+1} = regionsDiff(k); %#ok<AGROW>
            end
        end
    end    
    % 计算这一轮总周长
    total_length = 0;
    for k = 1:numel(final_results)
        total_length = total_length + perimeter(final_results{k});
    end
    fprintf('迭代 %d 总路径长度 = %.3f\n', iter, total_length);
    %%%判断outer_polys是否属于outer_polys_old区域，如果不是，请break
        % 停止条件
    if total_length < min_length
        fprintf('停止迭代：总路径长度 %.3f < 阈值 %.3f\n', total_length, min_length);
        break;
    end
    % 保存结果（cell 里再放一组 polyshape）
    if iter>1
        all_results{end+1} = final_results; %#ok<AGROW>    
    end
    % 非等距offset路径 基于base_path偏移一次
    % --------------------- 组合轮廓 ---------------------
    new_outer_path = {};
    new_inner_path = {};
    %%%%%%首先处理外轮廓
    for j=1:length(outer_path)
        path=outer_path{j};
        path_resampled=resample_path(path,0.1,true);
        path_resampled_cw = ensureClockwise(path_resampled);
        [offset_path, f_values_out]=offset_contour_by_normals(path_resampled_cw,offset_distance,'outer',pointCloud_data);
        % Diagnostic: print f-value statistics
        fv = f_values_out{1};
        fprintf('  [f值] min=%.3f, mean=%.3f, max=%.3f, 被钳位点数=%d/%d\n', ...
            min(fv), mean(fv), max(fv), sum(fv <= 0.5+1e-6 & fv > 0), length(fv));
        [filter_path]=Self_intersection_outer(offset_path,outer_polys_old);
        path_loops =filter_path;
       % 遍历每个环，将其加入 new_outer_path
       for k = 1:length(path_loops)
       loop = path_loops{k};
       new_outer_path = [new_outer_path; loop];  % 按行拼接
       end
    end
    outer_path=new_outer_path;
    outer_polys_old=polyshape.empty;
    if length(outer_path)<1
        fprintf('无新路径生成');
        if iter == 1 && ~isempty(final_results)
            fprintf('（回退：保存iter=1的轮廓作为首条路径）\n');
            all_results{end+1} = final_results;
        end
        break
    end
    %%%此处更新新的outer_polys_old
    if length(outer_path)>1
               outer_poly=cellfun(@(c) polyshape(c(:,2), c(:,1)), outer_path(1), 'UniformOutput', false);
               outer_polys_old=outer_poly{1};
               for j=2:length(outer_path)
                   outer_poly=cellfun(@(c) polyshape(c(:,2), c(:,1)), outer_path(j), 'UniformOutput', false);
                   outer_polys_old=union(outer_polys_old,outer_poly{1});
               end
    else
        outer_poly=cellfun(@(c) polyshape(c(:,2), c(:,1)), outer_path(1), 'UniformOutput', false);
        outer_polys_old=outer_poly{1};
    end
    %此处更新inner
    for j=1:length(inner_path)
        path=inner_path{j}{1};
        path_resampled=resample_path(path,0.1,true);
        [offset_path,~]=offset_contour_by_normals(path_resampled,offset_distance,'outer',pointCloud_data);
        %处理自交和小环
        path_safe=remove_self_intersection_loops(offset_path);
        new_inner_path{end+1}=path_safe;
        %new_inner_path{end+1}=offset_path{1};
    end
    inner_path=new_inner_path;
end
% --------------------- 删除小环 ---------------------
min_length = 3;
for iter = 1:numel(all_results)
    polys = all_results{iter};
    keep_polys = {};
    for k = 1:numel(polys)
        if perimeter(polys{k}) >= min_length
            keep_polys{end+1} = polys{k}; %#ok<AGROW>
        end
    end
    all_results{iter} = keep_polys;
end
all_rings_results = {};  % 保存每轮点集

for iter = 1:numel(all_results)
    polys = all_results{iter};
    rings_iter = {};   % 当前迭代所有环
    
    for k = 1:numel(polys)
        rings_cell = polyshape2rings(polys{k});  % 拆成多个环
        rings_iter = [rings_iter, rings_cell];   % 拼接
    end
    
    all_rings_results{iter} = rings_iter;
end
% --------------------- 基于NaN分割点集 ---------------------
all_rings_split = {};  % 保存分割后的结果

for iter = 1:numel(all_rings_results)
    rings_iter = all_rings_results{iter};
    rings_split_iter = {};  % 当前迭代分割后的环
    
    for k = 1:numel(rings_iter)
        pts = rings_iter{k};       
        % 检查是否存在NaN
        nan_indices = find(isnan(pts(:,1)) | isnan(pts(:,2)));        
        if isempty(nan_indices)
            % 没有NaN，直接添加整个点集
            rings_split_iter{end+1} = pts;
        else
            % 基于NaN分割点集
            start_idx = 1;
            for i = 1:length(nan_indices)
                end_idx = nan_indices(i) - 1;               
                % 确保有有效的点
                if end_idx >= start_idx
                    segment = pts(start_idx:end_idx, :);
                    if size(segment, 1) >= 2
                        rings_split_iter{end+1} = segment;
                    end
                end                
                start_idx = nan_indices(i) + 1;
            end           
            % 处理最后一段
            if start_idx <= size(pts, 1)
                segment = pts(start_idx:end, :);
                if size(segment, 1) >= 2
                    rings_split_iter{end+1} = segment;
                end
            end
        end
    end
    
    all_rings_split{iter} = rings_split_iter;
end
% 更新变量名
all_rings_results = all_rings_split;
% ... (前面的代码保持不变) ...
% --------------------- 确保每个点集首尾相接 ---------------------
for iter = 1:numel(all_rings_results)
    rings_iter = all_rings_results{iter};
    
    for k = 1:numel(rings_iter)
        pts = rings_iter{k};
        
        if ~isempty(pts) && size(pts, 1) >= 2
            % 如果首尾点不同，添加第一个点到最后使其闭合
            if ~isequal(pts(1, :), pts(end, :))
                pts = [pts; pts(1, :)];
                rings_iter{k} = pts;
            end
        end
    end    
    all_rings_results{iter} = rings_iter;
end
% --------------------- 点集插值，确保两点之间距离不大于0.5 ---------------------
max_distance = 0.5;  % 最大允许距离
for iter = 1:numel(all_rings_results)
    rings_iter = all_rings_results{iter};    
    for k = 1:numel(rings_iter)
        pts = rings_iter{k};
        if ~isempty(pts) && size(pts, 1) >= 2
            new_pts = pts(1, :);  % 从第一个点开始            
            for i = 1:size(pts, 1)-1
                p1 = pts(i, :);
                p2 = pts(i+1, :);
                distance = norm(p2 - p1);               
                if distance > max_distance
                    % 计算需要插入的点数
                    num_insertions = ceil(distance / max_distance) - 1;                  
                    % 线性插值
                    for j = 1:num_insertions
                        ratio = j / (num_insertions + 1);
                        new_point = p1 + ratio * (p2 - p1);
                        new_pts = [new_pts; new_point];
                    end
                end              
                new_pts = [new_pts; p2];
            end            
            rings_iter{k} = new_pts;
        end
    end   
    all_rings_results{iter} = rings_iter;
end
%all_rings_results = filter_paths_nested(all_rings_results);


% 添加这一步进行相交过滤
final_clean_results = filter_intersecting_paths(all_rings_results);

% 如果需要，可以将结果赋值回去
all_rings_results = final_clean_results;
end
function [offset_paths, f_values] = offset_contour_by_normals(contour, offset_distance, direction, pointCloud_data)
    % ---- Configurable parameters ----
    f_min = 0.5;           % f lower bound (prevents near-zero offsets on steep surfaces)
    f_smooth_window = 7;   % smoothing window for f values (odd number)
    contour_smooth_win = 5;% smoothing window for final offset contour
    % ---------------------------------

    if ~isequal(contour(1,:), contour(end,:))
        contour(end+1,:) = contour(1,:);
    end
    normals = compute_consistent_normals(contour, direction);
    n = size(contour,1);
    f_vals = zeros(n,1);

    % Step 1: compute raw f values
    for i = 1:n
        A = [normals(i,1), normals(i,2), 0];
        if norm(A) < 1e-12
            f_vals(i) = f_min;
            continue;
        end
        B = getNormalAtPoint(contour(i,1), contour(i,2), pointCloud_data);
        B = B / norm(B);
        A_xy = A(1:2);
        A_xy_unit = A_xy / norm(A_xy);
        B_xy = [B(1), B(2)];
        dot_product = dot(B_xy, A_xy_unit);
        if abs(B(3)) < 1e-12
            f_vals(i) = f_min;
        else
            f_vals(i) = 1 / sqrt(1 + (dot_product^2) / (B(3)^2));
            f_vals(i) = max(f_vals(i), f_min);
        end
    end

    % Step 2: smooth f values (circular boundary)
    half_w = floor(f_smooth_window / 2);
    if half_w >= 1 && n > f_smooth_window
        f_smooth = zeros(n,1);
        for i = 1:n
            indices = mod((i-half_w:i+half_w)-1, n) + 1;
            f_smooth(i) = mean(f_vals(indices));
        end
        f_smooth = max(f_smooth, f_min);  % re-clamp after smoothing
        f_vals = f_smooth;
    end
    f_vals(end) = f_vals(1);  % ensure closure

    % Step 3: apply offset with smoothed f values
    offset_contour = contour - offset_distance * (f_vals .* normals);

    % Step 4: smooth the offset contour (circular boundary)
    half_c = floor(contour_smooth_win / 2);
    if half_c >= 1 && n > contour_smooth_win
        smoothed = zeros(size(offset_contour));
        for i = 1:n
            indices = mod((i-half_c:i+half_c)-1, n) + 1;
            smoothed(i,:) = mean(offset_contour(indices,:), 1);
        end
        offset_contour = smoothed;
    end

    % Ensure closure
    offset_contour(end,:) = offset_contour(1,:);

    offset_paths = {offset_contour};
    f_values = {f_vals};
end

function dense_path = resample_path(path, target_spacing, preserve_ends)
    % Robust resampling of a 2D path.
    % - removes NaN/Inf rows
    % - removes consecutive duplicate points
    % - ensures strictly increasing s for interp1
    % - handles degenerate cases gracefully

    if nargin < 3, preserve_ends = true; end
    dense_path = [];

    if isempty(path)
        return;
    end

    % 1) 删除包含 NaN/Inf 的点
    finite_mask = all(isfinite(path), 2);
    path = path(finite_mask, :);

    if size(path,1) < 2
        % 如果仅剩 0 或 1 个点，直接返回（或重复以保证两点）
        if isempty(path)
            dense_path = [];
        else
            dense_path = repmat(path(1,:), 2, 1);
        end
        return;
    end

    % 2) 删除连续重复点（或几乎重复）
    deltas = sqrt(sum(diff(path).^2, 2));
    keep = true(size(path,1),1);
    keep(2:end) = deltas > eps;  % 可根据需要调整阈值
    path = path(keep, :);

    if size(path,1) < 2
        dense_path = repmat(path(1,:), 2, 1);
        return;
    end

    % 3) 累积弧长 s
    d = sqrt(sum(diff(path).^2, 2));
    s = [0; cumsum(d)];

    % 4) 确保 s 严格递增：去掉重复的 s（保留第一个）
    [s_unique, ia] = unique(s, 'stable');
    if numel(s_unique) < numel(s)
        s = s_unique;
        path = path(ia, :);
    end

    % 5) 处理退化情况（零长度）
    if s(end) <= eps
        dense_path = repmat(path(1,:), 2, 1);
        return;
    end

    % 6) 计算采样数并做插值
    n_samples = max(ceil(s(end) / target_spacing), 2);
    xi = linspace(0, s(end), n_samples);

    % interp1 在这里应该安全（s 和 path 都 finite 且严格递增）
    xq = interp1(s, path(:,1), xi, 'linear');
    yq = interp1(s, path(:,2), xi, 'linear');

    % 如果插值结果出现 NaN（理论上不应），做保护
    if any(~isfinite(xq)) || any(~isfinite(yq))
        % 退回到简单均匀插值（保守做法）
        xq = linspace(path(1,1), path(end,1), n_samples);
        yq = linspace(path(1,2), path(end,2), n_samples);
    end

    dense_path = [xq(:), yq(:)];

    % 7) 保持端点（可选）
    if preserve_ends && size(path,1) >= 2
        dense_path(1,:) = path(1,:);
        dense_path(end,:) = path(end,:);
    end
end

%% --------------------- 小环裁剪函数 ---------------------
function path_clean = remove_self_intersection_loops(path)
    n = size(path,1);
    path_clean = path;
    while true
        intersect_found = false;
        n = size(path_clean,1);
        for i = 1:n-3
            for j = i+2:n-1
                if i == 1 && j == n-1, continue; end
                if do_lines_intersect(path_clean(i,:), path_clean(i+1,:), path_clean(j,:), path_clean(j+1,:))
                    len1 = j-i;
                    len2 = n-len1;
                    if len1 <= len2
                        idx_start = i+1; idx_end = j;
                    else
                        idx_start = j+1; idx_end = mod(i+n-1,n)+1;
                    end
                    if idx_start <= idx_end
                        path_clean(idx_start:idx_end,:) = [];
                    else
                        path_clean([idx_start:end,1:idx_end],:) = [];
                    end
                    intersect_found = true;
                    break;
                end
            end
            if intersect_found, break; end
        end
        if ~intersect_found, break; end
    end
    if ~isequal(path_clean(1,:), path_clean(end,:))
        path_clean(end+1,:) = path_clean(1,:);
    end
end
function [filter_path]=Self_intersection_outer(offset_path,outer_polys_old)
%%%input:offset_path(cell点集), outer_polys_old(polyshape形式)
%%%output:filter_path(cell点集)，outer_polys_filtered(polyshape形式)
%%%如果判断output为空，就打断循环，直接结束
%% 示例闭合路径
points=offset_path{1};
points_swapped = points(:, [2, 1]);
%% 1️⃣ 创建 polyshape
pgon = polyshape(points_swapped);
%% 2 此处可将自交区域划分为多个区域
pgon_fixed = simplify(pgon); % 自动修复自交
faces = regions(pgon_fixed); % 返回 polyshape 数组
faces_filtered = polyshape.empty;  % 空 polyshape
if length(outer_polys_old) > 1
    combined_outer = outer_polys_old(1);
    for j = 2:length(outer_polys_old)
        combined_outer = union(combined_outer, outer_polys_old(j));
    end
else
    combined_outer = outer_polys_old;
end
for i = 1:length(faces)
    if area(faces(i)) >= 0.1
        C = intersect(faces(i), combined_outer);  % 安全调用
        area_A = area(faces(i));
        area_C = area(C);
        ratio = area_C / area_A;
        if ratio > 0.95  % slightly relaxed from 0.99
            faces_filtered(end+1) = faces(i);
        end
    end
end
%将faces_filter转化为cell点集
filter_path={};
for i = 1:length(faces_filtered)
    tem=faces_filtered(i).Vertices;
    tem = tem(:, [2, 1]);
    filter_path{end+1}=tem;   
end

%%%%此处测试输入和输出的一致性

%figure; hold on; 
% plot(outer_polys_filtered(1), 'FaceColor', 'b', 'FaceAlpha', 1);
% hold on;
%  plot(outer_polys_old{1}, 'FaceColor', 'y', 'FaceAlpha', 0.3);
% axis equal;
% hold off;
% figure; hold on; 
% axis equal
%for i = 1:length(offset_path)
%    plot(offset_path{i}(:,1),offset_path{i}(:,2), 'Color','r','LineWidth',0.5);
%end
%hold on
%for i = 1:length(filter_path)
%    plot(filter_path{i}(:,1),filter_path{i}(:,2),'Color','k','LineWidth',1);
%    hold on
%end
%hold off;
end

%% 辅助函数

function normals = compute_consistent_normals(contour, direction)
    n = size(contour,1);
    normals = zeros(n,2);
    area = polyarea(contour(:,1), contour(:,2));
    is_clockwise = area < 0;
    for i = 1:n
        if i == 1
            prev_point = contour(end-1,:);
            next_point = contour(i+1,:);
        elseif i == n
            prev_point = contour(i-1,:);
            next_point = contour(2,:);
        else
            prev_point = contour(i-1,:);
            next_point = contour(i+1,:);
        end
        tangent = next_point - prev_point;
        if is_clockwise
            normal = [tangent(2), -tangent(1)];
        else
            normal = [-tangent(2), tangent(1)];
        end
        normal_length = norm(normal);
        if normal_length > 0
            normal = normal / normal_length;
        end
        normals(i,:) = normal;
    end
    if strcmp(direction,'inner')
        if ~is_clockwise
            normals = -normals;
        end
    else
        if is_clockwise
            normals = -normals;
        end
    end
    window_size = min(10, floor(n/6));  % larger window for smoother normals
    if window_size > 1
        smoothed_normals = zeros(size(normals));
        for i = 1:n
            indices = mod((i-window_size:i+window_size)-1, n) + 1;
            smoothed_normals(i,:) = mean(normals(indices,:),1);
            smoothed_normals(i,:) = smoothed_normals(i,:) / norm(smoothed_normals(i,:));
        end
        normals = smoothed_normals;
    end
end
function rings_cell = polyshape2rings(p)
    % 输入: p - 一个 polyshape 对象
    % 输出: rings_cell - cell，每个 cell 是 [x,y] 点集，表示一个闭合环

    rings_cell = {};
    subregions = regions(p);  % 每个子区域不包含 hole
    
    for i = 1:numel(subregions)
        % 获取这个区域的顶点
        pts = subregions(i).Vertices;  % [x,y] 矩阵
        if ~isempty(pts)
            rings_cell{end+1} = pts; %#ok<AGROW>
        end
    end
end
function intersect = do_lines_intersect(p1, p2, p3, p4)
    % 使用向量叉积方法判断线段 p1-p2 和 p3-p4 是否相交
    d1 = (p4(1)-p3(1))*(p1(2)-p3(2)) - (p4(2)-p3(2))*(p1(1)-p3(1));
    d2 = (p4(1)-p3(1))*(p2(2)-p3(2)) - (p4(2)-p3(2))*(p2(1)-p3(1));
    d3 = (p2(1)-p1(1))*(p3(2)-p1(2)) - (p2(2)-p1(2))*(p3(1)-p1(1));
    d4 = (p2(1)-p1(1))*(p4(2)-p1(2)) - (p2(2)-p1(2))*(p4(1)-p1(1));
    intersect = (d1*d2 < 0) && (d3*d4 < 0);
end
function processed_contours = preprocessContours(contours)
processed_contours = {};
for k = 1:length(contours)
    boundary = contours{k};
    if length(boundary) > 200
        step = ceil(length(boundary)/200);
        boundary = boundary(1:step:end, :);
    end
    if ~isequal(boundary(1,:), boundary(end,:))
        boundary(end+1,:) = boundary(1,:);
    end
    window_size = 4;
    if length(boundary) > window_size*2
        x_points = smooth(boundary(:,1), window_size);
        y_points = smooth(boundary(:,2), window_size);
    else
        x_points = boundary(:,1);
        y_points = boundary(:,2);
    end
    boundary_xy = [x_points, y_points];
    boundary_xy = fix_sharp_corners(boundary_xy, 60, 5);
    d = sqrt(sum(diff(boundary_xy).^2,2));
    s = [0; cumsum(d)];
    total_len = s(end);
    n_samples = max(ceil(total_len/0.1), 2);
    
    % 修复：确保s是严格递增且唯一的
    if any(diff(s) <= 0)
        % 如果s不是严格递增，添加微小增量
        s = s + (0:length(s)-1)' * eps * 10;
    end
    
    xq = interp1(s, boundary_xy(:,1), linspace(0,total_len,n_samples), 'linear');
    yq = interp1(s, boundary_xy(:,2), linspace(0,total_len,n_samples), 'linear');
    boundary_dense = [xq(:), yq(:)];
    if ~isequal(boundary_dense(1,:), boundary_dense(end,:))
        boundary_dense(end+1,:) = boundary_dense(1,:);
    end
    processed_contours{end+1} = boundary_dense;
end
end
function boundary_out = fix_sharp_corners(boundary, min_angle_deg, corner_factor)
    N = size(boundary,1);
    boundary_out = boundary;
    i = 2;
    while i <= N-1
        p_prev = boundary_out(i-1,:);
        p_curr = boundary_out(i,:);
        p_next = boundary_out(i+1,:);
        v1 = p_prev - p_curr;
        v2 = p_next - p_curr;
        cosTheta = dot(v1,v2)/(norm(v1)*norm(v2)+1e-12);
        theta = acosd(max(min(cosTheta,1),-1));
        if theta < min_angle_deg
            n_add = ceil(corner_factor);
            new_pts1 = [linspace(p_prev(1), p_curr(1), n_add)', ...
                        linspace(p_prev(2), p_curr(2), n_add)'];
            new_pts2 = [linspace(p_curr(1), p_next(1), n_add)', ...
                        linspace(p_curr(2), p_next(2), n_add)'];
            boundary_out(i,:) = [];
            boundary_out = [boundary_out(1:i-1,:); ...
                            new_pts1(1:end-1,:); ...
                            new_pts2(1:end-1,:); ...
                            boundary_out(i:end,:)];
            N = size(boundary_out,1);
            i = i + n_add*2 - 1;
        else
            i = i + 1;
        end
    end
end
function output_cell = filter_paths_nested(rings_results_debug)
% FILTER_PATHS_NESTED 过滤路径并返回双层cell结构
%   输入：rings_results_debug - 原始嵌套结构
%   输出：output_cell - 1×n的cell，每个元素是一个1×1 cell

% 第一步：从嵌套结构中提取所有二维路径
all_paths = {};
for j = 1:length(rings_results_debug)
    rings_iter = rings_results_debug{j};
    for k = 1:length(rings_iter)
        pts_2d = rings_iter{k};
        if isempty(pts_2d) || size(pts_2d, 1) < 2
            continue;
        end
        all_paths{end+1} = pts_2d;
    end
end

disp(['从嵌套结构中提取出 ', num2str(length(all_paths)), ' 条路径']);

% 第二步：过滤路径
filtered_paths = filter_paths_simple(all_paths);

% 第三步：转换为双层cell结构
output_cell = cell(1, length(filtered_paths));
for i = 1:length(filtered_paths)
    inner_cell = cell(1, 1);
    inner_cell{1} = filtered_paths{i};
    output_cell{i} = inner_cell;
end

disp(['输出双层cell结构：1×', num2str(length(output_cell)), ' cell']);
disp('访问示例：output_cell{i}{1} 可获取第i条路径的点矩阵');

% 显示示例访问方法
if ~isempty(output_cell)
    disp(' ');
    disp('示例访问路径：');
    disp(['第一条路径点数：', num2str(size(output_cell{1}{1}, 1))]);
    disp(['第二条路径点数：', num2str(size(output_cell{2}{1}, 1))]);
end
end

function filtered_paths = filter_paths_simple(input_paths)
% 简单的路径过滤函数
n_paths = length(input_paths);
if n_paths == 0
    filtered_paths = {};
    return;
end

% 计算路径长度
path_lengths = zeros(1, n_paths);
for i = 1:n_paths
    path = input_paths{i};
    if size(path, 1) < 2
        path_lengths(i) = 0;
    else
        distances = sqrt(sum(diff(path).^2, 2));
        path_lengths(i) = sum(distances);
    end
end

% 标记要保留的路径
keep_path = true(1, n_paths);
threshold = 0.01;

% 比较所有路径对
for i = 1:n_paths
    if ~keep_path(i)
        continue;
    end
    
    for j = i+1:n_paths
        if ~keep_path(j)
            continue;
        end
        
        % 检查是否有共同点
        if has_common_points_simple(input_paths{i}, input_paths{j}, threshold)
            if path_lengths(i) >= path_lengths(j)
                keep_path(j) = false;
            else
                keep_path(i) = false;
                break;
            end
        end
    end
end

filtered_paths = input_paths(keep_path);
disp(['过滤后保留 ', num2str(sum(keep_path)), ' 条路径']);
end

function common = has_common_points_simple(path1, path2, threshold)
% 检查共同点的简化函数
common = false;
for i = 1:size(path1, 1)
    for j = 1:size(path2, 1)
        dx = path1(i,1) - path2(j,1);
        dy = path1(i,2) - path2(j,2);
        if sqrt(dx^2 + dy^2) < threshold
            common = true;
            return;
        end
    end
end
end
function all_rings_filtered = filter_intersecting_paths(all_rings_results)
    fprintf('--------------------------------------------------\n');
    fprintf('开始执行反向交叉检查（裁剪模式）...\n');
    
    all_rings_filtered = all_rings_results;
    num_layers = length(all_rings_filtered);
    trimmed_count = 0;
    deleted_count = 0;

    for i = num_layers:-1:2
        current_layer_rings = all_rings_filtered{i};
        if isempty(current_layer_rings), continue; end
        
        % 收集所有外层路径，合并为一个polyshape作为"合法区域"
        outer_polys = polyshape.empty;
        for j = 1:(i-1)
            prev_layer_rings = all_rings_filtered{j};
            for m = 1:length(prev_layer_rings)
                prev_pts = prev_layer_rings{m};
                if isempty(prev_pts) || size(prev_pts,1) < 3, continue; end
                try
                    % 外层路径围成的区域就是内层路径的合法范围
                    p = polyshape(prev_pts(:,1), prev_pts(:,2));
                    if isempty(outer_polys)
                        outer_polys = p;
                    else
                        outer_polys = union(outer_polys, p);
                    end
                catch
                end
            end
        end
        
        new_rings = {};
        for k = 1:length(current_layer_rings)
            curr_pts = current_layer_rings{k};
            if isempty(curr_pts) || size(curr_pts,1) < 3
                continue;
            end
            
            [xi, ~] = polyxpoly(curr_pts(:,1), curr_pts(:,2), ...
                outer_polys.Vertices(:,1), outer_polys.Vertices(:,2));
            
            if isempty(xi)
                % 无交叉，直接保留
                new_rings{end+1} = curr_pts;
            else
                % 有交叉：裁剪，只保留在外层路径内部的段
                try
                    curr_poly = polyshape(curr_pts(:,1), curr_pts(:,2));
                    clipped = intersect(curr_poly, outer_polys);
                    if area(clipped) > 0.1
                        clipped_regions = regions(clipped);
                        for r = 1:length(clipped_regions)
                            v = clipped_regions(r).Vertices;
                            if size(v,1) >= 3
                                % 闭合
                                if ~isequal(v(1,:), v(end,:))
                                    v = [v; v(1,:)];
                                end
                                new_rings{end+1} = v;
                                trimmed_count = trimmed_count + 1;
                            end
                        end
                    else
                        deleted_count = deleted_count + 1;
                    end
                catch
                    % polyshape操作失败，丢弃
                    deleted_count = deleted_count + 1;
                end
            end
        end
        
        all_rings_filtered{i} = new_rings;
    end
    
    fprintf('检查完成。裁剪了 %d 个路径环，删除了 %d 个无效环。\n', trimmed_count, deleted_count);
    fprintf('--------------------------------------------------\n');
end