function extended_streamlines = extendStreamlinesToContour(streamlines, outer_contours, inner_contours)
    % 修改版：延长流线并删除任何接触到内轮廓的流线

    % 1. 合并所有轮廓用于延长参考
    all_contours = {};
    if ~isempty(outer_contours), all_contours = [all_contours, outer_contours]; end
    if ~isempty(inner_contours), all_contours = [all_contours, inner_contours]; end

    if isempty(all_contours)
        extended_streamlines = streamlines;
        return;
    end

    % 按流线长度排序（从长到短）
    lengths = cellfun(@(s) size(s, 1), streamlines);
    [~, idx] = sort(lengths, 'descend');
    sorted_streamlines = streamlines(idx);
    temp_extended = cell(size(sorted_streamlines));

    all_processed = {};

    % --- 第一阶段：执行延长逻辑 ---
    for i = 1:length(sorted_streamlines)
        s = sorted_streamlines{i};
        if size(s, 1) < 2
            temp_extended{i} = s;
            all_processed{end+1} = s;
            continue;
        end

        % 延长起点
        start_pt = s(1,:);
        next_pt = s(2,:);
        [new_start] = extendToIntersection(start_pt, next_pt, all_contours, all_processed);
        if ~isempty(new_start), s = [new_start; s]; end

        % 延长终点
        end_pt = s(end,:);
        prev_pt = s(end-1,:);
        [new_end] = extendToIntersection(end_pt, prev_pt, all_contours, all_processed);
        if ~isempty(new_end), s = [s; new_end]; end

        temp_extended{i} = s;
        all_processed{end+1} = s;
    end

    % --- 第二阶段：碰撞检测并删除接触内轮廓的流线 ---
    keep_mask = true(size(temp_extended));
    
    if ~isempty(inner_contours)
        for i = 1:length(temp_extended)
            current_s = temp_extended{i};
            has_touch = false;
            
            % 遍历流线的每一段线段
            for seg_i = 1:size(current_s, 1) - 1
                p1 = current_s(seg_i, :);
                p2 = current_s(seg_i+1, :);
                
                % 与每一个内轮廓进行对比
                for c = 1:length(inner_contours)
                    contour = inner_contours{c};
                    for j = 1:size(contour, 1) - 1
                        p3 = contour(j, :);
                        p4 = contour(j+1, :);
                        
                        % 检测两线段是否相交（包含端点接触）
                        if checkSegmentsTouch(p1, p2, p3, p4)
                            has_touch = true;
                            break; 
                        end
                    end
                    if has_touch, break; end
                end
                if has_touch, break; end
            end
            
            if has_touch
                keep_mask(i) = false; % 标记为删除
            end
        end
    end

    % --- 第三阶段：清理并恢复顺序 ---
    final_extended = temp_extended(keep_mask);
    
    % 仅保留未被删除的流线，并根据原始索引重新整理
    extended_streamlines = cell(size(streamlines));
    original_keep_idx = idx(keep_mask);
    for k = 1:length(original_keep_idx)
        extended_streamlines{original_keep_idx(k)} = final_extended{k};
    end
    % 移除空单元
    extended_streamlines = extended_streamlines(~cellfun('isempty', extended_streamlines));
end

function touched = checkSegmentsTouch(p1, p2, p3, p4)
    % 判断两线段是否相交或接触
    touched = false;
    x1 = p1(1); y1 = p1(2); x2 = p2(1); y2 = p2(2);
    x3 = p3(1); y3 = p3(2); x4 = p4(1); y4 = p4(2);
    
    denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1);
    if denom == 0
        % 平行情况：检查是否共线且重叠（可选，此处简化为不处理平行重叠）
        return; 
    end
    
    ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom;
    ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / denom;
    
    % 判定范围：[0, 1] 闭区间表示只要有交点（哪怕是端点）就返回 true
    % 为了防止浮点数精度误差，引入微小的 eps
    eps = 1e-8;
    if ua >= -eps && ua <= 1+eps && ub >= -eps && ub <= 1+eps
        touched = true;
    end
end

% 辅助函数 extendToIntersection 和 lineSegmentIntersect 保持不变...
function intersection_point = extendToIntersection(point, ref_point, contours, processed_streamlines)
    intersection_point = [];

    dir_vec = point - ref_point;
    if norm(dir_vec) < 1e-6
        return;
    end
    dir_vec = dir_vec / norm(dir_vec);

    % 生成一条测试射线
    ray_end = point + dir_vec * 1e3; % 设定足够长的延长线

    min_dist = inf;
    nearest_pt = [];

    % --- 检查与轮廓的交点 ---
    for c = 1:length(contours)
        contour = contours{c};
        for j = 1:size(contour,1)-1
            p3 = contour(j,:);
            p4 = contour(j+1,:);
            [x, y, valid] = lineSegmentIntersect(point(1), point(2), ray_end(1), ray_end(2), p3(1), p3(2), p4(1), p4(2));
            if valid
                dist = norm([x - point(1), y - point(2)]);
                if dist < min_dist
                    min_dist = dist;
                    nearest_pt = [x, y];
                end
            end
        end
    end

    % --- 检查与其他流线的交点 ---
    for k = 1:length(processed_streamlines)
        s = processed_streamlines{k};
        for j = 1:size(s,1)-1
            p3 = s(j,:);
            p4 = s(j+1,:);
            [x, y, valid] = lineSegmentIntersect(point(1), point(2), ray_end(1), ray_end(2), p3(1), p3(2), p4(1), p4(2));
            if valid
                dist = norm([x - point(1), y - point(2)]);
                if dist < min_dist
                    min_dist = dist;
                    nearest_pt = [x, y];
                end
            end
        end
    end

    if ~isempty(nearest_pt)
        intersection_point = nearest_pt;
    end
end
function [x, y, valid] = lineSegmentIntersect(x1, y1, x2, y2, x3, y3, x4, y4)
    valid = false;
    x = NaN; y = NaN;
    denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1);
    if denom == 0, return; end
    ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom;
    ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / denom;
    if ua >= 0 && ub >= 0 && ub <= 1
        x = x1 + ua*(x2 - x1);
        y = y1 + ua*(y2 - y1);
        valid = true;
    end
end
