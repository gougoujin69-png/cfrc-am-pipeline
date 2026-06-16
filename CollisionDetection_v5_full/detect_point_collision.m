function [status, min_dist, closest_point] = detect_point_collision(point, path, collision_params)
%% DETECT_POINT_COLLISION 检测点与路径的碰撞
%
% 将路径建模为连续的圆柱体，计算点到路径的最小距离
%
% 输入:
%   point            - [x,y,z] 待检测点位置 (m)
%   path             - Nx3 已打印路径点 (m)
%   collision_params - 碰撞参数结构体
%
% 输出:
%   status        - 0=安全, 1=风险, 2=碰撞
%   min_dist      - 最小距离 (m)
%   closest_point - 最近点位置 (m)

    point = point(:)';
    min_dist = inf;
    closest_point = [nan, nan, nan];
    
    if isempty(path) || size(path, 1) < 2
        status = 0;
        return;
    end
    
    % 遍历所有路径段
    for i = 1:size(path, 1) - 1
        p1 = path(i, :);
        p2 = path(i + 1, :);
        
        % 点到线段距离
        [d, cp] = point_to_segment_distance(point, p1, p2);
        
        if d < min_dist
            min_dist = d;
            closest_point = cp;
        end
    end
    
    % 判断状态
    if min_dist < collision_params.warn_dist
        status = 2;  % 碰撞
    elseif min_dist < collision_params.safe_dist
        status = 1;  % 风险
    else
        status = 0;  % 安全
    end
end

function [dist, closest] = point_to_segment_distance(point, seg_start, seg_end)
%% 计算点到线段的距离
    v = seg_end - seg_start;
    w = point - seg_start;
    
    c1 = dot(w, v);
    c2 = dot(v, v);
    
    if c2 < 1e-14
        closest = seg_start;
        dist = norm(point - seg_start);
        return;
    end
    
    t = max(0, min(1, c1 / c2));
    closest = seg_start + t * v;
    dist = norm(point - closest);
end
