function [status, min_dist] = detect_cone_collision(cone_apex, cone_axis, cone_params, path, collision_params)
%% DETECT_CONE_COLLISION 检测锥面与已打印路径的碰撞
%
% 锥面物理模型 (代表喷嘴的锥形区域):
%
%         顶点 (cone_apex) ← 在喷嘴尖端上方
%            \    |    /
%             \   |   /
%              \  |  /     ← 锥面检测区域
%               \ | /
%                \|/
%           喷嘴尖端 (toolc_tip)
%                 |
%                 ↓ 
%           cone_axis (工具Z轴方向)
%
% 输入:
%   cone_apex        - [x,y,z] 锥面顶点位置 (m)
%   cone_axis        - [x,y,z] 锥面轴向 (单位向量，工具Z轴方向，指向喷嘴出口)
%   cone_params      - 锥面参数结构体 (.half_angle, .depth)
%   path             - Nx3 已打印路径 (m)
%   collision_params - 碰撞参数结构体
%
% 输出:
%   status   - 0=安全, 1=风险, 2=碰撞
%   min_dist - 最小距离 (m)

    cone_apex = cone_apex(:)';
    cone_axis = cone_axis(:)';
    cone_axis = cone_axis / norm(cone_axis);
    
    half_angle = cone_params.half_angle;
    depth = cone_params.depth;
    
    min_dist = inf;
    
    if isempty(path) || size(path, 1) < 2
        status = 0;
        return;
    end
    
    % 检测路径上每个点
    for i = 1:size(path, 1)
        d = point_to_cone_distance(path(i,:), cone_apex, cone_axis, half_angle, depth);
        min_dist = min(min_dist, d);
    end
    
    % 检测路径段上的采样点 (提高精度)
    for i = 1:size(path, 1) - 1
        p1 = path(i, :);
        p2 = path(i + 1, :);
        
        % 只检测线段长度较长的情况
        seg_len = norm(p2 - p1);
        if seg_len > 0.1e-3  % > 0.1mm
            num_samples = max(3, ceil(seg_len / 0.5e-3));  % 每0.5mm采样一次
            for t = linspace(0, 1, num_samples)
                pt = p1 + t * (p2 - p1);
                d = point_to_cone_distance(pt, cone_apex, cone_axis, half_angle, depth);
                min_dist = min(min_dist, d);
            end
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

function dist = point_to_cone_distance(point, apex, axis, half_angle, depth)
%% 计算点到锥面的距离
%
% 正值表示点在锥外，负值表示点在锥内
% 返回的是绝对距离

    point = point(:)';
    
    % 从顶点到点的向量
    v = point - apex;
    
    % 在锥轴方向上的投影 (沿axis方向的距离)
    d_ax = dot(v, axis);
    
    % 情况1: 点在顶点"后方" (d_ax < 0)
    % 即点在锥的上方，与顶点的距离
    if d_ax < 0
        dist = norm(v);
        return;
    end
    
    % 情况2: 点超出锥的深度 (d_ax > depth)
    % 点在锥的底部以下
    if d_ax > depth
        % 计算到底面圆盘的距离
        base_center = apex + depth * axis;
        base_radius = depth * tan(half_angle);
        
        v_to_base = point - base_center;
        d_base_ax = dot(v_to_base, axis);  % 点距底面的轴向距离
        v_perp = v_to_base - d_base_ax * axis;  % 垂直于轴的分量
        r = norm(v_perp);
        
        if r <= base_radius
            % 点在底面圆盘的正下方
            dist = d_ax - depth;
        else
            % 点在底面外侧，计算到底面边缘圆的距离
            edge_dir = v_perp / r;
            edge_point = base_center + base_radius * edge_dir;
            dist = norm(point - edge_point);
        end
        return;
    end
    
    % 情况3: 点在锥的高度范围内 (0 <= d_ax <= depth)
    % 计算该高度处锥面的半径
    v_perp = v - d_ax * axis;  % 垂直于轴的分量
    r = norm(v_perp);          % 点到轴的垂直距离
    cone_r = d_ax * tan(half_angle);  % 该高度处锥面的半径
    
    if r <= cone_r
        % 点在锥内 = 喷嘴实体侵入 -> 返回负的穿透量, 使其触发碰撞阈值。
        % FIX: 原来返回正值, 越深入锥内值越大 -> 反被判成"安全"。
        dist = -(cone_r - r) * cos(half_angle);
    else
        % 点在锥外，返回到锥面的最短距离
        dist = (r - cone_r) * cos(half_angle);
    end
end
