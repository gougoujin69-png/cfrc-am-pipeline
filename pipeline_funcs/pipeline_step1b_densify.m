function pipeline_step1b_densify(file_inout, target_carbon, target_resin, gap_thresh)
%PIPELINE_STEP1B_DENSIFY 在相邻同心 loop 之间插入中间路径, 保持线间距
%   切片放大后, 相邻 loop 之间的距离变大. 本模块在每对相邻 loop 之间
%   插入 N 条中间 loop, 让最终线间距 ~= target_spacing.
%
%   算法 (对每对相邻 loop L1, L2):
%     1. 估算平均距离 d_avg (双向最近点距离平均)
%     2. 计算插入条数 N = round(d_avg / target_spacing) - 1
%     3. 如果 N > 0:
%        a. 两条都重采样到相同点数 (等弧长)
%        b. 用 shoelace 公式对齐绕向
%        c. 找最近点对作为起点, 循环移位 L2
%        d. 线性插值生成 N 条中间 loop:
%           L_k = (1-t_k)*L1 + t_k*L2,  t_k = k/(N+1)
%     4. 如果相邻 loop 距离 > gap_thresh, 认为跨独立区域, 不插值
%
%   注意:
%     - 中间 loop 的 Z 是直接线性插值, 可能略偏离曲面;
%       后续 Step 3.1 会用 interp2 把碳纤维路径 Z 投影回曲面.
%     - 本模块同时处理 paths_3d (碳纤维) 和 paths_3dr (树脂),
%       通过两个独立的 target_spacing 控制.
%
%   输入:
%     file_inout    - mat 文件路径 (原地读写)
%     target_carbon - 碳纤维目标线间距 (mm), 设为 0 跳过碳纤维
%     target_resin  - 树脂目标线间距 (mm),   设为 0 跳过树脂
%     gap_thresh    - 跨区域判定阈值 (mm), 相邻 loop 距离超过此值不插值

    S = load(file_inout);
    all_layers_data = S.all_layers_data;
    n = length(all_layers_data);

    total_carbon_added = 0; total_resin_added = 0;

    for L = 1:n
        % --- 处理碳纤维路径 ---
        if target_carbon > 0 && isfield(all_layers_data(L), 'paths_3d')
            [new_paths, added] = densify_loop_group(all_layers_data(L).paths_3d, ...
                target_carbon, gap_thresh);
            all_layers_data(L).paths_3d = new_paths;
            total_carbon_added = total_carbon_added + added;
        end

        % --- 处理树脂路径 ---
        if target_resin > 0 && isfield(all_layers_data(L), 'paths_3dr')
            [new_paths, added] = densify_loop_group(all_layers_data(L).paths_3dr, ...
                target_resin, gap_thresh);
            all_layers_data(L).paths_3dr = new_paths;
            total_resin_added = total_resin_added + added;
        end
    end

    save(file_inout, 'all_layers_data', '-v7.3');
    fprintf('  Carbon loops added: %d (target spacing %.2fmm)\n', total_carbon_added, target_carbon);
    fprintf('  Resin  loops added: %d (target spacing %.2fmm)\n', total_resin_added, target_resin);
    fprintf('  Saved -> %s\n', file_inout);
end

%% ==================== LOCAL HELPERS ====================

function [paths_dense, added] = densify_loop_group(paths, target_spacing, gap_thresh)
%DENSIFY_LOOP_GROUP 对一组同心 loop 做细密化
    added = 0;
    if length(paths) < 2
        paths_dense = paths; return;
    end

    M = 500;  % 重采样点数, 大些保证形状精度
    paths_dense = {};

    for i = 1:length(paths)-1
        L1 = paths{i};
        L2 = paths{i+1};

        % 加入 L1 本身
        paths_dense{end+1} = L1;

        if isempty(L1) || isempty(L2) || size(L1,1) < 3 || size(L2,1) < 3
            continue;
        end

        % 估算平均距离
        d_avg = local_avg_distance(L1, L2);

        % 跨独立区域判断: 距离过大不插值
        if d_avg > gap_thresh
            continue;
        end

        % 计算需要插入的条数
        N = round(d_avg / target_spacing) - 1;
        N = max(0, N);

        if N == 0
            continue;
        end

        % 重采样到等弧长点数 M
        L1r = local_resample_loop(L1, M);
        L2r = local_resample_loop(L2, M);

        % 对齐绕向 (shoelace 判断)
        L2r = local_align_direction(L1r, L2r);

        % 对齐起点 (找 L2 上离 L1(1,:) 最近的点)
        L2r = local_align_start(L1r, L2r);

        % 生成 N 条中间 loop
        for k = 1:N
            t = k / (N + 1);
            L_mid = (1-t) * L1r + t * L2r;
            paths_dense{end+1} = L_mid;
            added = added + 1;
        end
    end

    % 加入最后一条
    paths_dense{end+1} = paths{end};
end

function d = local_avg_distance(L1, L2)
%LOCAL_AVG_DISTANCE 双向最近点距离的平均值 (近似 loop 间距)
    % 用降采样提速 (大 loop 时 pdist2 矩阵会很大)
    s1 = max(1, floor(size(L1,1)/100));
    s2 = max(1, floor(size(L2,1)/100));
    A = L1(1:s1:end, :);  B = L2(1:s2:end, :);
    D = pdist2(A, B);
    d1 = mean(min(D, [], 2));
    d2 = mean(min(D, [], 1));
    d = (d1 + d2) / 2;
end

function Lr = local_resample_loop(L, M)
%LOCAL_RESAMPLE_LOOP 等弧长重采样到 M 个点
    diffs = diff(L);
    seg = sqrt(sum(diffs.^2, 2));
    cd = [0; cumsum(seg)];
    td = cd(end);
    if td < 1e-9
        Lr = repmat(L(1,:), M, 1); return;
    end
    [cdu, um] = unique(cd);
    Lu = L(um, :);
    nq = linspace(0, td, M)';
    Lr = [interp1(cdu, Lu(:,1), nq, 'linear'), ...
          interp1(cdu, Lu(:,2), nq, 'linear'), ...
          interp1(cdu, Lu(:,3), nq, 'linear')];
end

function L2_out = local_align_direction(L1, L2)
%LOCAL_ALIGN_DIRECTION 用 shoelace 判断绕向, 不一致则翻转 L2
    s1 = local_shoelace(L1(:, 1:2));
    s2 = local_shoelace(L2(:, 1:2));
    if sign(s1) ~= sign(s2) && abs(s1) > 1e-9 && abs(s2) > 1e-9
        L2_out = flipud(L2);
    else
        L2_out = L2;
    end
end

function a = local_shoelace(xy)
%LOCAL_SHOELACE 闭合多边形的 signed area (正=逆时针, 负=顺时针)
    x = xy(:,1); y = xy(:,2);
    a = 0.5 * sum(x .* circshift(y, -1) - circshift(x, -1) .* y);
end

function L2_out = local_align_start(L1, L2)
%LOCAL_ALIGN_START 把 L2 循环移位, 让其起点对齐到离 L1(1) 最近的点
    D = sum((L2 - L1(1,:)).^2, 2);
    [~, idx] = min(D);
    if idx == 1
        L2_out = L2;
    else
        L2_out = [L2(idx:end, :); L2(1:idx-1, :)];
    end
end
