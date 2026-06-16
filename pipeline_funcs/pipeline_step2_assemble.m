function pipeline_step2_assemble(file_inout, in_field, out_field, adj_thresh, merge_thresh)
%PIPELINE_STEP2_ASSEMBLE 把多条同心 loop 组装成连续螺旋路径
%   合并原 Step2_2 (碳纤维) 和 Step2_3 (树脂) 的逻辑, 通过 in_field/out_field
%   参数控制处理对象, 通过 adj_thresh/merge_thresh 控制聚类阈值.
%
%   算法:
%     A. 用相邻 loop 端点距离 <= adj_thresh 把 loops 分成基础螺旋组,
%        每组反转后顺次拼接 (内 -> 外).
%     B. 用图论连通分量, 把空间上 nearest distance <= merge_thresh 的螺旋
%        归到同一簇, 每簇内部用 "最近点切开重组" 算法融合成一条路径.
%
%   输入:
%     file_inout   - mat 文件路径 (原地读写)
%     in_field     - 读入字段名, 如 'modified_paths' 或 'paths_3dr'
%     out_field    - 输出字段名, 如 'connected_paths' 或 'resin_connected_paths'
%     adj_thresh   - 同一螺旋内相邻 loop 阈值 (mm)
%     merge_thresh - 螺旋之间触发融合的阈值 (mm)

    S = load(file_inout);
    all_layers_data = S.all_layers_data;
    n = length(all_layers_data);

    for L = 1:n
        if ~isfield(all_layers_data(L), in_field), continue; end
        loops = all_layers_data(L).(in_field);
        if isempty(loops)
            all_layers_data(L).(out_field) = {}; continue;
        end
        nl = length(loops);

        % --- A. 相邻 loop 分组 ---
        adj_d = zeros(max(1, nl-1), 1);
        for i = 1:nl-1
            adj_d(i) = local_min_endpoint_dist(loops{i}, loops{i+1});
        end
        groups = {}; curr = 1;
        for i = 1:nl-1
            if adj_d(i) <= adj_thresh
                curr = [curr, i+1];
            else
                groups{end+1} = curr; curr = i+1;
            end
        end
        groups{end+1} = curr;

        % --- 基础螺旋组装 (反转: 内 -> 外) ---
        spirals = cell(1, length(groups));
        for s = 1:length(groups)
            g = fliplr(groups{s}); path = [];
            for idx = g
                if isempty(path), path = loops{idx};
                else, path = [path; loops{idx}]; end
            end
            spirals{s} = path;
        end

        % --- B. 螺旋聚类与融合 ---
        ns = length(spirals);
        if ns > 1
            dm = inf(ns);
            for i = 1:ns
                for j = i+1:ns
                    d = local_nearest_dist(spirals{i}, spirals{j});
                    dm(i, j) = d; dm(j, i) = d;
                end
            end
            adj_mat = dm <= merge_thresh;
            G = graph(adj_mat);
            bins = conncomp(G);
            out = {};
            for c = 1:max(bins)
                m = find(bins == c);
                if length(m) == 1
                    out{end+1} = spirals{m};
                else
                    out{end+1} = local_merge_cluster(spirals(m));
                end
            end
        else
            out = spirals;
        end
        all_layers_data(L).(out_field) = out;
    end

    save(file_inout, 'all_layers_data', '-v7.3');
    fprintf('  Done: %s -> %s\n', in_field, out_field);
end

%% ==================== LOCAL HELPERS ====================

function d = local_min_endpoint_dist(l1, l2)
%LOCAL_MIN_ENDPOINT_DIST 两条 loop 各自端点之间的最小距离 (XY 平面)
    pts1 = [l1(1, :); l1(end, :)];
    pts2 = [l2(1, :); l2(end, :)];
    d = min(min(pdist2(pts1(:, 1:2), pts2(:, 1:2))));
end

function d = local_nearest_dist(p1, p2)
%LOCAL_NEAREST_DIST 两条路径之间任意两点的最小距离 (采样以提速)
    s1 = max(1, floor(size(p1, 1) / 50));
    s2 = max(1, floor(size(p2, 1) / 50));
    D = pdist2(p1(1:s1:end, 1:2), p2(1:s2:end, 1:2));
    d = min(D(:));
end

function [i1, i2] = local_nearest_indices(p1, p2)
%LOCAL_NEAREST_INDICES 两条路径上最近点的索引 (精确, 不采样)
    D = pdist2(p1(:, 1:2), p2(:, 1:2));
    [mv, ri] = min(D, [], 1);
    [~, i2] = min(mv); i1 = ri(i2);
end

function merged = local_merge_cluster(paths)
%LOCAL_MERGE_CLUSTER 在最近点切开重组, 把一簇螺旋融合成一条路径
    np = length(paths);
    used = false(np, 1); merged = paths{1}; used(1) = true;
    for k = 1:np-1
        md = inf; ni = -1; bm = 0; bj = 0;
        for j = 1:np
            if ~used(j)
                [im, ij] = local_nearest_indices(merged, paths{j});
                d = norm(merged(im, 1:2) - paths{j}(ij, 1:2));
                if d < md, md = d; ni = j; bm = im; bj = ij; end
            end
        end
        if ni ~= -1
            pB = paths{ni};
            % 在 pathB 的最近点 bj 处切开, 重排为 bj 起绕一圈
            rB = [pB(bj:end, :); pB(1:bj, :)];
            % 插入到 merged 的最近点 bm 位置
            merged = [merged(1:bm, :); rB; merged(bm:end, :)];
            used(ni) = true;
        end
    end
end
