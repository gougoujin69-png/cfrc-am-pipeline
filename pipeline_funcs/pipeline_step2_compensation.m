function pipeline_step2_compensation(file_in, file_out, gpr_file, P)
%PIPELINE_STEP2_COMPENSATION 在尖角处插入送丝拖拽补偿点 (碳纤维)
%   遍历 paths_3d 中每条路径, 找出 turn_angle > SHARP_THRESHOLD 的拐点,
%   若该拐点前后均有足够长的直段, 沿外角平分线方向插入一个补偿点,
%   补偿距离由 GPR 模型 (若有) 或默认线性公式给出.
%
%   输入:
%     file_in   - Step 1 输出 mat 路径 (含 paths_3d)
%     file_out  - 输出 mat 路径 (新增字段 modified_paths, compensation_info)
%     gpr_file  - GPR 模型 mat 路径 (含变量 gprMdlv1), 不存在则降级为公式
%     P         - 参数结构体, 字段:
%                   SHARP_THRESHOLD, STRAIGHT_THRESHOLD,
%                   MIN_STRAIGHT_BEFORE, MIN_STRAIGHT_AFTER,
%                   REF_POINT_METHOD, REF_POINT_VALUE,
%                   GPR_T_TEMP, GPR_V_SPEED, GPR_H_HEIGHT

    S = load(file_in);
    all_layers_data = S.all_layers_data;
    n = length(all_layers_data);

    USE_GPR = false; gprMdlv1 = [];
    if exist(gpr_file, 'file')
        tmp = load(gpr_file); gprMdlv1 = tmp.gprMdlv1; USE_GPR = true;
        fprintf('  GPR model loaded\n');
    else
        fprintf('  No GPR model, using default formula\n');
    end

    total_sharp = 0; total_comp = 0;
    for L = 1:n
        if ~isfield(all_layers_data(L), 'paths_3d') || isempty(all_layers_data(L).paths_3d)
            all_layers_data(L).modified_paths = {};
            all_layers_data(L).compensation_info = {};
            continue;
        end
        paths = all_layers_data(L).paths_3d;
        np = length(paths);
        mp = cell(1, np); ci = cell(1, np);

        for pi = 1:np
            op = paths{pi};
            if size(op,1) < 3, mp{pi} = op; ci{pi} = []; continue; end

            [ta, sl, ~] = local_calc_geom(op);
            sharp_idx = find(ta > P.SHARP_THRESHOLD);
            total_sharp = total_sharp + length(sharp_idx);
            cp = [];

            for k = 1:length(sharp_idx)
                idx = sharp_idx(k); ang = ta(idx);
                [hb, lb] = local_check_before(idx, ta, sl, P.STRAIGHT_THRESHOLD, P.MIN_STRAIGHT_BEFORE);
                [ha, la] = local_check_after(idx, ta, sl, P.STRAIGHT_THRESHOLD, P.MIN_STRAIGHT_AFTER);
                if hb && ha
                    [iA, iC] = local_get_ref(idx, size(op,1), sl, P.REF_POINT_METHOD, P.REF_POINT_VALUE);
                    if USE_GPR
                        d = max(0, predict(gprMdlv1, [P.GPR_T_TEMP, P.GPR_V_SPEED, P.GPR_H_HEIGHT, 180 - ang/2]));
                    else
                        d = 0.5 + (ang - 90) / 90 * 1.5;
                    end
                    [cx, cy, cz] = local_comp_point(op, idx, iA, iC, d);
                    cp = [cp; idx, d, cx, cy, cz, ang, lb, la];
                    total_comp = total_comp + 1;
                end
            end

            if ~isempty(cp)
                cp_sorted = sortrows(cp, 1, 'descend');
                np_path = op;
                for q = 1:size(cp_sorted,1)
                    ii = cp_sorted(q, 1);
                    np_path = [np_path(1:ii, :); cp_sorted(q, 3:5); np_path(ii+1:end, :)];
                end
                mp{pi} = np_path;
            else
                mp{pi} = op;
            end
            ci{pi} = cp;
        end
        all_layers_data(L).modified_paths = mp;
        all_layers_data(L).compensation_info = ci;
    end
    save(file_out, 'all_layers_data', '-v7.3');
    fprintf('  Done: %d sharp turns, %d compensated -> %s\n', total_sharp, total_comp, file_out);
end

%% ==================== LOCAL HELPERS ====================

function [ta, sl, dr] = local_calc_geom(path)
%LOCAL_CALC_GEOM 计算路径的转角、段长、方向向量
    n = size(path, 1);
    ta = zeros(n, 1); sl = zeros(n-1, 1); dr = zeros(n-1, 3);
    for i = 1:n-1
        v = path(i+1, :) - path(i, :);
        sl(i) = norm(v);
        if sl(i) > 1e-10, dr(i, :) = v / sl(i); end
    end
    for i = 2:n-1
        if sl(i-1) > 1e-10 && sl(i) > 1e-10
            c = max(-1, min(1, dot(dr(i-1, :), dr(i, :))));
            ta(i) = acosd(c);
        end
    end
end

function [hs, tl] = local_check_before(idx, ta, sl, thr, mn)
%LOCAL_CHECK_BEFORE 检查 idx 之前是否有 >=mn 的连续直段 (转角< thr)
    tl = 0; hs = false;
    for i = idx-1:-1:1
        if i >= 2 && ta(i) >= thr
            if tl >= mn, hs = true; return;
            else, tl = 0; end
        end
        if i <= length(sl), tl = tl + sl(i); end
        if tl >= mn, hs = true; return; end
    end
    if tl >= mn, hs = true; end
end

function [hs, tl] = local_check_after(idx, ta, sl, thr, mn)
%LOCAL_CHECK_AFTER 检查 idx 之后是否有 >=mn 的连续直段
    n = length(ta); tl = 0; hs = false;
    for i = idx+1:n-1
        if ta(i) >= thr
            if tl >= mn, hs = true; return;
            else, tl = 0; end
        end
        if i-1 >= idx && i-1 <= length(sl), tl = tl + sl(i-1); end
        if tl >= mn, hs = true; return; end
    end
    if idx <= length(sl), tl = tl + sl(idx); end
    if tl >= mn, hs = true; end
end

function [iA, iC] = local_get_ref(idx, n, sl, method, value)
%LOCAL_GET_REF 选取外角平分线方向的参考点 A, C
    if strcmp(method, 'points')
        iA = max(1, idx - value); iC = min(n, idx + value);
    else
        d = 0; iA = idx;
        for i = idx-1:-1:1
            if i < length(sl), d = d + sl(i); end
            iA = i; if d >= value, break; end
        end
        d = 0; iC = idx;
        for i = idx:min(length(sl), n-1)
            d = d + sl(i); iC = i + 1;
            if d >= value, break; end
        end
    end
    iA = max(1, iA); iC = min(n, iC);
    if iA == idx && iA > 1, iA = iA - 1; end
    if iC == idx && iC < n, iC = iC + 1; end
end

function [cx, cy, cz] = local_comp_point(path, iB, iA, iC, dist)
%LOCAL_COMP_POINT 沿外角平分线方向, 在 B 点外侧 dist 处生成补偿点
    B = path(iB, :); A = path(iA, :); C = path(iC, :);
    vBA = A - B; vBC = C - B;
    lBA = norm(vBA); lBC = norm(vBC);
    if lBA < 1e-10 || lBC < 1e-10
        cx = B(1); cy = B(2); cz = B(3); return;
    end
    dBA = vBA / lBA; dBC = vBC / lBC;
    eb = -(dBA + dBC); en = norm(eb);
    if en < 1e-10
        eb = [-dBA(2), dBA(1), 0];
        if norm(eb) < 1e-10, eb = [0, -dBA(3), dBA(2)]; end
        en = norm(eb);
    end
    eb = eb / en;
    p = B + dist * eb;
    cx = p(1); cy = p(2); cz = p(3);
end
