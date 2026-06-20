function plot_streamline_partitioning_v6(with_text)
% PLOT_STREAMLINE_PARTITIONING_V6
%   复现 (d) Streamline generation and region partitioning.
%
%   关键约束 (用户要求):
%     - 流线两端严格在外轮廓上, 无任何内部种子点
%     - 三个 region 颜色清晰区分 (青绿 / 紫 / 橙)
%     - 无左侧入箭头, 无内部红点
%
%   数学约束: 要满足"起点在轮廓 + 3 个 region", 必须用 2 条不共点的贯穿流线
%   (3 条共一端点的流线 + 外轮廓 = 4 个 region; 这是 V-E+F=2 的必然结果)
%
%   每条流线: 从内部某点向 init_dir 双向积分到外轮廓
%   2 条流线: 上半 +55°, 下半 -55°. 切出 3 个 region (上/中/下)

    if nargin < 1
        plot_streamline_partitioning_v6(true);
        plot_streamline_partitioning_v6(false);
        return;
    end

    %% ============ 颜色 ============
    C_REGION1  = [191 224 224]/255;
    C_REGION2  = [216 199 224]/255;
    C_REGION3  = [245 213 187]/255;
    C_BOUNDARY = [0.05 0.05 0.05];
    C_STREAM   = [200  40  28]/255;
    C_VECTOR   = [119 132 150]/255;
    C_VEC_OUT  = [184 191 201]/255;
    C_BG       = [0.98 0.98 0.98];

    if with_text, suffix = 'full'; else, suffix = 'blank'; end
    PLOT_LO = -1.5;  PLOT_HI = 1.5;

    fig = figure('Color','w','Units','pixels', ...
                 'Position',[80 80 820 820], ...
                 'Name',['Streamline partitioning v6 (' suffix ')'], ...
                 'NumberTitle','off');
    ax = axes('Parent',fig,'Position',[0.04 0.04 0.92 0.92]);
    hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');

    rectangle(ax, 'Position',[PLOT_LO PLOT_LO PLOT_HI-PLOT_LO PLOT_HI-PLOT_LO], ...
              'FaceColor',C_BG, 'EdgeColor','k', 'LineWidth',1.5);

    %% ============ 1. 不规则区域 ============
    ctrl_theta = [0, 0.6, 1.2, 1.9, 2.6, 3.2, 3.9, 4.5, 5.2, 5.8];
    ctrl_r     = [1.05, 0.95, 1.10, 1.00, 0.95, 0.85, 1.00, 1.10, 0.90, 1.05];
    cx = ctrl_r .* cos(ctrl_theta);
    cy = ctrl_r .* sin(ctrl_theta);
    cx = [cx, cx(1)];   cy = [cy, cy(1)];
    pp_x = csape(1:length(cx), cx, 'periodic');
    pp_y = csape(1:length(cy), cy, 'periodic');
    u_pts = linspace(1, length(cx), 400);
    bx = fnval(pp_x, u_pts);
    by = fnval(pp_y, u_pts);

    %% ============ 2. 应力场 ============
    nelx = 140;  nely = 140;
    xx = linspace(PLOT_LO, PLOT_HI, nelx);
    yy = linspace(PLOT_LO, PLOT_HI, nely);
    [X, Y] = meshgrid(xx, yy);

    t_sharp = pi/3 * tanh(Y/0.20) + 0.15 * sin(pi*X/1.5);
    t_sharp(t_sharp > pi/2)  = t_sharp(t_sharp > pi/2)  - pi;
    t_sharp(t_sharp < -pi/2) = t_sharp(t_sharp < -pi/2) + pi;
    Cc = imgaussfilt(cos(t_sharp), 3);
    Ss = imgaussfilt(sin(t_sharp), 3);
    t = atan2(Ss, Cc);
    t(t > pi/2)  = t(t > pi/2)  - pi;
    t(t < -pi/2) = t(t < -pi/2) + pi;
    U_field = cos(t);  V_field = sin(t);
    flip = U_field < 0;
    U_field(flip) = -U_field(flip);
    V_field(flip) = -V_field(flip);

    %% ============ 3. 流线 (2 条贯穿流线, 两端在外轮廓) ============
    F_U = griddedInterpolant({yy, xx}, U_field, 'linear', 'nearest');
    F_V = griddedInterpolant({yy, xx}, V_field, 'linear', 'nearest');

    function pts = integrate_one_way(x0, y0, init_dir)
        max_len = 4.0;  step = 0.018;
        pts = [x0, y0];
        x = x0;  y = y0;
        prev_u = init_dir(1);  prev_v = init_dir(2);
        s_acc = 0;
        while s_acc < max_len
            if ~inpolygon(x, y, bx, by) && s_acc > 0.05
                break;
            end
            ux = F_U(y, x);   vx = F_V(y, x);
            if prev_u*ux + prev_v*vx < 0
                ux = -ux;  vx = -vx;
            end
            x = x + step*ux;
            y = y + step*vx;
            pts(end+1, :) = [x, y]; %#ok<AGROW>
            prev_u = ux;  prev_v = vx;
            s_acc = s_acc + step;
        end
    end

    function pts = trace_full(x0, y0, init_dir)
        fwd = integrate_one_way(x0, y0, init_dir);
        bwd = integrate_one_way(x0, y0, [-init_dir(1), -init_dir(2)]);
        % 反向段倒序 + 正向段
        pts = [flipud(bwd(2:end, :)); fwd];
    end

    function pts = trim_to_boundary(pts)
        n = size(pts, 1);
        si = 1;
        for i = 1:n
            if inpolygon(pts(i,1), pts(i,2), bx, by)
                si = max(i-1, 1);  break;
            end
        end
        ei = n;
        for i = n:-1:1
            if inpolygon(pts(i,1), pts(i,2), bx, by)
                ei = min(i+1, n);  break;
            end
        end
        pts = pts(si:ei, :);
    end

    streams = cell(2, 1);
    streams{1} = trim_to_boundary(trace_full( 0.0,  0.25, [cosd(55),  sind(55) ]));
    streams{2} = trim_to_boundary(trace_full( 0.0, -0.25, [cosd(-55), sind(-55)]));

    %% ============ 4. 用 polyshape 切分成 3 个 region ============
    poly_outer = polyshape(bx, by, 'Simplify', true);
    % 流线作为切刀: 用 polyshape 的 subtract 模拟 split
    % 由于 polyshape 不直接支持 LineString split, 我们手动构造 3 个 region:
    %   Region 1 (上): 流线1 上方 = 外轮廓上半段 + 流线1 反向
    %   Region 3 (下): 流线2 下方 = 外轮廓下半段 + 流线2 反向
    %   Region 2 (中): 剩余 = 流线1 + 外轮廓右段 + 流线2 + 外轮廓左段

    bcen_x = mean(bx);  bcen_y = mean(by);
    bang = atan2(by - bcen_y, bx - bcen_x);

    function ib = find_b_idx(pt)
        ang_p = atan2(pt(2) - bcen_y, pt(1) - bcen_x);
        diffs = abs(mod(bang - ang_p + pi, 2*pi) - pi);
        [~, ib] = min(diffs);
    end

    % 4 个外轮廓端点: 流线1 两端 + 流线2 两端
    s1 = streams{1};  s2 = streams{2};
    iL1 = find_b_idx(s1(1, :));    iR1 = find_b_idx(s1(end, :));
    iL2 = find_b_idx(s2(1, :));    iR2 = find_b_idx(s2(end, :));

    function [arc_x, arc_y] = boundary_arc_ccw(ib_a, ib_b)
        if ib_b > ib_a
            arc_x = bx(ib_a:ib_b);
            arc_y = by(ib_a:ib_b);
        else
            arc_x = [bx(ib_a:end), bx(1:ib_b)];
            arc_y = [by(ib_a:end), by(1:ib_b)];
        end
    end

    % Region 1 (上): 流线1 + 外轮廓段 (iR1 → iL1 ccw, 即流线1 上方那段轮廓)
    [bxa, bya] = boundary_arc_ccw(iR1, iL1);
    r1_x = [s1(:,1)', bxa];
    r1_y = [s1(:,2)', bya];

    % Region 3 (下): 流线2 + 外轮廓段 (iL2 → iR2 ccw, 即流线2 下方那段轮廓)
    [bxa, bya] = boundary_arc_ccw(iL2, iR2);
    r3_x = [bxa, fliplr(s2(:,1)')];
    r3_y = [bya, fliplr(s2(:,2)')];

    % Region 2 (中): 流线1 反向 + 外轮廓段 (iL1 → iL2 ccw, 经过左侧)
    %                + 流线2 + 外轮廓段 (iR2 → iR1 ccw, 经过右侧)
    [bxa1, bya1] = boundary_arc_ccw(iL1, iL2);   % 左侧段
    [bxa2, bya2] = boundary_arc_ccw(iR2, iR1);   % 右侧段
    r2_x = [fliplr(s1(:,1)'), bxa1, s2(:,1)', bxa2];
    r2_y = [fliplr(s1(:,2)'), bya1, s2(:,2)', bya2];

    patch(ax, r1_x, r1_y, C_REGION1, 'FaceAlpha',0.90, 'EdgeColor','none');
    patch(ax, r2_x, r2_y, C_REGION2, 'FaceAlpha',0.90, 'EdgeColor','none');
    patch(ax, r3_x, r3_y, C_REGION3, 'FaceAlpha',0.90, 'EdgeColor','none');

    %% ============ 5. 矢量箭头 ============
    skip = 8;
    Xs = X(1:skip:end, 1:skip:end);
    Ys = Y(1:skip:end, 1:skip:end);
    Us = U_field(1:skip:end, 1:skip:end);
    Vs = V_field(1:skip:end, 1:skip:end);
    inside_q = inpolygon(Xs, Ys, bx, by);
    margin = 0.12;
    not_edge = (Xs > PLOT_LO+margin) & (Xs < PLOT_HI-margin) & ...
               (Ys > PLOT_LO+margin) & (Ys < PLOT_HI-margin);
    arr_len = 0.14;

    mo = (~inside_q) & not_edge;
    Xo_c = Xs(mo) - Us(mo)*arr_len/2;
    Yo_c = Ys(mo) - Vs(mo)*arr_len/2;
    quiver(ax, Xo_c, Yo_c, Us(mo)*arr_len, Vs(mo)*arr_len, 0, ...
           'Color',C_VEC_OUT, 'LineWidth',0.7, ...
           'MaxHeadSize',0.8, 'AutoScale','off');

    mi = inside_q & not_edge;
    Xi_c = Xs(mi) - Us(mi)*arr_len/2;
    Yi_c = Ys(mi) - Vs(mi)*arr_len/2;
    quiver(ax, Xi_c, Yi_c, Us(mi)*arr_len, Vs(mi)*arr_len, 0, ...
           'Color',C_VECTOR, 'LineWidth',0.9, ...
           'MaxHeadSize',0.8, 'AutoScale','off');

    %% ============ 6. 边界 ============
    plot(ax, bx, by, 'Color',C_BOUNDARY, 'LineWidth',2.8);

    %% ============ 7. 流线 (无内部红点, 末端箭头朝外) ============
    for k = 1:2
        s = streams{k};
        plot(ax, s(:,1), s(:,2), 'Color',C_STREAM, 'LineWidth',2.6);
        if size(s,1) >= 4
            dx = s(end,1) - s(end-2,1);
            dy = s(end,2) - s(end-2,2);
            draw_arrow_head(ax, s(end,1), s(end,2), dx, dy, ...
                            C_STREAM, 0.08, 0.045);
        end
    end

    %% ============ 8. 文字 ============
    if with_text
        text(ax, 0, 1.78, ...
             {'(d) Streamline generation','and region partitioning'}, ...
             'FontSize',14.5,'FontWeight','bold', ...
             'HorizontalAlignment','center');
        text(ax,  0.00, 0.75, 'Region 1', 'FontSize',12,'FontWeight','bold', ...
             'HorizontalAlignment','center','Color',[26 64 64]/255);
        text(ax,  0.30, 0.00, 'Region 2', 'FontSize',12,'FontWeight','bold', ...
             'HorizontalAlignment','center','Color',[61 32 80]/255);
        text(ax,  0.00,-0.75, 'Region 3', 'FontSize',12,'FontWeight','bold', ...
             'HorizontalAlignment','center','Color',[109 56 20]/255);
        % 坐标轴
        plot(ax, [-1.78 -1.78], [-1.85 -1.55], 'k-', 'LineWidth',1.3);
        plot(ax, [-1.78 -1.48], [-1.85 -1.85], 'k-', 'LineWidth',1.3);
        draw_arrow_head(ax, -1.78, -1.55, 0, 0.05, [0 0 0], 0.06, 0.030);
        draw_arrow_head(ax, -1.48, -1.85, 0.05, 0, [0 0 0], 0.06, 0.030);
        text(ax, -1.86, -1.48, 'Y', 'FontSize',11, ...
             'HorizontalAlignment','right');
        text(ax, -1.42, -1.85, 'X', 'FontSize',11, ...
             'HorizontalAlignment','left');
    end

    xlim(ax, [-1.90 1.65]);   ylim(ax, [-2.00 2.00]);

    exportgraphics(fig, sprintf('streamline_partitioning_v6_%s.png', suffix), ...
                   'Resolution', 300);
end

function draw_arrow_head(ax, xt, yt, dx, dy, color, head_len, head_wid)
    L = hypot(dx, dy);
    if L < 1e-12, return; end
    ux = dx/L;  uy = dy/L;
    bx_h = xt - head_len*ux;   by_h = yt - head_len*uy;
    px = -uy;   py = ux;
    p1x = bx_h + head_wid*px;  p1y = by_h + head_wid*py;
    p2x = bx_h - head_wid*px;  p2y = by_h - head_wid*py;
    patch(ax, [xt p1x p2x], [yt p1y p2y], color, 'EdgeColor','none');
end