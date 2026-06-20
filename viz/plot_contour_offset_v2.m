function plot_contour_offset_v2(with_text)
% PLOT_CONTOUR_OFFSET_V2
%   复现 (e) Curved-surface-adaptive contour offset.
%   左:  Without f_min - 真实跑 offset_contour_by_normals (f_min ≈ 0)
%        没有自交修复, 展示真实的自交问题
%   中:  With f_min clamping - f_min = 0.5 + 自交修复 (polyshape buffer(0))
%   右:  Surface correction factor - 真实 3D 曲面 + 2D 投影路径
%
%   修复: LaTeX 用 'tex' interpreter + \frac (不用 \dfrac, 旧版 MATLAB 不支持)
%        ✗ ✓ 用 Unicode 字符避免乱码
%
%   两个版本: with_text = true / false. 无参数则两版本都生成.

    if nargin < 1
        plot_contour_offset_v2(true);
        plot_contour_offset_v2(false);
        return;
    end

    %% ============ 颜色 ============
    C_BAD_BG  = [251 228 220]/255;
    C_GOOD_BG = [232 240 251]/255;
    C_OFFSET  = [30  90 149]/255;
    C_BAD     = [200  40  28]/255;
    C_BOUNDARY = [0.05 0.05 0.05];
    C_GRAY    = [0.55 0.55 0.55];
    C_NOTE    = [90 59 26]/255;
    C_BLUES   = [44 110 163;  77 139 191;  110 164 207;
                 139 188 220; 168 207 231] / 255;

    if with_text, suffix = 'full'; else, suffix = 'blank'; end

    fig = figure('Color','w','Units','pixels', ...
                 'Position',[40 80 1400 580], ...
                 'Name',['Contour offset (' suffix ')'], ...
                 'NumberTitle','off');

    %% ============ 不规则区域 ============
    ctrl_theta = [0.0, 0.6, 1.1, 1.7, 2.3, 2.9, 3.3, 3.8, 4.3, 4.9, 5.4, 5.9];
    ctrl_r     = [1.00, 1.05, 0.90, 1.05, 0.95, 0.55, 0.80, 0.95, ...
                  0.85, 1.05, 0.90, 0.95];
    cx = ctrl_r .* cos(ctrl_theta);
    cy = ctrl_r .* sin(ctrl_theta);
    cx = [cx, cx(1)];   cy = [cy, cy(1)];
    pp_x = csape(1:length(cx), cx, 'periodic');
    pp_y = csape(1:length(cy), cy, 'periodic');
    u  = linspace(1, length(cx), 400);
    bx = fnval(pp_x, u);
    by = fnval(pp_y, u);

    contour0 = resample_contour([bx', by'], 200);

    %% ============ 跑两遍偏置 ============
    d_target = 0.15;  n_iter = 5;
    % Without f_min (f_min ~ 0, 不修复自交)
    contours_bad = run_offset_iterations(contour0, d_target, 0.001, n_iter, false);
    % With f_min clamping + 自交修复
    contours_good = run_offset_iterations(contour0, d_target, 0.5, n_iter, true);

    %% ============ 左: Without f_min ============
    ax_l = axes('Parent',fig,'Position',[0.03 0.10 0.30 0.78]);
    hold(ax_l,'on'); axis(ax_l,'equal'); axis(ax_l,'off');
    set(ax_l,'Color',C_BAD_BG);

    plot(ax_l, bx, by, 'Color',C_BOUNDARY, 'LineWidth',2.5);
    for k = 2:length(contours_bad)   % 跳过第一条 (=边界)
        c = contours_bad{k};
        plot(ax_l, c(:,1), c(:,2), '--', 'Color',C_BAD, 'LineWidth',1.4);
    end

    % 入箭头
    arx = [-1.55 -1.27 -1.27 -1.17 -1.27 -1.27 -1.55];
    ary = [ 0.05  0.05  0.15  0.00 -0.15 -0.05 -0.05];
    patch(ax_l, arx, ary, C_GRAY, 'EdgeColor','none');

    if with_text
        text(ax_l, 0.5, 1.40, 'Without {\itf}_{min}', ...
             'Interpreter','tex','FontSize',13,'FontWeight','bold', ...
             'HorizontalAlignment','center');

        % × 标 (自交点 - 在陡峭区中心附近)
        plot(ax_l, 0.45, -0.45, 'x', 'Color',C_BAD, ...
             'MarkerSize',15, 'LineWidth',3.2);
        text(ax_l, -0.40, -0.10, 'self-intersection', ...
             'FontSize',10,'Color',C_NOTE);
        plot(ax_l, [-0.40 0.40], [-0.15 -0.42], '-', ...
             'Color',C_NOTE, 'LineWidth',0.8);
        text(ax_l, 1.25, -1.05, 'steep zone', ...
             'FontSize',10,'Color',C_NOTE);
        plot(ax_l, [1.25 0.65], [-1.00 -0.65], '-', ...
             'Color',C_NOTE, 'LineWidth',0.8);

        % ✗ + 小标题
        text(ax_l, -1.10, -1.35, char(10007), ...
             'FontSize',38,'Color',C_BAD,'FontWeight','bold', ...
             'HorizontalAlignment','center','VerticalAlignment','middle');
        text(ax_l, 0.5, -1.40, 'Without {\itf}_{min}', ...
             'Interpreter','tex','FontSize',11, ...
             'HorizontalAlignment','center','VerticalAlignment','top', ...
             'Color',[0.27 0.27 0.27]);
    else
        text(ax_l, -1.10, -1.35, char(10007), ...
             'FontSize',38,'Color',C_BAD,'FontWeight','bold', ...
             'HorizontalAlignment','center','VerticalAlignment','middle');
    end
    xlim(ax_l,[-1.25 1.55]);  ylim(ax_l,[-1.50 1.55]);

    %% ============ 中: With f_min ============
    ax_m = axes('Parent',fig,'Position',[0.34 0.10 0.30 0.78]);
    hold(ax_m,'on'); axis(ax_m,'equal'); axis(ax_m,'off');
    set(ax_m,'Color',C_GOOD_BG);

    plot(ax_m, bx, by, 'Color',C_BOUNDARY, 'LineWidth',2.5);
    for k = 2:length(contours_good)
        c = contours_good{k};
        col = C_BLUES(min(k-1, size(C_BLUES,1)), :);
        plot(ax_m, c(:,1), c(:,2), '-', 'Color',col, 'LineWidth',2.0);
    end

    if with_text
        text(ax_m, 0.5, 1.40, ...
             'With {\itf}_{min} clamping + 3-tier smoothing', ...
             'Interpreter','tex','FontSize',13,'FontWeight','bold', ...
             'HorizontalAlignment','center');

        % d/2 与 d 标注 (右侧, 引线到外部)
        % 估算位置: 最外层 ~x=1.05, 第一层 (内移 d/2=0.075) ~x=0.975
        x_outer = 1.10;   x_in1 = x_outer - d_target/2;
        x_in2 = x_in1 - d_target;
        plot(ax_m, [x_outer x_in1], [0 0], 'k-', 'LineWidth',1.2);
        draw_arrow_head(ax_m, x_outer, 0,  0.04, 0, [0 0 0], 0.04, 0.025);
        draw_arrow_head(ax_m, x_in1,   0, -0.04, 0, [0 0 0], 0.04, 0.025);
        plot(ax_m, [(x_outer+x_in1)/2, 1.30], [0.05, 0.45], ...
             'k-','LineWidth',0.6);
        text(ax_m, 1.34, 0.50, '$\frac{d}{2}$', ...
             'Interpreter','latex','FontSize',15, ...
             'HorizontalAlignment','left','VerticalAlignment','middle', ...
             'FontWeight','bold');

        plot(ax_m, [x_in2 x_in1], [0 0], 'k-', 'LineWidth',1.2);
        draw_arrow_head(ax_m, x_in1, 0,  0.04, 0, [0 0 0], 0.04, 0.025);
        draw_arrow_head(ax_m, x_in2, 0, -0.04, 0, [0 0 0], 0.04, 0.025);
        plot(ax_m, [(x_in1+x_in2)/2, 1.30], [0.05, 0.10], ...
             'k-','LineWidth',0.6);
        text(ax_m, 1.34, 0.10, '{\itd}', ...
             'Interpreter','tex','FontSize',15,'FontWeight','bold', ...
             'HorizontalAlignment','left','VerticalAlignment','middle');

        text(ax_m, -1.10, -1.35, char(10003), ...
             'FontSize',32,'Color',C_OFFSET,'FontWeight','bold', ...
             'HorizontalAlignment','center','VerticalAlignment','middle');
        text(ax_m, 0.5, -1.40, ...
             'With {\itf}_{min} clamping + 3-tier smoothing', ...
             'Interpreter','tex','FontSize',11, ...
             'HorizontalAlignment','center','VerticalAlignment','top', ...
             'Color',[0.27 0.27 0.27]);
    else
        text(ax_m, -1.10, -1.35, char(10003), ...
             'FontSize',32,'Color',C_OFFSET,'FontWeight','bold', ...
             'HorizontalAlignment','center','VerticalAlignment','middle');
    end
    xlim(ax_m,[-1.25 1.55]);  ylim(ax_m,[-1.50 1.55]);

    %% ============ 右: 3D 曲面 correction factor 插图 ============
    ax_r = axes('Parent',fig,'Position',[0.66 0.08 0.32 0.82]);
    hold(ax_r, 'on');

    % 真实 3D 视图
    [Up, Vp] = meshgrid(linspace(0,1,25), linspace(0,1,25));
    Zp = 0.4*Up + 0.15*Vp - 0.08*Up.^2 + 0.05*sin(2*pi*Up).*Vp;
    surf(ax_r, Up, Vp, Zp, ...
         'FaceColor',[122 184 220]/255, 'FaceAlpha',0.70, ...
         'EdgeColor',C_OFFSET, 'LineWidth',0.3);

    % Δs_2D 段 (z=0)
    p1 = [0.20, 0.30]; p2 = [0.80, 0.30];
    plot3(ax_r, [p1(1) p2(1)], [p1(2) p2(2)], [0 0], ...
          'k-', 'LineWidth',2.0);
    % Δs_3D 段 (曲面上)
    ns = 30;
    xs3 = linspace(p1(1), p2(1), ns);
    ys3 = linspace(p1(2), p2(2), ns);
    zs3 = 0.4*xs3 + 0.15*ys3 - 0.08*xs3.^2 + 0.05*sin(2*pi*xs3).*ys3;
    plot3(ax_r, xs3, ys3, zs3, '-', 'Color',C_OFFSET, 'LineWidth',2.5);
    % 投影虚线 (两端)
    for ii = [1, ns]
        plot3(ax_r, [xs3(ii) xs3(ii)], [ys3(ii) ys3(ii)], [0 zs3(ii)], ...
              '--', 'Color',[0 0 0], 'LineWidth',0.6);
    end

    if with_text
        % Δs_2D, Δs_3D, f 标注
        text(ax_r, 0.5, 0.30, -0.07, ...
             '\Deltas_{2D}', 'Interpreter','tex','FontSize',12, ...
             'HorizontalAlignment','center');
        text(ax_r, 0.55, 0.30, 0.30, ...
             '\Deltas_{3D}', 'Interpreter','tex','FontSize',12, ...
             'Color',C_OFFSET);
        text(ax_r, 0.50, 0.65, 0.32, ...
             '{\itf}', 'Interpreter','tex','FontSize',16, ...
             'FontWeight','bold','Color',C_OFFSET);

        % 标题 + 公式 (用 figure-level text)
        annotation(fig, 'textbox', [0.66 0.88 0.32 0.06], ...
            'String','surface correction factor', ...
            'FontSize',12,'FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','middle', ...
            'EdgeColor','none');
        annotation(fig, 'textbox', [0.66 0.005 0.32 0.07], ...
            'String','$f = \frac{\Delta s_{2D}}{\Delta s_{3D}}$', ...
            'Interpreter','latex', ...
            'FontSize',15,'HorizontalAlignment','center', ...
            'VerticalAlignment','middle','EdgeColor','none');
    end

    % 视角与轴
    view(ax_r, -55, 25);
    axis(ax_r, 'off');
    xlim(ax_r,[0 1]); ylim(ax_r,[0 1]); zlim(ax_r,[0 0.6]);

    % 虚线框 (figure 级 rectangle)
    annotation(fig, 'rectangle', [0.66 0.06 0.32 0.88], ...
        'LineStyle','--','EdgeColor',[0.55 0.55 0.55],'LineWidth',1.2);

    %% ============ 总标题 ============
    if with_text
        annotation(fig, 'textbox', [0.20 0.0 0.30 0.06], ...
            'String','(e) Curved-surface-adaptive contour offset', ...
            'FontSize',14,'FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','middle', ...
            'EdgeColor','none');
    end

    exportgraphics(fig, sprintf('contour_offset_v2_%s.png', suffix), ...
                   'Resolution', 300);
end

%% =====================================================================
%% 偏置算法 (复刻 generate_offset_path2.m 的 offset_contour_by_normals)
%% =====================================================================
function [offset_c, f_vals] = offset_contour_by_normals(contour, d_off, f_min)
% 简化版 (用解析曲面替代 pointCloud_data)
    if ~isequal(contour(1,:), contour(end,:))
        contour(end+1,:) = contour(1,:);
    end
    n = size(contour, 1);
    normals = compute_2d_inward_normals(contour);
    f_vals = zeros(n, 1);

    for i = 1:n
        A_xy = normals(i, :);
        if norm(A_xy) < 1e-12
            f_vals(i) = f_min;  continue;
        end
        A_xy_u = A_xy / norm(A_xy);
        B = surface_normal_analytic(contour(i,1), contour(i,2));
        B_xy = B(1:2);
        dot_p = dot(B_xy, A_xy_u);
        if abs(B(3)) < 1e-12
            f_vals(i) = f_min;
        else
            f_vals(i) = 1.0 / sqrt(1.0 + dot_p^2 / B(3)^2);
            f_vals(i) = max(f_vals(i), f_min);
        end
    end

    % f 平滑 (环形, half_w=3)
    half_w = 3;
    f_smooth = zeros(n, 1);
    for i = 1:n
        idxs = mod((i-half_w:i+half_w) - 1, n) + 1;
        f_smooth(i) = mean(f_vals(idxs));
    end
    f_vals = max(f_smooth, f_min);
    f_vals(end) = f_vals(1);

    % 偏置 (inward 即 + normal, 因为 normals 已经是 inward)
    offset_c = contour + d_off * (f_vals .* normals);

    % 偏置轮廓平滑 (环形, half_c=2)
    half_c = 2;
    smoothed = zeros(size(offset_c));
    for i = 1:n
        idxs = mod((i-half_c:i+half_c) - 1, n) + 1;
        smoothed(i,:) = mean(offset_c(idxs, :), 1);
    end
    smoothed(end,:) = smoothed(1,:);
    offset_c = smoothed;
end

function nrm = compute_2d_inward_normals(contour)
    n = size(contour, 1);
    nrm = zeros(size(contour));
    for i = 1:n
        ip = mod(i, n) + 1;
        im = mod(i - 2, n) + 1;
        tx = contour(ip,1) - contour(im,1);
        ty = contour(ip,2) - contour(im,2);
        L = hypot(tx, ty) + 1e-12;
        nrm(i, :) = [-ty/L, tx/L];   % 左侧法线 (ccw 多边形的 inward)
    end
    % 判定方向
    area = 0.5 * sum(contour(:,1) .* circshift(contour(:,2), -1) - ...
                     circshift(contour(:,1), -1) .* contour(:,2));
    if area < 0,  nrm = -nrm;  end
end

function n = surface_normal_analytic(x, y)
% 解析曲面 z = 0.7*exp(-((x-0.5)^2 + (y+0.45)^2)/0.12) 的法线
    dx = x - 0.5;  dy = y + 0.45;
    z  = 0.7 * exp(-(dx^2 + dy^2) / 0.12);
    dz_dx = z * (-2*dx / 0.12);
    dz_dy = z * (-2*dy / 0.12);
    n = [-dz_dx, -dz_dy, 1.0];
    n = n / norm(n);
end

function out = resample_contour(c, n_pts)
    d = sqrt(sum(diff(c).^2, 2));
    s = [0; cumsum(d)];
    s_new = linspace(0, s(end), n_pts)';
    out = [interp1(s, c(:,1), s_new), interp1(s, c(:,2), s_new)];
end

function contours = run_offset_iterations(c0, d_target, f_min, n_iter, repair)
    contours = {c0};
    cur = c0;
    for k = 1:n_iter
        if k == 1, d_k = d_target/2;  else, d_k = d_target;  end
        try
            [cur, ~] = offset_contour_by_normals(cur, d_k, f_min);
            if repair
                ps = polyshape(cur(:,1), cur(:,2), 'Simplify',true);
                if ps.NumRegions == 0,  break;  end
                if ps.NumRegions > 1
                    rgs = regions(ps);
                    [~, i_max] = max(arrayfun(@area, rgs));
                    ps = rgs(i_max);
                end
                if area(ps) < 1e-3,  break;  end
                vv = ps.Vertices;
                vv = vv(all(~isnan(vv), 2), :);
                cur = vv;
                cur(end+1, :) = cur(1, :);
            end
            cur = resample_contour(cur, 200);
            contours{end+1} = cur;
        catch
            break;
        end
    end
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