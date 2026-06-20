function plot_offset_curved_layer_v3(with_text)
% PLOT_OFFSET_CURVED_LAYER_V3
%   复现 (c) Offset-based curved layer generation 概念图.
%   主图: 偏置切片视图 + 贯穿法线箭头
%   插图(a): Per-point offset along surface normal
%   插图(b): Self-intersection clipping (swallowtail + cusp 删除)
%
%   两个版本:
%     plot_offset_curved_layer_v3(true)   - 含所有文字
%     plot_offset_curved_layer_v3(false)  - 纯净版, 仅图形元素
%   无参数调用则两个版本都生成.

    if nargin < 1
        plot_offset_curved_layer_v3(true);
        plot_offset_curved_layer_v3(false);
        return;
    end

    % ============ 颜色 ============
    C_BASE   = [30  90 149]/255;
    C_LIGHT  = [122 184 220;  156 202 227;
                188 224 240;  214 235 245] / 255;
    C_NOTE   = [0.69 0.20 0.18];
    C_GRAY   = [0.55 0.55 0.55];
    C_NORMAL = [0.13 0.13 0.13];

    if with_text, suffix = 'full'; else, suffix = 'blank'; end

    fig = figure('Color','w','Units','pixels', ...
                 'Position',[50 50 1400 900], ...
                 'Name',['Offset curved layer (' suffix ')'], ...
                 'NumberTitle','off');

    %% =========================================================
    %  主图 (左) - 偏置切片视图
    %% =========================================================
    ax = axes('Parent',fig,'Position',[0.04 0.04 0.60 0.92]);
    hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');

    % --- 基准面 + 法线 ---
    x  = linspace(-1.6, 1.6, 400)';
    f0 = 0.55*cos(0.55*pi*x) + 0.06*sin(0.9*pi*x);
    fxv = gradient(f0, x);
    n_len = sqrt(fxv.^2 + 1);
    nx_unit = -fxv ./ n_len;
    nz_unit =  1.0  ./ n_len;
    % soft sigmoid clamp (与 v6 代码一致)
    nz_floor = 0.20;  sharp = 15.0;
    xd = sharp*(nz_unit - nz_floor);
    sp = zeros(size(xd));
    big = xd>20; mid = xd>-20 & xd<=20;
    sp(big) = xd(big);
    sp(mid) = log1p(exp(xd(mid)));
    nz_c = nz_floor + sp/sharp;

    offs_up = [0.18 0.36 0.54 0.72 0.90];
    offs_dn = [-0.18 -0.36 -0.54 -0.72 -0.90];

    % 远→近画偏置层
    [~,ou] = sort(abs(offs_up),'descend');
    for k = ou
        z = f0 + offs_up(k)*nz_c;
        c = C_LIGHT(min(k,4), :);
        plot(ax, x, z, 'Color',c, 'LineWidth',2.0);
    end
    [~,od] = sort(abs(offs_dn),'descend');
    for k = od
        z = f0 + offs_dn(k)*nz_c;
        c = C_LIGHT(min(k,4), :);
        plot(ax, x, z, 'Color',c, 'LineWidth',2.0);
    end
    plot(ax, x, f0, 'Color',C_BASE,'LineWidth',4.5);

    % --- voxel 方块 ---
    hw = 0.028;  hh = 0.045;
    for k = 1:length(offs_up)
        draw_squares(ax, x, f0 + offs_up(k)*nz_c, ...
                     C_LIGHT(min(k,4),:), C_BASE, 11, hw, hh);
    end
    for k = 1:length(offs_dn)
        draw_squares(ax, x, f0 + offs_dn(k)*nz_c, ...
                     C_LIGHT(min(k,4),:), C_BASE, 11, hw, hh);
    end
    draw_squares(ax, x, f0, C_BASE, C_BASE, 11, hw, hh);

    % --- 贯穿法线 (5 条虚线 + 上端箭头) ---
    L_pos = abs(offs_up(2));   % +2 偏置层
    L_neg = abs(offs_dn(2));   % -2 偏置层
    norm_idxs = [60 140 200 260 340];
    for i = norm_idxs
        x0 = x(i);  z0 = f0(i);
        nx = nx_unit(i);  nz = nz_unit(i);
        % 虚线段
        plot(ax, [x0 - L_neg*nx, x0 + L_pos*nx], ...
                  [z0 - L_neg*nz, z0 + L_pos*nz], ...
             'Color',C_NORMAL,'LineWidth',1.0,'LineStyle','--');
        % 上端箭头头
        draw_arrow_head(ax, x0 + L_pos*nx, z0 + L_pos*nz, ...
                            nx, nz, C_NORMAL, 0.055, 0.030);
    end

    %% ---------- 标注 (仅 with_text) ----------
    if with_text
        % Δ_offset
        it = 170;
        z1 = f0(it) + offs_up(end-1)*nz_c(it);
        z2 = f0(it) + offs_up(end)  *nz_c(it);
        draw_double_arrow(ax, x(it)-0.18, z1, x(it)-0.18, z2, 'k', 1.3);
        text(ax, x(it)-0.24, (z1+z2)/2, '$\Delta_{\mathrm{offset}}$', ...
             'Interpreter','latex','FontSize',13, ...
             'HorizontalAlignment','right','VerticalAlignment','middle');

        % n̂ 标注 (主图中)
        i_lab = norm_idxs(3);
        xl = x(i_lab); zl = f0(i_lab);
        nx = nx_unit(i_lab); nz = nz_unit(i_lab);
        text(ax, xl + L_pos*nx*0.7 + 0.10, zl + L_pos*nz*0.7 + 0.02, ...
             '$\hat n(x,y)$', 'Interpreter','latex', ...
             'FontSize',12, 'Color',C_NORMAL, 'FontAngle','italic');

        % 右侧 offset
        ir = length(x)-25;
        draw_double_arrow(ax, x(ir)+0.05, f0(ir)+offs_up(1)*nz_c(ir), ...
                              x(ir)+0.05, f0(ir)+offs_up(end)*nz_c(ir),'k',1.1);
        text(ax, x(ir)+0.10, f0(ir)+offs_up(3)*nz_c(ir), ...
             sprintf('offset\n+1, +2, ...'), 'FontSize',10, ...
             'HorizontalAlignment','left','VerticalAlignment','middle');
        draw_double_arrow(ax, x(ir)+0.05, f0(ir)+offs_dn(1)*nz_c(ir), ...
                              x(ir)+0.05, f0(ir)+offs_dn(end)*nz_c(ir),'k',1.1);
        text(ax, x(ir)+0.10, f0(ir)+offs_dn(3)*nz_c(ir), ...
             sprintf('offset\n-1, -2, ...'), 'FontSize',10, ...
             'HorizontalAlignment','left','VerticalAlignment','middle');

        % 左侧 offset
        il = 25;
        draw_double_arrow(ax, x(il)-0.05, f0(il)+offs_up(1)*nz_c(il), ...
                              x(il)-0.05, f0(il)+offs_up(end)*nz_c(il),'k',1.1);
        text(ax, x(il)-0.10, f0(il)+offs_up(3)*nz_c(il), ...
             sprintf('offset\n+1, +2, ...'), 'FontSize',10, ...
             'HorizontalAlignment','right','VerticalAlignment','middle');
        draw_double_arrow(ax, x(il)-0.05, f0(il)+offs_dn(1)*nz_c(il), ...
                              x(il)-0.05, f0(il)+offs_dn(end)*nz_c(il),'k',1.1);
        text(ax, x(il)-0.10, f0(il)+offs_dn(3)*nz_c(il), ...
             sprintf('offset\n-1, -2, ...'), 'FontSize',10, ...
             'HorizontalAlignment','right','VerticalAlignment','middle');

        % 100% voxel coverage
        draw_single_arrow(ax, -0.10, -1.55, x(140), f0(140)+offs_dn(end-1)*nz_c(140), 'k',1.2);
        draw_single_arrow(ax,  0.10, -1.55, x(260), f0(260)+offs_dn(end-1)*nz_c(260), 'k',1.2);
        text(ax, 0, -1.70, '100\% voxel coverage', ...
             'Interpreter','latex','FontSize',13,'FontWeight','bold', ...
             'HorizontalAlignment','center','VerticalAlignment','top');

        % 标题
        text(ax, 0, 2.10, '(c) Offset-based curved layer generation', ...
             'FontSize',15,'FontWeight','bold','HorizontalAlignment','center');
        text(ax, 0, 1.92, ...
             'each point: $(x,y,z) \rightarrow (x,y,z)+k\,\Delta_{\mathrm{off}}\,\hat n$', ...
             'Interpreter','latex','FontSize',11, ...
             'Color',[0.3 0.3 0.3], 'HorizontalAlignment','center', ...
             'FontAngle','italic');
    end

    % --- 入箭头 (灰色块状) - 两个版本都画 ---
    arx = [-2.25 -1.85 -1.85 -1.70 -1.85 -1.85 -2.25];
    ary = [ 0.10  0.10  0.25  0.00 -0.25 -0.10 -0.10];
    patch(ax, arx, ary, C_GRAY, 'EdgeColor','none');
    if with_text
        text(ax, -2.05, 0.62, sprintf('Surface\noptim-\nization'), ...
             'FontSize',11,'HorizontalAlignment','center','VerticalAlignment','bottom');
    end

    xlim(ax,[-2.6 2.3]); ylim(ax,[-2.0 2.3]);

    %% =========================================================
    %  插图 (a) Per-point offset along surface normal (右上)
    %% =========================================================
    ax_a = axes('Parent',fig,'Position',[0.66 0.55 0.33 0.41]);
    hold(ax_a,'on'); axis(ax_a,'equal'); axis(ax_a,'off');

    xs = linspace(-1, 1, 200)';
    fs = 0.40*cos(0.55*pi*xs);
    fxs = gradient(fs, xs);
    n_len_s = sqrt(fxs.^2 + 1);
    nxu_s = -fxs ./ n_len_s;
    nzu_s =  1.0  ./ n_len_s;
    L1 = 0.30;

    % 偏置层
    for sg = [-2 -1 1 2]
        z = fs + sg*L1*nzu_s;
        c = C_LIGHT(abs(sg), :);
        plot(ax_a, xs, z, 'Color',c, 'LineWidth',2.0);
    end
    plot(ax_a, xs, fs, 'Color',C_BASE, 'LineWidth',3.0);

    % 法线箭头
    for i = round(linspace(20,180,7))
        x0 = xs(i); z0 = fs(i);
        x1 = x0 - 2*L1*nxu_s(i);  z1 = z0 - 2*L1*nzu_s(i);
        x2 = x0 + 2*L1*nxu_s(i);  z2 = z0 + 2*L1*nzu_s(i);
        plot(ax_a, [x1 x2], [z1 z2], 'Color',C_NORMAL,'LineWidth',1.3);
        draw_arrow_head(ax_a, x2, z2, nxu_s(i), nzu_s(i), C_NORMAL, 0.06, 0.035);
    end

    xlim(ax_a,[-1.15 1.15]); ylim(ax_a,[-1.0 1.1]);

    if with_text
        title(ax_a, '(a) Per-point offset along surface normal', ...
              'FontSize',11.5,'FontWeight','bold','Units','normalized', ...
              'Position',[0.0 1.02 0],'HorizontalAlignment','left');
        text(ax_a, 0, -0.95, ...
             'each point moves along its own $\hat n(x,y)$', ...
             'Interpreter','latex','FontSize',9.5, ...
             'Color',C_BASE,'HorizontalAlignment','center','FontAngle','italic');
        i_lab = 100;
        text(ax_a, 0.55, 0.85, '$\hat n$', ...
             'Interpreter','latex','FontSize',14, ...
             'Color',C_NORMAL,'FontWeight','bold');
        plot(ax_a, [0.55, xs(i_lab)+1.6*L1*nxu_s(i_lab)], ...
                    [0.78, fs(i_lab)+1.6*L1*nzu_s(i_lab)], ...
             'Color',C_NORMAL,'LineWidth',0.7);
    end

    %% =========================================================
    %  插图 (b) Self-intersection clipping (右下)
    %% =========================================================
    ax_b = axes('Parent',fig,'Position',[0.66 0.07 0.33 0.42]);
    hold(ax_b,'on'); axis(ax_b,'equal'); axis(ax_b,'off');

    % 基准面 (尖锐凸起)
    s = linspace(-1, 1, 500)';
    sig2 = 0.12;
    f_s  = 0.55*exp(-s.^2/sig2);
    fx_s = -2*s/sig2 .* f_s;
    n_len2 = sqrt(fx_s.^2 + 1);
    n_x_arr = -fx_s ./ n_len2;
    n_z_arr =  1.0  ./ n_len2;

    % 沿 -n 方向偏置 (向下)
    off_d = 0.62;
    x_raw = s - off_d*n_x_arr;
    z_raw = f_s - off_d*n_z_arr;

    % --- 找 dx_raw/ds 变号位置 → 自交点 sa, sb ---
    ds_arr = diff(x_raw);
    sgn_arr = sign(ds_arr);
    sc_idx = find(diff(sgn_arr) ~= 0);
    has_swallowtail = length(sc_idx) >= 2;

    % --- 下包络 (clipped) ---
    Nxg = 300;
    x_grid = linspace(min(x_raw), max(x_raw), Nxg)';
    if has_swallowtail
        sa = sc_idx(1);    sb = sc_idx(end);
        lx = x_raw(1:sa+1); lz = z_raw(1:sa+1);
        rx = x_raw(sb:end); rz = z_raw(sb:end);
        if lx(end) < lx(1), lx = flip(lx); lz = flip(lz); end
        if rx(end) < rx(1), rx = flip(rx); rz = flip(rz); end
        % 去重 (interp1 要求 x 严格单调)
        [lx, iL] = unique(lx); lz = lz(iL);
        [rx, iR] = unique(rx); rz = rz(iR);
        z_l = interp1(lx, lz, x_grid, 'linear', Inf);
        z_r = interp1(rx, rz, x_grid, 'linear', Inf);
        z_clip = min(z_l, z_r);
    else
        z_clip = interp1(x_raw, z_raw, x_grid, 'linear', 'extrap');
    end

    % --- 阴影 (被删除 swallowtail) ---
    if has_swallowtail
        % swallowtail 段 (raw)
        raw_seg_x = x_raw(sa:sb);
        raw_seg_z = z_raw(sa:sb);
        x_lo = min(x_raw(sa), x_raw(sb));
        x_hi = max(x_raw(sa), x_raw(sb));
        clip_msk = x_grid >= x_lo & x_grid <= x_hi;
        clip_seg_x = x_grid(clip_msk);
        clip_seg_z = z_clip(clip_msk);
        poly_x = [raw_seg_x;  flip(clip_seg_x)];
        poly_z = [raw_seg_z;  flip(clip_seg_z)];
        patch(ax_b, poly_x, poly_z, C_NOTE, ...
              'FaceAlpha', 0.22, 'EdgeColor','none');
    end

    % --- 基准面 ---
    plot(ax_b, s, f_s, 'Color',C_BASE, 'LineWidth',2.5);

    % --- 法线 (灰色点线) ---
    for i = round(linspace(40, 460, 9))
        plot(ax_b, [s(i), s(i) - off_d*1.05*n_x_arr(i)], ...
                    [f_s(i), f_s(i) - off_d*1.05*n_z_arr(i)], ...
             'Color',[0.4 0.4 0.4],'LineWidth',0.8,'LineStyle',':');
    end

    % --- raw 自交曲线 (突出 swallowtail) ---
    if has_swallowtail
        plot(ax_b, x_raw(sa:sb), z_raw(sa:sb), ...
             'Color',C_NOTE,'LineWidth',1.8,'LineStyle','--');
        % 两端区段 (淡色)
        plot(ax_b, x_raw(1:sa), z_raw(1:sa), ...
             'Color',C_NOTE,'LineWidth',0.8,'LineStyle','--');
        plot(ax_b, x_raw(sb:end), z_raw(sb:end), ...
             'Color',C_NOTE,'LineWidth',0.8,'LineStyle','--');
    else
        plot(ax_b, x_raw, z_raw, ...
             'Color',C_NOTE,'LineWidth',1.8,'LineStyle','--');
    end

    % --- clipped 下包络 (蓝实线) ---
    plot(ax_b, x_grid, z_clip, 'Color',C_BASE, 'LineWidth',2.4);

    % --- 自交点 × 标记 ---
    if has_swallowtail
        plot(ax_b, x_raw(sa), z_raw(sa), 'x', ...
             'Color',C_NOTE, 'MarkerSize',11, 'LineWidth',2.4);
        plot(ax_b, x_raw(sb), z_raw(sb), 'x', ...
             'Color',C_NOTE, 'MarkerSize',11, 'LineWidth',2.4);
    end

    xlim(ax_b,[-1.05 1.05]); ylim(ax_b,[-1.20 0.85]);

    if with_text
        title(ax_b, '(b) Self-intersection clipping (downward offset)', ...
              'FontSize',11.5,'FontWeight','bold','Units','normalized', ...
              'Position',[0.0 1.02 0],'HorizontalAlignment','left');

        % raw 注释
        text(ax_b, 0.65, 0.50, sprintf('raw normal offset\n(self-crossing)'), ...
             'FontSize',9, 'Color',C_NOTE, 'FontWeight','bold', ...
             'HorizontalAlignment','center');
        if has_swallowtail
            i_mid = round((sa+sb)/2);
            plot(ax_b, [0.45, x_raw(i_mid)], [0.42, z_raw(i_mid)], ...
                 'Color',C_NOTE,'LineWidth',0.9);
            draw_arrow_head(ax_b, x_raw(i_mid), z_raw(i_mid), ...
                            x_raw(i_mid)-0.45, z_raw(i_mid)-0.42, ...
                            C_NOTE, 0.04, 0.025);
        end

        % clipped 注释
        text(ax_b, -0.85, -1.10, sprintf('clipped: lower envelope\n(kept after clip)'), ...
             'FontSize',9, 'Color',C_BASE, 'FontWeight','bold', ...
             'HorizontalAlignment','center');
        [~, imin] = min(z_clip);
        plot(ax_b, [-0.55, x_grid(imin)], [-1.05, z_clip(imin)], ...
             'Color',C_BASE,'LineWidth',0.9);
        draw_arrow_head(ax_b, x_grid(imin), z_clip(imin), ...
                        x_grid(imin)+0.55, z_clip(imin)+1.05, ...
                        C_BASE, 0.04, 0.025);

        % × 注释
        if has_swallowtail
            text(ax_b, -0.75, 0.55, '$\times$ self-cross point', ...
                 'Interpreter','latex','FontSize',9,'Color',C_NOTE);
            plot(ax_b, [-0.45, x_raw(sa)], [0.48, z_raw(sa)], ...
                 'Color',C_NOTE,'LineWidth',0.8);
            draw_arrow_head(ax_b, x_raw(sa), z_raw(sa), ...
                            x_raw(sa)+0.45, z_raw(sa)-0.48, ...
                            C_NOTE, 0.035, 0.022);
        end

        text(ax_b, 0, -1.18, ...
             'remove the upper "swallowtail", keep the lower envelope', ...
             'FontSize',9,'Color',[0.2 0.2 0.2],'FontAngle','italic', ...
             'HorizontalAlignment','center');
    end

    %% 导出
    exportgraphics(fig, ...
        sprintf('offset_curved_layer_v3_%s.png', suffix), ...
        'Resolution', 300);
end

%% =====================================================================
%% 辅助函数
%% =====================================================================
function draw_squares(ax, x_arr, z_arr, face_color, edge_color, n, hw, hh)
    idxs = round(linspace(20, length(x_arr)-20, n));
    for k = 1:length(idxs)
        i = idxs(k);
        rectangle(ax, 'Position',[x_arr(i)-hw, z_arr(i)-hh, 2*hw, 2*hh], ...
                  'FaceColor',face_color,'EdgeColor',edge_color, ...
                  'LineWidth',0.6,'Curvature',[0 0]);
    end
end

function draw_arrow_head(ax, xt, yt, dx, dy, color, head_len, head_wid)
% 在 (xt, yt) 处沿 (dx, dy) 画三角箭头
    L = hypot(dx, dy);
    if L < 1e-12, return; end
    ux = dx/L;  uy = dy/L;
    bx = xt - head_len*ux;  by = yt - head_len*uy;
    px = -uy;  py = ux;
    p1x = bx + head_wid*px;  p1y = by + head_wid*py;
    p2x = bx - head_wid*px;  p2y = by - head_wid*py;
    patch(ax, [xt p1x p2x], [yt p1y p2y], color, 'EdgeColor','none');
end

function draw_single_arrow(ax, x1, y1, x2, y2, color, lw)
    plot(ax, [x1 x2], [y1 y2], 'Color',color, 'LineWidth',lw);
    draw_arrow_head(ax, x2, y2, x2-x1, y2-y1, color, 0.075, 0.040);
end

function draw_double_arrow(ax, x1, y1, x2, y2, color, lw)
    plot(ax, [x1 x2], [y1 y2], 'Color',color, 'LineWidth',lw);
    draw_arrow_head(ax, x2, y2, x2-x1, y2-y1, color, 0.075, 0.040);
    draw_arrow_head(ax, x1, y1, x1-x2, y1-y2, color, 0.075, 0.040);
end