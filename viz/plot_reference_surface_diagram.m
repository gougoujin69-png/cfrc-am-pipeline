function plot_five_separate_figures()
% PLOT_FIVE_SEPARATE_FIGURES  (v2)
%   分别生成 5 个独立 figure:
%     fig1~fig4: 4 个低频余弦基函数, 红蓝绿色阶 (稀疏网格 + 柔化亮度)
%     fig5    : 优化参考曲面, 前光照亮, 不透明
%
%   v2 改动:
%     - 余弦基网格密度: 60 -> 22 (面片更稀疏, 网格清晰可辨)
%     - 色阶: 标准 jet -> softened_jet (混入白色 + 整体亮度 x 0.92)
%     - shading 改为 'faceted', 保留每个面片的方块感

    %% =========================================================
    %  Part 1~4 : 低频余弦基, jet 色阶柔化版
    %% =========================================================
    basis_params = [1 1; 1 2; 2 1; 2 2];
    Ngrid        = 22;                                  % <-- 网格密度
    [xi, eta]    = meshgrid(linspace(0,1,Ngrid), linspace(0,1,Ngrid));

    cmap_soft = softened_jet(256, 0.22);                % <-- 柔化 jet

    for k = 1:4
        m = basis_params(k,1);
        n = basis_params(k,2);
        Z = cos(m*pi*xi) .* cos(n*pi*eta);

        fig = figure('Color','w','Units','pixels', ...
                     'Position',[100+(k-1)*40, 100+(k-1)*40, 520, 480], ...
                     'Name',sprintf('Cosine basis m=%d n=%d',m,n), ...
                     'NumberTitle','off');
        ax  = axes('Parent',fig);

        surf(ax, xi, eta, Z, ...
             'EdgeColor',[0.30 0.30 0.30],'EdgeAlpha',0.60, ...
             'LineWidth',0.5, ...
             'FaceAlpha',0.95,'FaceLighting','none');
        colormap(ax, cmap_soft);
        shading(ax,'faceted');         % <-- 每个面片一块色, 不做插值

        view(ax,-55,30);
        axis(ax,'off');
        axis(ax,'tight');
        daspect(ax,[1 1 1.3]);

        exportgraphics(fig, sprintf('basis_m%d_n%d.png',m,n), ...
                       'Resolution', 300);
    end

    %% =========================================================
    %  Part 5 : 优化参考曲面
    %% =========================================================
    blue_cmap = make_cmap([188 224 240;
                           122 184 220;
                            63 138 194;
                            30  90 149]/255, 256);

    sigma_color = [0.84 0.18 0.18];
    arrow_color = [0.12 0.12 0.12];

    fig5 = figure('Color','w','Units','pixels', ...
                  'Position',[300 80 820 680], ...
                  'Name','Optimized reference surface','NumberTitle','off');
    ax5 = axes('Parent',fig5,'Position',[0.05 0.05 0.9 0.9]);

    [X, Y] = meshgrid(linspace(-1,1,100), linspace(-1,1,100));
    Z = 0.55*cos(0.65*pi*X).*cos(0.55*pi*Y) + 0.06*sin(0.9*pi*X);

    surf(ax5, X, Y, Z, ...
         'EdgeColor','none','FaceAlpha',1.0, ...
         'FaceLighting','gouraud', ...
         'AmbientStrength',0.45,'DiffuseStrength',0.85, ...
         'SpecularStrength',0.35,'SpecularExponent',12, ...
         'BackFaceLighting','reverselit');
    colormap(ax5, blue_cmap);
    shading(ax5,'interp');
    hold(ax5,'on');

    view(ax5,-58,28);
    delete(findall(ax5,'Type','Light'));
    camlight(ax5,'headlight');
    light(ax5,'Position',[ 0  0  10],'Style','infinite','Color',[1 1 1]*0.45);
    light(ax5,'Position',[-3 -3   5],'Style','infinite','Color',[1 1 1]*0.30);
    lighting(ax5,'gouraud');
    material(ax5,[0.55 0.75 0.35 12 0.6]);

    [Nx, Ny, Nz] = surfnorm(X, Y, Z);
    sample  = [10 10; 10 50; 10 90; 50 10; 50 90; 90 10; 90 50; 90 90];
    s_small = 0.28;
    for s = 1:size(sample,1)
        iy = sample(s,1); ix = sample(s,2);
        quiver3(ax5, X(iy,ix), Y(iy,ix), Z(iy,ix), ...
                Nx(iy,ix)*s_small, Ny(iy,ix)*s_small, Nz(iy,ix)*s_small, 0, ...
                'Color',arrow_color,'LineWidth',1.5, ...
                'MaxHeadSize',0.6,'AutoScale','off');
    end

    cx = 0; cy = 0;
    [~,cix] = min(abs(X(1,:) - cx));
    [~,ciy] = min(abs(Y(:,1) - cy));
    cz  = Z (ciy,cix);
    cnx = Nx(ciy,cix); cny = Ny(ciy,cix); cnz = Nz(ciy,cix);

    quiver3(ax5, cx, cy, cz, cnx*0.55, cny*0.55, cnz*0.55, 0, ...
            'Color',arrow_color,'LineWidth',2.8, ...
            'MaxHeadSize',0.5,'AutoScale','off');
    text(ax5, cx+cnx*0.66, cy+cny*0.66, cz+cnz*0.66, ...
         '$\hat{n}$','Interpreter','latex', ...
         'FontSize',22,'FontWeight','bold');

    Tx = 1.0; Ty = 0.25; Tz = 0.0;
    dotTN = Tx*cnx + Ty*cny + Tz*cnz;
    sx = Tx - dotTN*cnx;  sy = Ty - dotTN*cny;  sz = Tz - dotTN*cnz;
    sn = sqrt(sx^2+sy^2+sz^2); sx=sx/sn; sy=sy/sn; sz=sz/sn;

    quiver3(ax5, cx, cy, cz, sx*0.75, sy*0.75, sz*0.75, 0, ...
            'Color',sigma_color,'LineWidth',2.8, ...
            'MaxHeadSize',0.45,'AutoScale','off');
    text(ax5, cx+sx*0.88, cy+sy*0.88, cz+sz*0.88, ...
         '$\sigma_1$','Interpreter','latex', ...
         'FontSize',22,'Color',sigma_color);

    phi   = linspace(0,pi/2,30);
    r_arc = 0.24;
    arc_x = cx + r_arc*(cos(phi)*sx + sin(phi)*cnx);
    arc_y = cy + r_arc*(cos(phi)*sy + sin(phi)*cny);
    arc_z = cz + r_arc*(cos(phi)*sz + sin(phi)*cnz);
    plot3(ax5, arc_x, arc_y, arc_z,'Color',arrow_color,'LineWidth',1.6);

    mid = pi/4;
    tx = cx + (r_arc+0.08)*(cos(mid)*sx + sin(mid)*cnx);
    ty = cy + (r_arc+0.08)*(cos(mid)*sy + sin(mid)*cny);
    tz = cz + (r_arc+0.08)*(cos(mid)*sz + sin(mid)*cnz);
    text(ax5, tx, ty, tz, '$\theta$','Interpreter','latex','FontSize',19);

    axis(ax5,'off');  axis(ax5,'equal');
    xlim(ax5,[-1.15 1.15]); ylim(ax5,[-1.15 1.15]); zlim(ax5,[-0.5 1.1]);
    daspect(ax5,[1 1 0.8]);

    exportgraphics(fig5,'optimized_reference_surface.png','Resolution',300);
end

% -----------------------------------------------------------------------
function cmap = softened_jet(N, white_mix)
% 在标准 jet 基础上混入白色, 整体降亮, 得到柔和版红蓝绿色阶
%   N         : colormap 长度
%   white_mix : 与白色混合比例 (0=原 jet, 1=纯白)
    base = jet(N);
    soft = base*(1-white_mix) + ones(N,3)*white_mix;
    soft = soft * 0.92;          % 整体亮度再降一点
    cmap = max(min(soft,1),0);
end

function cmap = make_cmap(anchors, N)
% 锚点线性插值
    n_anchor = size(anchors,1);
    xq = linspace(1,n_anchor,N);
    cmap = zeros(N,3);
    for c = 1:3
        cmap(:,c) = interp1(1:n_anchor, anchors(:,c), xq, 'linear');
    end
    cmap = max(min(cmap,1),0);
end