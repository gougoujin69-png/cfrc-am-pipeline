%% =================================================================
%%  visualize_topo_stress_field.m  (v4)
%%  - 几何: 体素 wireframe
%%  - 应力方向: 内部点上画实心三角头箭头, 头面朝相机
%%  - F / X/Y/Z 标签拉远, 不再贴在箭头上
%% =================================================================
clear; clc; close all;

%% =================== 用户配置 ===================
MAT_FILE          = 'topo_stress_result.mat';
DENSITY_THRESHOLD = 0.30;

% --- 几何 wireframe ---
FACE_COLOR        = [0.92 0.94 0.97];
FACE_ALPHA        = 0.06;
EDGE_COLOR        = [0.32 0.34 0.38];
EDGE_ALPHA        = 0.45;
EDGE_WIDTH        = 0.40;

% --- 内部矢量 ---
INTERIOR_EROSION  = 2;
SAMPLE_FRACTION   = 0.70;       % 增加到 70%
ARROW_LEN_VOX     = 1.6;        % 箭头总长 (体素单位)
ARROW_LINE_WIDTH  = 1.5;        % 杆线宽
HEAD_LEN_FRAC     = 0.40;       % 三角头长 占总长比例
HEAD_HW_FRAC      = 0.18;       % 三角头半宽 占总长比例
N_COLOR_BINS      = 24;
ALIGN_W_POSITIVE  = true;

VIEW_AZEL         = [-35, 22];
ANGLE_UNIT        = 'deg';

% --- 载荷 ---
LOAD_CENTER_IJK   = [29.5, 30.5, 22];
LOAD_DIR          = [0, 0, -1];

% --- 5 个固定支撑 ---
SUPPORT_IJK = [ 5.0, 23.0, 2.0;
               14.4, 50.8, 2.0;
               29.0,  5.5, 2.0;
               44.0, 51.0, 2.0;
               53.0, 23.0, 2.0];

% --- 实物参考图叠加 ---
OVERLAY_REFERENCE = true;           % 是否叠加实物图
REF_IMG_PATH      = 'reference.png';% 实物参考图路径
OVERLAY_ALPHA     = 0.45;           % 参考图透明度 (0=完全透明, 1=完全不透明)
OVERLAY_SCALE     = 0.80;           % 参考图缩放比例 (1.0=贴满)
OVERLAY_DX_PX     = 0;              % 像素偏移 X (正=向右)
OVERLAY_DY_PX     = 0;              % 像素偏移 Y (正=向下)
BG_REMOVE_THRESH  = 25;             % 灰度 < 此值视为参考图的背景 (会被剔除)
SAVE_COMPOSITE    = true;           % 是否保存合成结果到磁盘

rng(7);

%% =================== 1. 加载数据 ===================
fprintf('加载 %s ...\n', MAT_FILE);
S     = load(MAT_FILE, 'nelx','nely','nelz','nele','xPhys','t_xoy','t_xoz');
xPhys = double(S.xPhys);
t_xoy = double(S.t_xoy);
t_xoz = double(S.t_xoz);
nelx  = double(S.nelx);
nely  = double(S.nely);
nelz  = double(S.nelz);

vsz = 1.0; org = [0 0 0];
try
    aux = load(MAT_FILE, 'voxel_size','origin_xyz');
    if isfield(aux,'voxel_size') && ~isempty(aux.voxel_size)
        vsz = double(aux.voxel_size);
    end
    if isfield(aux,'origin_xyz') && ~isempty(aux.origin_xyz)
        org = double(aux.origin_xyz(:))';
    end
catch
end
fprintf('  Grid: %dx%dx%d, voxel=%.3f, origin=[%g %g %g]\n', ...
        nelx, nely, nelz, vsz, org(1), org(2), org(3));

% --- 存储风格 ---
sz = size(xPhys); if numel(sz)==2, sz(3)=1; end
if sz(1)==nelx && sz(2)==nely && sz(3)==nelz
    fprintf('  存储: ndgrid\n');
elseif sz(1)==nely && sz(2)==nelx && sz(3)==nelz
    fprintf('  存储: meshgrid -> 自动 permute\n');
    xPhys = permute(xPhys, [2 1 3]);
    t_xoy = permute(t_xoy, [2 1 3]);
    t_xoz = permute(t_xoz, [2 1 3]);
else
    error('xPhys 形状 (%d,%d,%d) 不匹配 (nelx,nely,nelz)=(%d,%d,%d)', ...
          sz(1),sz(2),sz(3),nelx,nely,nelz);
end

%% =================== 2. sigma1 方向场 ===================
if strcmpi(ANGLE_UNIT, 'deg')
    Cz = cosd(t_xoz); Sz = sind(t_xoz);
    Cy = cosd(t_xoy); Sy = sind(t_xoy);
else
    Cz = cos(t_xoz);  Sz = sin(t_xoz);
    Cy = cos(t_xoy);  Sy = sin(t_xoy);
end
uu = Cz .* Cy;
vv = Cz .* Sy;
ww = -Sz;
mask_solid = xPhys >= DENSITY_THRESHOLD;

%% =================== 3. 内部点 ===================
fprintf('计算内部点 (侵蚀 %d 步) ...\n', INTERIOR_EROSION);
mask_interior = mask_solid;
for it = 1:INTERIOR_EROSION
    m = mask_interior;
    new_m = false(size(m));
    new_m(2:end-1, 2:end-1, 2:end-1) = ...
        m(2:end-1, 2:end-1, 2:end-1) & ...
        m(1:end-2, 2:end-1, 2:end-1) & m(3:end,   2:end-1, 2:end-1) & ...
        m(2:end-1, 1:end-2, 2:end-1) & m(2:end-1, 3:end,   2:end-1) & ...
        m(2:end-1, 2:end-1, 1:end-2) & m(2:end-1, 2:end-1, 3:end);
    mask_interior = new_m;
end
fprintf('  实体: %d, 内部: %d (%.1f%%)\n', ...
        nnz(mask_solid), nnz(mask_interior), ...
        100*nnz(mask_interior)/max(nnz(mask_solid),1));

if nnz(mask_interior) == 0
    error('内部点为空, 请减小 INTERIOR_EROSION');
end

%% =================== 4. 采样 ===================
ind_int = find(mask_interior);
n_sample = round(SAMPLE_FRACTION * numel(ind_int));
perm     = randperm(numel(ind_int));
ind_sel  = ind_int(perm(1:n_sample));
fprintf('  采样: %d / %d (%.0f%%)\n', n_sample, numel(ind_int), 100*SAMPLE_FRACTION);

[ii, jj, kk] = ind2sub(size(mask_interior), ind_sel);
x_vec = org(1) + ((1:nelx) - 0.5) * vsz;
y_vec = org(2) + ((1:nely) - 0.5) * vsz;
z_vec = org(3) + ((1:nelz) - 0.5) * vsz;

px = x_vec(ii(:)); px = px(:);
py = y_vec(jj(:)); py = py(:);
pz = z_vec(kk(:)); pz = pz(:);

u_sel = uu(ind_sel); u_sel = u_sel(:);
v_sel = vv(ind_sel); v_sel = v_sel(:);
w_sel = ww(ind_sel); w_sel = w_sel(:);

if ALIGN_W_POSITIVE
    flip = w_sel < 0;
    u_sel(flip) = -u_sel(flip);
    v_sel(flip) = -v_sel(flip);
    w_sel(flip) = -w_sel(flip);
end
abs_w = abs(w_sel);

%% =================== 5. Figure ===================
hf = figure('Color','w','Position',[80, 60, 1000, 820]);
ax = axes('Parent', hf); hold(ax,'on');

%% =================== 6. 体素 wireframe ===================
fprintf('生成体素暴露面 ...\n');
[Vw, Fw] = voxel_exposed_faces(mask_solid, x_vec, y_vec, z_vec);
fprintf('  暴露面: %d, 顶点: %d\n', size(Fw,1), size(Vw,1));
patch('Vertices', Vw, 'Faces', Fw, ...
      'FaceColor', FACE_COLOR, 'FaceAlpha', FACE_ALPHA, ...
      'EdgeColor', EDGE_COLOR, 'EdgeAlpha', EDGE_ALPHA, ...
      'LineWidth', EDGE_WIDTH, 'Parent', ax);

%% =================== 7. 视线方向 (用于箭头头朝相机) ===================
view_dir = compute_view_dir(VIEW_AZEL(1), VIEW_AZEL(2));

%% =================== 8. 实心箭头矢量场 (按 |sinθz| 分桶) ===================
fprintf('绘制 %d 个实心箭头 ...\n', numel(px));
cmap_bins = turbo(N_COLOR_BINS);
colormap(ax, turbo(256));
try, clim(ax, [0 1]); catch, caxis(ax, [0 1]); end

bin_idx = min(N_COLOR_BINS, max(1, ceil(abs_w * N_COLOR_BINS)));
bin_idx(abs_w == 0) = 1;

arr_L = ARROW_LEN_VOX * vsz;
ux = u_sel * arr_L;
uy = v_sel * arr_L;
uz = w_sel * arr_L;
sx = px - 0.5*ux;   % 箭头起点 (居中)
sy = py - 0.5*uy;
sz_= pz - 0.5*uz;
ex = px + 0.5*ux;   % 箭头终点
ey = py + 0.5*uy;
ez = pz + 0.5*uz;

for b = 1:N_COLOR_BINS
    bm = (bin_idx == b);
    if ~any(bm), continue; end
    P_start = [sx(bm), sy(bm), sz_(bm)];
    P_end   = [ex(bm), ey(bm), ez(bm)];
    draw_solid_arrows(ax, P_start, P_end, cmap_bins(b,:), ...
                      HEAD_LEN_FRAC, HEAD_HW_FRAC, ...
                      ARROW_LINE_WIDTH, view_dir);
end

%% =================== 9. 载荷箭头 F (实心, 大) ===================
xL = org(1) + (LOAD_CENTER_IJK(1)-0.5)*vsz;
yL = org(2) + (LOAD_CENTER_IJK(2)-0.5)*vsz;
zL = org(3) + (LOAD_CENTER_IJK(3)-0.5)*vsz;

Lz_phys   = nelz * vsz;
arr_len_F = 0.42 * Lz_phys;
P_start_F = [xL, yL, zL] - LOAD_DIR * arr_len_F * 1.15;
P_end_F   = [xL, yL, zL] - LOAD_DIR * arr_len_F * 0.15;

draw_solid_arrows(ax, P_start_F, P_end_F, [0.85 0.10 0.10], ...
                  0.30, 0.10, 4.0, view_dir);

% F 标签: 放在箭头尾部上方 (z 方向再多抬 3 个 voxel, 不再贴箭头)
text(P_start_F(1), P_start_F(2), P_start_F(3) + 3.0*vsz, ...
     '\bfF', 'FontSize', 22, 'Color',[0.7 0.05 0.05], ...
     'HorizontalAlignment','center', 'Parent', ax);

%% =================== 10. 5 个支撑 ▲ ===================
for s = 1:size(SUPPORT_IJK,1)
    xc = org(1) + (SUPPORT_IJK(s,1)-0.5)*vsz;
    yc = org(2) + (SUPPORT_IJK(s,2)-0.5)*vsz;
    zc = org(3) + (SUPPORT_IJK(s,3)-1.0)*vsz;
    draw_support_triangle(ax, xc, yc, zc, 2.5*vsz);
end

%% =================== 11. 角落坐标系 (实心箭头) ===================
xa = org(1) - 7*vsz;
ya = org(2) - 7*vsz;
za = org(3);
La = 7*vsz;
label_gap = 2.2*vsz;        % 标签距箭头尖端的额外间距

draw_solid_arrows(ax, [xa ya za], [xa+La, ya,   za],   [0 0 0], 0.20, 0.06, 1.8, view_dir);
draw_solid_arrows(ax, [xa ya za], [xa,    ya+La,za],   [0 0 0], 0.20, 0.06, 1.8, view_dir);
draw_solid_arrows(ax, [xa ya za], [xa,    ya,   za+La],[0 0 0], 0.20, 0.06, 1.8, view_dir);

text(xa+La+label_gap, ya,            za,            'X', ...
     'FontWeight','bold','FontSize',13,'HorizontalAlignment','center','Parent',ax);
text(xa,              ya+La+label_gap, za,          'Y', ...
     'FontWeight','bold','FontSize',13,'HorizontalAlignment','center','Parent',ax);
text(xa,              ya,            za+La+label_gap,'Z', ...
     'FontWeight','bold','FontSize',13,'HorizontalAlignment','center','Parent',ax);

%% =================== 12. 视图 ===================
axis(ax,'equal'); axis(ax,'vis3d'); axis(ax,'off');
view(ax, VIEW_AZEL);
camlight(ax,'headlight'); camlight(ax,'right');
lighting(ax,'gouraud'); material(ax,'dull');
set(ax,'Projection','perspective');

cb = colorbar(ax);
cb.Label.String   = '|sin\theta_z|   0 = in XY plane, 1 = along Z';
cb.Label.FontSize = 11;
title(ax, 'Topology optimization result and 1^{st} principal stress \sigma_1 field', ...
      'FontSize', 13, 'FontWeight','normal');

%% =================== 13. (可选) 叠加实物参考图 ===================
if OVERLAY_REFERENCE
    fprintf('叠加参考图 %s (alpha=%.2f, scale=%.2f) ...\n', ...
            REF_IMG_PATH, OVERLAY_ALPHA, OVERLAY_SCALE);
    compose_overlay(hf, REF_IMG_PATH, OVERLAY_ALPHA, OVERLAY_SCALE, ...
                    OVERLAY_DX_PX, OVERLAY_DY_PX, BG_REMOVE_THRESH, ...
                    SAVE_COMPOSITE);
end

fprintf('完成.\n');


%% =================== Helper: 批量绘制实心三角头箭头 ===================
function draw_solid_arrows(ax, P_start, P_end, color, head_len_frac, head_hw_frac, line_width, view_dir)
% P_start, P_end: Nx3 物理坐标
% head_len_frac : 头长占总长比例
% head_hw_frac  : 头三角形半宽占总长比例
% view_dir      : 1x3 相机视线方向, 用来定向头三角形使其面朝相机
    N = size(P_start, 1);
    if N == 0, return; end

    D  = P_end - P_start;
    L  = sqrt(sum(D.^2, 2));
    Dh = D ./ max(L, 1e-12);

    head_len    = head_len_frac * L;
    head_hw     = head_hw_frac  * L;
    P_shaft_end = P_end - Dh .* head_len;   % 杆终点 = 头根部

    % --- 杆: 用一个 patch 画 N 条独立线段 ---
    Vs = zeros(2*N, 3);
    Vs(1:2:end, :) = P_start;
    Vs(2:2:end, :) = P_shaft_end;
    Fs = [(1:2:2*N).', (2:2:2*N).'];
    patch('Vertices', Vs, 'Faces', Fs, ...
          'FaceColor','none', 'EdgeColor', color, ...
          'LineWidth', line_width, 'Parent', ax);

    % --- 头三角形 base 的侧向: cross(D, view_dir) (使三角面朝相机) ---
    side = cross(Dh, repmat(view_dir(:).', N, 1), 2);
    sn   = sqrt(sum(side.^2, 2));
    % D 平行 view_dir 时回退到与 Z 轴叉乘
    bad = sn < 1e-6;
    if any(bad)
        side(bad,:) = cross(Dh(bad,:), repmat([0 0 1], sum(bad), 1), 2);
        sn(bad) = sqrt(sum(side(bad,:).^2, 2));
        bad2 = sn < 1e-6;
        if any(bad2)
            side(bad2,:) = cross(Dh(bad2,:), repmat([1 0 0], sum(bad2),1), 2);
            sn(bad2) = sqrt(sum(side(bad2,:).^2, 2));
        end
    end
    side = side ./ max(sn, 1e-12);

    base_L = P_shaft_end + side .* head_hw;
    base_R = P_shaft_end - side .* head_hw;
    apex   = P_end;

    % --- 三角形头: 一个 patch 画 N 个实心三角形 ---
    Vh = zeros(3*N, 3);
    Vh(1:3:end, :) = apex;
    Vh(2:3:end, :) = base_L;
    Vh(3:3:end, :) = base_R;
    Fh = reshape(1:3*N, 3, [])';
    patch('Vertices', Vh, 'Faces', Fh, ...
          'FaceColor', color, 'EdgeColor', 'none', ...
          'Parent', ax);
end


%% =================== Helper: 视线方向 from MATLAB view(az,el) ===================
function v = compute_view_dir(az_deg, el_deg)
% 返回从相机指向 origin (目标) 的单位方向向量
% MATLAB 约定: az=0 时相机在 -Y 方向, 顺时针为正
    cam = [sind(az_deg)*cosd(el_deg), ...
          -cosd(az_deg)*cosd(el_deg), ...
           sind(el_deg)];
    v = -cam;
    v = v / max(norm(v), 1e-12);
end


%% =================== Helper: 体素暴露面 ===================
function [V, F] = voxel_exposed_faces(mask, x_vec, y_vec, z_vec)
    [nx, ny, nz] = size(mask);
    if numel(x_vec) >= 2, dxh = (x_vec(2)-x_vec(1))/2; else, dxh = 0.5; end
    if numel(y_vec) >= 2, dyh = (y_vec(2)-y_vec(1))/2; else, dyh = 0.5; end
    if numel(z_vec) >= 2, dzh = (z_vec(2)-z_vec(1))/2; else, dzh = 0.5; end

    face_local = { ...
        [-1 -1 -1; +1 -1 -1; +1 +1 -1; -1 +1 -1];  ...
        [-1 -1 +1; +1 -1 +1; +1 +1 +1; -1 +1 +1];  ...
        [-1 -1 -1; +1 -1 -1; +1 -1 +1; -1 -1 +1];  ...
        [-1 +1 -1; +1 +1 -1; +1 +1 +1; -1 +1 +1];  ...
        [-1 -1 -1; -1 +1 -1; -1 +1 +1; -1 -1 +1];  ...
        [+1 -1 -1; +1 +1 -1; +1 +1 +1; +1 -1 +1]   ...
    };
    face_dirs = [0 0 -1; 0 0 +1; 0 -1 0; 0 +1 0; -1 0 0; +1 0 0];

    Vc = cell(6,1); Fc = cell(6,1); nV_total = 0;
    for f = 1:6
        di = face_dirs(f,1); dj = face_dirs(f,2); dk = face_dirs(f,3);
        neigh = false(nx, ny, nz);
        i_s1 = max(1, 1+di); i_s2 = min(nx, nx+di);
        j_s1 = max(1, 1+dj); j_s2 = min(ny, ny+dj);
        k_s1 = max(1, 1+dk); k_s2 = min(nz, nz+dk);
        neigh(i_s1-di:i_s2-di, j_s1-dj:j_s2-dj, k_s1-dk:k_s2-dk) = ...
            mask(i_s1:i_s2, j_s1:j_s2, k_s1:k_s2);
        expose = mask & ~neigh;
        idx = find(expose);
        if isempty(idx), continue; end
        [ii, jj, kk] = ind2sub([nx ny nz], idx);
        cx = x_vec(ii(:)); cx = cx(:);
        cy = y_vec(jj(:)); cy = cy(:);
        cz = z_vec(kk(:)); cz = cz(:);
        nF = numel(idx);
        local = face_local{f}; offsets = local .* [dxh dyh dzh];
        Vf = zeros(nF*4, 3);
        for v = 1:4
            Vf((v-1)*nF + (1:nF), 1) = cx + offsets(v,1);
            Vf((v-1)*nF + (1:nF), 2) = cy + offsets(v,2);
            Vf((v-1)*nF + (1:nF), 3) = cz + offsets(v,3);
        end
        Ff = zeros(nF, 4);
        for v = 1:4
            Ff(:, v) = nV_total + (v-1)*nF + (1:nF)';
        end
        Vc{f} = Vf; Fc{f} = Ff;
        nV_total = nV_total + nF*4;
    end
    V = cat(1, Vc{:});
    F = cat(1, Fc{:});
end


%% =================== Helper: 支撑三角形 ▲ ===================
function draw_support_triangle(ax, xc, yc, zc, s)
    apex = [xc,     yc, zc];
    bL   = [xc - s, yc, zc - s];
    bR   = [xc + s, yc, zc - s];
    patch('Vertices', [apex; bL; bR], 'Faces', [1 2 3], ...
          'FaceColor',[0.93 0.95 0.97], 'EdgeColor','k', ...
          'LineWidth', 1.2, 'Parent', ax);
    line([bL(1) - 0.30*s, bR(1) + 0.30*s], [yc, yc], [bL(3), bL(3)], ...
         'Color','k','LineWidth',1.2,'Parent',ax);
    nh = 5;
    for k = 1:nh
        t  = (k - 0.5)/nh;
        xa = bL(1) + 2*s*t;
        xb = xa - 0.45*s;
        za = bL(3);
        zb = za - 0.45*s;
        line([xa, xb], [yc, yc], [za, zb], ...
             'Color','k','LineWidth',0.7,'Parent',ax);
    end
end


%% =================== Helper: 实物图叠加合成 ===================
function compose_overlay(hf, ref_path, alpha_overlay, scale, dx_px, dy_px, bg_thresh, do_save)
% 把当前 figure 渲染到画布, 再叠加实物参考图 (已经移除背景), 弹出新窗口显示
    drawnow;
    frame = getframe(hf);
    base = frame.cdata;
    [H, W, ~] = size(base);

    if ~exist(ref_path, 'file')
        warning('找不到参考图: %s. 跳过叠加.', ref_path);
        return;
    end
    [ref, ~, ref_alpha_in] = imread(ref_path);
    if ndims(ref) == 2
        ref = repmat(ref, 1, 1, 3);
    end
    if size(ref,3) == 4
        % RGBA
        ref_alpha = double(ref(:,:,4))/255;
        ref = ref(:,:,1:3);
    elseif ~isempty(ref_alpha_in)
        ref_alpha = double(ref_alpha_in)/255;
    else
        % 没有 alpha 通道, 用黑底阈值做透明 mask
        gray = rgb2gray(ref);
        ref_alpha = double(gray > bg_thresh);
        % 形态学清理 + 边缘羽化 (需要 Image Processing Toolbox; 失败也无所谓)
        try, ref_alpha = imopen(ref_alpha, strel('square',3)); catch, end
        try, ref_alpha = imgaussfilt(ref_alpha, 1.5);          catch, end
    end

    % --- 等比缩放到 figure ---
    Hr = size(ref,1); Wr = size(ref,2);
    base_scale = min(W/Wr, H/Hr) * scale;
    Hn = max(1, round(Hr * base_scale));
    Wn = max(1, round(Wr * base_scale));
    try
        ref_rs   = imresize(ref,       [Hn Wn]);
        alpha_rs = imresize(ref_alpha, [Hn Wn]);
    catch
        warning('imresize 失败, 跳过叠加.');
        return;
    end

    % --- 居中粘贴, 含像素偏移 ---
    canvas_ref = zeros(H, W, 3, 'uint8');
    canvas_a   = zeros(H, W);
    y_off = round((H - Hn)/2) + dy_px;
    x_off = round((W - Wn)/2) + dx_px;

    y1 = max(1, y_off+1);   y2 = min(H, y_off+Hn);
    x1 = max(1, x_off+1);   x2 = min(W, x_off+Wn);
    sy1 = y1 - y_off;        sy2 = y2 - y_off;
    sx1 = x1 - x_off;        sx2 = x2 - x_off;
    if y1 > y2 || x1 > x2
        warning('overlay 偏移过大, 完全落在画布外.');
        return;
    end
    canvas_ref(y1:y2, x1:x2, :) = ref_rs(sy1:sy2, sx1:sx2, :);
    canvas_a(y1:y2, x1:x2)      = alpha_rs(sy1:sy2, sx1:sx2);

    % --- alpha blend: out = base*(1-M) + ref*M, M = alpha_overlay * ref_alpha ---
    A = double(base);
    B = double(canvas_ref);
    M = canvas_a * alpha_overlay;   % HxW, 范围 [0, alpha_overlay]
    merged = zeros(H, W, 3);
    for c = 1:3
        merged(:,:,c) = A(:,:,c) .* (1 - M) + B(:,:,c) .* M;
    end
    merged = uint8(merged);

    % --- 显示合成图 ---
    hf2 = figure('Color','w','Position', [120 120 W H], ...
                 'Name','Composite Overlay','NumberTitle','off');
    ax2 = axes('Parent', hf2, 'Position', [0 0 1 1]);
    imshow(merged, 'Parent', ax2);

    % --- 保存到磁盘 ---
    if do_save
        out = 'composite_overlay.png';
        try
            imwrite(merged, out);
            fprintf('  合成图已保存: %s\n', out);
        catch ME
            warning('保存失败: %s', ME.message);
        end
    end
end
