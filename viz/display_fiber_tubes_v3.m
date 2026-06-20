%% =====================================================================
%% display_fiber_tubes_v3.m  (V3: 中间附近几层 + 微偏移 + 半透明体素背景)
%% ---------------------------------------------------------------------
%% 改进:
%%   - 只展示中间附近的连续几层 (避开顶部柱体/底部稀疏的极端层)
%%   - Z 方向微偏移 1 mm, 保留分层感但不夸张
%%   - 加回半透明灰色体素背景, F/支撑符号回到真实位置
%%   - 圆柱细 (~0.3 mm), 每层全部路径
%%
%% 运行:
%%   >> display_fiber_tubes_v3
%% =====================================================================

clear; clc; close all;

%% ===== 配置 =====
PATH_MAT          = 'all_layers_path_results_v3.mat';
VOXEL_MAT         = 'voxel_refined_latest.mat';

N_SHOW            = 5;        % 选中间附近几层 (3~7)
LAYER_PICK_MODE   = 'middle'; % 'middle' = 围绕中心层; 'spread' = 整体等距
EXTRA_Z_GAP_MM    = 0.3;      % 层间额外 Z 间隔 (微偏移)
TUBE_RADIUS_MM    = 0.10;     % 圆柱半径
TUBE_NSIDES       = 12;

SHOW_VOXEL        = true;     % 半透明体素背景
VOXEL_ALPHA       = 0.05;     % 体素透明度 (越小越淡)
VOXEL_COLOR       = [0.55 0.55 0.58];
VOXEL_HALF_SIZE   = 0.5;      % 体素半边长 (按你的细化体素调整)

VIEW_AZ           = -60;
VIEW_EL           = 22;

% BC
BC_FIXED_CENTERS = [ ...
    5.0,  23.0, 0.0;
    14.4, 50.8, 0.0;
    29.0,  5.5, 0.0;
    44.0, 51.0, 0.0;
    53.0, 23.0, 0.0];
BC_LOAD_CENTER = [29.3, 30.4, 20.0];

%% ===== 1. 加载路径 =====
fprintf('[1] Loading %s ...\n', PATH_MAT);
assert(exist(PATH_MAT,'file')==2, 'File not found: %s', PATH_MAT);
tmp = load(PATH_MAT);
results = tmp.results;
all_layers_data = results.all_layers_data;
num_layers = results.num_layers;
fprintf('    num_layers = %d\n', num_layers);

%% ===== 2. 计算最长路径 =====
longest.layer = -1; longest.idx = -1; longest.len = 0;
for li = 1:num_layers
    if ~all_layers_data(li).success, continue; end
    P = all_layers_data(li).paths_3d;
    for pi = 1:length(P)
        pts = P{pi};
        if isempty(pts) || size(pts,1) < 2, continue; end
        L = sum(vecnorm(diff(pts,1,1), 2, 2));
        if L > longest.len
            longest.len = L; longest.layer = li; longest.idx = pi;
        end
    end
end
fprintf('[2] Longest path: layer %d, idx %d, length = %.2f mm\n', ...
        longest.layer, longest.idx, longest.len);

%% ===== 3. 选层: 中间附近连续 N_SHOW 层 =====
valid_layers = find(arrayfun(@(s) s.success, all_layers_data));

% 评估每层"有效路径点数", 用于选最饱满的中段
layer_richness = zeros(num_layers,1);
for li = valid_layers(:).'
    P = all_layers_data(li).paths_3d;
    n = 0;
    for pi = 1:length(P)
        if ~isempty(P{pi}), n = n + size(P{pi},1); end
    end
    layer_richness(li) = n;
end

switch lower(LAYER_PICK_MODE)
    case 'middle'
        % 找路径点最多的层作为"中心层", 然后向两侧扩
        [~, mid_li] = max(layer_richness);
        half = floor((N_SHOW-1)/2);
        cand = (mid_li - half) : (mid_li - half + N_SHOW - 1);
        cand = cand(cand >= 1 & cand <= num_layers);
        % 只保留 success 的
        cand = intersect(cand, valid_layers, 'stable');
        % 如果不够, 向两侧补
        while length(cand) < N_SHOW
            lo = min(cand) - 1; hi = max(cand) + 1;
            added = false;
            if lo >= 1 && ismember(lo, valid_layers) && ~ismember(lo, cand)
                cand = [lo, cand]; added = true;
            end
            if length(cand) < N_SHOW && hi <= num_layers && ...
                    ismember(hi, valid_layers) && ~ismember(hi, cand)
                cand = [cand, hi]; added = true;
            end
            if ~added, break; end
        end
        shown_layers = sort(cand);
    case 'spread'
        idx = round(linspace(1, length(valid_layers), N_SHOW));
        shown_layers = valid_layers(idx);
    otherwise
        error('Unknown LAYER_PICK_MODE');
end
fprintf('[3] Shown layers: %s  (richness: %s)\n', mat2str(shown_layers), ...
        mat2str(layer_richness(shown_layers)'));

N_SHOW_actual = length(shown_layers);
center_show_idx = (N_SHOW_actual + 1) / 2;
dz_per = ((1:N_SHOW_actual)' - center_show_idx) * EXTRA_Z_GAP_MM;

%% ===== 4. (可选) 加载体素 =====
voxel_data = [];
if SHOW_VOXEL && exist(VOXEL_MAT,'file')==2
    fprintf('[4] Loading %s ...\n', VOXEL_MAT);
    vm = load(VOXEL_MAT);
    if isfield(vm, 'refined_data')
        rd = vm.refined_data;
        voxel_data.grid_data = rd.grid_data;
        voxel_data.valid_grid_mask = rd.valid_grid_mask;
        voxel_data.nelx = rd.grid_size.nelx;
        voxel_data.nely = rd.grid_size.nely;
        voxel_data.nelz = rd.grid_size.nelz;
        fprintf('    voxel grid %d x %d x %d, valid = %d\n', ...
                voxel_data.nelx, voxel_data.nely, voxel_data.nelz, ...
                sum(voxel_data.valid_grid_mask(:)));
    else
        SHOW_VOXEL = false;
        fprintf('    refined_data missing, skip voxel.\n');
    end
else
    if SHOW_VOXEL
        fprintf('[4] %s not found, skip voxel.\n', VOXEL_MAT);
    end
    SHOW_VOXEL = false;
end

%% ===== 5. 绘图 =====
fprintf('[5] Drawing ...\n');
fig = figure('Name','Fiber paths (middle layers, voxel bg)', ...
             'NumberTitle','off','Position',[80 60 1400 1000],'Color','w');
ax = axes('Parent', fig); hold(ax,'on');

% ----- 5.1 半透明体素背景 -----
if SHOW_VOXEL
    cube_face = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    h = VOXEL_HALF_SIZE;
    n_vox = 0;

    % 收集所有体素顶点, 一次性 patch (远比循环 patch 快)
    nelx = voxel_data.nelx; nely = voxel_data.nely; nelz = voxel_data.nelz;
    valid_mask = voxel_data.valid_grid_mask;
    grid_data  = voxel_data.grid_data;

    % 预估
    n_total = sum(valid_mask(:));
    VertAll = zeros(8*n_total, 3);
    FaceAll = zeros(6*n_total, 4);
    cnt = 0;

    for i = 1:nelx
        for j = 1:nely
            for k = 1:nelz
                if ~valid_mask(i,j,k), continue; end
                gx = grid_data(i,j,k).x;
                gy = grid_data(i,j,k).y;
                gz = grid_data(i,j,k).z;
                v = [gx-h, gy-h, gz-h; gx+h, gy-h, gz-h; ...
                     gx+h, gy-h, gz+h; gx-h, gy-h, gz+h; ...
                     gx-h, gy+h, gz-h; gx+h, gy+h, gz-h; ...
                     gx+h, gy+h, gz+h; gx-h, gy+h, gz+h];
                base = cnt*8;
                VertAll(base+1:base+8, :) = v;
                FaceAll(cnt*6+1:cnt*6+6, :) = cube_face + base;
                cnt = cnt + 1;
            end
        end
    end
    VertAll = VertAll(1:cnt*8, :);
    FaceAll = FaceAll(1:cnt*6, :);

    patch('Parent', ax, 'Faces', FaceAll, 'Vertices', VertAll, ...
          'FaceColor', VOXEL_COLOR, 'EdgeColor', 'none', ...
          'FaceAlpha', VOXEL_ALPHA, 'FaceLighting','none');
    fprintf('    Drew %d voxels (one patch call)\n', cnt);
end

% ----- 5.2 路径圆柱 -----
layer_colors = jet(num_layers);
total_tubes = 0;
for ii = 1:N_SHOW_actual
    li = shown_layers(ii);
    dz = dz_per(ii);
    c  = layer_colors(li, :);
    P  = all_layers_data(li).paths_3d;

    for pi = 1:length(P)
        pts = P{pi};
        if isempty(pts) || size(pts,1) < 2, continue; end

        pts_s = pts;
        pts_s(:,3) = pts(:,3) + dz;

        pts_rs = resample_path_uniform(pts_s, max(0.2, TUBE_RADIUS_MM*0.7));
        [Xs, Ys, Zs] = tube_along_path(pts_rs, TUBE_RADIUS_MM, TUBE_NSIDES);

        surf(ax, Xs, Ys, Zs, ...
             'FaceColor', c, 'EdgeColor', 'none', ...
             'FaceLighting','gouraud','AmbientStrength',0.50, ...
             'DiffuseStrength',0.65,'SpecularStrength',0.20);
        total_tubes = total_tubes + 1;
    end
end
fprintf('    Drew %d tubes across %d layers\n', total_tubes, N_SHOW_actual);

%% ===== 6. F 箭头 (真实位置) =====
fx = BC_LOAD_CENTER(1); fy = BC_LOAD_CENTER(2); fz = BC_LOAD_CENTER(3);
arrow_len = 11;
quiver3(ax, fx, fy, fz + arrow_len*1.3, 0, 0, -arrow_len, 0, ...
        'Color','r','LineWidth',3.5,'MaxHeadSize',0.55);
text(ax, fx+1.6, fy+1.6, fz + arrow_len*1.5, 'F', ...
     'Color','r','FontSize',24,'FontWeight','bold','FontAngle','italic');

%% ===== 7. 固定支撑符号 (真实底面) =====
for s = 1:size(BC_FIXED_CENTERS,1)
    cz = BC_FIXED_CENTERS(s,3) - 1.5;   % 在 z=2 底面稍下方
    draw_fixed_support(ax, BC_FIXED_CENTERS(s,1), BC_FIXED_CENTERS(s,2), ...
                       cz, 2.8);
end
fc = BC_FIXED_CENTERS(3, :);
text(ax, fc(1)+5, fc(2)-9, fc(3) - 4.5, ...
     'fixed support', 'FontSize', 14, 'FontAngle','italic');

%% ===== 8. 样式 =====
axis(ax,'equal'); axis(ax,'tight');
view(ax, VIEW_AZ, VIEW_EL);
camproj(ax,'perspective');
camlight(ax,'headlight'); camlight(ax,'left');
lighting(ax,'gouraud');
material(ax,'dull');
grid(ax,'off'); axis(ax,'off');
set(fig,'Renderer','opengl');

fprintf('\n=================================================================\n');
fprintf('  Shown layers      : %s\n', mat2str(shown_layers));
fprintf('  >>> Longest path  : layer %d, idx %d, length = %.2f mm <<<\n', ...
        longest.layer, longest.idx, longest.len);
fprintf('=================================================================\n');

saveas(fig, 'fiber_tubes_v3.png');
saveas(fig, 'fiber_tubes_v3.fig');
fprintf('Saved: fiber_tubes_v3.png / .fig\n');


%% =====================================================================
%%                          辅助函数
%% =====================================================================

function pts_rs = resample_path_uniform(pts, step)
    if size(pts,1) < 2, pts_rs = pts; return; end
    d = [0; cumsum(vecnorm(diff(pts,1,1),2,2))];
    L = d(end);
    if L < 2*step, pts_rs = pts; return; end
    nn = max(8, ceil(L/step));
    d_new = linspace(0, L, nn).';
    pts_rs = [interp1(d, pts(:,1), d_new, 'pchip'), ...
              interp1(d, pts(:,2), d_new, 'pchip'), ...
              interp1(d, pts(:,3), d_new, 'pchip')];
end

function [Xs, Ys, Zs] = tube_along_path(pts, radius, n_sides)
    N = size(pts,1);
    T = zeros(N,3);
    T(1,:) = pts(2,:) - pts(1,:);
    T(N,:) = pts(N,:) - pts(N-1,:);
    for i = 2:N-1
        T(i,:) = pts(i+1,:) - pts(i-1,:);
    end
    Tn = vecnorm(T,2,2); Tn(Tn<1e-12)=1; T = T ./ Tn;

    ref = [0 0 1];
    if abs(dot(T(1,:), ref)) > 0.95, ref = [0 1 0]; end
    Nv = zeros(N,3); Bv = zeros(N,3);
    Nv(1,:) = ref - dot(ref, T(1,:))*T(1,:);
    Nv(1,:) = Nv(1,:)/max(norm(Nv(1,:)),1e-12);
    Bv(1,:) = cross(T(1,:), Nv(1,:));

    for i = 1:N-1
        v1 = pts(i+1,:) - pts(i,:);
        c1 = dot(v1,v1);
        rL = Nv(i,:) - (2/c1)*dot(v1, Nv(i,:))*v1;
        tL = T(i,:)  - (2/c1)*dot(v1, T(i,:))*v1;
        v2 = T(i+1,:) - tL;
        c2 = dot(v2,v2);
        if c2 < 1e-14
            Nv(i+1,:) = Nv(i,:);
        else
            Nv(i+1,:) = rL - (2/c2)*dot(v2, rL)*v2;
        end
        Nv(i+1,:) = Nv(i+1,:) - dot(Nv(i+1,:), T(i+1,:))*T(i+1,:);
        Nv(i+1,:) = Nv(i+1,:)/max(norm(Nv(i+1,:)),1e-12);
        Bv(i+1,:) = cross(T(i+1,:), Nv(i+1,:));
    end

    theta = linspace(0, 2*pi, n_sides+1);
    ct = cos(theta); st = sin(theta);
    Xs = zeros(N, n_sides+1);
    Ys = zeros(N, n_sides+1);
    Zs = zeros(N, n_sides+1);
    for i = 1:N
        for k = 1:n_sides+1
            off = radius*(ct(k)*Nv(i,:) + st(k)*Bv(i,:));
            Xs(i,k) = pts(i,1) + off(1);
            Ys(i,k) = pts(i,2) + off(2);
            Zs(i,k) = pts(i,3) + off(3);
        end
    end
end

function draw_fixed_support(ax, cx, cy, cz, sz)
    V = [cx - sz, cy, cz - sz*0.7;
         cx + sz, cy, cz - sz*0.7;
         cx,      cy, cz + sz*0.6];
    patch('Parent',ax,'Faces',[1 2 3],'Vertices',V,...
          'FaceColor',[0.85 0.85 0.85],'EdgeColor','k','LineWidth',1.0);
    nL = 5;
    for k = 0:nL-1
        x0 = cx - sz + (2*sz)*k/(nL-1);
        plot3(ax, [x0, x0 - 0.7], [cy, cy], ...
              [cz - sz*0.7, cz - sz*0.7 - 1.0], ...
              'k-', 'LineWidth', 1);
    end
end