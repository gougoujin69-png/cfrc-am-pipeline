function support_data = pipeline_step0_generate_support_voxel(voxel_file, carbon_file, user_params)
%% PIPELINE_STEP0_GENERATE_SUPPORT_VOXEL  v1
%  基于【有效结构体素网格】的内部空腔支撑生成 (与碳纤维路径无关)
% =====================================================================
% 取代 pipeline_step0_generate_support 的"从碳纤维路径推截面"做法.
% 支撑区域完全由体素占用 refined_data.valid_grid_mask 计算; 碳纤维文件
% 只用来提供【曲面层 Z_k(X,Y)】(把体素支撑投影/归到曲面层, 供逐层穿插打印).
%
% 数据来源:
%   voxel_file  : voxel_refined_latest.mat
%       refined_data.valid_grid_mask  [nelx_f, nely_f, nelz_f] 0/1, 有效体素=结构
%       refined_data.grid_data(i,j,k).{x,y,z}  物理 mm 坐标 (= 体素中心)
%       refined_data.parameters.{ELEM_SIZE, REFINE_FACTOR, SCALE_FACTOR}
%       体素物理边长 GRID_STEP = ELEM_SIZE / REFINE_FACTOR
%   carbon_file : all_layers_path_results_v3.mat (results.all_layers_data)
%       all_layers_data(k).pointCloud_data.{X,Y,Z}  第 k 层曲面(同一物理坐标系)
%       all_layers_data(k).offset
%
% 算法:
%   1) occ = valid_grid_mask (逻辑). 成型方向 +Z (= 第 3 维 k).
%   2) 列向封闭空腔: cav(i,j,k)= 空 & 该(i,j)列下方有实体 & 上方有实体.
%      (开口到底/到顶的悬垂、底部间隙 -> 不在此列, 交给底座 STL.)
%   3) 接触分类: 距实体 <= CONTACT_THICK(沿 Z, 折算成体素层数) 的空腔 -> 实心直线;
%      更深 -> 20% 网格.
%   4) 把 cav_solid / cav_grid 当 3D 占用场, 用最近邻插值【沿每层曲面 Z_k(X,Y)
%      采样】-> 每层的支撑截面 mask; 再扫描填充, 投影到该层曲面 -> 该层支撑路径.
%      => 支撑天然归到曲面层、与碳纤维同坐标, 可逐层穿插.
%
% 打印时序(下游 merge / 序列生成沿用):
%   support_data(k) = 第 k 层支撑, 打印于"碳 L_k 之后". 空腔顶盖在更高层 ->
%   支撑必先于被支撑碳纤维; 空腔地板在更低层 -> 已打印. 同层碳纤维一次打完.
%
% 输出(与原 resin_data 同构, 可直接喂 merge_carbon_resin_data):
%   support_data(k).paths_3d   cell, 每个 M×3
%   support_data(k).is_solid   cell<logical>
%   support_data(k).layer_pair [k, k+1]
%   support_data(k).support_type 'cavity'
%   support_data(k).signal_tag [0 23 222]
%
% 用法:
%   sd = pipeline_step0_generate_support_voxel('voxel_refined_latest.mat', ...
%           'all_layers_path_results_v3.mat');
% =====================================================================

%% ---------- 参数 ----------
p.RESIN_LINE_WIDTH = 0.6;     % 支撑道宽
p.CONTACT_THICK    = 0.6;     % 距实体 <= 此值(沿Z) -> 实心
p.INFILL_DENSITY   = 0.20;    % 网格填充率
p.GRID_RES         = 0.4;     % 填充用 XY 分辨率(mm)
p.GRID_ANGLES      = [0 90];  % 网格扫描角
p.SOLID_ANGLE      = 0;       % 实心扫描角
p.MIN_AREA         = 1.0;     % 单区域最小面积(mm^2)
p.SIGNAL_TAG       = [0 23 222];
p.SAVE_FILE        = 'support_paths_step0.mat';
p.VISUALIZE        = true;
if nargin>=3 && ~isempty(user_params)
    fn = fieldnames(user_params);
    for i = 1:numel(fn), p.(fn{i}) = user_params.(fn{i}); end
end
if nargin<1 || isempty(voxel_file),  voxel_file  = 'voxel_refined_latest.mat'; end
if nargin<2 || isempty(carbon_file), carbon_file = 'all_layers_path_results_v3.mat'; end
warning('off','MATLAB:polyshape:repairedBySimplify');
warning('off','MATLAB:polyshape:boolOperationFailed');

%% ---------- 读体素 ----------
fprintf('[Step0-voxel] 读取体素 %s ...\n', voxel_file);
V = load(voxel_file, 'refined_data');
rd = V.refined_data;
occ = logical(rd.valid_grid_mask);                 % [nx,ny,nz]
[nx,ny,nz] = size(occ);
if isfield(rd,'parameters') && isfield(rd.parameters,'ELEM_SIZE') ...
        && isfield(rd.parameters,'REFINE_FACTOR')
    GRID_STEP = rd.parameters.ELEM_SIZE / rd.parameters.REFINE_FACTOR;
else
    GRID_STEP = 1.0;
    warning('refined_data.parameters 缺 ELEM_SIZE/REFINE_FACTOR, GRID_STEP 暂取 1mm');
end
% 物理坐标轴: 从 grid_data 角点取, 不迭代整个 struct (省内存)
gd = rd.grid_data;
x0 = gd(1,1,1).x;  x1 = gd(nx,1,1).x;
y0 = gd(1,1,1).y;  y1 = gd(1,ny,1).y;
z0 = gd(1,1,1).z;  z1 = gd(1,1,nz).z;
xs = linspace(x0, x1, nx);  ys = linspace(y0, y1, ny);  zs = linspace(z0, z1, nz);
fprintf('  体素网格 %dx%dx%d, 物理边长 %.3fmm\n', nx, ny, nz, GRID_STEP);
fprintf('  物理范围 X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]\n', x0,x1,y0,y1,z0,z1);
fprintf('  有效体素(结构) %d / %d\n', nnz(occ), numel(occ));

%% ---------- 列向封闭空腔 (纯体素) ----------
cb = cumsum(occ,3);  solid_below = false(nx,ny,nz);
solid_below(:,:,2:end) = cb(:,:,1:end-1) > 0;          % 该列 k 以下有实体
cf = cumsum(occ(:,:,end:-1:1),3); cf = cf(:,:,end:-1:1);
solid_above = false(nx,ny,nz);
solid_above(:,:,1:end-1) = cf(:,:,2:end) > 0;          % 该列 k 以上有实体
cav = ~occ & solid_below & solid_above;                % 内部封闭空腔
if ~any(cav(:))
    fprintf('[Step0-voxel] 体素网格无内部封闭空腔 -> 树脂支撑为空.\n');
    fprintf('             开放悬垂/底部由底座处理. 若需把悬垂也做成树脂, 告诉我改方案.\n');
    support_data = empty_support(carbon_file, p);
    save(p.SAVE_FILE, 'support_data', 'p', '-v7.3');
    return;
end

%% ---------- 接触分类: 沿 Z 距实体 <= CONTACT_THICK -> 实心 ----------
nb = max(1, round(p.CONTACT_THICK / GRID_STEP));       % 接触带体素层数
occ_near = occ;
for s = 1:nb
    occ_near(:,:,1:end-s) = occ_near(:,:,1:end-s) | occ(:,:,1+s:end);   % 上方 s 内有实体
    occ_near(:,:,1+s:end) = occ_near(:,:,1+s:end) | occ(:,:,1:end-s);   % 下方 s 内有实体
end
cav_solid = cav &  occ_near;
cav_grid  = cav & ~occ_near;
fprintf('  空腔体素 %d (实心接触 %d, 网格 %d), 接触带 %d 层(%.2fmm)\n', ...
        nnz(cav), nnz(cav_solid), nnz(cav_grid), nb, nb*GRID_STEP);

% 空腔物理范围(用于按层预筛)
[ci,cj,ck] = ind2sub(size(cav), find(cav));
cav_x = xs(ci); cav_y = ys(cj); cav_z = zs(ck);
cav_z_min = min(cav_z); cav_z_max = max(cav_z);

%% ---------- 填充用 XY 网格(贴着空腔 XY 包围盒 + 余量) ----------
mrg = 2*GRID_STEP;
gx = (min(cav_x)-mrg):p.GRID_RES:(max(cav_x)+mrg);
gy = (min(cav_y)-mrg):p.GRID_RES:(max(cav_y)+mrg);
[XX,YY] = meshgrid(gx, gy);                            % 注意: 行=Y, 列=X
nGY = numel(gy); nGX = numel(gx);

% 3D 占用场最近邻插值器
Fsolid = griddedInterpolant({xs(:),ys(:),zs(:)}, double(cav_solid), 'nearest','none');
Fgrid  = griddedInterpolant({xs(:),ys(:),zs(:)}, double(cav_grid),  'nearest','none');

%% ---------- 读碳纤维文件取曲面层 ----------
fprintf('[Step0-voxel] 读取碳纤维曲面 %s ...\n', carbon_file);
C = load(carbon_file);
if isfield(C,'results') && isfield(C.results,'all_layers_data')
    ALD = C.results.all_layers_data;
elseif isfield(C,'all_layers_data')
    ALD = C.all_layers_data;
else
    error('在 %s 找不到 all_layers_data', carbon_file);
end
nL = numel(ALD);
grid_spacing = p.RESIN_LINE_WIDTH / p.INFILL_DENSITY;
fprintf('  层数 %d, 网格间距 %.2fmm\n', nL, grid_spacing);

%% ---------- 逐层: 沿曲面采样空腔 -> 填充 -> 投影 ----------
support_data = struct('paths_3d',{},'is_solid',{},'layer_pair',{}, ...
                      'support_type',{},'signal_tag',{});
tic;
for k = 1:nL
    [emptyk, pc] = layer_surface(ALD(k));
    if emptyk
        support_data(k) = pack_layer({}, {}, k, nL, p.SIGNAL_TAG); continue;
    end
    zmn = min(pc.Z(:),[],'omitnan'); zmx = max(pc.Z(:),[],'omitnan');
    if ~isfinite(zmn) || zmx < cav_z_min-GRID_STEP || zmn > cav_z_max+GRID_STEP
        support_data(k) = pack_layer({}, {}, k, nL, p.SIGNAL_TAG); continue;   % 该层不穿过任何空腔
    end

    % 该层曲面 Z_k(X,Y)
    Fz = scatteredInterpolant(pc.X(:), pc.Y(:), pc.Z(:), 'linear','none');
    ZL = Fz(XX, YY);
    valid = ~isnan(ZL);
    if ~any(valid(:))
        support_data(k) = pack_layer({}, {}, k, nL, p.SIGNAL_TAG); continue;
    end

    % 沿曲面采样空腔占用
    Smask = false(nGY,nGX); Gmask = false(nGY,nGX);
    sv = Fsolid(XX(valid), YY(valid), ZL(valid));
    gv = Fgrid (XX(valid), YY(valid), ZL(valid));
    Smask(valid) = sv > 0.5;
    Gmask(valid) = gv > 0.5;
    Gmask = Gmask & ~Smask;
    if ~any(Smask(:)) && ~any(Gmask(:))
        support_data(k) = pack_layer({}, {}, k, nL, p.SIGNAL_TAG); continue;
    end

    % mask -> polyshape -> 扫描填充
    solid_poly = rmsmall(mask2poly(Smask, gx, gy), p.MIN_AREA);
    grid_poly  = rmsmall(mask2poly(Gmask, gx, gy), p.MIN_AREA);

    segs = {}; flags = [];
    s1 = scan_fill(solid_poly, p.RESIN_LINE_WIDTH, p.SOLID_ANGLE);
    segs = [segs s1]; flags = [flags true(1,numel(s1))]; %#ok<AGROW>
    for a = p.GRID_ANGLES
        s2 = scan_fill(grid_poly, grid_spacing, a);
        segs = [segs s2]; flags = [flags false(1,numel(s2))]; %#ok<AGROW>
    end

    % 投影到该层曲面
    paths3 = cell(1,numel(segs));
    for i = 1:numel(segs)
        xy = segs{i};
        z  = Fz(xy(:,1), xy(:,2));
        ok = ~isnan(z);
        if sum(ok)>=2, paths3{i} = [xy(ok,1), xy(ok,2), z(ok)];
        else,          paths3{i} = zeros(0,3); end
    end
    keep = cellfun(@(c) size(c,1)>=2, paths3);
    support_data(k) = pack_layer(paths3(keep), num2cell(logical(flags(keep))), ...
                                 k, nL, p.SIGNAL_TAG);

    if mod(k,10)==0 || k==nL
        nseg = sum(keep);
        fprintf('  层 %3d/%-3d  支撑段 %4d  (%.1fs)\n', k, nL, nseg, toc);
    end
end

%% ---------- 保存 ----------
save(p.SAVE_FILE, 'support_data', 'p', 'GRID_STEP', '-v7.3');
n_seg = sum(arrayfun(@(s) numel(s.paths_3d), support_data));
n_lay = sum(arrayfun(@(s) ~isempty(s.paths_3d), support_data));
fprintf('[Step0-voxel] 已保存 %s (含支撑层 %d, 总段 %d)\n', p.SAVE_FILE, n_lay, n_seg);

%% ---------- 可视化(全层 3D + 体素空腔对照) ----------
if p.VISUALIZE
    visualize_voxel_support(support_data, cav, xs, ys, zs);
end
fprintf('[Step0-voxel] 完成.\n');
end

%% ================= 子函数 =================
function sd = pack_layer(paths3, issolid, k, nL, tag)
    sd.paths_3d     = paths3;
    sd.is_solid     = issolid;
    sd.layer_pair   = [k, min(k+1,nL)];
    sd.support_type = 'cavity';
    sd.signal_tag   = tag;
end

function sd = empty_support(carbon_file, p)
    C = load(carbon_file);
    if isfield(C,'results') && isfield(C.results,'all_layers_data'), ALD = C.results.all_layers_data;
    elseif isfield(C,'all_layers_data'), ALD = C.all_layers_data; else, ALD = []; end
    nL = max(1, numel(ALD));
    sd = struct('paths_3d',{},'is_solid',{},'layer_pair',{},'support_type',{},'signal_tag',{});
    for k = 1:nL, sd(k) = pack_layer({}, {}, k, nL, p.SIGNAL_TAG); end
end

function [isEmpty, pc] = layer_surface(ld)
    isEmpty = true; pc = [];
    if ~isfield(ld,'pointCloud_data') || isempty(ld.pointCloud_data), return; end
    pc = ld.pointCloud_data;
    if ~isfield(pc,'X') || ~isfield(pc,'Y') || ~isfield(pc,'Z'), pc = []; return; end
    if numel(pc.Z) < 3, pc = []; return; end
    isEmpty = false;
end

function tf = isempty_poly(pg)
    tf = isempty(pg) || pg.NumRegions==0 || area(pg)<1e-9;
end

function pg = rmsmall(pg, minA)
    if isempty_poly(pg), return; end
    try
        R = regions(pg); R = R(arrayfun(@area,R)>=minA);
        if isempty(R), pg = polyshape(); else, pg = union(R); end
    catch, end
end

function pg = mask2poly(mask, gx, gy)
    pg = polyshape();
    if ~any(mask(:)), return; end
    try
        B = bwboundaries(mask, 8, 'holes');     % [row=Y, col=X]
        for i = 1:numel(B)
            yx = B{i};
            if size(yx,1)<3, continue; end
            xy = [gx(yx(:,2)).', gy(yx(:,1)).'];
            try, pg = union(pg, polyshape(xy,'Simplify',true)); catch, end
        end
    catch, end
end

function segs = scan_fill(pg, spacing, ang_deg)
    segs = {};
    if isempty_poly(pg), return; end
    Vv = pg.Vertices;
    ctr = [mean(Vv(~isnan(Vv(:,1)),1)), mean(Vv(~isnan(Vv(:,2)),2))];
    pr = rotate(pg, -ang_deg, ctr);
    [xb, yb] = boundingbox(pr);
    y = yb(1) + spacing*0.5;
    while y < yb(2)
        seg = intersect(pr, [xb(1)-1, y; xb(2)+1, y]);
        if ~isempty(seg)
            idx = [0; find(isnan(seg(:,1))); size(seg,1)+1];
            for j = 1:numel(idx)-1
                sub = seg(idx(j)+1:idx(j+1)-1, :);
                if size(sub,1)>=2, segs{end+1} = rotate_pts(sub, ang_deg, ctr); end %#ok<AGROW>
            end
        end
        y = y + spacing;
    end
end

function q = rotate_pts(pts, ang_deg, ctr)
    a = deg2rad(ang_deg); R = [cos(a) -sin(a); sin(a) cos(a)];
    q = (pts - ctr)*R' + ctr;
end

function visualize_voxel_support(support_data, cav, xs, ys, zs)
    nL = numel(support_data);
    RS = []; RG = [];
    for k = 1:nL
        sd = support_data(k);
        for i = 1:numel(sd.paths_3d)
            c = sd.paths_3d{i};
            if sd.is_solid{i}, RS=[RS;c;nan(1,3)]; else, RG=[RG;c;nan(1,3)]; end %#ok<AGROW>
        end
    end
    [ci,cj,ck] = ind2sub(size(cav), find(cav));
    cvx = xs(ci); cvy = ys(cj); cvz = zs(ck);

    figure('Color','w','Name','Step0-voxel 体素空腔支撑','Position',[60 60 1100 480]);
    subplot(1,2,1); hold on;
    plot3(cvx, cvy, cvz, '.', 'Color',[0.8 0.8 0.85], 'MarkerSize',6);
    if ~isempty(RS), plot3(RS(:,1),RS(:,2),RS(:,3),'-','Color',[0.9 0.1 0.1],'LineWidth',1.1); end
    if ~isempty(RG), plot3(RG(:,1),RG(:,2),RG(:,3),'-','Color',[0.1 0.45 1],'LineWidth',0.8); end
    axis equal; grid on; view(35,22); xlabel('X'); ylabel('Y'); zlabel('Z');
    title('灰=体素空腔  红=实心支撑  蓝=网格支撑');

    subplot(1,2,2); hold on;
    rsP = RS(~isnan(RS(:,1)),:); rgP = RG(~isnan(RG(:,1)),:);
    plot(cvx, cvz, '.', 'Color',[0.8 0.8 0.85], 'MarkerSize',6);
    if ~isempty(rsP), plot(rsP(:,1),rsP(:,3),'.','Color',[0.9 0.1 0.1],'MarkerSize',5); end
    if ~isempty(rgP), plot(rgP(:,1),rgP(:,3),'.','Color',[0.1 0.45 1],'MarkerSize',5); end
    axis equal; grid on; xlabel('X (mm)'); ylabel('Z (mm)');
    title('XZ 投影: 支撑落在体素空腔内(核对不串到结构里)');
end
