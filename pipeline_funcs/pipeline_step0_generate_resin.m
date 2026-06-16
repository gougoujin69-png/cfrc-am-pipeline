function resin_data = pipeline_step0_generate_resin(input_file, user_params)
%% PIPELINE_STEP0_GENERATE_RESIN  v2  (悬空柱垂直分析版)
%
% 修正 v1 的结构性错误: 之前只看相邻上下层差集, 漏掉 ~90% 的深悬空柱.
%
% 新逻辑 (2.5D 垂直柱分析):
%   对每个 XY 细网格点:
%     z_above = 该 XY 上方最近的碳纤维 z (从 k+1 向上扫描)
%     z_below = 该 XY 下方最近的支撑 z   (从 k 向下扫描, 找不到则 = z_base 底座)
%     若 (z_above - z_below) > HANG_THRESHOLD_LAYERS × 层高:
%        且当前层间中线 z_mid ∈ (z_below, z_above)
%     → 该 XY 在当前层间需要填树脂
%
%   实心/网格判定 (按距悬空柱顶距离):
%     dist = z_above - z_mid
%     dist < CONTACT_THICK → 100% 直线实心 (接触上方碳纤维)
%     否则                 → 网格低密度  (悬空柱深处)
%
% 关键参数 (可由 user_params 覆盖):
%   HANG_THRESHOLD_LAYERS - 悬空阈值, 默认 2 个层高 (向下扫描这么远内无支撑则悬空)
%   CONTACT_THICK         - 接触层厚, 默认 0.6mm (距上方碳纤维这么近则实心)
%   FILAMENT_WIDTH        - 碳纤维丝宽, 默认 0.6mm
%   RESIN_LINE_WIDTH      - 树脂道宽, 默认 0.6mm
%   INFILL_DENSITY        - 网格填充率, 默认 0.20
%   GRID_RES              - XY 分析网格分辨率, 默认 0.4mm
%   Z_BASE                - 底座顶面 Z, 默认 = min(所有曲面 Z) - 1mm

%% ---------- 默认参数 ----------
p.FILAMENT_WIDTH        = 0.6;
p.RESIN_LINE_WIDTH      = 0.6;
p.CONTACT_THICK         = 0.6;
p.INFILL_DENSITY        = 0.20;
p.HANG_THRESHOLD_LAYERS = 2;       % 关键: 2 层高内无支撑才算悬空
p.GRID_RES              = 0.4;     % XY 分析网格分辨率(mm)
p.Z_BASE                = [];      % [] = 自动 (min surface - 1mm)
p.GRID_ANGLES           = [0 90];
p.SOLID_ANGLE           = 0;
p.MIN_AREA              = 0.5;
p.SIGNAL_TAG            = [0 23 222];
p.SAVE_FILE             = 'resin_paths_step0.mat';
p.VISUALIZE             = true;
if nargin>=2 && ~isempty(user_params)
    fn=fieldnames(user_params);
    for i=1:numel(fn), p.(fn{i})=user_params.(fn{i}); end
end
warning('off','MATLAB:polyshape:repairedBySimplify');
warning('off','MATLAB:polyshape:boolOperationFailed');

%% ---------- 读数据 + 自动层间距 ----------
fprintf('[Step0 v2] 读取 %s ...\n', input_file);
S = load(input_file, 'results');
ALD = S.results.all_layers_data;
nL  = numel(ALD);
offs = arrayfun(@(s) s.offset, ALD);
layer_pitch = median(diff(sort(offs(:))));
hang_dist = p.HANG_THRESHOLD_LAYERS * layer_pitch;
grid_spacing = p.RESIN_LINE_WIDTH / p.INFILL_DENSITY;
fprintf('  层数: %d, 层间距: %.3fmm\n', nL, layer_pitch);
fprintf('  悬空阈值: %d 层高 = %.2fmm   (向下扫描内无支撑则悬空)\n', p.HANG_THRESHOLD_LAYERS, hang_dist);
fprintf('  接触层厚: %.2fmm   网格间距: %.2fmm (填充率 %.0f%%)\n', p.CONTACT_THICK, grid_spacing, p.INFILL_DENSITY*100);

%% ---------- 建统一 XY 分析网格 ----------
all_xy=[];
for k=1:nL
    P=ALD(k).paths_3d;
    if iscell(P)
        for i=1:numel(P)
            seg=P{i};
            if isempty(seg)||~isnumeric(seg), continue; end
            if size(seg,1)==3 && size(seg,2)~=3, seg=seg'; end
            if size(seg,1)>=1, all_xy=[all_xy; seg(:,1:2)]; end
        end
    end
end
margin=2;
x_min=min(all_xy(:,1))-margin; x_max=max(all_xy(:,1))+margin;
y_min=min(all_xy(:,2))-margin; y_max=max(all_xy(:,2))+margin;
xg=x_min:p.GRID_RES:x_max;  yg=y_min:p.GRID_RES:y_max;
[XG,YG]=meshgrid(xg,yg);
nY=numel(yg); nX=numel(xg);
% 预线性化网格 (polyshape/isinterior 在多数 MATLAB 版本只接受列向量)
XG_vec = XG(:);  YG_vec = YG(:);
fprintf('  XY 分析网格: %d × %d = %d 点  (res %.2fmm)\n', nY, nX, nY*nX, p.GRID_RES);

%% ---------- 预计算: 每层 carbon_mask + surface_z ----------
fprintf('  预计算每层 carbon_mask + surface_z...\n');
carbon_mask = false(nY, nX, nL);
surface_z   = nan(nY, nX, nL);
tic;
for k=1:nL
    cpoly = build_carbon_poly(ALD(k).paths_3d, p.FILAMENT_WIDTH);
    if ~isempty_poly(cpoly)
        mask_vec = isinterior(cpoly, XG_vec, YG_vec);
        carbon_mask(:,:,k) = reshape(mask_vec, nY, nX);
    end
    F = build_surface_interp(ALD(k).pointCloud_data);
    if ~isempty(F)
        try
            sz_vec = F(XG_vec, YG_vec);
            surface_z(:,:,k) = reshape(sz_vec, nY, nX);
        catch
        end
    end
    if mod(k,20)==0||k==nL
        fprintf('    层 %d/%d 完成 (累计 %.1fs)\n', k, nL, toc);
    end
end

if isempty(p.Z_BASE)
    z_base = min(surface_z(:),[],'omitnan') - 1.0;
else
    z_base = p.Z_BASE;
end
fprintf('  z_base = %.3fmm\n', z_base);

%% ---------- 主循环: 逐层间生成树脂 (新悬空逻辑) ----------
fprintf('  逐层间生成树脂 (新逻辑: 向下找最近支撑)...\n');
resin_data = struct('paths_3d',{},'is_solid',{},'layer_pair',{},'signal_tag',{});

for k = 1:nL-1
    % 上方最近碳纤维 z (从 k+1 向上扫描)
    z_above = nan(nY,nX);
    for u = k+1:nL
        m_u = carbon_mask(:,:,u);
        s_u = surface_z(:,:,u);
        need = isnan(z_above) & m_u & ~isnan(s_u);
        z_above(need) = s_u(need);
    end
    % 下方最近支撑 (碳纤维 z, 从 k 向下扫描)
    z_below = nan(nY,nX);
    for u = k:-1:1
        m_u = carbon_mask(:,:,u);
        s_u = surface_z(:,:,u);
        need = isnan(z_below) & m_u & ~isnan(s_u);
        z_below(need) = s_u(need);
    end
    z_below(isnan(z_below)) = z_base;   % 没碳纤维支撑则用底座

    % 当前层间中线 z
    z_k   = surface_z(:,:,k);
    z_kp1 = surface_z(:,:,k+1);
    z_mid = (z_k + z_kp1) / 2;

    % 悬空判定: 间隙 > 2 层高, 且当前层间在悬空段内
    gap = z_above - z_below;
    in_seg = ~isnan(z_above) & ~isnan(z_mid) & (z_mid<z_above) & (z_mid>z_below);
    hang_mask = in_seg & (gap > hang_dist);
    if ~any(hang_mask(:))
        resin_data(k) = pack_empty(k, p.SIGNAL_TAG); continue;
    end

    % 实心 vs 网格 (按距悬空柱顶距离)
    dist_top  = z_above - z_mid;
    solid_mask  = hang_mask & (dist_top <  p.CONTACT_THICK);
    sparse_mask = hang_mask & (dist_top >= p.CONTACT_THICK);

    % mask 转 polyshape
    solid_poly  = mask2poly(solid_mask,  xg, yg);
    sparse_poly = mask2poly(sparse_mask, xg, yg);
    solid_poly  = rmsmall(solid_poly,  p.MIN_AREA);
    sparse_poly = rmsmall(sparse_poly, p.MIN_AREA);

    % 扫描填充
    segs = {}; flags = [];
    s1 = scan_fill(solid_poly, p.RESIN_LINE_WIDTH, p.SOLID_ANGLE);
    segs=[segs s1]; flags=[flags true(1,numel(s1))];
    for a = p.GRID_ANGLES
        s2 = scan_fill(sparse_poly, grid_spacing, a);
        segs=[segs s2]; flags=[flags false(1,numel(s2))];
    end

    % 投影到层 k 曲面 (用预计算的 surface_z[k], 直接 interp2 双线性)
    F_k = griddedInterpolant({yg, xg}, surface_z(:,:,k), 'linear', 'none');
    paths3 = cell(1,numel(segs));
    for i=1:numel(segs)
        xy = segs{i};
        z = F_k(xy(:,2), xy(:,1));   % griddedInterpolant: (y, x)
        ok = ~isnan(z);
        if sum(ok)>=2, paths3{i} = [xy(ok,1), xy(ok,2), z(ok)];
        else, paths3{i} = zeros(0,3); end
    end
    keep = cellfun(@(c) size(c,1)>=2, paths3);
    resin_data(k).paths_3d   = paths3(keep);
    resin_data(k).is_solid   = num2cell(logical(flags(keep)));
    resin_data(k).layer_pair = [k, k+1];
    resin_data(k).signal_tag = p.SIGNAL_TAG;

    if mod(k,5)==0||k==nL-1
        n_hang_xy = sum(hang_mask(:));
        fprintf('  层间 %3d->%3d: 悬空XY %5d, 实心段 %4d, 网格段 %4d\n', ...
            k,k+1,n_hang_xy,sum(flags),sum(~flags));
    end
end

%% ---------- 保存 ----------
save(p.SAVE_FILE, 'resin_data', 'p', 'layer_pitch', 'z_base');
fprintf('[Step0 v2] 已保存: %s\n', p.SAVE_FILE);

%% ---------- 可视化抽查 ----------
if p.VISUALIZE
    kk = find(arrayfun(@(s) ~isempty(s.paths_3d), resin_data), 1, 'last');
    if ~isempty(kk)
        figure('Color','w','Name','Step0 v2 抽查');
        rd = resin_data(kk);
        hold on;
        for i=1:numel(rd.paths_3d)
            c = rd.paths_3d{i};
            if rd.is_solid{i}, col=[1 0.1 0.1]; lw=1.4;
            else,              col=[0.1 0.45 1]; lw=0.9; end
            plot3(c(:,1),c(:,2),c(:,3),'-','Color',col,'LineWidth',lw);
        end
        axis equal; grid on; view(35,25);
        title(sprintf('层间 %d->%d  红=实心100%%  蓝=网格20%%', rd.layer_pair(1), rd.layer_pair(2)));
        xlabel('X'); ylabel('Y'); zlabel('Z');
    end
end

fprintf('[Step0 v2] 完成. 下一步: show_resin_paths 可视化, 或喂下游链\n');
end

%% ================= 子函数 =================
function pg = build_carbon_poly(paths_3d, fil_w)
    pg = polyshape();
    if isempty(paths_3d) || ~iscell(paths_3d), return; end
    for i=1:numel(paths_3d)
        seg = paths_3d{i};
        if isempty(seg)||~isnumeric(seg), continue; end
        if size(seg,1)==3 && size(seg,2)~=3, seg=seg'; end
        if size(seg,2)<2 || size(seg,1)<2, continue; end
        try, buf = polybuffer(seg(:,1:2),'lines',fil_w/2); pg = union(pg,buf); catch, end
    end
end
function F = build_surface_interp(pc)
    X=pc.X(:); Y=pc.Y(:); Z=pc.Z(:);
    m = ~(isnan(X)|isnan(Y)|isnan(Z));
    if sum(m)<3, F=[]; return; end
    try, F = scatteredInterpolant(X(m),Y(m),Z(m),'linear','none'); catch, F=[]; end
end
function tf = isempty_poly(pg)
    tf = isempty(pg) || pg.NumRegions==0 || area(pg)<1e-9;
end
function pg = rmsmall(pg, minA)
    if isempty_poly(pg), return; end
    try
        R = regions(pg);
        R = R(arrayfun(@area,R)>=minA);
        if isempty(R), pg=polyshape(); else, pg=union(R); end
    catch, end
end
function pg = mask2poly(mask, xg, yg)
% binary mask -> polyshape (使用 bwboundaries)
    pg = polyshape();
    if ~any(mask(:)), return; end
    try
        B = bwboundaries(mask, 8, 'holes');
        for i=1:numel(B)
            yx = B{i};
            if size(yx,1)<3, continue; end
            % 注: bwboundaries 返回 [row, col]
            xy = [xg(yx(:,2)).', yg(yx(:,1)).'];
            try, pg = union(pg, polyshape(xy,'Simplify',true)); catch, end
        end
    catch, end
end
function segs = scan_fill(pg, spacing, ang_deg)
    segs = {};
    if isempty_poly(pg), return; end
    V = pg.Vertices; ctr=[mean(V(~isnan(V(:,1)),1)), mean(V(~isnan(V(:,2)),2))];
    pr = rotate(pg, -ang_deg, ctr);
    [xb, yb] = boundingbox(pr);
    y = yb(1) + spacing*0.5;
    while y < yb(2)
        seg = intersect(pr, [xb(1)-1, y; xb(2)+1, y]);
        if ~isempty(seg)
            idx = [0; find(isnan(seg(:,1))); size(seg,1)+1];
            for j=1:numel(idx)-1
                sub = seg(idx(j)+1:idx(j+1)-1, :);
                if size(sub,1)>=2
                    sb = rotate_pts(sub, ang_deg, ctr);
                    segs{end+1} = sb; %#ok<AGROW>
                end
            end
        end
        y = y + spacing;
    end
end
function q = rotate_pts(pts, ang_deg, ctr)
    a = deg2rad(ang_deg); R=[cos(a) -sin(a); sin(a) cos(a)];
    q = (pts - ctr)*R' + ctr;
end
function e = pack_empty(k, tag)
    e.paths_3d={}; e.is_solid={}; e.layer_pair=[k,k+1]; e.signal_tag=tag;
end
