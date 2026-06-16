function support_data = pipeline_step0_generate_support(input_file, user_params)
%% PIPELINE_STEP0_GENERATE_SUPPORT  v3.1  基于模型的"内部空腔"支撑生成
% =====================================================================
% 取代逐层悬空监测; 把碳纤维结构当 3D 实体, 只在【内部封闭空腔】生成支撑.
%
% v3.1 关键修复(树脂跑到碳纤维道间的 bug):
%   旧版每层碳纤维截面 = 路径缓冲 + 按"面积"填小孔. 但碳纤维道与道之间是
%   【细长缝隙】, 面积常大于阈值而被保留 -> 与邻层错位的道叠加后被误判成
%   "空腔" -> 层间夹缝里到处长树脂. 这是错的: 那里结构是实心的.
%   现在改成: 栅格化路径 -> 按丝宽膨胀 -> 【形态学闭运算】按缝隙*宽度*桥接
%   道间空隙, 合并成实心截面; 只有远大于道间距的【真实空腔】才保留.
%   判别尺度由 STRUCT_CLOSE_MM 控制(设为略大于碳纤维道间距/2).
%
% 责任划分(配合 generate_base_stl):
%   支撑(树脂)= 内部封闭空腔(某层无碳纤维, 但该 XY 列上下方都有碳纤维).
%   底座 STL  = 底部间隙 + 开放悬垂(底座上表面=结构下包络, 自然垫到位).
%   两者互不重叠. (想用平面顶底座+稀疏垫底, 把 FILL_BOTTOM_GAP 设 true.)
%
% 填充规则: 空腔内部 20% 网格; 临近碳纤维 0.6mm 实心; 紧贴碳纤维相邻层必实心.
%
% 打印时序: support_data(k)=第 k 层空腔填充, 投影到第 k 层曲面, "碳 L_k 之后"打.
%
% 输出(与原 resin_data 同构): support_data(k).{paths_3d, is_solid, layer_pair,
%   support_type, signal_tag=[0 23 222]} -> support_paths_step0.mat
% =====================================================================

%% ---------- 默认参数 ----------
p.FILAMENT_WIDTH   = 0.6;     % 碳纤维丝宽
p.RESIN_LINE_WIDTH = 0.6;     % 支撑道宽
p.CONTACT_THICK    = 0.6;     % 距碳纤维 < 此值 -> 实心
p.INFILL_DENSITY   = 0.20;    % 网格填充率
p.GRID_RES         = 0.4;     % XY 分析分辨率(mm)
p.GRID_ANGLES      = [0 90];  % 网格扫描角度
p.SOLID_ANGLE      = 0;       % 实心扫描角度
p.MIN_AREA         = 1.0;     % 单个填充区域最小面积(mm^2), 去碎
p.STRUCT_CLOSE_MM  = 3.0;     % 闭运算半径: 桥接碳纤维道间空隙(关键!)
                              %   设为略大于碳纤维道间距/2; 太大填掉真实空腔,
                              %   太小则道间缝被误判成空腔(老 bug).
p.CLOSE_GAPS       = true;    % 是否做闭运算桥接(建议 true)
p.FILL_BOTTOM_GAP  = false;   % 底部交给底座 STL
p.Z_BASE           = [];      % 仅 FILL_BOTTOM_GAP=true 用; []=min(曲面)-1mm
p.SIGNAL_TAG       = [0 23 222];
p.SAVE_FILE        = 'support_paths_step0.mat';
p.VISUALIZE        = true;
if nargin>=2 && ~isempty(user_params)
    fn = fieldnames(user_params);
    for i = 1:numel(fn), p.(fn{i}) = user_params.(fn{i}); end
end
warning('off','MATLAB:polyshape:repairedBySimplify');
warning('off','MATLAB:polyshape:boolOperationFailed');

%% ---------- 读数据 + 估计层间距 ----------
fprintf('[Step0 v3.1] 读取 %s ...\n', input_file);
S = load(input_file);
if isfield(S,'results') && isfield(S.results,'all_layers_data')
    ALD = S.results.all_layers_data;
elseif isfield(S,'all_layers_data')
    ALD = S.all_layers_data;
else
    error('在 %s 找不到 all_layers_data', input_file);
end
nL = numel(ALD);

if isfield(ALD,'offset')
    offs = arrayfun(@(s) s.offset, ALD);
    layer_pitch = median(diff(sort(offs(:))));
else
    zmed = arrayfun(@(s) median(s.pointCloud_data.Z(:),'omitnan'), ALD);
    layer_pitch = median(abs(diff(sort(zmed(:)))));
end
if ~isfinite(layer_pitch) || layer_pitch<=0, layer_pitch = 0.2; end
grid_spacing = p.RESIN_LINE_WIDTH / p.INFILL_DENSITY;
fprintf('  层数 %d, 估计层间距 %.3fmm, 闭运算桥接半径 %.2fmm\n', ...
        nL, layer_pitch, p.STRUCT_CLOSE_MM);
fprintf('  接触层厚 %.2fmm, 网格间距 %.2fmm (填充率 %.0f%%)\n', ...
        p.CONTACT_THICK, grid_spacing, p.INFILL_DENSITY*100);

%% ---------- 统一 XY 分析网格 ----------
all_xy = [];
for k = 1:nL
    P = ALD(k).paths_3d;
    if iscell(P)
        for i = 1:numel(P)
            seg = P{i};
            if isempty(seg) || ~isnumeric(seg), continue; end
            if size(seg,1)==3 && size(seg,2)~=3, seg = seg.'; end
            if size(seg,1)>=1, all_xy = [all_xy; seg(:,1:2)]; end %#ok<AGROW>
        end
    end
end
if isempty(all_xy), error('没有任何碳纤维路径点'); end
margin = 2;
x_min = min(all_xy(:,1))-margin; x_max = max(all_xy(:,1))+margin;
y_min = min(all_xy(:,2))-margin; y_max = max(all_xy(:,2))+margin;
xg = x_min:p.GRID_RES:x_max;  yg = y_min:p.GRID_RES:y_max;
nY = numel(yg); nX = numel(xg);
filPx   = max(1, round((p.FILAMENT_WIDTH/2)/p.GRID_RES));
closePx = max(1, round(p.STRUCT_CLOSE_MM/p.GRID_RES));
fprintf('  XY 网格 %d x %d = %d 点 (res %.2fmm)\n', nY, nX, nY*nX, p.GRID_RES);

%% ---------- 预计算: 每层 碳纤维实心截面 + 曲面 Z ----------
% 截面: 栅格化路径 -> 丝宽膨胀 -> 闭运算桥接道间空隙 -> 实心(保留真实空腔)
fprintf('  预计算每层碳纤维实心截面 + 曲面 Z ...\n');
carbon_mask = false(nY, nX, nL);
surface_z   = nan(nY, nX, nL);
tic;
for k = 1:nL
    m = rasterize_layer(ALD(k).paths_3d, xg(1), yg(1), p.GRID_RES, nX, nY, p.GRID_RES*0.5);
    if any(m(:))
        m = imdilate(m, strel('disk', filPx));            % 到丝外缘
        if p.CLOSE_GAPS
            m = imclose(m, strel('disk', closePx));       % 桥接道间空隙 -> 实心
        end
    end
    carbon_mask(:,:,k) = m;

    F = build_surface_interp(ALD(k).pointCloud_data);
    if ~isempty(F)
        try
            [Xg,Yg] = meshgrid(xg,yg);
            surface_z(:,:,k) = reshape(F(Xg(:),Yg(:)), nY, nX);
        catch
        end
    end
    if mod(k,20)==0 || k==nL
        fprintf('    层 %d/%d (累计 %.1fs)\n', k, nL, toc);
    end
end

if p.FILL_BOTTOM_GAP
    if isempty(p.Z_BASE), z_base = min(surface_z(:),[],'omitnan') - 1.0;
    else,                 z_base = p.Z_BASE; end
    fprintf('  [底部间隙填充: 开] z_base = %.3fmm\n', z_base);
else
    z_base = NaN;
    fprintf('  [底部间隙交给底座 STL, 此处仅内部空腔支撑]\n');
end

%% ---------- 预计算: 每层"上方最近碳纤维 Z" (反向一次) ----------
z_above_layer = nan(nY, nX, nL);
z_last = nan(nY,nX);
for k = nL:-1:1
    z_above_layer(:,:,k) = z_last;
    cm = carbon_mask(:,:,k); sz = surface_z(:,:,k);
    upd = cm & ~isnan(sz);  z_last(upd) = sz(upd);
end

%% ---------- 主循环: 逐层生成内部空腔支撑 ----------
fprintf('  逐层检测内部空腔并填充 ...\n');
support_data = struct('paths_3d',{},'is_solid',{},'layer_pair',{}, ...
                      'support_type',{},'signal_tag',{});
z_below_run = nan(nY,nX);

for k = 1:nL
    sz_k = surface_z(:,:,k);
    za   = z_above_layer(:,:,k);
    zb   = z_below_run;
    have_above = ~isnan(za);
    have_below = ~isnan(zb);
    no_carbon  = ~carbon_mask(:,:,k) & ~isnan(sz_k);

    cavity = no_carbon & have_above & have_below;     % 内部封闭空腔

    if p.FILL_BOTTOM_GAP
        bottom = no_carbon & have_above & ~have_below;
        zb_eff = zb;  zb_eff(bottom) = z_base;
        have_below_eff = have_below | bottom;
        void = cavity | bottom;
    else
        zb_eff = zb;  have_below_eff = have_below;
        void = cavity;
    end

    if any(void(:))
        dist_above = za - sz_k;
        dist_below = sz_k - zb_eff;
        adj_above = false(nY,nX); if k<nL, adj_above = carbon_mask(:,:,k+1); end
        adj_below = false(nY,nX); if k>1,  adj_below = carbon_mask(:,:,k-1); end
        near_carbon = (dist_above < p.CONTACT_THICK) | ...
                      (have_below_eff & (dist_below < p.CONTACT_THICK)) | ...
                      adj_above | adj_below;
        solid_mask  = void &  near_carbon;
        sparse_mask = void & ~near_carbon;

        solid_poly  = rmsmall(mask2poly(solid_mask,  xg, yg), p.MIN_AREA);
        sparse_poly = rmsmall(mask2poly(sparse_mask, xg, yg), p.MIN_AREA);

        segs = {}; flags = [];
        s1 = scan_fill(solid_poly, p.RESIN_LINE_WIDTH, p.SOLID_ANGLE);
        segs = [segs s1]; flags = [flags true(1,numel(s1))]; %#ok<AGROW>
        for a = p.GRID_ANGLES
            s2 = scan_fill(sparse_poly, grid_spacing, a);
            segs = [segs s2]; flags = [flags false(1,numel(s2))]; %#ok<AGROW>
        end

        Fk = griddedInterpolant({yg, xg}, surface_z(:,:,k), 'linear', 'none');
        paths3 = cell(1,numel(segs));
        for i = 1:numel(segs)
            xy = segs{i};
            z  = Fk(xy(:,2), xy(:,1));
            ok = ~isnan(z);
            if sum(ok)>=2, paths3{i} = [xy(ok,1), xy(ok,2), z(ok)];
            else,          paths3{i} = zeros(0,3); end
        end
        keep = cellfun(@(c) size(c,1)>=2, paths3);
        support_data(k).paths_3d     = paths3(keep);
        support_data(k).is_solid     = num2cell(logical(flags(keep)));
        support_data(k).layer_pair   = [k, min(k+1,nL)];
        support_data(k).support_type = 'cavity';
        support_data(k).signal_tag   = p.SIGNAL_TAG;

        if mod(k,5)==0 || k==nL
            fprintf('  层 %3d: 空腔XY %5d, 实心段 %4d, 网格段 %4d\n', ...
                    k, sum(void(:)), sum(flags), sum(~flags));
        end
    else
        support_data(k).paths_3d     = {};
        support_data(k).is_solid     = {};
        support_data(k).layer_pair   = [k, min(k+1,nL)];
        support_data(k).support_type = '';
        support_data(k).signal_tag   = p.SIGNAL_TAG;
    end

    cm = carbon_mask(:,:,k);  sz = surface_z(:,:,k);
    upd = cm & ~isnan(sz);    z_below_run(upd) = sz(upd);
end

%% ---------- 保存 ----------
save(p.SAVE_FILE, 'support_data', 'p', 'layer_pitch', 'z_base', '-v7.3');
n_seg = sum(arrayfun(@(s) numel(s.paths_3d), support_data));
n_lay = sum(arrayfun(@(s) ~isempty(s.paths_3d), support_data));
fprintf('[Step0 v3.1] 已保存 %s  (含支撑的层 %d, 总段 %d)\n', p.SAVE_FILE, n_lay, n_seg);

%% ---------- 完整可视化 ----------
if p.VISUALIZE
    visualize_support_full(support_data, carbon_mask, ALD, xg, yg, p);
    if n_seg==0
        fprintf('[Step0 v3.1] 未检测到内部封闭空腔(结构可能无封闭空洞).\n');
        fprintf('             开放悬垂/底部由底座 STL 处理. 若需把悬垂也做成树脂支撑, 告诉我改方案.\n');
    end
end

fprintf('[Step0 v3.1] 完成.\n');
end

%% ================= 可视化 =================
function visualize_support_full(support_data, carbon_mask, ALD, xg, yg, p)
    nL = numel(support_data);
    % 汇总树脂(线段用 NaN 分隔, 便于一次 plot3)
    RS = []; RG = [];
    for k = 1:nL
        rd = support_data(k);
        for i = 1:numel(rd.paths_3d)
            c = rd.paths_3d{i};
            if rd.is_solid{i}, RS = [RS; c; nan(1,3)]; %#ok<AGROW>
            else,              RG = [RG; c; nan(1,3)]; end %#ok<AGROW>
        end
    end
    % 碳纤维点(抽样, 作上下文/剖面)
    CB = [];
    for k = 1:nL
        Pc = ALD(k).paths_3d;
        if ~iscell(Pc), continue; end
        for i = 1:numel(Pc)
            seg = Pc{i};
            if isempty(seg)||~isnumeric(seg), continue; end
            if size(seg,1)==3 && size(seg,2)~=3, seg = seg.'; end
            if size(seg,2)>=3, CB = [CB; seg(1:max(1,round(end/40)):end,1:3)]; end %#ok<AGROW>
        end
    end
    rsP = RS(~isnan(RS(:,1)),:);  rgP = RG(~isnan(RG(:,1)),:);

    figure('Color','w','Name','Step0 v3.1 完整树脂区域/路径','Position',[60 60 1180 820]);

    % (1) 3D 全部树脂 + 碳纤维上下文
    subplot(2,2,1); hold on;
    if ~isempty(CB), plot3(CB(:,1),CB(:,2),CB(:,3),'.','Color',[0.78 0.78 0.8],'MarkerSize',1); end
    if ~isempty(RS), plot3(RS(:,1),RS(:,2),RS(:,3),'-','Color',[0.9 0.1 0.1],'LineWidth',1.1); end
    if ~isempty(RG), plot3(RG(:,1),RG(:,2),RG(:,3),'-','Color',[0.1 0.45 1],'LineWidth',0.8); end
    axis equal; grid on; view(35,22); xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D 全部树脂  红=实心 蓝=网格  灰=碳纤维');

    % (2) 俯视: 碳纤维覆盖(并集) vs 树脂覆盖
    subplot(2,2,2); hold on;
    carbon_union = any(carbon_mask,3);
    imagesc(xg, yg, carbon_union); set(gca,'YDir','normal');
    colormap(gca,[1 1 1; 0.82 0.82 0.86]); caxis([0 1]);
    if ~isempty(rsP), plot(rsP(:,1),rsP(:,2),'.','Color',[0.9 0.1 0.1],'MarkerSize',3); end
    if ~isempty(rgP), plot(rgP(:,1),rgP(:,2),'.','Color',[0.1 0.45 1],'MarkerSize',3); end
    axis equal tight; xlabel('X (mm)'); ylabel('Y (mm)');
    title('俯视: 灰=碳纤维占据(任一层)  红/蓝=树脂');

    % (3) 纵向剖面(XZ, 中间 Y 切片): 看树脂只夹在空腔里
    subplot(2,2,3); hold on;
    if ~isempty(CB)
        yc = median(CB(:,2)); band = max(1.5, 3*p.GRID_RES);
        sc = abs(CB(:,2)-yc)<band;
        plot(CB(sc,1),CB(sc,3),'.','Color',[0.6 0.6 0.65],'MarkerSize',4);
        if ~isempty(rsP), s2=abs(rsP(:,2)-yc)<band; plot(rsP(s2,1),rsP(s2,3),'.','Color',[0.9 0.1 0.1],'MarkerSize',6); end
        if ~isempty(rgP), s3=abs(rgP(:,2)-yc)<band; plot(rgP(s3,1),rgP(s3,3),'.','Color',[0.1 0.45 1],'MarkerSize',6); end
        xlabel('X (mm)'); ylabel('Z (mm)'); axis equal; grid on;
        title(sprintf('纵剖面 Y≈%.1fmm: 灰=碳纤维 红/蓝=树脂(应只在空腔)', yc));
    end

    % (4) 每层树脂段数分布
    subplot(2,2,4);
    cnt = arrayfun(@(s) numel(s.paths_3d), support_data);
    bar(1:nL, cnt, 'FaceColor',[0.3 0.55 0.85], 'EdgeColor','none');
    xlabel('层号'); ylabel('树脂段数'); grid on;
    title('各层树脂段数分布(空腔随高度变化)');
end

%% ================= 子函数 =================
function m = rasterize_layer(paths_3d, x0, y0, res, nX, nY, step)
    m = false(nY,nX);
    if ~iscell(paths_3d), return; end
    for i = 1:numel(paths_3d)
        seg = paths_3d{i};
        if isempty(seg) || ~isnumeric(seg), continue; end
        if size(seg,1)==3 && size(seg,2)~=3, seg = seg.'; end
        if size(seg,1)<1 || size(seg,2)<2, continue; end
        idx = path_cells(seg(:,1:2), x0, y0, res, nX, nY, step);
        m(idx) = true;
    end
end

function idx = path_cells(seg, x0, y0, res, nX, nY, step)
    idx = [];
    if isempty(seg), return; end
    if size(seg,1)==1
        pts = seg;
    else
        d = sqrt(sum(diff(seg,1,1).^2,2));
        Lc = [0; cumsum(d)];
        if Lc(end)==0, pts = seg(1,:);
        else
            q = (0:step:Lc(end)).';
            if q(end)<Lc(end), q=[q;Lc(end)]; end
            [Lu,ia] = unique(Lc); pts = interp1(Lu, seg(ia,:), q);
        end
    end
    ix = min(max(round((pts(:,1)-x0)/res)+1,1),nX);
    iy = min(max(round((pts(:,2)-y0)/res)+1,1),nY);
    idx = unique(sub2ind([nY,nX], iy, ix));
end

function F = build_surface_interp(pc)
    X = pc.X(:); Y = pc.Y(:); Z = pc.Z(:);
    m = ~(isnan(X)|isnan(Y)|isnan(Z));
    if sum(m)<3, F = []; return; end
    try, F = scatteredInterpolant(X(m),Y(m),Z(m),'linear','none'); catch, F = []; end
end

function tf = isempty_poly(pg)
    tf = isempty(pg) || pg.NumRegions==0 || area(pg)<1e-9;
end

function pg = rmsmall(pg, minA)
    if isempty_poly(pg), return; end
    try
        Rr = regions(pg);
        Rr = Rr(arrayfun(@area,Rr)>=minA);
        if isempty(Rr), pg = polyshape(); else, pg = union(Rr); end
    catch, end
end

function pg = mask2poly(mask, xg, yg)
    pg = polyshape();
    if ~any(mask(:)), return; end
    try
        B = bwboundaries(mask, 8, 'holes');
        for i = 1:numel(B)
            yx = B{i};
            if size(yx,1)<3, continue; end
            xy = [xg(yx(:,2)).', yg(yx(:,1)).'];
            try, pg = union(pg, polyshape(xy,'Simplify',true)); catch, end
        end
    catch, end
end

function segs = scan_fill(pg, spacing, ang_deg)
    segs = {};
    if isempty_poly(pg), return; end
    V = pg.Vertices;
    ctr = [mean(V(~isnan(V(:,1)),1)), mean(V(~isnan(V(:,2)),2))];
    pr = rotate(pg, -ang_deg, ctr);
    [xb, yb] = boundingbox(pr);
    y = yb(1) + spacing*0.5;
    while y < yb(2)
        seg = intersect(pr, [xb(1)-1, y; xb(2)+1, y]);
        if ~isempty(seg)
            idx = [0; find(isnan(seg(:,1))); size(seg,1)+1];
            for j = 1:numel(idx)-1
                sub = seg(idx(j)+1:idx(j+1)-1, :);
                if size(sub,1)>=2
                    segs{end+1} = rotate_pts(sub, ang_deg, ctr); %#ok<AGROW>
                end
            end
        end
        y = y + spacing;
    end
end

function q = rotate_pts(pts, ang_deg, ctr)
    a = deg2rad(ang_deg); R = [cos(a) -sin(a); sin(a) cos(a)];
    q = (pts - ctr)*R' + ctr;
end