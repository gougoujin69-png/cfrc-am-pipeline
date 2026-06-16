function base_support_data = generate_base_and_interface_support(carbon_file, stl_out, P, user_params)
%% GENERATE_BASE_AND_INTERFACE_SUPPORT  v2  (足迹来自真实碳纤维, 不是切片曲面)
% =====================================================================
% v2 修复: v1 误用 pointCloud_data(覆盖整个成型域的切片参考面)当结构, 导致支撑
%          铺满一大片没用的曲面. v2 改用【真实碳纤维路径 connected_paths】的 XY 投影
%          作结构足迹, 只在结构所在的那一小块区域算下包络/挖支撑.
%
% 思路:
%   1) 收集所有层 connected_paths(回退 paths_3d) 的点 = 真实结构, 施加变换 P.
%   2) 这些点 XY 投影 + 闭运算(补合相邻打印道间隙) = 结构足迹 fp (那一小块).
%   3) 足迹内: 每个 XY 取碳纤维最低 Z = 下包络 Senv = 底座顶面.
%   4) Senv 的曲率/坡度 -> 高曲率掩膜 Hc; 顶部 INTERFACE_BAND 厚 = 界面带.
%   5) 产物: (a) 降低底座 STL = 足迹挖掉高曲率列、顶面留 Δ 间隙 (给 FDM);
%            (b) base_support_data = 挖出区(高曲率列全高 + 顶部界面带)按平 Z 层切片,
%                在碳纤维 L1 之前自底向上打.
%
% 坐标系: 传已变换完的 Manufacturing_printing_path.mat + P=单位阵, 与碰撞校验同框.
%
% 用法:
%   P = struct('SCALE_FACTOR',1,'ROT_Z_DEG',0,'ROT_X_DEG',0,'TRANS',[0 0 0]);
%   generate_base_and_interface_support('Manufacturing_printing_path.mat','Base_Reduced.stl',P);
% =====================================================================

%% ---------- 参数 ----------
q.KAPPA_MAX        = 0.30;  % 平均曲率阈值(1/mm)
q.SLOPE_MAX        = 35;    % 坡度阈值(deg)
q.INTERFACE_BAND   = 1.0;   % 顶部界面带厚 Δ(mm)
q.CURV_DILATE      = 0.8;   % 高曲率区膨胀(mm)
q.BASE_LAYER_H     = 0.2;   % 底座支撑平Z层高(mm)
q.GRID_RES         = 0.5;   % 包络/填充XY分辨率(mm)
q.Z_BASE           = 0;     % 底座下表面Z
q.RESIN_LINE_WIDTH = 0.6;
q.CONTACT_THICK    = 0.6;   % 距结构底<此->实心
q.INFILL_DENSITY   = 0.20;
q.GRID_ANGLES      = [0 90];
q.SOLID_ANGLE      = 0;
q.MIN_AREA         = 1.0;
q.MIN_FOOTPRINT_AREA = 2.0; % 丢掉<此面积(mm^2)的碎足迹
q.FOOTPRINT_CLOSE  = 3.0;   % 闭运算半径(mm), 补合相邻打印道间隙成实心足迹
q.FILL_HOLES       = false; % 是否填充足迹内部孔洞(默认否, 保留真实通孔)
q.CARBON_FIELD     = '';    % 留空=自动 connected_paths->paths_3d
q.SIGNAL_TAG       = [0 23 222];
q.SAVE_FILE        = 'base_support_paths.mat';
q.VISUALIZE        = true;
if nargin>=4 && ~isempty(user_params)
    fn=fieldnames(user_params); for i=1:numel(fn), q.(fn{i})=user_params.(fn{i}); end
end
if nargin<2 || isempty(stl_out), stl_out='Base_Reduced.stl'; end
if nargin<3 || isempty(P), P=struct(); end
if ~isfield(P,'SCALE_FACTOR'), P.SCALE_FACTOR=1; end
if ~isfield(P,'ROT_Z_DEG'), P.ROT_Z_DEG=0; end
if ~isfield(P,'ROT_X_DEG'), P.ROT_X_DEG=0; end
if ~isfield(P,'TRANS'), P.TRANS=[0 0 0]; end
warning('off','MATLAB:polyshape:repairedBySimplify');
warning('off','MATLAB:polyshape:boolOperationFailed');

%% ---------- 读碳纤维路径 ----------
fprintf('[Base+IF v2] 读取 %s ...\n', carbon_file);
C = load(carbon_file);
if isfield(C,'results') && isfield(C.results,'all_layers_data'), ALD=C.results.all_layers_data;
elseif isfield(C,'all_layers_data'), ALD=C.all_layers_data;
else, error('在 %s 找不到 all_layers_data', carbon_file); end
nL = numel(ALD);

% 选碳纤维字段
if ~isempty(q.CARBON_FIELD) && isfield(ALD,q.CARBON_FIELD), cF=q.CARBON_FIELD;
elseif isfield(ALD,'connected_paths'), cF='connected_paths';
elseif isfield(ALD,'paths_3d'), cF='paths_3d';
else, error('找不到 connected_paths / paths_3d 字段'); end
fprintf('  结构源字段: %s (真实碳纤维路径)\n', cF);

tz=deg2rad(P.ROT_Z_DEG); tx=deg2rad(P.ROT_X_DEG);
Rz=[cos(tz) -sin(tz) 0; sin(tz) cos(tz) 0; 0 0 1];
Rx=[1 0 0; 0 cos(tx) -sin(tx); 0 sin(tx) cos(tx)];
R=Rx*Rz; T=P.TRANS(:).'; mm=P.SCALE_FACTOR;

% 收集所有层碳纤维点
acc={};
for L=1:nL
    if ~isfield(ALD(L),cF) || isempty(ALD(L).(cF)), continue; end
    c=ALD(L).(cF); if ~iscell(c), continue; end
    for i=1:numel(c)
        s=c{i}; if isempty(s)||~isnumeric(s), continue; end
        if size(s,2)~=3 && size(s,1)==3, s=s.'; end
        if size(s,2)>=3, acc{end+1}=s(:,1:3); end %#ok<AGROW>
    end
end
if isempty(acc), error('没有碳纤维路径点 (检查 %s 字段)', cF); end
allP = cat(1, acc{:});
allP = (allP*mm)*R.' + T;
fprintf('  碳纤维点 %d, XY 范围 X[%.1f,%.1f] Y[%.1f,%.1f]\n', size(allP,1), ...
        min(allP(:,1)),max(allP(:,1)),min(allP(:,2)),max(allP(:,2)));

%% ---------- 结构足迹 + 下包络 (只在结构所在区域) ----------
res = q.GRID_RES; mrg = 2;
x0=min(allP(:,1))-mrg; x1=max(allP(:,1))+mrg;
y0=min(allP(:,2))-mrg; y1=max(allP(:,2))+mrg;
gx=x0:res:x1; gy=y0:res:y1; nX=numel(gx); nY=numel(gy);
ix=min(max(round((allP(:,1)-x0)/res)+1,1),nX);
iy=min(max(round((allP(:,2)-y0)/res)+1,1),nY);
lin=sub2ind([nY,nX], iy, ix);
Senv = accumarray(lin, allP(:,3), [nY*nX,1], @min, NaN);   % 每 cell 取碳纤维最低 Z
Senv = reshape(Senv, nY, nX);
covered = ~isnan(Senv);

% 足迹: 去碎块 + 闭运算补道间隙 (默认不填孔, 保留真实通孔)
minAreaPx = max(1, round(q.MIN_FOOTPRINT_AREA/res^2));
fp = bwareaopen(covered, minAreaPx);
fp = imclose(fp, strel('disk', max(1,round(q.FOOTPRINT_CLOSE/res))));
if q.FILL_HOLES, fp = imfill(fp,'holes'); end
fprintf('  结构足迹: %d cell (%.0f mm^2)\n', nnz(fp), nnz(fp)*res^2);

% 足迹内补齐 Senv (闭运算新增的 cell 用插值填)
need = fp & ~covered;
if any(need(:))
    [cy,cx]=find(covered); vals=Senv(sub2ind([nY,nX],cy,cx));
    Fz=scatteredInterpolant(gx(cx).',gy(cy).',vals,'linear','nearest');
    [ny_,nx_]=find(need); Senv(sub2ind([nY,nX],ny_,nx_))=Fz(gx(nx_).',gy(ny_).');
end
Senv(~fp) = NaN;

%% ---------- (1) 曲率 + 坡度 -> 高曲率掩膜 ----------
S = Senv;
[Sy,Sx]   = gradient(S, res, res);
[Syy,~]   = gradient(Sy, res, res);
[Sxy,Sxx] = gradient(Sx, res, res);
H = ((1+Sx.^2).*Syy - 2*Sx.*Sy.*Sxy + (1+Sy.^2).*Sxx) ./ (2*(1+Sx.^2+Sy.^2).^1.5);
slope = atand(hypot(Sx, Sy));
Hc = fp & ((abs(H) > q.KAPPA_MAX) | (slope > q.SLOPE_MAX));
Hc(isnan(H)) = false;
if q.CURV_DILATE>0, Hc = imdilate(Hc, strel('disk', max(1,round(q.CURV_DILATE/res)))) & fp; end
fprintf('  高曲率/陡坡 cell %d (占足迹 %.1f%%)\n', nnz(Hc), 100*nnz(Hc)/max(1,nnz(fp)));

%% ---------- (2) 降低底座 STL ----------
fp_base = fp & ~Hc;
Ztop = Senv - q.INTERFACE_BAND;  Ztop = max(Ztop, q.Z_BASE + 1e-3);
write_prism_stl(stl_out, fp_base, gx, gy, Ztop, q.Z_BASE);
fprintf('[Base+IF v2] 降低底座 STL: %s\n', stl_out);

%% ---------- (3) 挖出区 -> 平 Z 层支撑 ----------
grid_spacing = q.RESIN_LINE_WIDTH / q.INFILL_DENSITY;
z_top_max = max(Senv(fp));
z_levels = (q.Z_BASE + q.BASE_LAYER_H) : q.BASE_LAYER_H : z_top_max;
fprintf('  挖出区支撑: %d 个平 Z 层 (层高 %.2f, 至 %.2f)\n', numel(z_levels), q.BASE_LAYER_H, z_top_max);

base_support_data = struct('paths_3d',{},'is_solid',{},'z_level',{}, ...
                           'support_type',{},'signal_tag',{},'print_before_carbon',{});
[GX,GY] = meshgrid(gx,gy);
tic;
for idx = 1:numel(z_levels)
    z = z_levels(idx);
    under   = fp & (z <= Senv);
    in_band = z >= (Senv - q.INTERFACE_BAND);
    region  = under & (Hc | in_band);
    if ~any(region(:)), base_support_data(idx)=pack_base({},{},z,q.SIGNAL_TAG); continue; end
    solid_m = region & (z >= (Senv - q.CONTACT_THICK));
    grid_m  = region & ~solid_m;
    solid_poly = rmsmall(mask2poly(solid_m, gx, gy), q.MIN_AREA);
    grid_poly  = rmsmall(mask2poly(grid_m,  gx, gy), q.MIN_AREA);
    segs={}; flags=[];
    s1 = scan_fill(solid_poly, q.RESIN_LINE_WIDTH, q.SOLID_ANGLE);
    segs=[segs s1]; flags=[flags true(1,numel(s1))]; %#ok<AGROW>
    for a=q.GRID_ANGLES
        s2=scan_fill(grid_poly, grid_spacing, a);
        segs=[segs s2]; flags=[flags false(1,numel(s2))]; %#ok<AGROW>
    end
    paths3=cell(1,numel(segs));
    for i=1:numel(segs), xy=segs{i}; paths3{i}=[xy(:,1),xy(:,2),z*ones(size(xy,1),1)]; end
    keep=cellfun(@(c) size(c,1)>=2, paths3);
    base_support_data(idx)=pack_base(paths3(keep), num2cell(logical(flags(keep))), z, q.SIGNAL_TAG);
    if mod(idx,10)==0 || idx==numel(z_levels)
        fprintf('  Z层 %3d/%-3d z=%.2f 段 %d (%.1fs)\n', idx, numel(z_levels), z, sum(keep), toc);
    end
end

save(q.SAVE_FILE, 'base_support_data','q','P','-v7.3');
nseg = sum(arrayfun(@(s) numel(s.paths_3d), base_support_data));
fprintf('[Base+IF v2] 已保存 %s (底座支撑总段 %d)\n', q.SAVE_FILE, nseg);

%% ---------- 可视化 ----------
if q.VISUALIZE
    figure('Color','w','Name','底座分析v2: 仅结构足迹','Position',[60 60 1150 460]);
    subplot(1,2,1);
    Sp=Senv; surf(GX,GY,Sp,double(Hc),'EdgeColor','none'); hold on;
    colormap(gca,[0.7 0.78 0.85; 0.95 0.3 0.2]); caxis([0 1]);
    view(40,30); axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
    title('结构(碳纤维)下包络面: 红=高曲率/陡坡');
    subplot(1,2,2); hold on;
    RS=[]; RG=[];
    for idx=1:numel(base_support_data)
        sd=base_support_data(idx);
        for i=1:numel(sd.paths_3d)
            c=sd.paths_3d{i};
            if sd.is_solid{i}, RS=[RS;c;nan(1,3)]; else, RG=[RG;c;nan(1,3)]; end %#ok<AGROW>
        end
    end
    if ~isempty(RS), plot3(RS(:,1),RS(:,2),RS(:,3),'-','Color',[0.9 0.1 0.1],'LineWidth',1.0); end
    if ~isempty(RG), plot3(RG(:,1),RG(:,2),RG(:,3),'-','Color',[0.1 0.45 1],'LineWidth',0.7); end
    axis equal; grid on; view(40,25); xlabel('X'); ylabel('Y'); zlabel('Z');
    title('挖出区支撑(平Z层, 仅结构下方): 红=实心 蓝=网格');
end
fprintf('[Base+IF v2] 完成.\n');
end

%% ================= 子函数 =================
function sd = pack_base(paths3, issolid, z, tag)
    sd.paths_3d=paths3; sd.is_solid=issolid; sd.z_level=z;
    sd.support_type='base_interface'; sd.signal_tag=tag; sd.print_before_carbon=1;
end
function write_prism_stl(stl_out, fp, gx, gy, Ztop, Zbase)
    nY=numel(gy); nX=numel(gx); [GX,GY]=meshgrid(gx,gy);
    Vtop=[GX(:),GY(:),Ztop(:)]; Vbot=[GX(:),GY(:),Zbase*ones(nY*nX,1)];
    nN=nY*nX; V=[Vtop;Vbot];
    filled=fp(1:end-1,1:end-1)&fp(2:end,1:end-1)&fp(1:end-1,2:end)&fp(2:end,2:end);
    if ~any(filled(:)), warning('降低底座足迹为空, 不写 STL'); return; end
    [fi,fj]=find(filled);
    c00=sub2ind([nY,nX],fi,fj); c10=sub2ind([nY,nX],fi+1,fj);
    c01=sub2ind([nY,nX],fi,fj+1); c11=sub2ind([nY,nX],fi+1,fj+1);
    Ftop=[c00 c01 c11; c00 c11 c10];
    Fbot=[c00+nN c11+nN c01+nN; c00+nN c10+nN c11+nN];
    above=[false(1,size(filled,2)); filled(1:end-1,:)];
    below=[filled(2:end,:); false(1,size(filled,2))];
    left =[false(size(filled,1),1), filled(:,1:end-1)];
    right=[filled(:,2:end), false(size(filled,1),1)];
    eB=filled&~above; eT=filled&~below; eL=filled&~left; eR=filled&~right;
    Fs=zeros(0,3);
    [bi,bj]=find(eB); A=sub2ind([nY,nX],bi,bj); B=sub2ind([nY,nX],bi,bj+1); Fs=[Fs; A A+nN B; B A+nN B+nN];
    [ti,tj]=find(eT); A=sub2ind([nY,nX],ti+1,tj); B=sub2ind([nY,nX],ti+1,tj+1); Fs=[Fs; A B A+nN; B B+nN A+nN];
    [li,lj]=find(eL); A=sub2ind([nY,nX],li,lj); B=sub2ind([nY,nX],li+1,lj); Fs=[Fs; A B A+nN; B B+nN A+nN];
    [ri,rj]=find(eR); A=sub2ind([nY,nX],ri,rj+1); B=sub2ind([nY,nX],ri+1,rj+1); Fs=[Fs; A A+nN B; B A+nN B+nN];
    F=[Ftop;Fbot;Fs];
    used=unique(F(:)); remap=zeros(size(V,1),1); remap(used)=1:numel(used);
    TR=triangulation(remap(F), V(used,:)); stlwrite(TR, stl_out);
end
function tf = isempty_poly(pg), tf = isempty(pg)||pg.NumRegions==0||area(pg)<1e-9; end
function pg = rmsmall(pg, minA)
    if isempty_poly(pg), return; end
    try, Rr=regions(pg); Rr=Rr(arrayfun(@area,Rr)>=minA);
        if isempty(Rr), pg=polyshape(); else, pg=union(Rr); end
    catch, end
end
function pg = mask2poly(mask, gx, gy)
    pg=polyshape(); if ~any(mask(:)), return; end
    try
        B=bwboundaries(mask,8,'holes');
        for i=1:numel(B)
            yx=B{i}; if size(yx,1)<3, continue; end
            xy=[gx(yx(:,2)).', gy(yx(:,1)).'];
            try, pg=union(pg, polyshape(xy,'Simplify',true)); catch, end
        end
    catch, end
end
function segs = scan_fill(pg, spacing, ang_deg)
    segs={}; if isempty_poly(pg), return; end
    Vv=pg.Vertices; ctr=[mean(Vv(~isnan(Vv(:,1)),1)), mean(Vv(~isnan(Vv(:,2)),2))];
    pr=rotate(pg,-ang_deg,ctr); [xb,yb]=boundingbox(pr);
    y=yb(1)+spacing*0.5;
    while y<yb(2)
        seg=intersect(pr,[xb(1)-1,y; xb(2)+1,y]);
        if ~isempty(seg)
            idx=[0; find(isnan(seg(:,1))); size(seg,1)+1];
            for j=1:numel(idx)-1
                sub=seg(idx(j)+1:idx(j+1)-1,:);
                if size(sub,1)>=2, segs{end+1}=rotate_pts(sub,ang_deg,ctr); end %#ok<AGROW>
            end
        end
        y=y+spacing;
    end
end
function w = rotate_pts(pts, ang_deg, ctr)
    a=deg2rad(ang_deg); Rm=[cos(a) -sin(a); sin(a) cos(a)]; w=(pts-ctr)*Rm'+ctr;
end