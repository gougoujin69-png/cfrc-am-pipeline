function generate_base_stl(carbon_file, stl_out, P, opt)
%% GENERATE_BASE_STL  v2  贴合"有效结构"的底座 STL (棱柱)
% =====================================================================
% v2 关键改动(修足迹过大):
%   足迹不再用参考面点云(那是铺满整个域的光滑面, 会把底座撑得巨大),
%   而是用【碳纤维路径实际覆盖区】= 有效结构投影:
%     1) 把每层碳纤维路径栅格化 + 按丝宽膨胀;
%     2) 闭运算桥接碳纤维道间空隙 -> 实心截面(不被稀疏路径骗);
%     3) 各层取并集 -> 结构在 XY 的真实投影;
%     4) 再向外适当扩一圈(EXPAND_MM)生成底座足迹.
%   底座上表面 = 结构【下包络】(每个 XY 取所有层中最低的碳纤维 Z), 因此
%   自然贴合曲面底、并把开放悬垂/桥接段垫到位.
%
% 坐标系: 同 pipeline_step1_transform (R=Rx*Rz, pts*scale*R'+T). 在【原始】
%         碳纤维文件上调用并传入与 Step1 一致的变换; 或在【已变换】文件上调
%         用并把 P 留空. "抬高量" = P.TRANS(3), 决定底座高度.
%
% 输入:
%   carbon_file, stl_out, P(变换), opt(选项)
%   P:  .SCALE_FACTOR .ROT_Z_DEG .ROT_X_DEG .TRANS([dx dy dz])
%   opt:
%     .GRID_RES           网格分辨率(mm), 默认 0.5
%     .Z_BASE             底座下表面 Z, 默认 0
%     .EXPAND_MM          足迹向外扩多少(mm), 默认 3.0  <-- "适当扩大一点"
%     .STRUCT_CLOSE_MM    闭运算半径(mm), 桥接碳纤维道间空隙, 默认 3.0
%                         (设为略大于碳纤维道间距/2; 太大可能填掉真实通孔)
%     .FILAMENT_WIDTH     碳纤维丝宽(mm), 默认 0.6
%     .FILL_THROUGH_HOLES 填实通孔(实心底座), 默认 true
%     .MIN_FOOTPRINT_AREA 去除小碎片(mm^2), 默认 2.0
%     .USE_LAYER1_ONLY    只用第 1 层定足迹/上表面, 默认 false
%     .VISUALIZE          预览(含足迹 vs 碳纤维覆盖对照), 默认 true
%
% 用法:
%   P = struct('SCALE_FACTOR',1,'ROT_Z_DEG',0,'ROT_X_DEG',0,'TRANS',[0 0 5]);
%   generate_base_stl('all_layers_path_results_v3.mat','Base_Model.stl',P, ...
%                     struct('EXPAND_MM',3,'STRUCT_CLOSE_MM',3));
% =====================================================================

if nargin<3 || isempty(P), P = struct(); end
if ~isfield(P,'SCALE_FACTOR'), P.SCALE_FACTOR = 1;  end
if ~isfield(P,'ROT_Z_DEG'),    P.ROT_Z_DEG = 0;     end
if ~isfield(P,'ROT_X_DEG'),    P.ROT_X_DEG = 0;     end
if ~isfield(P,'TRANS'),        P.TRANS = [0 0 0];   end
if nargin<4 || isempty(opt), opt = struct(); end
if ~isfield(opt,'GRID_RES'),           opt.GRID_RES = 0.5;           end
if ~isfield(opt,'Z_BASE'),             opt.Z_BASE = 0;               end
if ~isfield(opt,'EXPAND_MM'),          opt.EXPAND_MM = 3.0;          end
if ~isfield(opt,'STRUCT_CLOSE_MM'),    opt.STRUCT_CLOSE_MM = 3.0;    end
if ~isfield(opt,'FILAMENT_WIDTH'),     opt.FILAMENT_WIDTH = 0.6;     end
if ~isfield(opt,'FILL_THROUGH_HOLES'), opt.FILL_THROUGH_HOLES = true;end
if ~isfield(opt,'MIN_FOOTPRINT_AREA'), opt.MIN_FOOTPRINT_AREA = 2.0; end
if ~isfield(opt,'USE_LAYER1_ONLY'),    opt.USE_LAYER1_ONLY = false;  end
if ~isfield(opt,'VISUALIZE'),          opt.VISUALIZE = true;         end

%% ---------- 读结构 ----------
fprintf('[Base v2] 读取 %s ...\n', carbon_file);
S = load(carbon_file);
if isfield(S,'results') && isfield(S.results,'all_layers_data')
    ALD = S.results.all_layers_data;
elseif isfield(S,'all_layers_data')
    ALD = S.all_layers_data;
else
    error('在 %s 找不到 all_layers_data', carbon_file);
end
nL = numel(ALD);

tz = deg2rad(P.ROT_Z_DEG); tx = deg2rad(P.ROT_X_DEG);
Rz = [cos(tz) -sin(tz) 0; sin(tz) cos(tz) 0; 0 0 1];
Rx = [1 0 0; 0 cos(tx) -sin(tx); 0 sin(tx) cos(tx)];
R  = Rx*Rz;  T = P.TRANS(:).';
if opt.USE_LAYER1_ONLY, layers = 1; else, layers = 1:nL; end
fprintf('  变换 scale=%.3g rotZ=%.3g rotX=%.3g 抬升dz=%.3g mm\n', ...
        P.SCALE_FACTOR, P.ROT_Z_DEG, P.ROT_X_DEG, T(3));

%% ---------- 收集变换后碳纤维路径(2D 段 + 3D 点) ----------
segs2d = {};  pts3 = [];
for L = layers
    Pc = ALD(L).paths_3d;
    if ~iscell(Pc), continue; end
    for i = 1:numel(Pc)
        seg = Pc{i};
        if isempty(seg) || ~isnumeric(seg), continue; end
        if size(seg,1)==3 && size(seg,2)~=3, seg = seg.'; end
        if size(seg,2)<3 || size(seg,1)<1, continue; end
        seg = seg(all(~isnan(seg(:,1:3)),2), :);
        if isempty(seg), continue; end
        seg = (seg(:,1:3) * P.SCALE_FACTOR) * R.' + T;   % 变换
        segs2d{end+1} = seg(:,1:2); %#ok<AGROW>
        pts3 = [pts3; seg(:,1:3)]; %#ok<AGROW>
    end
end
if isempty(pts3), error('结构无有效碳纤维路径点'); end

%% ---------- XY 网格(贴着碳纤维范围 + 余量) ----------
res = opt.GRID_RES;
margin = opt.EXPAND_MM + opt.FILAMENT_WIDTH + 2*res;
x0 = min(pts3(:,1))-margin; x1 = max(pts3(:,1))+margin;
y0 = min(pts3(:,2))-margin; y1 = max(pts3(:,2))+margin;
xg = x0:res:x1; yg = y0:res:y1; nX = numel(xg); nY = numel(yg);

%% ---------- 足迹 = 碳纤维覆盖 -> 膨胀(丝宽) -> 闭运算(桥接) -> 外扩 ----------
cov = false(nY,nX);
for i = 1:numel(segs2d)
    idx = path_cells(segs2d{i}, x0, y0, res, nX, nY, res*0.5);
    cov(idx) = true;
end
filPx    = max(1, round((opt.FILAMENT_WIDTH/2)/res));
closePx  = max(1, round(opt.STRUCT_CLOSE_MM/res));
expandPx = max(0, round(opt.EXPAND_MM/res));
minAreaPx= max(1, round(opt.MIN_FOOTPRINT_AREA/(res^2)));

cov = bwareaopen(cov, max(1,round(minAreaPx/4)));            % 去单像素噪声
solid = imdilate(cov, strel('disk', filPx));                % 到丝外缘
solid = imclose(solid, strel('disk', closePx));             % 桥接道间空隙 -> 实心
fp = solid;
if expandPx>0, fp = imdilate(fp, strel('disk', expandPx)); end  % 适当外扩
if opt.FILL_THROUGH_HOLES, fp = imfill(fp,'holes'); end
fp = bwareaopen(fp, minAreaPx);
if ~any(fp(:)), error('足迹为空'); end
fprintf('  碳纤维覆盖 cell=%d -> 实心 %d -> 足迹(扩%.1fmm) %d  (域 %dx%d)\n', ...
        nnz(cov), nnz(solid), opt.EXPAND_MM, nnz(fp), nY, nX);

%% ---------- 上表面 = 碳纤维下包络(每 cell 取最低 Z), 在足迹内插值/外推 ----------
ix = min(max(round((pts3(:,1)-x0)/res)+1,1),nX);
iy = min(max(round((pts3(:,2)-y0)/res)+1,1),nY);
lin = sub2ind([nY,nX], iy, ix);
envZ = accumarray(lin, pts3(:,3), [nY*nX,1], @min, NaN);    % 下包络
envZ = reshape(envZ, nY, nX);
covered = ~isnan(envZ);
[cy,cx] = find(covered);
Fz = scatteredInterpolant(xg(cx).', yg(cy).', envZ(covered), 'linear','nearest');
[fy,fx] = find(fp);
envZ_fp = nan(nY,nX);
envZ_fp(sub2ind([nY,nX],fy,fx)) = Fz(xg(fx).', yg(fy).');

%% ---------- 构网格(顶盖+底盖+侧壁), 水密+外法线(已验证逻辑) ----------
[XGn, YGn] = meshgrid(xg, yg);
Ztop = envZ_fp; Ztop(~fp) = opt.Z_BASE;
Ztop = max(Ztop, opt.Z_BASE + 1e-3);                        % 正高度防退化
Zbot = opt.Z_BASE * ones(nY,nX);
Vtop = [XGn(:), YGn(:), Ztop(:)];
Vbot = [XGn(:), YGn(:), Zbot(:)];
nN = nY*nX;  V = [Vtop; Vbot];

filled = fp(1:end-1,1:end-1) & fp(2:end,1:end-1) & ...
         fp(1:end-1,2:end)   & fp(2:end,2:end);
[fi,fj] = find(filled);
c00 = sub2ind([nY,nX], fi,   fj);
c10 = sub2ind([nY,nX], fi+1, fj);
c01 = sub2ind([nY,nX], fi,   fj+1);
c11 = sub2ind([nY,nX], fi+1, fj+1);
Ftop = [c00 c01 c11; c00 c11 c10];                          % +Z
Fbot = [c00+nN c11+nN c01+nN; c00+nN c10+nN c11+nN];        % -Z

above = [false(1,size(filled,2)); filled(1:end-1,:)];
below = [filled(2:end,:); false(1,size(filled,2))];
left  = [false(size(filled,1),1), filled(:,1:end-1)];
right = [filled(:,2:end), false(size(filled,1),1)];
eB = filled&~above; eT = filled&~below; eL = filled&~left; eR = filled&~right;

Fside = zeros(0,3);
[bi,bj]=find(eB); A=sub2ind([nY,nX],bi,bj);   B=sub2ind([nY,nX],bi,bj+1);
Fside=[Fside; A A+nN B; B A+nN B+nN];                       % 底边 -Y
[ti,tj]=find(eT); A=sub2ind([nY,nX],ti+1,tj); B=sub2ind([nY,nX],ti+1,tj+1);
Fside=[Fside; A B A+nN; B B+nN A+nN];                       % 顶边 +Y
[li,lj]=find(eL); A=sub2ind([nY,nX],li,lj);   B=sub2ind([nY,nX],li+1,lj);
Fside=[Fside; A B A+nN; B B+nN A+nN];                       % 左边 -X
[ri,rj]=find(eR); A=sub2ind([nY,nX],ri,rj+1); B=sub2ind([nY,nX],ri+1,rj+1);
Fside=[Fside; A A+nN B; B A+nN B+nN];                       % 右边 +X

F = [Ftop; Fbot; Fside];
used = unique(F(:));
remap = zeros(size(V,1),1); remap(used) = 1:numel(used);
V2 = V(used,:);  F2 = remap(F);

%% ---------- 写 STL ----------
TR = triangulation(F2, V2);
stlwrite(TR, stl_out);
fprintf('[Base v2] 已写出: %s\n', stl_out);
fprintf('  顶点 %d, 三角面 %d\n', size(V2,1), size(F2,1));
fprintf('  X[%.2f,%.2f] Y[%.2f,%.2f]  底座高度 %.2f~%.2f mm\n', ...
        min(V2(:,1)),max(V2(:,1)),min(V2(:,2)),max(V2(:,2)), ...
        min(Ztop(fp))-opt.Z_BASE, max(Ztop(fp))-opt.Z_BASE);

%% ---------- 预览(3D + 足迹 vs 碳纤维覆盖对照) ----------
if opt.VISUALIZE
    figure('Color','w','Name','底座 STL v2 (足迹贴合有效结构)','Position',[80 80 1100 460]);
    subplot(1,2,1);
    trisurf(TR,'FaceColor',[0.75 0.78 0.82],'EdgeColor','none','FaceAlpha',0.95);
    hold on; camlight; lighting gouraud; axis equal; grid on; view(35,20);
    xlabel('X'); ylabel('Y'); zlabel('Z'); title('底座 (上=结构下包络, 下=Z\_base)');

    subplot(1,2,2); hold on; axis equal; grid on;
    Bf = bwboundaries(fp,8,'noholes');
    for i=1:numel(Bf), b=Bf{i}; fill(xg(b(:,2)),yg(b(:,1)),[0.85 0.85 0.9],'EdgeColor',[0.4 0.4 0.5]); end
    Bc = bwboundaries(solid,8,'noholes');
    for i=1:numel(Bc), b=Bc{i}; plot(xg(b(:,2)),yg(b(:,1)),'-','Color',[0.1 0.3 0.9],'LineWidth',1.2); end
    xlabel('X (mm)'); ylabel('Y (mm)');
    title(sprintf('俯视: 灰=底座足迹(扩%.1fmm)  蓝=有效结构', opt.EXPAND_MM));
end
end

%% ================= 子函数 =================
function idx = path_cells(seg, x0, y0, res, nX, nY, step)
% 把一条折线按 step 加密后, 返回它经过的栅格 cell 线性索引
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