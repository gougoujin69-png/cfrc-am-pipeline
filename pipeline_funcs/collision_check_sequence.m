function R = collision_check_sequence(seq, base_pts, config, opt)
%% COLLISION_CHECK_SEQUENCE  v1  (对应第4点: 碰撞感知时序的核心校验器)
% =====================================================================
% 给定一条【有序打印序列】(ToolC 尖端路径, 打印顺序), 逐点用双喷头碰撞模型检查
% 喷嘴是否撞到【已打印几何】= 前序已打印点(折线) + 静态降低底座(表面采样点云).
%
% 复刻你 colisiondection 里的几何 (compute_tool_positions / 锥面 / 点-线段),
% config 用你的 get_default_config()(单位米). 两点增强(已注明):
%   - 已打印折线按 path_id/索引跳变插 NaN 断点, 避免跨段连线误判.
%   - 锥面按【实体喷嘴外壳】判定: 点落在锥体内部直接判碰撞(你原版只测到锥面距离,
%     深入锥体内部反而读作"远"会漏判).
%
% 输入:
%   seq      : N×5 [x y z(mm)  v1 v2(deg)]  ToolC 尖端 + 姿态(=Excel 第1-5列).
%              可选第6列 path_id(整数); 第7列 type(1=碳,2=支撑). 用于断点/报告.
%   base_pts : M×3 (mm) 静态障碍点云(降低底座表面采样), 或 []
%   config   : get_default_config() 结构体 (米制)
%   opt      : 可选
%       .radius_mm   空间预筛半径(mm), 只与该半径内的已打印点/底座点比, 默认 120
%       .retry       碰撞点是否搜索姿态(位置不动), 默认 false
%       .retry_range_deg  姿态搜索范围(±, 绕标称), 默认 25
%       .retry_coarse_deg 粗步长, 默认 5
%       .retry_fine_deg   细步长, 默认 1
%       .check_flange  是否把法兰盘也当点检测, 默认 true
%       .verbose     默认 true
%
% 输出 R:
%   .status        N×1  逐点综合状态 0安全/1风险/2碰撞
%   .toolr_status, .cone_status, .flange_status  N×1 分项
%   .min_dist      N×1  最小间距(mm; 负=深入锥体)
%   .feature       N×1 cellstr  触发源: 'prior'(前序打印) / 'base'(底座) / ''(安全)
%   .v1_fix,.v2_fix N×1  (retry 时)修正后的姿态(deg); 未改=原值
%   .resolved      N×1 logical  (retry 时)是否已解到安全
%   .summary       结构: n_safe/n_risk/n_collision, 碰撞点索引, 未解点索引
%
% 用法:
%   cfg = get_default_config();
%   R = collision_check_sequence(seq, base_pts, cfg, struct('retry',true));
% =====================================================================
if nargin<3 || isempty(config), error('需要 config = get_default_config()'); end
if nargin<4 || isempty(opt), opt = struct(); end
if ~isfield(opt,'radius_mm'),       opt.radius_mm = 120;   end
if ~isfield(opt,'retry'),           opt.retry = false;     end
if ~isfield(opt,'retry_range_deg'), opt.retry_range_deg = 25; end
if ~isfield(opt,'retry_coarse_deg'),opt.retry_coarse_deg = 5; end
if ~isfield(opt,'retry_fine_deg'),  opt.retry_fine_deg = 1; end
if ~isfield(opt,'check_flange'),    opt.check_flange = true; end
if ~isfield(opt,'verbose'),         opt.verbose = true;    end

N = size(seq,1);
xyz_m = seq(:,1:3) / 1000;                 % mm -> m
v1 = seq(:,4);  v2 = seq(:,5);
if size(seq,2)>=6, pid = seq(:,6); else, pid = (1:N).'; end
rad_m = opt.radius_mm/1000;
half = config.cone.half_angle;  depth = config.cone.depth;
coll = config.collision;
if isempty(base_pts), base_m = zeros(0,3); else, base_m = base_pts/1000; end

R.status        = zeros(N,1);
R.toolr_status  = zeros(N,1);
R.cone_status   = zeros(N,1);
R.flange_status = zeros(N,1);
R.min_dist      = inf(N,1);
R.feature       = repmat({''}, N, 1);
R.v1_fix        = v1;  R.v2_fix = v2;
R.resolved      = true(N,1);

if opt.verbose
    fprintf('[collision] 序列 %d 点, 底座障碍 %d 点, 预筛半径 %.0fmm, 姿态重试=%d\n', ...
            N, size(base_m,1), opt.radius_mm, opt.retry);
    prog = max(1, floor(N/40));
end

for i = 1:N
    tip = xyz_m(i,:);
    % --- 已打印折线(前序点, 半径预筛, 跳变插 NaN) ---
    printed = build_printed(xyz_m, pid, i, tip, rad_m);
    % --- 底座点云(半径预筛) ---
    if ~isempty(base_m)
        bnear = base_m(sum((base_m - tip).^2,2) < rad_m^2, :);
    else
        bnear = zeros(0,3);
    end

    [s, sr, sc, sf, dmin, feat] = check_pose(tip, deg2rad(v1(i)), deg2rad(v2(i)), ...
        printed, bnear, config, half, depth, coll, opt.check_flange);

    if s>=1 && opt.retry
        [v1n, v2n, s2, sr2, sc2, sf2, dmin2, feat2] = retry_orientation( ...
            tip, v1(i), v2(i), printed, bnear, config, half, depth, coll, opt);
        R.v1_fix(i)=v1n; R.v2_fix(i)=v2n;
        s=s2; sr=sr2; sc=sc2; sf=sf2; dmin=dmin2; feat=feat2;
        R.resolved(i) = (s2==0);
    elseif s>=2
        R.resolved(i) = false;
    end

    R.status(i)=s; R.toolr_status(i)=sr; R.cone_status(i)=sc; R.flange_status(i)=sf;
    R.min_dist(i)=dmin*1000; R.feature{i}=feat;        % m -> mm

    if opt.verbose && mod(i,prog)==0, fprintf('.'); end
end
if opt.verbose, fprintf(' done\n'); end

R.summary.n_safe      = sum(R.status==0);
R.summary.n_risk      = sum(R.status==1);
R.summary.n_collision = sum(R.status==2);
R.summary.collision_idx = find(R.status==2);
R.summary.unresolved_idx = find(~R.resolved);
fprintf('[collision] 安全 %d | 风险 %d | 碰撞 %d', ...
        R.summary.n_safe, R.summary.n_risk, R.summary.n_collision);
if opt.retry, fprintf(' | 重试后未解 %d', numel(R.summary.unresolved_idx)); end
fprintf('\n');
end

%% ================= 位姿检查(一次姿态) =================
function [s, sr, sc, sf, dmin, feat] = check_pose(tip, Rx, Ry, printed, bnear, ...
        config, half, depth, coll, check_flange)
    [flange, toolr_tip, ~, capex, caxis] = compute_tool_positions( ...
        tip, Rx, Ry, config.toolc, config.toolr, config.cone);

    % ToolR 尖端(点) vs 已打印折线 + 底座点云
    [dr1] = min_point_to_poly(toolr_tip, printed);
    [dr2] = min_point_to_cloud(toolr_tip, bnear);
    [sr, dr, fr] = pick(dr1, dr2, coll);

    % 锥面(实体喷嘴外壳) vs 已打印折线 + 底座点云
    [dc1, sc1] = min_cone_to_poly(capex, caxis, half, depth, printed, coll);
    [dc2, sc2] = min_cone_to_cloud(capex, caxis, half, depth, bnear, coll);
    if sc1>=sc2, sc=sc1; else, sc=sc2; end
    if dc1<=dc2, dc=dc1; fc='prior'; else, dc=dc2; fc='base'; end
    if isempty(printed)&&~isempty(bnear)&&sc==sc2, fc='base'; end

    % 法兰盘(点) vs 两者
    sf=0; df=inf; ff='';
    if check_flange
        [df1]=min_point_to_poly(flange, printed);
        [df2]=min_point_to_cloud(flange, bnear);
        [sf, df, ff] = pick(df1, df2, coll);
    end

    [s, k] = max([sr, sc, sf]);
    dd = [dr, dc, df]; ff_all = {fr, fc, ff};
    dmin = min([dr, dc, df]);
    feat = ff_all{k}; if isempty(feat), [~,kk]=min(dd); feat=ff_all{kk}; end
    if s==0, feat=''; end
end

function [status, d, feat] = pick(d1, d2, coll)
    if d1<=d2, d=d1; feat='prior'; else, d=d2; feat='base'; end
    status = status_of(d, coll);
    if status==0, feat=''; end
end
function s = status_of(d, coll)
    if d < coll.warn_dist, s=2; elseif d < coll.safe_dist, s=1; else, s=0; end
end

%% ================= 姿态重试(位置不动, 仅搜索 Rx/Ry) =================
function [v1b, v2b, sb, srb, scb, sfb, db, fb] = retry_orientation( ...
        tip, v1, v2, printed, bnear, config, half, depth, coll, opt)
    v1b=v1; v2b=v2;
    [sb, srb, scb, sfb, db, fb] = check_pose(tip, deg2rad(v1), deg2rad(v2), ...
        printed, bnear, config, half, depth, coll, opt.check_flange);
    best_dev = 0;
    for stage = 1:2
        if stage==1, step=opt.retry_coarse_deg; rng=opt.retry_range_deg; c1=v1; c2=v2;
        else,        step=opt.retry_fine_deg;   rng=2*opt.retry_coarse_deg; c1=v1b; c2=v2b; end
        for a = -rng:step:rng
            for b = -rng:step:rng
                t1=c1+a; t2=c2+b;
                [s,sr,sc,sf,d,f] = check_pose(tip, deg2rad(t1), deg2rad(t2), ...
                    printed, bnear, config, half, depth, coll, opt.check_flange);
                dev = hypot(t1-v1, t2-v2);
                better = (s < sb) || (s==sb && s>0 && d>db) || ...
                         (s==0 && sb==0 && dev<best_dev);
                if better
                    sb=s; srb=sr; scb=sc; sfb=sf; db=d; fb=f;
                    v1b=t1; v2b=t2; best_dev=dev;
                end
            end
        end
    end
end

%% ================= 距离基元 (自包含) =================
function d = min_point_to_cloud(pt, cloud)
    if isempty(cloud), d=inf; return; end
    d = sqrt(min(sum((cloud - pt).^2,2)));
end

function d = min_point_to_poly(pt, poly)
    % poly: K×3, 含 NaN 断点. 点到各线段最小距离.
    d = inf;
    if size(poly,1)<2, return; end
    for i = 1:size(poly,1)-1
        p1=poly(i,:); p2=poly(i+1,:);
        if any(isnan(p1))||any(isnan(p2)), continue; end
        v=p2-p1; w=pt-p1; c2=dot(v,v);
        if c2<1e-18, dd=norm(pt-p1);
        else, t=max(0,min(1,dot(w,v)/c2)); dd=norm(pt-(p1+t*v)); end
        if dd<d, d=dd; end
    end
end

function [dmin, smax] = min_cone_to_poly(apex, axis, half, depth, poly, coll)
    dmin=inf; smax=0;
    if size(poly,1)<1, return; end
    for i=1:size(poly,1)
        if any(isnan(poly(i,:))), continue; end
        [s,d]=cone_status(poly(i,:), apex, axis, half, depth, coll);
        if d<dmin, dmin=d; end; if s>smax, smax=s; end
    end
    for i=1:size(poly,1)-1
        p1=poly(i,:); p2=poly(i+1,:);
        if any(isnan(p1))||any(isnan(p2)), continue; end
        L=norm(p2-p1);
        if L>0.1e-3
            ns=max(3, ceil(L/0.5e-3));
            for t=linspace(0,1,ns)
                [s,d]=cone_status(p1+t*(p2-p1), apex, axis, half, depth, coll);
                if d<dmin, dmin=d; end; if s>smax, smax=s; end
            end
        end
    end
end

function [dmin, smax] = min_cone_to_cloud(apex, axis, half, depth, cloud, coll)
    dmin=inf; smax=0;
    for i=1:size(cloud,1)
        [s,d]=cone_status(cloud(i,:), apex, axis, half, depth, coll);
        if d<dmin, dmin=d; end; if s>smax, smax=s; end
    end
end

function [status, dist] = cone_status(pt, apex, axis, half, depth, coll)
    % 锥体(实体喷嘴外壳)状态. dist<0 表示点在锥体内部(碰撞).
    pt=pt(:)'; apex=apex(:)'; axis=axis(:)'/norm(axis);
    v = pt - apex; d_ax = dot(v, axis);
    if d_ax < 0
        dist = norm(v);
    elseif d_ax > depth
        bc = apex + depth*axis; br = depth*tan(half);
        vb = pt - bc; dba = dot(vb,axis); vp = vb - dba*axis; r=norm(vp);
        if r<=br, dist = d_ax - depth; else, ep=bc+br*(vp/r); dist=norm(pt-ep); end
    else
        vp = v - d_ax*axis; r = norm(vp); cone_r = d_ax*tan(half);
        if r <= cone_r
            status = 2; dist = -(cone_r - r)*cos(half); return;   % 锥体内部 -> 碰撞
        else
            dist = (r - cone_r)*cos(half);
        end
    end
    status = status_of(dist, coll);
end

function printed = build_printed(xyz_m, pid, i, tip, rad_m)
    if i<=1, printed=zeros(0,3); return; end
    prior = xyz_m(1:i-1,:); pp = pid(1:i-1);
    keep = sum((prior - tip).^2, 2) < rad_m^2;
    idx = find(keep);
    if isempty(idx), printed=zeros(0,3); return; end
    % 在索引跳变或 path_id 变化处插 NaN 断点
    out = prior(idx(1),:);
    for t = 2:numel(idx)
        if idx(t)-idx(t-1) > 1 || pp(idx(t)) ~= pp(idx(t-1))
            out = [out; nan(1,3)]; %#ok<AGROW>
        end
        out = [out; prior(idx(t),:)]; %#ok<AGROW>
    end
    printed = out;
end

%% ================= compute_tool_positions (复刻你的脚本) =================
function [flange, toolr_tip, tool_z_axis, cone_apex, cone_axis] = compute_tool_positions( ...
        toolc_tip, Rx, Ry, toolc, toolr, cone_params)
    toolc_tip = toolc_tip(:)';
    c_pi=cos(pi); s_pi=sin(pi);
    R_base=[c_pi,0,s_pi; 0,1,0; -s_pi,0,c_pi];          % troty(pi)
    c_x=cos(Rx); s_x=sin(Rx); R_x=[1,0,0; 0,c_x,-s_x; 0,s_x,c_x];
    c_y=cos(Ry); s_y=sin(Ry); R_y=[c_y,0,s_y; 0,1,0; -s_y,0,c_y];
    R_e = R_x*R_y*R_base;
    tool_z_axis = (R_e*[0;0;1])'; tool_z_axis = tool_z_axis/norm(tool_z_axis);
    c_a=cos(-toolc.a); s_a=sin(-toolc.a);
    R_tn=[c_a,0,s_a; 0,1,0; -s_a,0,c_a];                % troty(-tool_a)
    flange = toolc_tip + (R_e*R_tn*[-toolc.h; -toolc.y; -toolc.l])';
    toolr_tip = flange + (R_e*R_tn*[toolr.h; toolr.y; toolr.l])';
    if nargin>=6 && isfield(cone_params,'apex_offset'), ao=cone_params.apex_offset; else, ao=0.5e-3; end
    cone_apex = toolc_tip - ao*tool_z_axis;
    cone_axis = -tool_z_axis;
end
