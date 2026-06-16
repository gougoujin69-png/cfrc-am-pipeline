function R = RZ_Optimize_PerPoint(xlsx, opts)
% RZ_OPTIMIZE_PERPOINT  Mature, per-point roll (RZ) optimisation for the
% ABB IRB-1200 dual-nozzle FP printer.
%
% Replaces YZ_AMV1_RZ_Modified.m's single hand-tuned global  delta_rz  (and
% the missing RZ_Offset_Analysis.m) with a PER-POINT search of the roll
% about the tool axis that jointly minimises singularity risk and idle-
% nozzle collision, with roll/joint continuity. Uses the fixed analytic
% kinematics (ikabb / ikbest / rscfx) and needs_moveabsj for the (improved)
% singularity test.
%
% USAGE
%   R = RZ_Optimize_PerPoint('path_23_1.xlsx');
%
% The result R has per-point fields (delta, qi, rcf, sing, clearance) plus a
% comparison vs the two baselines (roll=0 and roll=-90deg). It is saved to
% '<xlsx>_RZopt.mat' for the MOD generator to consume (use R.delta(i) in
% place of the global delta_rz, and R.sing(i) / R.qi(i,:) for MoveAbsJ).
%
% Columns of the Excel (no header): [x y z(mm)  rx ry(deg)  pot  vcs  tool].

if nargin < 2, opts = struct(); end
addpath(pwd);

% ---- geometry / placement constants (match YZ_AMV1_RZ_Modified.m) ----
firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;
base_x = 0.55;
carbon = struct('h',-50.7321e-3,'y',-50.0753e-3,'l',70.963e-3,'a', 30/180*pi, ...
                'cx',-0.5,'cy',-1.5,'cz',0.2);     % carbon xyz compensation (mm)
resin  = struct('h',-50.3541e-3,'y', 50.9039e-3,'l',71.4291e-3,'a',-30/180*pi, ...
                'cx',0,'cy',0,'cz',0);
ql0 = [0 0 pi/6 0 0 0];

YM = xlsread(xlsx);
n  = size(YM,1);
if size(YM,2) < 8, YM(:,end+1:8) = 0; YM(:,8) = 111; end

% world tip coords (m) for every point (active deposit = collision history)
tip = zeros(n,3); rx = zeros(n,1); ry = zeros(n,1); ttype = YM(:,8);
for i = 1:n
    if ttype(i)==111, t=carbon; else, t=resin; end
    tip(i,:) = [base_x + (YM(i,1)+t.cx)/1000, (YM(i,2)+t.cy)/1000, firstl + (YM(i,3)+t.cz)/1000];
    rx(i) = YM(i,4)*pi/180;  ry(i) = YM(i,5)*pi/180;
end

% ---- three strategies, each with continuity seeding ----
R.n = n; R.tip_mm = tip*1000; R.tool = ttype;
[R.delta,  R.qi,  R.rcf,  R.sing,  R.clearance,  R.feasible ] = run_perpoint(tip,rx,ry,ttype,carbon,resin,ql0,opts);
[d0, q0, ~, s0, c0] = run_fixed(tip,rx,ry,ttype,carbon,resin,ql0, 0);
[d9, q9, ~, s9, c9] = run_fixed(tip,rx,ry,ttype,carbon,resin,ql0, -pi/2);

R.baseline0   = struct('delta',d0,'qi',q0,'sing',s0,'clearance',c0);
R.baselineN90 = struct('delta',d9,'qi',q9,'sing',s9,'clearance',c9);

% ---- summary ----
fprintf('\n==== RZ per-point optimisation: %s  (%d points) ====\n', xlsx, n);
fprintf('strategy        singularities(MoveAbsJ)   min idle-clearance(mm)   IK-infeasible\n');
fprintf('roll = 0 deg          %5d                    %8.2f                 %d\n', sum(s0), 1000*minfin(c0), sum(isnan(c0)));
fprintf('roll = -90 deg        %5d                    %8.2f                 %d\n', sum(s9), 1000*minfin(c9), sum(isnan(c9)));
fprintf('per-point optimised   %5d                    %8.2f                 %d\n', sum(R.sing), 1000*minfin(R.clearance), sum(~R.feasible));
fprintf('roll range optimised: [%.1f, %.1f] deg\n', 180/pi*min(R.delta), 180/pi*max(R.delta));

outmat = [erase(xlsx,'.xlsx') '_RZopt.mat'];
save(outmat, 'R');
fprintf('saved -> %s\n', outmat);
end

% ===========================================================================
function [delta,qi,rcf,sing,clr,feas] = run_perpoint(tip,rx,ry,ttype,carbon,resin,ql0,uopts)
n=size(tip,1); delta=zeros(n,1); qi=nan(n,6); rcf=nan(n,4); sing=false(n,1);
clr=nan(n,1); feas=false(n,1);
ql=ql0; prev_delta=0; rcf_prev=[]; qprev=[];
for i=1:n
    [act,idl] = pick(ttype(i),carbon,resin);
    o = uopts; o.rcf_prev = rcf_prev;
    res = optimize_print_roll(tip(i,:), rx(i), ry(i), act, idl, ql, prev_delta, tip(1:i-1,:), o);
    if ~res.feasible
        % fall back to roll 0 so the run still produces something usable
        res.delta=0; res.qi=ql; res.rcf=rscfx(ql); res.clearance=NaN;
    end
    delta(i)=res.delta; qi(i,:)=res.qi; rcf(i,:)=res.rcf; clr(i)=res.clearance; feas(i)=res.feasible;
    [sing(i),~] = needs_moveabsj(res.qi, res.rcf, qprev, rcf_prev);
    ql=res.qi; prev_delta=res.delta; rcf_prev=res.rcf; qprev=res.qi;
end
end

function [delta,qi,rcf,sing,clr] = run_fixed(tip,rx,ry,ttype,carbon,resin,ql0,dfix)
n=size(tip,1); delta=dfix*ones(n,1); qi=nan(n,6); rcf=nan(n,4); sing=false(n,1); clr=nan(n,1);
ql=ql0; rcf_prev=[]; qprev=[];
for i=1:n
    [act,idl]=pick(ttype(i),carbon,resin);
    E1 = transl(tip(i,1),tip(i,2),tip(i,3))*troty(pi)*trotx(rx(i))*troty(ry(i))*trotz(dfix);
    Q = ikabb(E1, act.l, act.h, act.y, act.a);
    q=[]; try, q=ikbest(ql,Q); catch, q=[]; end
    if isempty(q)||any(~isfinite(q)), continue; end
    r = rscfx(q);
    P = dual_tool_points(E1,act,idl);
    cc = Inf; for k=1:size(P.body,1), if i>1, cc=min(cc, sqrt(min(sum((tip(1:i-1,:)-P.body(k,:)).^2,2)))); end, end
    qi(i,:)=q; rcf(i,:)=r; clr(i)=cc;
    [sing(i),~]=needs_moveabsj(q,r,qprev,rcf_prev);
    ql=q; rcf_prev=r; qprev=q;
end
end

function [act,idl]=pick(tt,carbon,resin)
if tt==111, act=carbon; idl=resin; else, act=resin; idl=carbon; end
end

function m = minfin(v)
v = v(isfinite(v)); if isempty(v), m=NaN; else, m=min(v); end
end
