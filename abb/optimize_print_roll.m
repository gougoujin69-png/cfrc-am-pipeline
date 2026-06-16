function out = optimize_print_roll(tip_m, rx, ry, active, idle, ql, prev_delta, printed_m, opts)
% OPTIMIZE_PRINT_ROLL  Per-point roll-about-tool-axis optimiser (FAST).
%
% Searches the redundant DOF delta (a roll about the tool Z = surface normal,
% appended as ...*trotz(delta)) to avoid robot singularities (|J5| away from 0,
% inside joint limits, no config flips) and idle-nozzle collisions, with roll/
% joint continuity. Rolling about the normal does not change the nozzle
% direction, so print quality is preserved.
%
% PERFORMANCE (why the previous version took hours on 842k points):
%   * NO exceptions: a local no-throw IK selector replaces ikbest+try/catch
%     (ikbest's error() in a 70M-iteration loop cost ~ms each).
%   * LAZY/adaptive (opts.lazy, default true): first test ONLY the incumbent
%     roll (prev_delta); if it is already non-singular / in-limits / clear,
%     keep it and skip the full sweep. Singular points are ~8-15%, so the
%     expensive sweep runs only there (~10x fewer IK evals overall).
%   * The static pose base transl*troty(pi)*trotx(rx)*troty(ry) is built ONCE
%     per point; each roll candidate only multiplies by an inline trotz.
%
% OUTPUT struct OUT: .delta .qi .rcf .clearance .j5 .lim .cfg .cost .feasible

if nargin < 9, opts = struct(); end
D = struct('range',[-pi pi],'coarse',5*pi/180,'fine',1*pi/180, ...
           'j5_safe',15*pi/180,'lim_safe',10*pi/180,'coll_safe',8e-3, ...
           'w_coll',10,'w_j5',8,'w_lim',3,'w_cfg',2,'w_cont',0.5,'w_jump',0.2, ...
           'rcf_prev',[],'coll_stride',1,'lazy',true);
fn = fieldnames(D); for i=1:numel(fn), if ~isfield(opts,fn{i}), opts.(fn{i})=D.(fn{i}); end, end

lo = [-170 -100 -200 -270 -130 -242]*pi/180;
hi = [ 170  130   70  270  130  242]*pi/180;

% static pose base for THIS point (built once, not per candidate)
Pbase = transl(tip_m(1),tip_m(2),tip_m(3))*troty(pi)*trotx(rx)*troty(ry);

% ---- lazy: try the incumbent roll first ----
if opts.lazy
    inc = eval_one(prev_delta, Pbase, active, idle, ql, prev_delta, printed_m, opts, lo, hi);
    if inc.feasible && is_good(inc, opts)
        out = inc; return;        % incumbent already fine -> no sweep
    end
end

% ---- full coarse sweep then local refine ----
best = eval_grid(opts.range(1):opts.coarse:opts.range(2), Pbase, active, idle, ql, prev_delta, printed_m, opts, lo, hi);
if best.feasible
    best2 = eval_grid((best.delta-opts.coarse):opts.fine:(best.delta+opts.coarse), Pbase, active, idle, ql, prev_delta, printed_m, opts, lo, hi);
    if best2.feasible && best2.cost <= best.cost, best = best2; end
end
out = best;
end

% ---------------------------------------------------------------------------
function best = eval_grid(cand, Pbase, active, idle, ql, prev_delta, printed_m, opts, lo, hi)
best = empty_res();
for d = cand
    r = eval_one(d, Pbase, active, idle, ql, prev_delta, printed_m, opts, lo, hi);
    if r.feasible && r.cost < best.cost, best = r; end
end
end

function r = eval_one(d, Pbase, active, idle, ql, prev_delta, printed_m, opts, lo, hi)
r = empty_res(); r.delta = d;
E1 = Pbase * trotz_h(d);
Q  = ikabb(E1, active.l, active.h, active.y, active.a);
qi = pick_ik(ql, Q, lo, hi);          % no-throw
if isempty(qi), return; end

j5  = abs(qi(5));
lim = min(min(qi - lo), min(hi - qi));
rcf = rscfx(qi);
cfg = 0; if ~isempty(opts.rcf_prev), cfg = any(rcf(:).' ~= opts.rcf_prev(:).'); end
c_sing = opts.w_j5*ramp(j5,opts.j5_safe) + opts.w_lim*ramp(lim,opts.lim_safe) + opts.w_cfg*cfg;

if opts.w_coll > 0
    clr = tool_clearance(E1, active, idle, printed_m, opts.coll_stride);
    c_coll = opts.w_coll*ramp(clr, opts.coll_safe);
else
    clr = Inf; c_coll = 0;
end
c_cont = opts.w_cont*angdiff(d,prev_delta) + opts.w_jump*max(abs(qi-ql));

r.qi=qi; r.rcf=rcf; r.clearance=clr; r.j5=j5; r.lim=lim; r.cfg=cfg;
r.cost = c_sing + c_coll + c_cont; r.feasible = true;
end

function tf = is_good(r, opts)
tf = r.j5 >= opts.j5_safe && r.lim >= opts.lim_safe && ~r.cfg && r.clearance >= opts.coll_safe;
end

function r = empty_res()
r = struct('delta',0,'qi',[],'rcf',[],'clearance',NaN,'j5',NaN,'lim',NaN,'cfg',0,'cost',Inf,'feasible',false);
end

function qi = pick_ik(qn, Q, lo, hi)
% no-throw nearest-within-limits selector (replaces ikbest+try/catch)
qi = [];
s = sum(abs(Q - qn(:).'), 2); s(any(~isfinite(Q),2)) = Inf;
for a = 1:8
    [sb,b] = min(s); if ~isfinite(sb), return; end
    if all(Q(b,:) >= lo) && all(Q(b,:) <= hi), qi = Q(b,:); return; end
    s(b) = Inf;
end
end

function T = trotz_h(a)
c=cos(a); s=sin(a); T=[c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end

function p = ramp(x, safe)
p = max(0, (safe - x)) / max(safe, eps);
end

function a = angdiff(x, y)
a = abs(mod(x - y + pi, 2*pi) - pi);
end

function clr = tool_clearance(E1, active, idle, printed_m, stride)
if nargin < 5 || isempty(stride), stride = 1; end
if isempty(printed_m), clr = Inf; return; end
if stride > 1, printed_m = printed_m(1:stride:end, :); end
P = dual_tool_points(E1, active, idle);
B = P.body;
clr = Inf;
for k = 1:size(B,1)
    dd = printed_m - B(k,:);
    clr = min(clr, sqrt(min(sum(dd.^2,2))));
end
end
