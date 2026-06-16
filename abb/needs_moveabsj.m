function [flag, reason] = needs_moveabsj(qi, rcf, qprev, rcf_prev, opts)
% NEEDS_MOVEABSJ  Decide whether a path point must be emitted as MoveAbsJ
% (joint-space) instead of MoveL (linear), i.e. whether it is a singularity
% / configuration-change point.
%
% ---- Improves on the original YZ_AMV1 / Duanliang rule ----
% Original: flag only when rcf(4) (cfx) changed between consecutive points.
% That misses:
%   (a) changes in cf1 / cf4 / cf6 (J1/J4/J6 turn flips) -> a MoveL across
%       such a flip is rejected / reconfigured by the controller;
%   (b) the TRUE wrist singularity J5 ~ 0, where J4 and J6 align and the
%       analytic IK branch is ill-conditioned (no cfx change need occur);
%   (c) large per-joint jumps between consecutive solutions.
%
% qi,rcf      : current joint solution (rad, 1x6) and config flags (1x4)
% qprev,rcf_prev : previous point's solution / flags (use [] for the first)
% opts (optional): .j5_eps (rad, default 5deg) wrist-singularity band
%                  .qjump  (rad, default 30deg) max allowed per-joint jump
%
% flag   : true if MoveAbsJ is required
% reason : char tag ('init' | 'cf' | 'j5' | 'jump' | '')

if nargin < 5 || isempty(opts), opts = struct(); end
if ~isfield(opts,'j5_eps'), opts.j5_eps = 5*pi/180; end
if ~isfield(opts,'qjump'),  opts.qjump  = 30*pi/180; end

reason = '';
% (b) wrist singularity: J5 near zero
if abs(qi(5)) < opts.j5_eps
    flag = true; reason = 'j5'; return;
end
% first point / no predecessor
if isempty(qprev) || isempty(rcf_prev)
    flag = true; reason = 'init'; return;
end
% (a) ANY configuration-flag change (not just cfx)
if any(rcf(:).' ~= rcf_prev(:).')
    flag = true; reason = 'cf'; return;
end
% (c) large joint jump (reconfiguration the controller can't do linearly)
if max(abs(qi(:).' - qprev(:).')) > opts.qjump
    flag = true; reason = 'jump'; return;
end
flag = false;
end
