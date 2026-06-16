function qi = ikbest(qn, Q)
% IKBEST  Pick the IK solution (row of Q, 8x6 rad) that is within joint
%   limits and closest (L1) to the previous joint vector qn.
%
% ---- BUG FIX (see _backup_pre_fix_* for the original) ----
%  The joint-limit gates for J5 and J6 did NOT match CreateIRB1200.m qlim:
%    J5 was gated to +-170deg (17*pi/18) but the real limit is +-130deg
%        -> the original could SELECT an unreachable J5 pose and emit it.
%    J6 was gated to +-130deg (13*pi/18) but the real limit is +-242deg
%        -> the original REJECTED valid J6 poses and could throw a spurious
%           'solution does not exist'.
%  Gates below now match CreateIRB1200 qlim exactly:
%    J1 +-170, J2 [-100,130], J3 [-200,70], J4 +-270, J5 +-130, J6 +-242 (deg)
%  Rows that are NaN (unreachable branch from ikabb) are skipped.

lo = [-170, -100, -200, -270, -130, -242] * pi/180;
hi = [ 170,  130,   70,  270,  130,  242] * pi/180;

s = sum(abs(Q - qn(:).'), 2);    % L1 distance of each of the 8 rows to qn
s(any(~isfinite(Q), 2)) = Inf;   % drop unreachable (NaN) branches

for attempt = 1:8
    [sb, b] = min(s);
    if ~isfinite(sb)
        break;                   % no finite candidates left
    end
    if all(Q(b,:) >= lo) && all(Q(b,:) <= hi)
        qi = Q(b,:);
        return;
    end
    s(b) = Inf;                  % reject this row and try the next-closest
end
error('The solution does not exist!');
end
