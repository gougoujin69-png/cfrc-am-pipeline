function cfx = rscfx(q)
% RSCFX  ABB IRB-1200 robot configuration flags rcf = [cf1, cf4, cf6, cfx].
%   cf1/cf4/cf6 : turn (quadrant) index of joints 1/4/6
%   cfx         : compatible wrist/arm configuration code 0..7
%   q           : 6x1 joint solution (rad), as emitted to the controller.
%
% ---- BUG FIXES (see _backup_pre_fix_* for the original) ----
%  1) cfx sub-flag from J3 (cfx_b) used TWO different thresholds in the
%     original: cfx_b > -1.459095  vs  cfx_b < -1.50971. That leaves an
%     UNCOVERED band  (-1.50971, -1.459095)  (about -86.5deg .. -83.6deg)
%     in which none of the 8 branches matched, so the original hit the
%     final  error('The Cfx dose not exist!')  and ABORTED MOD generation
%     (and near the band the flag jumped 2 -> abort -> 0, i.e. spurious /
%     missed singularity flags). Unified to a SINGLE threshold J3_THR so
%     the J3 split is continuous and total -> never errors.
%  2) cf1/cf4/cf6: the first/last branches had explicit outer bounds
%     (q > d-pi ... q <= d+pi) so a value exactly at / just past a joint
%     limit fell through every branch and the flag silently stayed 0
%     (wrong config -> spurious singularity). First/last branches are now
%     open-ended, covering the whole joint range. In-range classification
%     is unchanged.

cfx = [0 0 0 0];
d = 1e-14;

% ---- cf1 : J1 quadrant ----
if     q(1) <= d - pi/2,                 cfx(1) = -2;
elseif q(1) >  d - pi/2 && q(1) <= -d,   cfx(1) = -1;
elseif q(1) > -d        && q(1) <= d+pi/2, cfx(1) = 0;
else,                                    cfx(1) = 1;
end

% ---- cf4 : J4 quadrant ----
if     q(4) <= d - pi,                   cfx(2) = -3;
elseif q(4) >  d - pi   && q(4) <= d-pi/2, cfx(2) = -2;
elseif q(4) >  d - pi/2 && q(4) <= -d,   cfx(2) = -1;
elseif q(4) > -d        && q(4) <= d+pi/2, cfx(2) = 0;
elseif q(4) >  d + pi/2 && q(4) <= d+pi, cfx(2) = 1;
else,                                    cfx(2) = 2;
end

% ---- cf6 : J6 quadrant ----
if     q(6) <= d - pi,                   cfx(3) = -3;
elseif q(6) >  d - pi   && q(6) <= d-pi/2, cfx(3) = -2;
elseif q(6) >  d - pi/2 && q(6) <= -d,   cfx(3) = -1;
elseif q(6) > -d        && q(6) <= d+pi/2, cfx(3) = 0;
elseif q(6) >  d + pi/2 && q(6) <= d+pi, cfx(3) = 1;
else,                                    cfx(3) = 2;
end

% ---- cfx : wrist/arm configuration 0..7 ----
cfx_a = 448*sin(q(2)) + 42*sin(q(2)+q(3)) + 451*sin(pi/2-(q(2)+q(3)));
cfx_b = q(3);
cfx_c = q(5);
J3_THR = -1.484;            % unified J3 boundary (midpoint of the old pair)
posA = (cfx_a >= 0);        % arm front/back
hiB  = (cfx_b >= J3_THR);   % elbow region (single, gap-free threshold)
posC = (cfx_c >= 0);        % wrist (sign of J5)

if      posA &&  hiB &&  posC, cfx(4) = 0;
elseif  posA &&  hiB && ~posC, cfx(4) = 1;
elseif  posA && ~hiB &&  posC, cfx(4) = 2;
elseif  posA && ~hiB && ~posC, cfx(4) = 3;
elseif ~posA &&  hiB &&  posC, cfx(4) = 4;
elseif ~posA &&  hiB && ~posC, cfx(4) = 5;
elseif ~posA && ~hiB &&  posC, cfx(4) = 6;
else,                          cfx(4) = 7;
end
end
