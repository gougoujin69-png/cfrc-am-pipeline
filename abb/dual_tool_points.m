function P = dual_tool_points(E1, active, idle)
% DUAL_TOOL_POINTS  Geometry of the dual-nozzle end effector for a given
% active-tool-tip pose E1 (4x4, world, metres).
%
%   active / idle : struct with fields .h .y .l .a  (metres, rad), the tool
%                   mount offset (matches Duanliang/YZ_AMV1: flange =
%                   tip * troty(-a) * transl(-h,-y,-l) ).
%
% Returns struct P:
%   P.tip       active nozzle tip (= path point)            [1x3 m]
%   P.flange    common flange centre                         [1x3 m]
%   P.idle_tip  the OTHER (idle) nozzle tip                  [1x3 m]
%   P.axis      active nozzle pointing direction (tool +Z)   [1x3]
%   P.body      sampled points along flange->idle_tip and    [Nx3 m]
%               flange->tip, used as the moving collision body
%               (the idle nozzle + holder is the main hazard).

Rot = E1(1:3,1:3);
tip = E1(1:3,4).';

Tf = E1 * troty(-active.a) * transl(-active.h, -active.y, -active.l);
flange = Tf(1:3,4).';

Ti = Tf * transl(idle.h, idle.y, idle.l) * troty(idle.a);
idle_tip = Ti(1:3,4).';

P.tip      = tip;
P.flange   = flange;
P.idle_tip = idle_tip;
P.axis     = Rot(:,3).';

% Collision body = the IDLE nozzle + flange only. The active nozzle tip
% sits ON the bead it is printing, so the active side (flange->tip) is NOT
% part of the hazard and is deliberately excluded (including it would make
% the clearance ~0 everywhere). The idle nozzle, sticking out to the side,
% is the real collision risk against previously-printed material.
seg1 = seg_pts(flange, idle_tip, 6);
P.body = [idle_tip; seg1];
end

function S = seg_pts(a, b, n)
t = linspace(0, 1, n).';
S = a + t.*(b - a);
end
