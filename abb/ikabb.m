function Q = ikabb(To, tool_l, tool_h, tool_y, tool_a)
% IKABB  Analytic inverse kinematics for the ABB IRB-1200 (8 solutions).
%   To   : 4x4 desired tool pose in world frame (m)
%   Q    : 8x6 joint solutions (rad), one per [theta2,theta3,theta5] branch.
%
% ---- BUG FIX (see _backup_pre_fix_* for the original) ----
%  The original had 8 copy-pasted blocks with NO numerical guards, so an
%  unreachable / borderline target produced complex or NaN garbage that
%  flowed silently into ikbest, rscfx and the emitted jointtarget:
%    - sqrt(a3^2+d4^2-k^2)      (theta3) went complex when out of reach
%    - sqrt(u1^2+u2^2-T34^2)    (theta2) went complex when out of reach
%    - sqrt(1-c5^2)             (theta5) went complex when |c5|>1 (round-off)
%    - theta1 was UNDEFINED when v1==0 (leaked the previous branch's value)
%  This version factors the 8 branches into one guarded helper:
%    * an out-of-reach radical -> that branch returns a NaN row (so ikbest
%      skips it) instead of complex garbage;
%    * c5 is clamped to [-1,1];
%    * theta1 is defined for v1>=0 and v1<0.
%  For reachable targets the numbers are identical to the original.

T = To * trotx(-tool_a) * transl(-tool_h, -tool_y, -tool_l - 0.082);
T(3,4) = T(3,4) - 0.3991;          % subtract base height -> joint-2 frame

a2 = 0.448; a3 = 0.042; d4 = 0.451;
k  = ((T(1,4)^2 + T(2,4)^2 + T(3,4)^2) - a2^2 - a3^2 - d4^2) / (2*a2);
nx = T(1,1); ny = T(2,1); nz = T(3,1);
ox = T(1,2); oy = T(2,2); oz = T(3,2);
ax = T(1,3); ay = T(2,3); az = T(3,3);

% branch signs for [theta2-radical, theta3-radical, theta5-radical]
S = [ 1  1  1;  1  1 -1;  1 -1  1;  1 -1 -1; ...
     -1  1  1; -1  1 -1; -1 -1  1; -1 -1 -1];
Q = nan(8,6);
for r = 1:8
    Q(r,:) = ik_branch(T, a2, a3, d4, k, nx,ny,nz, ox,oy,oz, ax,ay,az, ...
                       S(r,1), S(r,2), S(r,3));
end
end

function ik = ik_branch(T, a2, a3, d4, k, nx,ny,nz, ox,oy,oz, ax,ay,az, s2, s3, s5)
ik = nan(1,6);

rad3 = a3^2 + d4^2 - k^2;
if rad3 < 0, return; end            % wrist centre out of reach
theta3 = atan2(a3, d4) - atan2(k, s3*sqrt(rad3));

u1 = a3*cos(theta3) - d4*sin(theta3) + a2;
u2 = a3*sin(theta3) + d4*cos(theta3);
rad2 = u1^2 + u2^2 - T(3,4)^2;
if rad2 < 0, return; end             % out of reach for this elbow branch
theta2 = atan2(-T(3,4), s2*sqrt(rad2)) - atan2(u2, u1);

v1 = cos(theta2)*u1 - sin(theta2)*u2;
if v1 < 0
    theta1 = atan2(-T(2,4), -T(1,4));
else
    theta1 = atan2( T(2,4),  T(1,4));   % v1>0, and v1==0 default
end

c1 = cos(theta1); s1 = sin(theta1);
c23 = cos(theta2+theta3); s23 = sin(theta2+theta3);
c5 = -ax*c1*s23 - ay*s1*s23 - az*c23;
c5 = max(-1, min(1, c5));             % clamp round-off
theta5 = atan2(s5*sqrt(1 - c5^2), c5);

if sin(theta5) < 0
    theta4 = atan2( ax*s1 - ay*c1,  ax*c1*c23 + ay*s1*c23 - az*s23);
    theta6 = atan2( ox*c1*s23 + oy*s1*s23 + oz*c23, -nx*c1*s23 - ny*s1*s23 - nz*c23);
else
    theta4 = atan2(-ax*s1 + ay*c1, -ax*c1*c23 - ay*s1*c23 + az*s23);
    theta6 = atan2(-ox*c1*s23 - oy*s1*s23 - oz*c23,  nx*c1*s23 + ny*s1*s23 + nz*c23);
end

theta2 = theta2 + pi/2;               % CreateIRB1200 L2 offset compensation
ik = [theta1, theta2, theta3, theta4, theta5, theta6];
end
