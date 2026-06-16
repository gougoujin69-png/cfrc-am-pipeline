function quat = rot2quat(R)
% ROT2QUAT  rotation matrix -> unit quaternion [w x y z] (ABB convention).
%
% ---- BUG FIX (see _backup_pre_fix_* for the original) ----
%  The original used four INDEPENDENT  if d==w / if d==a / if d==b / if d==c
%  blocks (not elseif). On a tie (two of |w|,|a|,|b|,|c| equal to the max,
%  e.g. a 180deg rotation) more than one block ran and a later one
%  OVERWROTE the correct quaternion. It could also take sqrt of a slightly
%  negative number from round-off. This version selects exactly one branch
%  (the numerically largest component) and clamps the radicands to >= 0.
%  The per-branch formulas are unchanged.

w = 0.5*sqrt(max(0,  R(1,1) + R(2,2) + R(3,3) + 1));
a = 0.5*sqrt(max(0,  R(1,1) - R(2,2) - R(3,3) + 1));
b = 0.5*sqrt(max(0, -R(1,1) + R(2,2) - R(3,3) + 1));
c = 0.5*sqrt(max(0, -R(1,1) - R(2,2) + R(3,3) + 1));

[~, idx] = max([w, a, b, c]);
switch idx
    case 1
        quat = [w, (R(3,2)-R(2,3))/(4*w), (R(1,3)-R(3,1))/(4*w), (R(2,1)-R(1,2))/(4*w)];
    case 2
        quat = [(R(3,2)-R(2,3))/(4*a), a, (R(1,2)+R(2,1))/(4*a), (R(3,1)+R(1,3))/(4*a)];
    case 3
        quat = [(R(1,3)-R(3,1))/(4*b), (R(1,2)+R(2,1))/(4*b), b, (R(2,3)+R(3,2))/(4*b)];
    otherwise
        quat = [(R(2,1)-R(1,2))/(4*c), (R(3,1)+R(1,3))/(4*c), (R(2,3)+R(3,2))/(4*c), c];
end
end
