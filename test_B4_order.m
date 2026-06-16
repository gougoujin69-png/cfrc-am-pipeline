function test_B4_order()
% Demonstrate why B4 matters: snapping carbon Z to the surface must happen
% BEFORE the rigid rotation, not after. Uses the same interp2+scatteredInterpolant
% snap as Step3_1, and the original Step1_2 placement (Rz=90, Rx=165, T).

% curved surface as a plaid meshgrid (like pointCloud_data)
[X,Y] = meshgrid(linspace(0,50,60), linspace(0,40,50));
Z = 6*sin(X/12) + 4*cos(Y/15);
pc.X=X; pc.Y=Y; pc.Z=Z;

% a carbon path in XY (z will be snapped onto the surface)
t = linspace(0,1,200)'; px = 5+40*t; py = 5+30*t;

tz=deg2rad(90); tx=deg2rad(165);
Rz=[cos(tz) -sin(tz) 0; sin(tz) cos(tz) 0;0 0 1];
Rx=[1 0 0;0 cos(tx) -sin(tx);0 sin(tx) cos(tx)];
Rtot=Rx*Rz; Tr=[30 70 130];

% ---- Method B (B4, CORRECT): snap Z in original plaid frame, then rotate ----
[zB, errB] = snapZ(pc, px, py);
pathB = [px, py, zB]*Rtot' + Tr;

% ---- Method A (ORIGINAL order: rotate first, then snap on rotated cloud) ----
pcA = rotpc(pc, Rtot, Tr);
xyA = [px, py, zeros(size(px))]*Rtot' + Tr;
[zA, errA] = snapZ(pcA, xyA(:,1), xyA(:,2));   % interp2 on NON-plaid rotated grid
pathA = [xyA(:,1), xyA(:,2), zA];

d = sqrt(sum((pathA - pathB).^2, 2));
fprintf('\n=== B4: snap-then-rotate (correct) vs rotate-then-snap (original) ===\n');
fprintf('Method B interp2 used plaid grid?  %s\n', tf(errB==0));
fprintf('Method A interp2 on rotated grid threw -> fell back to scatteredInterp?  %s\n', tf(errA~=0));
fprintf('carbon displacement A vs B:  mean = %.3f mm,  max = %.3f mm\n', mean(d), max(d));
fprintf('(large displacement = original order puts carbon in the WRONG place)\n');
end

function [z, threw] = snapZ(pc, qx, qy)
threw = 0;
try
    z = interp2(pc.X, pc.Y, pc.Z, qx, qy, 'linear');
    if any(isnan(z))
        F = scatteredInterpolant(pc.X(:),pc.Y(:),pc.Z(:),'linear','nearest');
        m = isnan(z); z(m) = F(qx(m), qy(m));
    end
catch
    threw = 1;
    F = scatteredInterpolant(pc.X(:),pc.Y(:),pc.Z(:),'linear','nearest');
    z = F(qx, qy);
end
end

function pcr = rotpc(pc, R, T)
P = [pc.X(:) pc.Y(:) pc.Z(:)]*R' + T;
pcr.X = reshape(P(:,1), size(pc.X));
pcr.Y = reshape(P(:,2), size(pc.Y));
pcr.Z = reshape(P(:,3), size(pc.Z));
end

function s = tf(b), if b, s='YES'; else, s='no'; end, end
