function validate_fixes()
% Validate the corrected ikabb / ikbest / rscfx / rot2quat against the
% Robotics Toolbox FK, by forward->inverse->forward round-trip.
addpath(pwd);
rng(0);

% carbon tool params (from YZ_AMV1)
tool_h = -50.7321e-3; tool_y = -50.0753e-3; tool_l = 70.963e-3; tool_a = 30/180*pi;

% Robot whose tool frame is CONSISTENT with ikabb's tool offset, so
% fkine(q) returns exactly the pose ikabb inverts.
R = CreateIRB1200('t', tool_l, tool_h, tool_a);
R.tool = transl(tool_h, tool_y, tool_l + 0.082) * trotx(tool_a);

lo = [-170 -100 -200 -270 -130 -242]*pi/180;
hi = [ 170  130   70  270  130  242]*pi/180;
margin = 5*pi/180;                  % keep away from hard limits
loS = lo + margin; hiS = hi - margin;

N = 300; okpos = 0; okrot = 0; nbad = 0; maxp = 0; maxr = 0; singerr = 0;
for t = 1:N
    q = loS + (hiS-loS).*rand(1,6);     % random reachable config
    Tm = fk(R, q);
    Q = ikabb(Tm, tool_l, tool_h, tool_y, tool_a);
    try
        qi = ikbest(q, Q);              % seed with truth -> should match
    catch
        nbad = nbad + 1; continue;
    end
    if any(~isfinite(qi)), nbad = nbad + 1; continue; end
    Tc = fk(R, qi);
    ep = norm(Tm(1:3,4) - Tc(1:3,4));   % position error (m)
    er = norm(Tm(1:3,1:3) - Tc(1:3,1:3), 'fro');  % rotation error
    maxp = max(maxp, ep); maxr = max(maxr, er);
    if ep < 1e-6, okpos = okpos + 1; end
    if er < 1e-6, okrot = okrot + 1; end
    try
        rscfx(qi);                      % must never error now
    catch
        singerr = singerr + 1;
    end
end
fprintf('\n=== IK round-trip over %d random reachable poses ===\n', N);
fprintf('  pos match (<1e-6 m): %d/%d   rot match (<1e-6): %d/%d\n', okpos, N, okrot, N);
fprintf('  max pos err = %.3e m   max rot err = %.3e\n', maxp, maxr);
fprintf('  ikbest failures: %d   rscfx errors: %d\n', nbad, singerr);

% --- dead-band: rscfx must NOT error and must classify continuously ---
fprintf('\n=== rscfx former dead-band sweep (J3) ===\n');
errs = 0;
for j3 = -1.55:0.005:-1.42
    q = [0, pi/4, j3, 0, 0.3, 0];
    try, r = rscfx(q); catch, errs = errs + 1; end
end
fprintf('  errors across former dead-band: %d (expect 0)\n', errs);
fprintf('  rscfx at J3=-1.48 (was abort): cfx=[%d %d %d %d]\n', rscfx([0 pi/4 -1.48 0 0.3 0]));

% --- rot2quat round-trip ---
fprintf('\n=== rot2quat round-trip ===\n');
maxq = 0;
for t = 1:200
    ax = randn(3,1); ax = ax/norm(ax); th = pi*rand;
    Rm = axang(ax, th);
    qq = rot2quat(Rm);
    Rb = quat2R(qq);
    maxq = max(maxq, norm(Rm - Rb, 'fro'));
end
fprintf('  max ||R - quat2R(rot2quat(R))|| = %.3e (expect ~0)\n', maxq);
fprintf('\nDONE.\n');
end

function Tm = fk(R, q)
T = R.fkine(q);
if isa(T,'SE3'), Tm = T.T; else, Tm = double(T); end
end

function Rm = axang(ax, th)
x=ax(1); y=ax(2); z=ax(3); c=cos(th); s=sin(th); v=1-c;
Rm = [c+x*x*v, x*y*v-z*s, x*z*v+y*s; ...
      y*x*v+z*s, c+y*y*v, y*z*v-x*s; ...
      z*x*v-y*s, z*y*v+x*s, c+z*z*v];
end

function Rm = quat2R(q)
w=q(1); x=q(2); y=q(3); z=q(4);
Rm = [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w); ...
      2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w); ...
      2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)];
end
