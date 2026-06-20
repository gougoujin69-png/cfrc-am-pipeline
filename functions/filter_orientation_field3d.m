function [dirs_f, coh] = filter_orientation_field3d(dirs, ijk, w, gsz, sigma_s, range_deg, iters, adapt)
%FILTER_ORIENTATION_FIELD3D  Structure-aware (edge-preserving) cleaning of a 3D
%  principal-stress DIRECTION field, intended for reference-surface fitting only.
%
%  This is the 3D analogue of filter_orientation_field.m (the 2D bilateral filter
%  applied to each layer BEFORE streamline tracing). A principal-stress direction
%  is a LINE (d == -d), so we smooth in the STRUCTURE-TENSOR domain  M = d (x) d
%  -- the 3D counterpart of the 2D double-angle cos2t/sin2t. M is invariant under
%  d -> -d, so AVERAGING tensors and taking the dominant eigenvector follows the
%  field faithfully with NO 180-deg cancellation (a plain vector average would
%  cancel anti-parallel-but-collinear neighbours to zero).
%
%  Each neighbour's contribution is weighted by
%        spatial_gauss(distance) * reliability(neighbour) * range_gauss(line-angle diff)
%  so that:
%    - similar-orientation, RELIABLE neighbours ARE averaged   -> noise removed;
%    - across a STRUCTURAL boundary (orientation flips sharply) neighbours are NOT
%      averaged                                                 -> distinct load
%      paths / sub-structures stay distinct (edge-preserving);
%    - low-reliability voxels (weak stress / low density, where sigma1 is ill
%      defined) contribute little and get pulled toward trustworthy neighbours.
%
%  This is deliberately NOT a global blur. It is bilateral + reliability-weighted
%  + LOCALLY ADAPTIVE (the range tolerance widens where the field is locally
%  incoherent and stays tight where it is already clean), so it self-tunes to each
%  part's geometry and to each part's stress distribution rather than applying one
%  fixed smoothing everywhere.
%
%  The cleaned LINE orientation is recovered as the dominant eigenvector of the
%  averaged tensor; the original arrow direction (sign / pointing) is then restored
%  from the input field -- exactly as filter_orientation_field.m and
%  extract_layer_2d_projection.m do, so the field's true sense is preserved.
%
%  IN : dirs  N x 3  sigma1 vectors (need not be unit)
%       ijk   N x 3  integer voxel indices (1-based) into the [nelx nely nelz] grid
%       w     N x 1  reliability weight >= 0 (e.g. density .* normalized stress mag)
%       gsz   [nelx nely nelz]
%       sigma_s    spatial radius in VOXELS (scale-free across parts)   [default 1.2]
%       range_deg  base line-orientation tolerance, deg                 [default 30]
%       iters      passes                                               [default 2]
%       adapt      0..1, how much the range widens where incoherent     [default 1.0]
%  OUT: dirs_f N x 3  cleaned UNIT vectors (sign restored to match dirs)
%       coh    N x 1  local coherence in [0,1] (1 = perfectly aligned neighbourhood)

    if nargin < 5 || isempty(sigma_s),   sigma_s   = 1.2; end
    if nargin < 6 || isempty(range_deg), range_deg = 30;  end
    if nargin < 7 || isempty(iters),     iters     = 2;   end
    if nargin < 8 || isempty(adapt),     adapt     = 1.0; end

    nx = gsz(1); ny = gsz(2); nz = gsz(3);
    N  = size(dirs,1);

    % --- normalize input directions; flag degenerate (zero) vectors ---
    dn   = sqrt(sum(dirs.^2,2));
    good = dn > 1e-9;
    d0   = zeros(N,3);
    d0(good,:) = dirs(good,:) ./ dn(good);
    w = max(w(:),0);

    lin = sub2ind([nx ny nz], ijk(:,1), ijk(:,2), ijk(:,3));

    % --- scatter per-voxel data into volumes ---
    Wv  = zeros(nx,ny,nz);                 % reliability weight
    D0x = zeros(nx,ny,nz); D0y = D0x; D0z = D0x;   % original unit dir (seed + sign restore)
    Wv(lin)  = w;
    D0x(lin) = d0(:,1); D0y(lin) = d0(:,2); D0z(lin) = d0(:,3);

    % weighted structure tensor  w*(d (x) d)  -- 6 unique symmetric components
    Txx = zeros(nx,ny,nz); Tyy = Txx; Tzz = Txx; Txy = Txx; Txz = Txx; Tyz = Txx;
    Txx(lin) = w.*d0(:,1).^2;     Tyy(lin) = w.*d0(:,2).^2;     Tzz(lin) = w.*d0(:,3).^2;
    Txy(lin) = w.*d0(:,1).*d0(:,2); Txz(lin) = w.*d0(:,1).*d0(:,3); Tyz(lin) = w.*d0(:,2).*d0(:,3);

    % current dominant-direction estimate (seed = original direction)
    Ux = D0x; Uy = D0y; Uz = D0z;

    % --- 3D spatial (ball) kernel ---
    r = max(1, ceil(2.5*sigma_s));
    [ox,oy,oz] = ndgrid(-r:r, -r:r, -r:r);
    ball = (ox.^2 + oy.^2 + oz.^2) <= r^2;
    ox = ox(ball); oy = oy(ball); oz = oz(ball);
    Ws = exp(-(ox.^2 + oy.^2 + oz.^2) / (2*sigma_s^2));
    rng_rad = deg2rad(max(range_deg,1e-3));

    occ  = Wv > 0;            % material voxels
    cohv = double(occ);       % coherence (init 1 -> first pass is conservative/tight)

    for it = 1:max(1,iters)
        % locally-adaptive range: widen where the field is incoherent (coh small),
        % stay tight where it is already clean (coh ~ 1) -> NOT a global tolerance.
        rad_eff = rng_rad .* (1 + adapt*(1 - cohv));

        Axx = zeros(nx,ny,nz); Ayy = Axx; Azz = Axx;
        Axy = Axx; Axz = Axx; Ayz = Axx; Wacc = Axx;

        for kk = 1:numel(ox)
            o = [ox(kk) oy(kk) oz(kk)];
            sUx = shift3(Ux,o); sUy = shift3(Uy,o); sUz = shift3(Uz,o);
            sW  = shift3(Wv,o);
            sxx = shift3(Txx,o); syy = shift3(Tyy,o); szz = shift3(Tzz,o);
            sxy = shift3(Txy,o); sxz = shift3(Txz,o); syz = shift3(Tyz,o);

            % line-angle difference between centre dir U and neighbour dir sU
            % (squared cosine -> sign-free, correct for the d == -d ambiguity)
            cdot  = Ux.*sUx + Uy.*sUy + Uz.*sUz;
            c2    = min(1, cdot.^2);
            dline = acos(sqrt(c2));                          % in [0, pi/2]
            rw    = exp(-(dline.^2) ./ (2*rad_eff.^2));       % bilateral range term

            wgt = (Ws(kk) .* sW) .* rw;                       % spatial * reliability * range
            Axx = Axx + wgt.*sxx; Ayy = Ayy + wgt.*syy; Azz = Azz + wgt.*szz;
            Axy = Axy + wgt.*sxy; Axz = Axz + wgt.*sxz; Ayz = Ayz + wgt.*syz;
            Wacc = Wacc + wgt;
        end

        Wsafe = Wacc; Wsafe(Wsafe < 1e-12) = 1;
        Bxx = Axx./Wsafe; Byy = Ayy./Wsafe; Bzz = Azz./Wsafe;
        Bxy = Axy./Wsafe; Bxz = Axz./Wsafe; Byz = Ayz./Wsafe;

        % dominant eigenvector of the averaged 3x3 symmetric tensor, via a few
        % power-iteration steps seeded with the current estimate (fully vectorized).
        vx = Ux; vy = Uy; vz = Uz;
        for pit = 1:5
            px = Bxx.*vx + Bxy.*vy + Bxz.*vz;
            py = Bxy.*vx + Byy.*vy + Byz.*vz;
            pz = Bxz.*vx + Byz.*vy + Bzz.*vz;
            pn = sqrt(px.^2 + py.^2 + pz.^2); pn(pn < 1e-12) = 1;
            vx = px./pn; vy = py./pn; vz = pz./pn;
        end

        % coherence = (lambda1/trace - 1/3)/(2/3): 1 for a perfectly aligned
        % neighbourhood (rank-1 tensor), 0 for isotropic noise.
        lam1 = vx.*(Bxx.*vx + Bxy.*vy + Bxz.*vz) ...
             + vy.*(Bxy.*vx + Byy.*vy + Byz.*vz) ...
             + vz.*(Bxz.*vx + Byz.*vy + Bzz.*vz);
        tr  = Bxx + Byy + Bzz; trs = tr; trs(trs < 1e-12) = 1;
        cohv = (lam1./trs - 1/3) / (2/3);
        cohv = min(1, max(0, cohv)); cohv(~occ) = 0;

        % update direction estimate where there was support; keep old otherwise
        hassup = (Wacc > 1e-12) & occ;
        Ux(hassup) = vx(hassup); Uy(hassup) = vy(hassup); Uz(hassup) = vz(hassup);
        un = sqrt(Ux.^2 + Uy.^2 + Uz.^2); un(un < 1e-12) = 1;
        Ux = Ux./un; Uy = Uy./un; Uz = Uz./un;
    end

    % --- gather back to the point list; restore sign from the ORIGINAL direction ---
    ufx = Ux(lin); ufy = Uy(lin); ufz = Uz(lin);
    s = sign(ufx.*d0(:,1) + ufy.*d0(:,2) + ufz.*d0(:,3)); s(s == 0) = 1;
    dirs_f = [ufx.*s, ufy.*s, ufz.*s];

    % voxels with degenerate input or no reliable support keep the raw direction
    bad = ~good | (Wv(lin) <= 0);
    dirs_f(bad,:) = d0(bad,:);

    coh = cohv(lin);
end

function B = shift3(A, o)
% B(i,j,k) = A(i-o1, j-o2, k-o3), zero-padded outside the grid (no wrap-around).
    [nx,ny,nz] = size(A); B = zeros(nx,ny,nz);
    a = o(1); b = o(2); c = o(3);
    ti = max(1,1+a):min(nx,nx+a); si = ti - a;
    tj = max(1,1+b):min(ny,ny+b); sj = tj - b;
    tk = max(1,1+c):min(nz,nz+c); sk = tk - c;
    if isempty(ti) || isempty(tj) || isempty(tk), return; end
    B(ti,tj,tk) = A(si,sj,sk);
end
