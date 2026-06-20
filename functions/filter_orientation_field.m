function [tf, coh] = filter_orientation_field(t, mask, sigma_s, range_deg, iters)
%FILTER_ORIENTATION_FIELD  Edge-preserving (bilateral) cleaning of a 2D
%  principal-stress orientation field.
%
%  Smooths the line field in the DOUBLE-ANGLE domain (cos2t/sin2t, correct for
%  the 180-deg ambiguity). Each neighbour's weight is
%        spatial_gauss(distance) * material * range_gauss(orientation difference)
%  so that:
%    - similar-orientation neighbours ARE averaged   -> removes noise, smooth transitions
%    - very-different neighbours (feature boundaries) are NOT averaged
%                                                     -> distinct regions stay distinct
%  This is a bilateral filter: unlike a plain Gaussian it cleans noise WITHOUT
%  washing out the large-scale stress pattern.
%
%  IN : t         nely x nelx orientation angle (rad)
%       mask      nely x nelx logical (material region)
%       sigma_s   spatial radius / strength (px)            [default 1.0]
%       range_deg orientation tolerance (deg): neighbours whose line-angle
%                 differs by <~range_deg are smoothed together; larger features
%                 are preserved. (Inf -> plain Gaussian.)   [default 30]
%       iters     passes                                     [default 2]
%  OUT: tf  cleaned angle (rad); void cells unchanged.  coh  coherence in [0,1].

    if nargin < 3 || isempty(sigma_s),   sigma_s   = 1; end
    if nargin < 4 || isempty(range_deg), range_deg = 30;  end
    if nargin < 5 || isempty(iters),     iters     = 2;   end

    [ny,nx] = size(t);
    w0 = double(mask > 0);
    c2 = cos(2*t) .* w0;  s2 = sin(2*t) .* w0;          % double-angle, 0 in void

    r  = max(1, ceil(2.5*sigma_s));
    [ax,ay] = meshgrid(-r:r, -r:r);
    Ws = exp(-(ax.^2 + ay.^2) / (2*sigma_s^2));         % spatial kernel
    rng_rad = deg2rad(max(range_deg, 1e-3));
    coh = w0;

    for it = 1:max(1,iters)
        m = hypot(c2,s2); m(m<1e-9) = 1;
        uc = c2./m;  us = s2./m;                         % unit center field
        C = zeros(ny,nx);  Sm = zeros(ny,nx);  W = zeros(ny,nx);
        for kk = 1:numel(ax)
            a = ax(kk);  b = ay(kk);
            sc = shift2(c2,a,b);  ss = shift2(s2,a,b);  sw = shift2(w0,a,b);
            sm = hypot(sc,ss);  sm(sm<1e-9) = 1;
            sim = max(-1, min(1, uc.*(sc./sm) + us.*(ss./sm)));   % cos(2*lineΔ)
            dline = 0.5*acos(sim);                                % line-angle diff
            wgt = (Ws(kk) .* sw) .* exp(-(dline.^2)/(2*rng_rad^2));% spatial*material*range
            C  = C  + wgt.*sc;  Sm = Sm + wgt.*ss;  W = W + wgt;
        end
        W(W<1e-9) = 1;
        c2 = C./W;  s2 = Sm./W;
        coh = hypot(c2,s2);                              % coherence (pre-renorm)
        nz = coh > 1e-9;
        c2(nz) = c2(nz)./coh(nz);  s2(nz) = s2(nz)./coh(nz);
        c2 = c2 .* w0;  s2 = s2 .* w0;
    end

    % Recover the cleaned LINE orientation, then RESTORE the original arrow
    % direction (sign) from the input field. Double-angle smoothing is invariant
    % to t->t+pi, so 0.5*atan2 alone collapses every vector into [-90,90] deg
    % (all pointing +x) and destroys the field's actual orientation (e.g. an
    % inward-converging field would look outward on one side). Aligning the sign
    % with the input keeps the cleaned orientation AND the true pointing.
    tf = t;  in = w0 > 0;
    tline = 0.5 * atan2(s2(in), c2(in));
    flip = (cos(tline).*cos(t(in)) + sin(tline).*sin(t(in))) < 0;
    tline(flip) = tline(flip) + pi;
    tf(in) = tline;
    coh(~in) = 0;
end

function B = shift2(A, a, b)
% B(i,j) = A(i-b, j-a), zero-padded outside (a along cols/x, b along rows/y)
    [ny,nx] = size(A);  B = zeros(ny,nx);
    ti = max(1,1+b):min(ny,ny+b);  si = ti - b;
    tj = max(1,1+a):min(nx,nx+a);  sj = tj - a;
    B(ti,tj) = A(si,sj);
end
