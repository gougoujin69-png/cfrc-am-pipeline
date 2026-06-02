function streamlines = trace_principal_streamlines(mask, t, nelx, nely, opt)
%% trace_principal_streamlines
%  Long, stress-aligned, evenly-spaced principal-direction streamlines for
%  continuous-fiber path planning.
%
%  Strategy (addresses seed distribution / selection / integration all at once):
%    1. DOUBLE-ANGLE interpolation. Principal stress is a LINE field (t == t+pi).
%       We interpolate cos(2t)/sin(2t) (continuous across 180-deg flips) and
%       recover the direction per step -> streamlines follow the field faithfully
%       with NO kinks at sign flips (stream2 cannot do this) and NO Gaussian
%       blurring of the true stress angle.
%    2. SIGN-COHERENT BIDIRECTIONAL RK2 integration: each step aligns with the
%       previous one; forward + backward from each seed -> full-length lines.
%    3. LONGEST-FIRST EVENLY-SPACED selection: integrate many full-length
%       candidates, then greedily keep the longest ones that don't overlap
%       already-kept lines (spacing d_sep). Maximizes path LENGTH and continuity
%       while keeping coverage uniform -- instead of truncating at crossings
%       (Jobard-Lefebvre) or cherry-picking straight bits (old straightness filter).
%    4. Singularities detected by orientation COHERENCE (interpolated |c2,s2|).
%
%  IN : mask  nely x nelx logical;  t  nely x nelx orientation angle (rad)
%  OUT: streamlines  cell array of [x y] (=[col row]) pixel polylines
%
%  opt.*: d_sep 2.0  step 0.5  min_len 6  min_coh 0.30  max_overlap 0.35
%         smooth_sigma 0  max_steps 6000  max_lines 5000

    if nargin < 5, opt = struct(); end
    % Defaults tuned for FEW, LONG, STRAIGHT, SYMMETRIC, well-aligned paths
    % (not maximum coverage): larger d_sep -> fewer lines; min_len -> drop fragments.
    def = struct('d_sep',3.0,'step',0.5,'min_len',5,'min_len_frac',0.35,'min_coh',0.30, ...
                 'max_overlap',0.35,'smooth_sigma',0.0,'symmetry','auto', ...
                 'straight_w',0.5,'max_steps',6000,'max_lines',5000, ...
                 'd_merge',0,'merge_ang_deg',20,'merge_frac',0.7);
    % d_merge: 两条流线靠得比它近(px)且近似平行(<merge_ang_deg)且重叠超过 merge_frac
    %          时, 视为冗余 -> 只保留较长的一条. d_merge<=0 时取 d_sep.
    fns = fieldnames(def);
    for i = 1:numel(fns), if ~isfield(opt,fns{i}), opt.(fns{i}) = def.(fns{i}); end, end

    streamlines = {};
    if ~any(mask(:)), return; end

    % --- double-angle field (correct interpolation across 180-deg line flips) ---
    c2 = cos(2*t); s2 = sin(2*t);
    if opt.smooth_sigma > 0
        if exist('imgaussfilt','file')
            c2 = imgaussfilt(c2, opt.smooth_sigma); s2 = imgaussfilt(s2, opt.smooth_sigma);
        else
            h = fspecial_gauss(opt.smooth_sigma); c2 = conv2(c2,h,'same'); s2 = conv2(s2,h,'same');
        end
    end
    % --- SYMMETRIZE the orientation field about the part's symmetry axis/axes.
    %     Done in the double-angle domain (c2 even, s2 odd under reflection).
    %     This makes streamlines symmetric AND removes asymmetric noise from the
    %     2D layer projection -> straighter, longer, better-aligned paths. ---
    [c2, s2, symc] = symmetrize_da(c2, s2, mask, opt.symmetry);
    gy = 1:nely; gx = 1:nelx;
    Fc2 = griddedInterpolant({gy,gx}, c2,          'linear','none');
    Fs2 = griddedInterpolant({gy,gx}, s2,          'linear','none');
    Fm  = griddedInterpolant({gy,gx}, double(mask),'linear','none');
    Fcos = griddedInterpolant({gy,gx}, cos(t), 'linear','none');  % SIGNED field, used only
    Fsin = griddedInterpolant({gy,gx}, sin(t), 'linear','none');  % to orient output lines

    % occupancy grid (oversampled) for spacing
    os  = max(1, round(2/max(opt.d_sep*0.5,0.25)));
    OW  = round((nelx-1)*os)+1; OH = round((nely-1)*os)+1;
    cellx = @(x) min(max(round((x-1)*os)+1,1),OW);
    celly = @(y) min(max(round((y-1)*os)+1,1),OH);
    function dd = disk_off(rad)
        [ox,oy] = meshgrid(-rad:rad,-rad:rad); kk = (ox.^2+oy.^2)<=rad^2; dd = [ox(kk) oy(kk)];
    end

    function d = fdir(p, dprev)
        a = Fc2(p(2),p(1)); b = Fs2(p(2),p(1));
        if isnan(a) || isnan(b), d = []; return; end
        if hypot(a,b) < opt.min_coh, d = []; return; end          % singularity
        ang = 0.5*atan2(b,a); d = [cos(ang) sin(ang)];
        if ~isempty(dprev) && (d(1)*dprev(1)+d(2)*dprev(2)) < 0, d = -d; end
    end
    function tf = inmat(p), mv = Fm(p(2),p(1)); tf = ~isnan(mv) && mv >= 0.5; end
    function pts = trace_one(seed, d0)
        % physical step cap (backstop) + closed-loop detection (revisit -> stop)
        cap = min(opt.max_steps, ceil(5*(nelx+nely)/opt.step));
        buf = zeros(cap+1,2); buf(1,:) = seed; np = 1; dprev = d0;
        seenk = zeros(OH,OW,'int32'); seenk(celly(seed(2)),cellx(seed(1))) = 1;
        for k = 1:cap
            p = buf(np,:);
            d1 = fdir(p, dprev);   if isempty(d1), break; end
            pm = p + 0.5*opt.step*d1;
            d2 = fdir(pm, d1);     if isempty(d2), d2 = d1; end
            pn = p + opt.step*d2;
            if pn(1)<1||pn(1)>nelx||pn(2)<1||pn(2)>nely, break; end
            if ~inmat(pn), break; end
            cyp = celly(pn(2)); cxp = cellx(pn(1));
            if seenk(cyp,cxp) > 0 && (k - seenk(cyp,cxp)) > 4, break; end   % closed orbit / self-revisit
            if seenk(cyp,cxp) == 0, seenk(cyp,cxp) = k; end
            np = np+1; buf(np,:) = pn; dprev = d2;
        end
        pts = buf(1:np,:);
    end

    % ---- 1) candidate full-length streamlines from a dense seed grid ----
    COV = false(OH, OW);                          % candidate dedup (~0.8 px radius)
    dcov = disk_off(max(1,round(0.8*os)));
    function covmark(p)
        ix = cellx(p(1))+dcov(:,1); iy = celly(p(2))+dcov(:,2);
        ok = ix>=1&ix<=OW&iy>=1&iy<=OH; COV(sub2ind([OH OW],iy(ok),ix(ok))) = true;
    end
    seed_step = max(1, round(opt.d_sep/2));
    cand = {}; candL = []; candS = [];
    for ry = 1:seed_step:nely
        for cx = 1:seed_step:nelx
            if ~mask(ry,cx), continue; end
            if COV(celly(ry),cellx(cx)), continue; end
            s = [cx ry]; v0 = fdir(s, []); if isempty(v0), continue; end
            fwd = trace_one(s,  v0); bwd = trace_one(s, -v0);
            sl = [flipud(bwd); fwd(2:end,:)];
            if size(sl,1) >= 2
                for i = 1:size(sl,1), covmark(sl(i,:)); end   % so other seeds on this line skip
                L = sum(sqrt(sum(diff(sl).^2,2)));
                chord = hypot(sl(end,1)-sl(1,1), sl(end,2)-sl(1,2));
                if L >= opt.min_len
                    cand{end+1} = sl; candL(end+1) = L; candS(end+1) = chord/max(L,1e-9); %#ok<AGROW>
                end
            else
                covmark(s);
            end
        end
    end
    if isempty(cand), return; end

    % ---- 2) longest-first selection with even spacing (d_sep) ----
    OCC = false(OH, OW); dspace = disk_off(max(1,round(opt.d_sep*0.5*os)));
    function stamp(p)
        ix = cellx(p(1))+dspace(:,1); iy = celly(p(2))+dspace(:,2);
        ok = ix>=1&ix<=OW&iy>=1&iy<=OH; OCC(sub2ind([OH OW],iy(ok),ix(ok))) = true;
    end
    % rank by LENGTH x STRAIGHTNESS^w  -> prefer long AND straight main paths
    score = candL .* (candS.^opt.straight_w);
    [~,order] = sort(score,'descend');
    min_len_eff = max(opt.min_len, opt.min_len_frac*max(candL));   % drop fragments short RELATIVE to main paths
    for jj = 1:numel(order)
        if numel(streamlines) >= opt.max_lines, break; end
        if candL(order(jj)) < min_len_eff, continue; end
        sl = cand{order(jj)}; n = size(sl,1);
        nov = 0;
        for i = 1:n, if OCC(celly(sl(i,2)),cellx(sl(i,1))), nov = nov+1; end, end
        if nov/n > opt.max_overlap, continue; end          % too close to a kept line
        streamlines{end+1} = sl; %#ok<AGROW>
        for i = 1:n, stamp(sl(i,:)); end
    end

    % --- enforce SYMMETRY: add mirror image of each kept line (dedup vs spacing) ---
    if symc.x || symc.y
        base = streamlines; nb = numel(base);
        for bi = 1:nb
            sl = base{bi}; mlist = {};
            if symc.x,           mlist{end+1} = [2*symc.cx - sl(:,1),  sl(:,2)]; end %#ok<AGROW>
            if symc.y,           mlist{end+1} = [sl(:,1),  2*symc.cy - sl(:,2)]; end %#ok<AGROW>
            if symc.x && symc.y, mlist{end+1} = [2*symc.cx - sl(:,1), 2*symc.cy - sl(:,2)]; end %#ok<AGROW>
            for mi = 1:numel(mlist)
                mln = mlist{mi}; nm = size(mln,1); nov = 0; nin = 0;
                for i = 1:nm
                    if mln(i,1)>=1 && mln(i,1)<=nelx && mln(i,2)>=1 && mln(i,2)<=nely && inmat(mln(i,:))
                        nin = nin + 1;
                        if OCC(celly(mln(i,2)),cellx(mln(i,1))), nov = nov + 1; end
                    end
                end
                if nin >= 0.6*nm && nov/max(nin,1) <= opt.max_overlap
                    streamlines{end+1} = mln; %#ok<AGROW>
                    for i = 1:nm
                        if mln(i,1)>=1&&mln(i,1)<=nelx&&mln(i,2)>=1&&mln(i,2)<=nely, stamp(mln(i,:)); end
                    end
                end
            end
        end
    end

    % --- prune redundant lines: drop a line if it is mostly close-AND-parallel to a
    %     longer kept line (two near-parallel neighbours -> keep one). ---
    dM = opt.d_merge; if dM <= 0, dM = opt.d_sep; end
    streamlines = prune_redundant(streamlines, dM, opt.merge_ang_deg, opt.merge_frac);

    % --- orient each output line to traverse WITH the (signed) stress field ---
    %     (lines are sign-agnostic; this makes the traversal follow the field arrows) ---
    for k = 1:numel(streamlines)
        sl = streamlines{k}; if size(sl,1) < 2, continue; end
        seg = diff(sl); mid = (sl(1:end-1,:)+sl(2:end,:))/2;
        fc = Fcos(mid(:,2),mid(:,1)); fs = Fsin(mid(:,2),mid(:,1));
        if sum(seg(:,1).*fc + seg(:,2).*fs, 'omitnan') < 0
            streamlines{k} = flipud(sl);
        end
    end
end

function h = fspecial_gauss(sigma)
    r = max(1,ceil(3*sigma)); [x,y] = meshgrid(-r:r,-r:r);
    h = exp(-(x.^2+y.^2)/(2*sigma^2)); h = h/sum(h(:));
end

function [c2,s2,symc] = symmetrize_da(c2, s2, mask, symmetry)
% Symmetrize a double-angle orientation field (c2=cos2t, s2=sin2t) about the
% material centroid. Under reflection: c2 is EVEN, s2 is ODD.
% Returns symc.{cx,cy,x,y} so the caller can mirror the selected streamlines.
    [ny,nx] = size(c2);
    symc = struct('cx',(1+nx)/2,'cy',(1+ny)/2,'x',false,'y',false);
    [rr,cc] = find(mask); if isempty(rr), return; end
    cx = mean(cc); cy = mean(rr);
    [Xg,Yg] = meshgrid(1:nx,1:ny);
    do_x = false; do_y = false;
    if     strcmpi(symmetry,'x'),  do_x = true;
    elseif strcmpi(symmetry,'y'),  do_y = true;
    elseif strcmpi(symmetry,'xy'), do_x = true; do_y = true;
    elseif strcmpi(symmetry,'none')
    else  % 'auto': detect from mask overlap with its reflection
        mrx = interp2(Xg,Yg,double(mask),2*cx-Xg,Yg,'nearest',0);
        mry = interp2(Xg,Yg,double(mask),Xg,2*cy-Yg,'nearest',0);
        sm  = sum(mask(:));
        do_x = sum(mask(:)&(mrx(:)>0.5))/sm > 0.85;
        do_y = sum(mask(:)&(mry(:)>0.5))/sm > 0.85;
    end
    if do_x
        c2r = interp2(Xg,Yg,c2,2*cx-Xg,Yg,'linear',NaN);
        s2r = interp2(Xg,Yg,s2,2*cx-Xg,Yg,'linear',NaN);
        v = ~isnan(c2r) & ~isnan(s2r);
        c2(v) = 0.5*(c2(v)+c2r(v)); s2(v) = 0.5*(s2(v)-s2r(v));
    end
    if do_y
        c2r = interp2(Xg,Yg,c2,Xg,2*cy-Yg,'linear',NaN);
        s2r = interp2(Xg,Yg,s2,Xg,2*cy-Yg,'linear',NaN);
        v = ~isnan(c2r) & ~isnan(s2r);
        c2(v) = 0.5*(c2(v)+c2r(v)); s2(v) = 0.5*(s2(v)-s2r(v));
    end
    symc = struct('cx',cx,'cy',cy,'x',do_x,'y',do_y);
end

function S = prune_redundant(S, d_merge, ang_deg, frac)
% Remove streamlines that add little new information: a line is dropped if a
% fraction >= frac of its points lie within d_merge of, AND nearly parallel to
% (< ang_deg), an already-kept (longer) line. Keeps the longer of each near-
% parallel pair -> no "two close parallel lines where one would do".
    m = numel(S); if m <= 1, return; end
    Ls = zeros(m,1);
    for k = 1:m, p = S{k}; if size(p,1) >= 2, Ls(k) = sum(sqrt(sum(diff(p).^2,2))); end, end
    [~, ord] = sort(Ls, 'descend');
    ct = cos(deg2rad(ang_deg)); d2 = d_merge^2;
    kept = {}; KP = zeros(0,2); KT = zeros(0,2);
    for jj = ord(:)'
        p = S{jj}; n = size(p,1);
        if n < 2, continue; end
        tg = local_tangents(p);
        if isempty(KP)
            kept{end+1} = p; KP = [KP; p]; KT = [KT; tg]; continue; %#ok<AGROW>
        end
        cov = 0;
        for i = 1:n
            dd = (KP(:,1)-p(i,1)).^2 + (KP(:,2)-p(i,2)).^2;
            [md, mi] = min(dd);
            if md <= d2 && abs(KT(mi,1)*tg(i,1)+KT(mi,2)*tg(i,2)) >= ct
                cov = cov + 1;
            end
        end
        if cov/n < frac
            kept{end+1} = p; KP = [KP; p]; KT = [KT; tg]; %#ok<AGROW>
        end
    end
    S = kept;
end

function T = local_tangents(p)
    n = size(p,1); d = zeros(n,2);
    if n >= 3, d(2:n-1,:) = p(3:n,:) - p(1:n-2,:); end
    d(1,:) = p(min(2,n),:) - p(1,:); d(n,:) = p(n,:) - p(max(n-1,1),:);
    nn = hypot(d(:,1),d(:,2)); nn(nn < 1e-9) = 1; T = d ./ nn;
end
