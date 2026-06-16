function info = gen_robot_excel(matfile, outfile, opts)
% GEN_ROBOT_EXCEL  Manufacturing_printing_path.mat -> robot path table.
%
% Bridges the print pipeline to the ABB MOD generator. For every point of
% every carbon segment (connected_paths) it computes the surface normal
% (getPathNormals against that layer's pointCloud_data) and the two print
% orientation angles using the SAME convention as Step3_2_Robotic_carbon_path
% / calculate_angles_from_normal (tool Z = surface normal):
%     rx = atan2(-ny,-nz)  (deg, col 4)
%     ry = asin(-nx)       (deg, col 5)
% and writes the 8-column format the MOD generator reads:
%     [ x  y  z(mm)   rx  ry(deg)   pot   vcs   tool ]
%   pot  = 1 on the LAST point of each segment (dwell), else 0
%   vcs  = 10 (travel) on the FIRST point of each segment, else 14 (print 10mm/s)
%   tool = 111 (carbon)
%
% opts (optional): .layers  (vector of layer indices; default = all)
%                  .stride  (keep every k-th point; default 1)
%                  .vcs_print (default 14)  .vcs_travel (default 10)
%
% Output extension decides format (.xlsx or .csv). Returns info struct.

if nargin < 2 || isempty(outfile), outfile = 'carbon_robot_path.csv'; end
if nargin < 3, opts = struct(); end
if ~isfield(opts,'stride'),     opts.stride = 1;   end
if ~isfield(opts,'vcs_print'),  opts.vcs_print = 14; end  % 10mm/s print
if ~isfield(opts,'vcs_travel'), opts.vcs_travel = 10; end % travel (idle)
if ~isfield(opts,'vcs_feed'),   opts.vcs_feed  = 12; end  % feed after travel (segment start)
if ~isfield(opts,'vcs_cut'),    opts.vcs_cut   = 11; end  % cut fiber (segment end)
if ~isfield(opts,'travel'),     opts.travel    = true; end% insert lift+travel between segments
if ~isfield(opts,'lift_mm'),    opts.lift_mm   = 5;  end  % retract/lift height (mm)
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'pipeline_funcs'));   % getPathNormals

S = load(matfile,'all_layers_data');
ald = S.all_layers_data;
nL = numel(ald);
if ~isfield(opts,'layers') || isempty(opts.layers), opts.layers = 1:nL; end

rows = cell(numel(opts.layers),1);
totpts = 0; totseg = 0; ntravel = 0;
prev = [];   % [x y z rx ry] last printed point, for lift+travel between segments
for li = 1:numel(opts.layers)
    k = opts.layers(li);
    if k<1 || k>nL || ~isfield(ald,'connected_paths') || isempty(ald(k).connected_paths)
        continue;
    end
    cp = ald(k).connected_paths;
    pc = ald(k).pointCloud_data;
    layerrows = [];
    for s = 1:numel(cp)
        P = cp{s};
        if isempty(P) || size(P,1) < 2, continue; end
        if opts.stride > 1
            idx = unique([1:opts.stride:size(P,1), size(P,1)]);
            P = P(idx,:);
        end
        n = size(P,1);
        N = getPathNormals(P, pc);                % n x 3 surface normals
        nx = N(:,1); ny = N(:,2); nz = N(:,3);
        rx = atan2(-ny, -nz) * 180/pi;
        ry = asin(max(-1,min(1,-nx))) * 180/pi;

        % --- lift + travel from the previous segment end to this start ---
        % (avoids dragging the nozzle across already-printed material)
        if opts.travel && ~isempty(prev)
            up   = [prev(1), prev(2), prev(3)+opts.lift_mm, prev(4), prev(5), 0, opts.vcs_travel, 111]; % lift
            over = [P(1,1),  P(1,2),  P(1,3)+opts.lift_mm,  rx(1),   ry(1),   0, opts.vcs_travel, 111]; % over start
            layerrows = [layerrows; up; over]; %#ok<AGROW>
            ntravel = ntravel + 2;
        end

        % --- segment rows: feed at start, print, cut at end (fiber semantics) ---
        vcs = opts.vcs_print*ones(n,1);
        vcs(1)   = opts.vcs_feed;     % feed fiber after travelling in
        vcs(end) = opts.vcs_cut;      % cut fiber at the path end
        pot = zeros(n,1);  pot(end) = 1;
        tool = 111*ones(n,1);
        layerrows = [layerrows; P(:,1), P(:,2), P(:,3), rx, ry, pot, vcs, tool]; %#ok<AGROW>
        totseg = totseg + 1; totpts = totpts + n;
        prev = [P(end,1), P(end,2), P(end,3), rx(end), ry(end)];
    end
    rows{li} = layerrows;
end
M = cat(1, rows{:});
if isempty(M), error('No carbon points found for the requested layers.'); end

writematrix(M, outfile);
info = struct('file',outfile,'rows',size(M,1),'segments',totseg, ...
              'travel_pts',ntravel,'layers',numel(opts.layers));
fprintf('[gen_robot_excel] wrote %s : %d rows (%d travel), %d segments, %d layers\n', ...
        outfile, size(M,1), ntravel, totseg, numel(opts.layers));
end
