function app = launch_CFRC()
%LAUNCH_CFRC  Set up paths + working dir and open the CFRC Pipeline Console.
%
%   >> launch_CFRC
%
%   This is the ONLY thing you need to run. It puts scripts/ functions/ viz/
%   python/ on the MATLAB path, makes data/ the working directory (so every
%   stage's load()/save() resolves there), and opens the app window.

here = fileparts(mfilename('fullpath'));        % .../CFRC_App
addpath(here, fullfile(here, 'app_helpers'));
L = cfrc_layout();

% --- source folders on the path (NOT _gh_repo / _backup, to avoid clashes) ---
addpath(L.scripts, L.functions, L.viz, L.python);

% --- make sure the data/output folders exist ---
for d = {L.data, L.output, L.figures, L.fea, L.logs}
    if ~exist(d{1}, 'dir'); mkdir(d{1}); end
end

% --- work inside data/ so bare load('x.mat')/save('x.mat') land there ---
cd(L.data);

fprintf('[CFRC] paths added; working dir = %s\n', L.data);
app = CFRC_Pipeline_App();
if nargout == 0; clear app; end
end
