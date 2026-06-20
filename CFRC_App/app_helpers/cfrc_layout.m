function L = cfrc_layout()
%CFRC_LAYOUT  Return the project folder layout as a struct.
%   All other code asks this function where things live, so the folder
%   split (scripts / functions / viz / python / data / output / docs)
%   is defined in exactly one place.
%
%   L.root      project root
%   L.app       the CFRC_App folder
%   L.scripts   pipeline stage entry-point .m files
%   L.functions helper functions
%   L.viz       visualization scripts
%   L.python    python / Abaqus scripts
%   L.data      .mat artifacts  (this is the working directory when stages run)
%   L.output    figures + FEA work dir + logs
%   L.figures   output/figures
%   L.fea       C:\temp\cfrc_fea   (Abaqus working dir; ASCII temp path)
%   L.logs      output/logs
%   L.docs      markdown docs

here = fileparts(mfilename('fullpath'));   % .../CFRC_App/app_helpers
app  = fileparts(here);                     % .../CFRC_App
root = fileparts(app);                       % project root

L = struct();
L.root      = root;
L.app       = app;
L.scripts   = fullfile(root, 'scripts');
L.functions = fullfile(root, 'functions');
L.viz       = fullfile(root, 'viz');
L.python    = fullfile(root, 'python');
L.data      = fullfile(root, 'data');
L.output    = fullfile(root, 'output');
L.figures   = fullfile(root, 'output', 'figures');
% Abaqus working dir MUST be an ASCII path: Abaqus + its bundled Python 2.7
% choke on the project's non-ASCII path (E:\308\傅里叶\...). Keep it identical
% to abaqus_cfrc_compare.py Config.BASE_DIR / run_compare.m / run_full_comparison.m
% so the App and the standalone Abaqus workflow share one FEA directory.
L.fea       = 'C:\temp\cfrc_fea';
L.logs      = fullfile(root, 'output', 'logs');
L.docs      = fullfile(root, 'docs');
end
