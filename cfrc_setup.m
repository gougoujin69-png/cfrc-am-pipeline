function cfrc_setup()
%CFRC_SETUP  Bootstrap the CFRC-AM pipeline for either run mode.
%
%   Run ONCE per MATLAB session from the repo root:
%       >> cd <repo-root>
%       >> cfrc_setup
%
%   Then pick a mode:
%       App mode    :  >> launch_CFRC          % GUI console (16 stages)
%       Script mode :  >> cd data; run_full_comparison
%
%   It puts scripts/ functions/ viz/ python/ and CFRC_App/ (+app_helpers) on
%   the MATLAB path, so every stage script is callable BY NAME regardless of
%   which folder it physically lives in, and creates the data/ + output/
%   folders (kept out of git).
%
%   The Abaqus working directory is NOT inside the repo: it is fixed to
%   C:\temp\cfrc_fea (a short ASCII path) because Abaqus' bundled Python 2.7
%   cannot handle non-ASCII project paths. The export stage copies the 4 path
%   sets + helper scripts there automatically.

here = fileparts(mfilename('fullpath'));   % repo root

addpath( ...
    fullfile(here, 'scripts'), ...
    fullfile(here, 'functions'), ...
    fullfile(here, 'viz'), ...
    fullfile(here, 'python'), ...
    fullfile(here, 'CFRC_App'), ...
    fullfile(here, 'CFRC_App', 'app_helpers'));

for d = {fullfile(here,'data'), fullfile(here,'output'), ...
         fullfile(here,'output','figures'), fullfile(here,'output','logs')}
    if ~exist(d{1}, 'dir'); mkdir(d{1}); end
end

fprintf('[cfrc_setup] paths added (scripts/ functions/ viz/ python/ CFRC_App/).\n');
fprintf('  App  mode  : launch_CFRC\n');
fprintf('  Script mode: cd(''%s''); run_full_comparison\n', fullfile(here,'data'));
fprintf('  Abaqus work dir = C:\\temp\\cfrc_fea (ASCII path; auto-populated by export stage)\n');
end
