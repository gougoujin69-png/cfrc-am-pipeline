function run_full_comparison(varargin)
% run_full_comparison  CFRC 4-way 切片+路径生成+FEA 部署 一键脚本
%
%   run_full_comparison                  % 默认: 检测已有的产物, 只跑缺的步骤
%   run_full_comparison('force', true)   % 强制全部重跑 (覆盖已有 mat)
%   run_full_comparison('stages', [7,8,9]) % 只跑指定 stage
%
% Stages:
%   1) Curved (mine) slicing      -> slice_results_refined_latest.mat
%   2) Planar slicing             -> slice_results_refined_latest_PLANAR.mat
%   3) mine_stream path planning  -> all_layers_paths_only_v3.mat
%   4) mine_offset path planning  -> all_layers_paths_only_mine_offset.mat
%   5) planar_stream path planning-> all_layers_paths_only_planar_stream.mat
%   6) planar_offset path planning-> all_layers_paths_only_planar_offset.mat
%   7) Verify outputs (sanity check 4 path mats)
%   8) Export 4 path sets + host inp -> C:\temp\cfrc_fea\<cfg>\beam_paths\
%   9) Copy helper scripts (Python + run_compare.m) to C:\temp\cfrc_fea
%   10) Print next-step instructions

% ===== 配置 =====
script_dir = fileparts(mfilename('fullpath'));
fea_dir = 'C:\temp\cfrc_fea';

helper_files = {
    'abaqus_cfrc_compare.py', ...
    'extract_fea_results.py', ...
    'run_compare.m', ...
    'compare_fea_results.m', ...
    'compute_path_statistics.m', ...
    'diagnose_loadpoint.py', ...
};

% ===== 参数 =====
p = inputParser;
addParameter(p, 'force', false);
addParameter(p, 'stages', []);
parse(p, varargin{:});
opt = p.Results;

force_all = opt.force;
stages_to_run = opt.stages;
if isempty(stages_to_run); stages_to_run = 1:10; end

% ===== 头信息 =====
fprintf('\n');
fprintf('================================================================\n');
fprintf(' CFRC 4-way Comparison: Full MATLAB Pipeline\n');
fprintf(' Script dir:  %s\n', script_dir);
fprintf(' FEA dir:     %s\n', fea_dir);
fprintf(' Force mode:  %s\n', mat2str(force_all));
fprintf(' Stages:      %s\n', mat2str(stages_to_run));
fprintf('================================================================\n\n');

if ~exist(fea_dir, 'dir')
    mkdir(fea_dir);
    fprintf('[Setup] Created FEA dir: %s\n', fea_dir);
end

% ===== Stage 1: Curved (mine) slicing =====
if any(stages_to_run == 1)
    stage_header(1, 'Curved (mine) slicing');
    target = 'slice_results_refined_latest.mat';
    if exist(target, 'file') && ~force_all
        fprintf('  [SKIP] %s already exists\n', target);
    else
        fprintf('  [RUN]  slice_refined_model_v6 (curved/mine)...\n');
        try
            slice_refined_model_v6('voxel_refined_latest.mat', target);
            fprintf('  [OK]   produced %s\n', target);
        catch err
            fprintf('  [FAIL] %s\n', err.message);
            error('Stage 1 failed: %s', err.message);
        end
    end
end

% ===== Stage 2: Planar slicing =====
if any(stages_to_run == 2)
    stage_header(2, 'Planar slicing');
    target = 'slice_results_refined_latest_PLANAR.mat';
    if exist(target, 'file') && ~force_all
        fprintf('  [SKIP] %s already exists\n', target);
    else
        fprintf('  [RUN]  generate_planar_slicing (planar layers)...\n');
        try
            generate_planar_slicing('voxel_refined_latest.mat', target);
            fprintf('  [OK]   produced %s\n', target);
        catch err
            fprintf('  [FAIL] %s\n', err.message);
            error('Stage 2 failed: %s', err.message);
        end
    end
end

% ===== Stage 3: mine_stream paths =====
if any(stages_to_run == 3)
    stage_header(3, 'mine_stream path planning');
    target = 'all_layers_paths_only_v3.mat';
    if exist(target, 'file') && ~force_all
        fprintf('  [SKIP] %s already exists\n', target);
    else
        fprintf('  [RUN]  all_layers_path_generation (stream on curved)...\n');
        try
            run_v6_with_slice('slice_results_refined_latest.mat', target);
            fprintf('  [OK]   produced %s\n', target);
        catch err
            fprintf('  [FAIL] %s\n', err.message);
            error('Stage 3 failed: %s', err.message);
        end
    end
end

% ===== Stage 4: mine_offset paths =====
if any(stages_to_run == 4)
    stage_header(4, 'mine_offset path planning');
    target = 'all_layers_paths_only_mine_offset.mat';
    if exist(target, 'file') && ~force_all
        fprintf('  [SKIP] %s already exists\n', target);
    else
        fprintf('  [RUN]  offset path generation on curved slice...\n');
        try
            generate_offset_paths('slice_results_refined_latest.mat', target);
            fprintf('  [OK]   produced %s\n', target);
        catch err
            fprintf('  [FAIL] %s\n', err.message);
            error('Stage 4 failed: %s', err.message);
        end
    end
end

% ===== Stage 5: planar_stream paths =====
if any(stages_to_run == 5)
    stage_header(5, 'planar_stream path planning');
    target = 'all_layers_paths_only_planar_stream.mat';
    if exist(target, 'file') && ~force_all
        fprintf('  [SKIP] %s already exists\n', target);
    else
        fprintf('  [RUN]  all_layers_path_generation (stream on planar)...\n');
        try
            run_v6_with_slice('slice_results_refined_latest_PLANAR.mat', target);
            fprintf('  [OK]   produced %s\n', target);
        catch err
            fprintf('  [FAIL] %s\n', err.message);
            error('Stage 5 failed: %s', err.message);
        end
    end
end

% ===== Stage 6: planar_offset paths =====
if any(stages_to_run == 6)
    stage_header(6, 'planar_offset path planning');
    target = 'all_layers_paths_only_planar_offset.mat';
    if exist(target, 'file') && ~force_all
        fprintf('  [SKIP] %s already exists\n', target);
    else
        fprintf('  [RUN]  offset path generation on planar slice...\n');
        try
            generate_offset_paths('slice_results_refined_latest_PLANAR.mat', target);
            fprintf('  [OK]   produced %s\n', target);
        catch err
            fprintf('  [FAIL] %s\n', err.message);
            error('Stage 6 failed: %s', err.message);
        end
    end
end

% ===== Stage 7: Verify =====
if any(stages_to_run == 7)
    stage_header(7, 'Verify all 4 path-mat outputs');
    expected = {
        'all_layers_paths_only_v3.mat', ...
        'all_layers_paths_only_mine_offset.mat', ...
        'all_layers_paths_only_planar_stream.mat', ...
        'all_layers_paths_only_planar_offset.mat', ...
    };
    missing = {};
    for k = 1:numel(expected)
        if ~exist(expected{k}, 'file')
            missing{end+1} = expected{k};  %#ok<AGROW>
        else
            d = dir(expected{k});
            fprintf('  [OK]  %s  (%.1f MB)\n', expected{k}, d.bytes/1024/1024);
        end
    end
    if ~isempty(missing)
        fprintf('  [FAIL] Missing: %s\n', strjoin(missing, ', '));
        error('Stage 7: not all path mats present');
    end
    fprintf('  All 4 path mats present.\n');
end

% ===== Stage 8: Export paths + host inp =====
if any(stages_to_run == 8)
    stage_header(8, 'Export paths and host inp to FEA dir');
    pairs = {
        'all_layers_paths_only_v3.mat',             'mine_stream'; ...
        'all_layers_paths_only_mine_offset.mat',    'mine_offset'; ...
        'all_layers_paths_only_planar_stream.mat',  'planar_stream'; ...
        'all_layers_paths_only_planar_offset.mat',  'planar_offset'; ...
    };

    % Detect missing host/ data. The standalone export_paths_to_fea.m writes
    % host/mesh_params.txt + host/valid_elements.txt only on first invocation
    % (when valid_elements.txt does not yet exist). If beam_paths from a
    % previous run already exist, all 4 configs SKIP and host never gets
    % written. Force-rerun the first config when host is missing to break
    % that deadlock.
    host_valid_txt   = fullfile(fea_dir, 'host', 'valid_elements.txt');
    host_needs_write = ~exist(host_valid_txt, 'file');
    if host_needs_write
        fprintf('  [INFO] %s missing\n', host_valid_txt);
        fprintf('         will force-run first config to populate host/\n');
    end

    for k = 1:size(pairs, 1)
        mat_file = pairs{k, 1};
        cfg_name = pairs{k, 2};
        out_subdir   = fullfile(fea_dir, cfg_name, 'beam_paths');
        summary_file = fullfile(fea_dir, cfg_name, 'beam_paths_summary.txt');
        n_existing   = numel(dir(fullfile(out_subdir, 'path_*.txt')));
        has_sentinel = exist(summary_file, 'file') > 0;

        % abaqus_cfrc_compare.py uses beam_paths_summary.txt as the
        % sentinel that decides whether a config gets processed. So the SKIP
        % check here must require BOTH path_*.txt and the sentinel; if the
        % sentinel is missing (legacy state from the old buggy local helper),
        % redo the export.
        force_this  = (host_needs_write && k == 1) || ...
                      (n_existing > 0 && ~has_sentinel);
        can_skip    = (n_existing > 0) && has_sentinel && ~force_all && ~force_this;

        if can_skip
            fprintf('  [SKIP] %s: %d path_*.txt + summary already in %s\n', ...
                cfg_name, n_existing, out_subdir);
            continue;
        end

        % Diagnostic preamble so the user knows why we are re-running.
        if force_this && host_needs_write && k == 1
            fprintf('  [RUN]  export_paths_to_fea %s -> %s  (also writes host/)\n', ...
                mat_file, cfg_name);
        elseif n_existing > 0 && ~has_sentinel
            fprintf(['  [RUN]  export_paths_to_fea %s -> %s  ' ...
                     '(re-export: %d path_*.txt present but summary missing)\n'], ...
                mat_file, cfg_name, n_existing);
        else
            fprintf('  [RUN]  export_paths_to_fea %s -> %s\n', mat_file, cfg_name);
        end

        try
            export_paths_to_fea(mat_file, cfg_name, fea_dir);
            fprintf('  [OK]   %s exported\n', cfg_name);
            host_needs_write = false;
        catch err
            fprintf('  [FAIL] %s\n', err.message);
        end
    end

    host_inp = fullfile(fea_dir, 'EmbeddedBeamModel.inp');
    if exist(host_inp, 'file') && ~force_all
        fprintf('  [SKIP] host inp already at %s\n', host_inp);
    else
        fprintf('  [WARN] host inp missing at %s\n', host_inp);
        fprintf('         Build it separately (from voxels) and place here.\n');
    end
end

% ===== Stage 9: Copy helper scripts =====
if any(stages_to_run == 9)
    stage_header(9, 'Copy helper scripts to FEA dir');
    for k = 1:numel(helper_files)
        src = fullfile(script_dir, helper_files{k});
        dst = fullfile(fea_dir, helper_files{k});
        if ~exist(src, 'file')
            fprintf('  [WARN] source missing: %s\n', src);
            fprintf('         (script_dir = %s)\n', script_dir);
            continue;
        end
        try
            copyfile(src, dst, 'f');
            fprintf('  [OK]   %s  -> %s\n', helper_files{k}, dst);
        catch err
            fprintf('  [FAIL] copy %s: %s\n', helper_files{k}, err.message);
        end
    end

    % --- 同步复制 4 个 path mat 到 fea_dir ---
    % compute_path_statistics 是从 pwd 搜 mat 的, 而 run_compare cd 到 fea_dir
    % 之后 pwd = fea_dir. 把 4 份 mat 拷过来才能让 run_compare('all') / ('stats')
    % 在 fea_dir 自给自足 (不依赖 MATLAB 项目目录在 path 里).
    fprintf('\n  Copying 4 path mat files for run_compare(''stats''/''all''):\n');
    path_mats = { ...
        'all_layers_paths_only_v3.mat', ...
        'all_layers_paths_only_mine_offset.mat', ...
        'all_layers_paths_only_planar_stream.mat', ...
        'all_layers_paths_only_planar_offset.mat', ...
    };
    for k = 1:numel(path_mats)
        src = fullfile(script_dir, path_mats{k});
        dst = fullfile(fea_dir, path_mats{k});
        if ~exist(src, 'file')
            fprintf('  [WARN] path mat missing: %s\n', src);
            continue;
        end
        try
            copyfile(src, dst, 'f');
            d = dir(dst);
            fprintf('  [OK]   %s  (%.1f MB)\n', path_mats{k}, d.bytes/1e6);
        catch err
            fprintf('  [FAIL] copy %s: %s\n', path_mats{k}, err.message);
        end
    end
end

% ===== Stage 10: Next steps =====
if any(stages_to_run == 10)
    stage_header(10, 'NEXT STEPS - Run Abaqus & post-process');
    fprintf('\n');
    fprintf('  All MATLAB output is in:  %s\n', fea_dir);
    fprintf('\n');
    fprintf('  STEP A. (Optional) Build template.cae for the host part:\n');
    fprintf('    -- Abaqus CAE (GUI):\n');
    fprintf('      >> execfile(''%s/abaqus_cfrc_compare.py'')\n', fea_dir);
    fprintf('      >> step1_build_template()\n');
    fprintf('    -- Manually add *Cload, BC etc. to template.cae, save (once)\n');
    fprintf('\n');
    fprintf('  STEP B. Run 4 Abaqus jobs (auto-retry on errors):\n');
    fprintf('    -- Abaqus CAE command line:\n');
    fprintf('      >> execfile(''abaqus_cfrc_compare.py'')\n');
    fprintf('      >> run_with_auto_retry([''mine_stream'',''mine_offset'',''planar_stream'',''planar_offset''])\n');
    fprintf('      >> dump_blacklist()    %% Paste blacklist back into source\n');
    fprintf('\n');
    fprintf('  STEP C. Extract ODB results (Windows cmd in %s):\n', fea_dir);
    fprintf('      cd /d %s\n', fea_dir);
    fprintf('      abaqus cae noGUI=extract_fea_results.py\n');
    fprintf('\n');
    fprintf('  STEP D. MATLAB compare:\n');
    fprintf('      >> cd %s\n', fea_dir);
    fprintf('      >> run_compare         %% Just compare\n');
    fprintf('      >> run_compare(''all'')  %% stats + compare\n');
    fprintf('\n');
end

fprintf('\n=== run_full_comparison DONE ===\n\n');
end


% =================================================================
% Helper functions
% =================================================================
function stage_header(num, title)
fprintf('\n----------------------------------------------------------------\n');
fprintf(' Stage %d: %s\n', num, title);
fprintf('----------------------------------------------------------------\n');
end


function run_v6_with_slice(slice_mat, target_mat)
% Wrapper around all_layers_path_generation_v6, explicitly using given slice file.
% Also gives each run a unique full-results filename so consecutive stages
% (mine_stream and planar_stream) don't overwrite each other's intermediate.
fprintf('    using slice: %s\n', slice_mat);
fprintf('    target:      %s\n', target_mat);
[~, base, ~] = fileparts(target_mat);
full_results = sprintf('%s_full.mat', base);
all_layers_path_generation_v6(slice_mat, target_mat, full_results);
end


function generate_offset_paths(slice_mat, target_mat)
% Pure offset path generation (no stream).
fprintf('    using slice: %s\n', slice_mat);
fprintf('    target:      %s\n', target_mat);
path_generation_offset_only(slice_mat, target_mat);
end

