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
    'diagnose_loadpoint.py', ...
};
% NOTE: compute_path_statistics.m is NOT copied here - it lives in script_dir
% and reads/writes from there directly.

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

    % ===== [NEW 2026-06] Host 尺度一致性守卫 =====
    % 问题: export_paths_to_fea 的 host (mesh_params.txt + valid_elements.txt) 是
    %   "只写一次"的 (valid_elements.txt 存在就跳过). 当你在 voxel_refinement 里把
    %   ELEM_SIZE 拉大 (或改 REFINE_FACTOR/原始网格), 当前体素的物理网格步长 dx 变了,
    %   但旧 host 仍是旧尺度 -> abaqus_cfrc_compare.py 按旧 dx 建 host -> beam 路径范围
    %   远大于 host 结构范围 -> 嵌入计算 (EmbeddedRegion) 出错.
    % 守卫: 比对 host/mesh_params.txt 记录的 (nelx,nely,nelz,dx,dy,dz) 与当前体素 mat,
    %   不一致 (或 force=true) 就删掉旧 host, 让本 Stage 按当前 ELEM_SIZE 重建.
    host_dir_chk     = fullfile(fea_dir, 'host');
    voxel_mat_host   = 'voxel_refined_latest.mat';   % export_paths_to_fea 也用这个建 host
    [host_stale, stale_msg] = host_scale_mismatch(host_dir_chk, voxel_mat_host);
    if force_all && exist(fullfile(host_dir_chk, 'valid_elements.txt'), 'file')
        host_stale = true;
        stale_msg  = 'force=true: 强制按当前 ELEM_SIZE 重建 host';
    end
    if host_stale
        fprintf('  [HOST] %s\n', stale_msg);
        for fdel = {'mesh_params.txt', 'valid_elements.txt'}
            fp = fullfile(host_dir_chk, fdel{1});
            if exist(fp, 'file')
                delete(fp);
                fprintf('  [HOST] 已删除旧 %s (将按新尺度重写)\n', fdel{1});
            end
        end
    end

    % Detect missing host/ data. The standalone export_paths_to_fea.m writes
    % host/mesh_params.txt + host/valid_elements.txt only on first invocation
    % (when valid_elements.txt does not yet exist). If beam_paths from a
    % previous run already exist, all 4 configs SKIP and host never gets
    % written. Force-rerun the first config when host is missing to break
    % that deadlock. (尺度守卫已在上面把陈旧 host 删掉, 这里会判定为缺失并重写.)
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

    % ===== [NEW 2026-06] 导出后核对: beam 路径 bbox vs host 结构 bbox =====
    % 直观确认"路径范围没有超出结构范围"(尺度一致). 超出会红字告警.
    verify_path_host_bbox(fea_dir, pairs);

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


% =================================================================
% Host 尺度一致性 (ELEM_SIZE 改变检测)
% =================================================================
function [stale, msg] = host_scale_mismatch(host_dir, voxel_mat)
% 比对已写出的 host/mesh_params.txt 与当前体素 mat 的网格尺度.
%   stale=true  -> 需要按当前 ELEM_SIZE 重建 host
%   host 不存在 -> stale=false (交给后续"缺失"逻辑处理)
stale = false; msg = '';
mp = fullfile(host_dir, 'mesh_params.txt');
ve = fullfile(host_dir, 'valid_elements.txt');
if ~exist(mp, 'file') || ~exist(ve, 'file')
    return;   % host 尚未生成, 不算 stale
end
if ~exist(voxel_mat, 'file')
    return;   % 没有体素 mat 可比对
end

% 当前体素 mat 的网格尺度 (与 export_paths_to_fea/write_host_mesh 同源算法)
S = load(voxel_mat, 'refined_data');
rd = S.refined_data;
nx = rd.grid_size.nelx; ny = rd.grid_size.nely; nz = rd.grid_size.nelz;
gd = rd.grid_data;
if nx >= 2, cur_dx = gd(2,1,1).x - gd(1,1,1).x; else, cur_dx = 1.0; end
if ny >= 2, cur_dy = gd(1,2,1).y - gd(1,1,1).y; else, cur_dy = 1.0; end
if nz >= 2, cur_dz = gd(1,1,2).z - gd(1,1,1).z; else, cur_dz = 1.0; end

P = read_mesh_params_file(mp);
tol = 1e-3;
reasons = {};

% 整数维度比对
dim_keys = {'nelx', nx; 'nely', ny; 'nelz', nz};
for di = 1:size(dim_keys, 1)
    kk = dim_keys{di, 1}; cur = dim_keys{di, 2};
    if isfield(P, kk) && P.(kk) ~= cur
        reasons{end+1} = sprintf('%s %g->%d', kk, P.(kk), cur); %#ok<AGROW>
    end
end

% 物理步长比对 (dx/dy/dz)
step_keys = {'dx', cur_dx; 'dy', cur_dy; 'dz', cur_dz};
for di = 1:size(step_keys, 1)
    kk = step_keys{di, 1}; cur = step_keys{di, 2};
    if isfield(P, kk) && abs(P.(kk) - cur) > tol
        reasons{end+1} = sprintf('%s %.4f->%.4f', kk, P.(kk), cur); %#ok<AGROW>
    end
end

% 旧格式 mesh_params.txt 没写 dx 字段: Python 端会回退 1.0; 若当前 dx≠1.0 则尺度会错
if ~isfield(P, 'dx') && abs(cur_dx - 1.0) > tol
    reasons{end+1} = sprintf('host 缺 dx 字段且当前 dx=%.4f≠1.0 (Python 回退 1.0 -> 尺度错)', cur_dx); %#ok<AGROW>
end

if ~isempty(reasons)
    stale = true;
    msg = sprintf('检测到 host 尺度与当前体素不一致 [%s]', strjoin(reasons, ', '));
end
end


function P = read_mesh_params_file(mp)
% 解析 host/mesh_params.txt 的 "key value" 行 (# 为注释).
P = struct();
fid = fopen(mp, 'r');
if fid < 0, return; end
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line) || line(1) == '#', continue; end
    parts = strsplit(line);
    if numel(parts) >= 2
        key = matlab.lang.makeValidName(parts{1});
        val = str2double(parts{2});
        if ~isnan(val), P.(key) = val; end
    end
end
fclose(fid);
end


% =================================================================
% 导出后核对: beam 路径 bbox vs host 结构 bbox
% =================================================================
function verify_path_host_bbox(fea_dir, pairs)
host_ve = fullfile(fea_dir, 'host', 'valid_elements.txt');
if ~exist(host_ve, 'file')
    fprintf('  [BBOX] host valid_elements.txt 不存在, 跳过核对\n');
    return;
end
M = read_valid_elements_xyz(host_ve);
if isempty(M)
    fprintf('  [BBOX] host valid_elements.txt 无可读坐标, 跳过\n');
    return;
end
hx = [min(M(:,1)) max(M(:,1))];
hy = [min(M(:,2)) max(M(:,2))];
hz = [min(M(:,3)) max(M(:,3))];

pbb = []; used_cfg = '';
for k = 1:size(pairs,1)
    mf = pairs{k,1};
    if exist(mf, 'file')
        pbb = path_mat_bbox(mf);
        if ~isempty(pbb), used_cfg = pairs{k,2}; break; end
    end
end
if isempty(pbb)
    fprintf('  [BBOX] 找不到可读的 path mat, 跳过核对\n');
    return;
end

fprintf('  [BBOX] host 结构范围(体素中心 mm): X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]\n', ...
    hx(1),hx(2), hy(1),hy(2), hz(1),hz(2));
fprintf('  [BBOX] beam 路径范围(%s mm):  X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]\n', ...
    used_cfg, pbb(1),pbb(2), pbb(3),pbb(4), pbb(5),pbb(6));

spanx = hx(2)-hx(1); spany = hy(2)-hy(1); spanz = hz(2)-hz(1);
tolx = max(0.2*spanx, 3); toly = max(0.2*spany, 3); tolz = max(0.2*spanz, 3);
over = (pbb(1) < hx(1)-tolx) || (pbb(2) > hx(2)+tolx) || ...
       (pbb(3) < hy(1)-toly) || (pbb(4) > hy(2)+toly) || ...
       (pbb(5) < hz(1)-tolz) || (pbb(6) > hz(2)+tolz);
if over
    fprintf(2, '  [BBOX][WARN] beam 路径明显超出 host 结构范围! host 尺度可能与路径不一致.\n');
    fprintf(2, '              请确认 voxel_refinement 的 ELEM_SIZE 与生成这些路径时一致;\n');
    fprintf(2, '              如刚改过 ELEM_SIZE, 用 run_full_comparison(''force'', true) 全量重跑.\n');
else
    fprintf('  [BBOX] OK: beam 路径落在 host 结构范围内 (尺度一致)\n');
end
end


function M = read_valid_elements_xyz(fp)
% 读 valid_elements.txt 的 xc yc zc (第 5,6,7 列), # 为注释.
M = [];
fid = fopen(fp, 'r');
if fid < 0, return; end
C = textscan(fid, '%f %f %f %f %f %f %f', 'CommentStyle', '#');
fclose(fid);
if numel(C) >= 7 && ~isempty(C{5})
    M = [C{5}, C{6}, C{7}];
end
end


function bb = path_mat_bbox(mat_file)
% 返回 paths_only.layer_paths_3d 所有点的 bbox = [xmin xmax ymin ymax zmin zmax].
bb = [];
try
    S = load(mat_file, 'paths_only');
    po = S.paths_only;
    if ~isfield(po, 'layer_paths_3d'), return; end
    mn = [inf inf inf]; mx = [-inf -inf -inf];
    L = po.layer_paths_3d;
    for li = 1:numel(L)
        pl = L{li};
        if isempty(pl), continue; end
        for q = 1:numel(pl)
            pts = pl{q};
            if isempty(pts) || size(pts,2) < 3, continue; end
            mn = min(mn, min(pts(:,1:3), [], 1));
            mx = max(mx, max(pts(:,1:3), [], 1));
        end
    end
    if all(isfinite(mn))
        bb = [mn(1) mx(1) mn(2) mx(2) mn(3) mx(3)];
    end
catch
end
end