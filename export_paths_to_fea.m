function export_paths_to_fea(paths_mat_or_struct, cfg_name, base_dir, opts)
%% ================================================================
%% 把 paths_only mat 导出到 Abaqus FEA 期望的目录结构
%% ================================================================
%
% 用法 1 (从 mat 文件):
%   export_paths_to_fea('all_layers_paths_only_v3.mat', 'mine_stream');
%
% 用法 2 (传入 struct):
%   export_paths_to_fea(paths_only, 'mine_offset');
%
% 用法 3 (指定 base_dir 和选项):
%   opts.write_host = true;      % 是否写 host mesh (默认 true, 已存在则跳过)
%   opts.min_path_pts = 2;       % 路径最少点数 (默认 2)
%   opts.min_seg_len = 0.01;     % 段最小长度 mm, 去掉零长度段 (默认 0.01)
%   opts.path_precision = 6;     % 输出坐标小数位 (默认 6)
%   export_paths_to_fea(..., 'C:/temp/cfrc_fea', opts);
%
% 输出目录结构 (与 abaqus_cfrc_compare.py 期望完全一致):
%   <base_dir>/
%     host/
%       mesh_params.txt        # nelx N \n nely N \n nelz N \n
%       valid_elements.txt     # ix iy iz density xc yc zc (0-based)
%     <cfg_name>/
%       beam_paths/
%         path_0001.txt        # 每行: x y z
%         path_0002.txt
%         ...
%       beam_paths_summary.txt # 总路径数 + per-layer 信息 (用作 sentinel 文件)
%
% 重要: ix iy iz 是 0-based, 与 abaqus_cfrc_compare.py 内的循环一致;
%       xc yc zc 用 MATLAB 体素中心坐标 (mm), Abaqus 端会通过 auto-offset
%       自动对齐到 host node 网格.
%

if nargin < 3 || isempty(base_dir)
    base_dir = 'C:/temp/cfrc_fea';
end
if nargin < 4 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'write_host'),     opts.write_host = true; end
if ~isfield(opts, 'min_path_pts'),   opts.min_path_pts = 2; end
if ~isfield(opts, 'min_seg_len'),    opts.min_seg_len = 0.01; end
if ~isfield(opts, 'path_precision'), opts.path_precision = 6; end

fprintf('\n================================================================\n');
fprintf('  Exporting paths to FEA directory\n');
fprintf('  Config : %s\n', cfg_name);
fprintf('  BaseDir: %s\n', base_dir);
fprintf('================================================================\n');

%% --- 加载 paths_only ---
if ischar(paths_mat_or_struct) || isstring(paths_mat_or_struct)
    if ~exist(paths_mat_or_struct, 'file')
        error('paths file not found: %s', paths_mat_or_struct);
    end
    S = load(paths_mat_or_struct, 'paths_only');
    paths_only = S.paths_only;
else
    paths_only = paths_mat_or_struct;
end

if ~isfield(paths_only, 'layer_paths_3d')
    error('Input does not look like paths_only (missing layer_paths_3d)');
end
num_layers = length(paths_only.layer_paths_3d);
fprintf('Loaded paths_only: %d layers\n', num_layers);

%% --- 1) 写 host mesh (只在 base_dir/host/valid_elements.txt 不存在时写) ---
host_dir = fullfile(base_dir, 'host');
if ~exist(host_dir, 'dir'), mkdir(host_dir); end

mesh_params_path     = fullfile(host_dir, 'mesh_params.txt');
valid_elements_path  = fullfile(host_dir, 'valid_elements.txt');

need_write_host = opts.write_host && ~exist(valid_elements_path, 'file');
if need_write_host
    fprintf('\nWriting host mesh (one-time):\n');
    write_host_mesh(host_dir);
else
    if exist(valid_elements_path, 'file')
        fprintf('\nHost mesh already exists, skipping.\n');
    else
        fprintf('\nHost mesh writing disabled (opts.write_host=false).\n');
    end
end

%% --- 2) 写 per-config beam paths ---
cfg_dir = fullfile(base_dir, cfg_name);
if ~exist(cfg_dir, 'dir'), mkdir(cfg_dir); end

paths_dir = fullfile(cfg_dir, 'beam_paths');
if exist(paths_dir, 'dir')
    % 清空旧文件
    old_files = dir(fullfile(paths_dir, 'path_*.txt'));
    for f = 1:length(old_files)
        delete(fullfile(paths_dir, old_files(f).name));
    end
    fprintf('\nCleared %d old path files from %s\n', length(old_files), paths_dir);
else
    mkdir(paths_dir);
end

fprintf('\nWriting per-path files to: %s\n', paths_dir);

path_counter = 0;
per_layer_counts = zeros(num_layers, 1);
total_length = 0;
n_dropped_short = 0;
n_dropped_few   = 0;

fmt = sprintf('%%.%df %%.%df %%.%df\n', opts.path_precision, ...
    opts.path_precision, opts.path_precision);

for L = 1:num_layers
    paths_L = paths_only.layer_paths_3d{L};
    if isempty(paths_L), continue; end
    for k = 1:length(paths_L)
        pts = paths_L{k};
        if isempty(pts) || size(pts, 1) < opts.min_path_pts
            n_dropped_few = n_dropped_few + 1;
            continue;
        end
        % 去掉零长度连续段
        pts_clean = dedup_path(pts, opts.min_seg_len);
        if size(pts_clean, 1) < opts.min_path_pts
            n_dropped_short = n_dropped_short + 1;
            continue;
        end
        path_counter = path_counter + 1;
        per_layer_counts(L) = per_layer_counts(L) + 1;
        path_len = sum(sqrt(sum(diff(pts_clean).^2, 2)));
        total_length = total_length + path_len;

        fname = fullfile(paths_dir, sprintf('path_%04d.txt', path_counter));
        fid = fopen(fname, 'w');
        if fid < 0, error('Cannot open %s for writing', fname); end
        fprintf(fid, '# CFRC beam path layer=%d k=%d N=%d length=%.4f mm\n', ...
            L, k, size(pts_clean,1), path_len);
        fprintf(fid, '# columns: x y z (mm)\n');
        for p = 1:size(pts_clean, 1)
            fprintf(fid, fmt, pts_clean(p,1), pts_clean(p,2), pts_clean(p,3));
        end
        fclose(fid);
    end
end

%% --- 3) 写 summary (作为 sentinel 文件, abaqus_cfrc_compare 会检查它) ---
summary_path = fullfile(cfg_dir, 'beam_paths_summary.txt');
fid = fopen(summary_path, 'w');
fprintf(fid, '# CFRC beam paths summary for config: %s\n', cfg_name);
fprintf(fid, '# generated: %s\n', datestr(now));
fprintf(fid, '# source: %s\n', class(paths_mat_or_struct));
if isfield(paths_only, 'mode')
    fprintf(fid, '# mode: %s\n', paths_only.mode);
end
if isfield(paths_only, 'source_slice_file')
    fprintf(fid, '# source_slice_file: %s\n', paths_only.source_slice_file);
end
fprintf(fid, 'total_paths %d\n', path_counter);
fprintf(fid, 'total_length_mm %.4f\n', total_length);
fprintf(fid, 'num_layers %d\n', num_layers);
fprintf(fid, 'dropped_short_paths %d\n', n_dropped_short);
fprintf(fid, 'dropped_few_pts_paths %d\n', n_dropped_few);
fprintf(fid, '# per-layer path counts (layer_idx count):\n');
for L = 1:num_layers
    fprintf(fid, 'layer %d %d\n', L, per_layer_counts(L));
end
fclose(fid);

fprintf('\nDone:\n');
fprintf('  Total paths written: %d\n', path_counter);
fprintf('  Total length: %.2f mm\n', total_length);
fprintf('  Dropped: %d short, %d too-few-points\n', n_dropped_short, n_dropped_few);
fprintf('  Summary: %s\n', summary_path);

fprintf('\n[Next step] In Abaqus CAE:\n');
fprintf('  execfile(''abaqus_cfrc_compare.py'')\n');
fprintf('  run_single(''%s'')   %% or run_with_auto_retry({''%s''})\n\n', cfg_name, cfg_name);

end


%% ========================================================================
%% 子函数
%% ========================================================================
function write_host_mesh(host_dir)
% 写 mesh_params.txt + valid_elements.txt
S = load('voxel_refined_latest.mat', 'refined_data');
refined_data = S.refined_data;
grid_data       = refined_data.grid_data;
valid_grid_mask = refined_data.valid_grid_mask;
nelx = refined_data.grid_size.nelx;
nely = refined_data.grid_size.nely;
nelz = refined_data.grid_size.nelz;

% mesh_params.txt
fid = fopen(fullfile(host_dir, 'mesh_params.txt'), 'w');
fprintf(fid, '# host mesh params for CFRC FEA\n');
fprintf(fid, 'nelx %d\n', nelx);
fprintf(fid, 'nely %d\n', nely);
fprintf(fid, 'nelz %d\n', nelz);
fclose(fid);

% valid_elements.txt
fid = fopen(fullfile(host_dir, 'valid_elements.txt'), 'w');
fprintf(fid, '# columns: ix(0-based) iy iz density xc yc zc (xc,yc,zc are voxel centers in mm)\n');
n_written = 0;
for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if valid_grid_mask(i,j,k)
                g = grid_data(i,j,k);
                fprintf(fid, '%d %d %d %.6f %.6f %.6f %.6f\n', ...
                    i-1, j-1, k-1, g.xPhys, g.x, g.y, g.z);
                n_written = n_written + 1;
            end
        end
    end
end
fclose(fid);

fprintf('  mesh_params.txt: nelx=%d, nely=%d, nelz=%d\n', nelx, nely, nelz);
fprintf('  valid_elements.txt: %d valid voxels\n', n_written);
end


function pts_out = dedup_path(pts, min_seg_len)
% 去掉相邻间距 < min_seg_len 的点 (避免零长度 B31)
if size(pts, 1) < 2
    pts_out = pts;
    return;
end
keep = true(size(pts, 1), 1);
last = 1;
min_sq = min_seg_len^2;
for q = 2:size(pts, 1)
    d2 = sum((pts(q,:) - pts(last,:)).^2);
    if d2 < min_sq
        keep(q) = false;
    else
        last = q;
    end
end
pts_out = pts(keep, :);
end