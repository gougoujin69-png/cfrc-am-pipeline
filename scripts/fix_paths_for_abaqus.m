function fix_paths_for_abaqus(paths_mat, cfg_name, opts)
%% =================================================================
%% 过滤掉 Abaqus B31 单元处理不了的 segment, 在坏 segment 处切断路径,
%% 重新保存 paths_only mat 并重新导出到 FEA 目录.
%%
%% 用法:
%%   fix_paths_for_abaqus('all_layers_paths_only_planar_stream.mat', 'planar_stream')
%%
%% 选项 opts (可不传, 使用默认):
%%   opts.n1            = [0.309, 0.619, 0.722]   (必须和 abaqus_cfrc_compare.py 里 Config.BEAM_N1 一致)
%%   opts.min_seg_len   = 0.05  mm                (零长 / 极短 segment 阈值)
%%   opts.cos_threshold = 0.99                    (|dot(t_unit, n1_unit)| 超过此值 = 共线)
%%   opts.uturn_deg     = 178.0                   (相邻 segment 折角超过此值 = 反向 U-turn)
%%   opts.base_dir      = 'C:/temp/cfrc_fea'
%%   opts.backup        = true                    (覆盖原 mat 前先备份成 .bak.mat)
%%   opts.reexport      = true                    (修完后是否重新导出到 FEA 目录)
%%
%% 三种坏 segment 对应的 Abaqus 错误:
%%   - 零/极短     -> ErrElemZeroLength, ErrElemLenSmallNegZero
%%   - 共线 n1     -> ErrElemNormal, ErrElemBeamSecNormal, ErrElemBeamSecDirVect
%%   - U-turn      -> 同样会导致法线计算失败
%% =================================================================

if nargin < 2, error('需要 paths_mat 和 cfg_name 两个参数'); end
if nargin < 3, opts = struct(); end
if ~isfield(opts, 'n1'),            opts.n1 = [0.309, 0.619, 0.722]; end
if ~isfield(opts, 'min_seg_len'),   opts.min_seg_len = 0.05; end
if ~isfield(opts, 'cos_threshold'), opts.cos_threshold = 0.99; end
if ~isfield(opts, 'uturn_deg'),     opts.uturn_deg = 178.0; end
if ~isfield(opts, 'base_dir'),      opts.base_dir = 'C:/temp/cfrc_fea'; end
if ~isfield(opts, 'backup'),        opts.backup = true; end
if ~isfield(opts, 'reexport'),      opts.reexport = true; end

n1 = opts.n1(:)' / norm(opts.n1);
cos_uturn = cosd(opts.uturn_deg);   % e.g., cosd(178) = -0.9994

fprintf('\n=========================================================\n');
fprintf('  fix_paths_for_abaqus\n');
fprintf('  paths_mat     : %s\n', paths_mat);
fprintf('  cfg_name      : %s\n', cfg_name);
fprintf('  n1 (norm)     : (%.4f, %.4f, %.4f)\n', n1);
fprintf('  min_seg_len   : %.3f mm\n', opts.min_seg_len);
fprintf('  cos_threshold : %.3f  (|cos(t,n1)| 超此值 = 共线)\n', opts.cos_threshold);
fprintf('  uturn_deg     : %.1f deg\n', opts.uturn_deg);
fprintf('=========================================================\n');

if exist(paths_mat, 'file') ~= 2
    error('paths_mat not found: %s', paths_mat);
end

% --- 备份 ---
if opts.backup
    bak = strrep(paths_mat, '.mat', '.bak.mat');
    if exist(bak, 'file') == 2, delete(bak); end
    copyfile(paths_mat, bak);
    fprintf('\nBackup: %s\n', bak);
end

S = load(paths_mat, 'paths_only');
paths_only = S.paths_only;
n_layers = length(paths_only.layer_paths_3d);

% --- 统计 ---
orig_path_count = 0; new_path_count = 0; dropped_path_count = 0;
n_bad_short = 0; n_bad_parallel = 0; n_bad_uturn = 0;
n_split_paths = 0;
orig_pts = 0; new_pts = 0;

for L = 1:n_layers
    pL = paths_only.layer_paths_3d{L};
    if isempty(pL), continue; end
    new_pL = {};
    for k = 1:length(pL)
        pts = pL{k};
        orig_path_count = orig_path_count + 1;
        if isempty(pts) || size(pts, 1) < 2
            dropped_path_count = dropped_path_count + 1;
            continue;
        end
        orig_pts = orig_pts + size(pts, 1);
        [sub_pts, ns, np, nu] = split_path_on_bad_segs(...
            pts, n1, opts.min_seg_len, opts.cos_threshold, cos_uturn);
        n_bad_short    = n_bad_short    + ns;
        n_bad_parallel = n_bad_parallel + np;
        n_bad_uturn    = n_bad_uturn    + nu;

        if isempty(sub_pts)
            dropped_path_count = dropped_path_count + 1;
        else
            if length(sub_pts) > 1
                n_split_paths = n_split_paths + 1;
            end
            for s = 1:length(sub_pts)
                if size(sub_pts{s}, 1) >= 2
                    new_pL{end+1} = sub_pts{s}; %#ok<AGROW>
                    new_path_count = new_path_count + 1;
                    new_pts = new_pts + size(sub_pts{s}, 1);
                end
            end
        end
    end
    paths_only.layer_paths_3d{L} = new_pL;
end

% 同时清掉对应的 2d (可选, 这里直接置空, FEA 用不上)
if isfield(paths_only, 'layer_paths_2d')
    for L = 1:length(paths_only.layer_paths_2d)
        % 不动 2d, 它只在路径生成阶段用; FEA 不读 2d
    end
end

fprintf('\n--- Filter results ---\n');
fprintf('  Original paths     : %6d   (%d pts)\n', orig_path_count, orig_pts);
fprintf('  Output paths       : %6d   (%d pts)\n', new_path_count, new_pts);
fprintf('  Paths split        : %6d   (path 被切成多段)\n', n_split_paths);
fprintf('  Paths dropped      : %6d   (整条剩 < 2 个点)\n', dropped_path_count);
fprintf('  Bad segs removed   : %d short + %d parallel + %d uturn = %d total\n', ...
    n_bad_short, n_bad_parallel, n_bad_uturn, ...
    n_bad_short + n_bad_parallel + n_bad_uturn);

save(paths_mat, 'paths_only', '-v7.3');
fprintf('\nSaved cleaned mat: %s\n', paths_mat);

% --- 重新导出到 FEA 目录 ---
if opts.reexport
    fprintf('\nRe-exporting %s to %s ...\n', cfg_name, opts.base_dir);
    exp_opts = struct('write_host', false);   % host 已写过, 不动
    export_paths_to_fea(paths_only, cfg_name, opts.base_dir, exp_opts);
end

fprintf('\n=========================================================\n');
fprintf('  Done. 下一步在 Abaqus CAE 里:\n');
fprintf('  >>> step2_run_batch_comparison()    %% 跳过已成功的, 只重跑 %s\n', cfg_name);
fprintf('  或定向跑这一个 config:\n');
fprintf('  >>> run_config(''%s'')\n', cfg_name);
fprintf('=========================================================\n\n');

end


%% =================================================================
%% 子函数: 在坏 segment 处切断路径
%% =================================================================
function [sub_paths, n_short, n_parallel, n_uturn] = split_path_on_bad_segs(...
    pts, n1, min_seg_len, cos_threshold, cos_uturn)

n_pts = size(pts, 1);
if n_pts < 2
    sub_paths = {}; n_short = 0; n_parallel = 0; n_uturn = 0;
    return;
end

% segment 向量和长度
seg_vec = diff(pts, 1, 1);                    % (N-1) x 3
seg_len = sqrt(sum(seg_vec.^2, 2));
seg_unit = bsxfun(@rdivide, seg_vec, max(seg_len, 1e-12));

% 三种坏标记
bad_short    = seg_len < min_seg_len;
cos_n1       = abs(seg_unit * n1');
bad_parallel = cos_n1 > cos_threshold;

% U-turn: 第 q 段和 q+1 段几乎反向 (dot(t_q, t_{q+1}) < cos_uturn, 即两 segment
% 夹角接近 180 度). 把第 q+1 段标坏 (留下第 q 段, 切断 q+1).
bad_uturn = false(size(bad_short));
if length(seg_unit) >= 2
    dot_tt = sum(seg_unit(1:end-1,:) .* seg_unit(2:end,:), 2);
    uturn_mask = dot_tt < cos_uturn;
    bad_uturn(2:end) = uturn_mask;
end

n_short    = sum(bad_short);
n_parallel = sum(bad_parallel & ~bad_short);
n_uturn    = sum(bad_uturn & ~bad_short & ~bad_parallel);

bad_seg = bad_short | bad_parallel | bad_uturn;

if all(~bad_seg)
    sub_paths = {pts};
    return;
end

% 切分: 遇到坏 segment 就结束当前 sub-path, 从坏 segment 的终点重新开始一条
sub_paths = {};
cur = pts(1, :);
for q = 1:length(bad_seg)
    if bad_seg(q)
        if size(cur, 1) >= 2
            sub_paths{end+1} = cur; %#ok<AGROW>
        end
        cur = pts(q+1, :);
    else
        cur = [cur; pts(q+1, :)]; %#ok<AGROW>
    end
end
if size(cur, 1) >= 2
    sub_paths{end+1} = cur;
end
end
