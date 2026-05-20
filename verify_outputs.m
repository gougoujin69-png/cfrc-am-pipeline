%% verify_outputs.m
%% ================================================================
%% 检查 4 个 config 的路径 mat 和 2 个切片 mat 是否都正确生成
%% 报告:
%%   - 每个文件是否存在 + 大小
%%   - 切片: 层数 + 每层激活体素数 (min/max/total)
%%   - 路径: 层数 + 总路径数 + 总点数 + 总长度
%%   - 一致性检查: mine_*/planar_* 的层数是否和对应切片匹配
%% ================================================================

clear; clc;

fprintf('\n');
fprintf('================================================================\n');
fprintf('  CFRC 5-way Comparison - Output Verification\n');
fprintf('================================================================\n');

%% ---------- 待检查的文件 ----------
SLICE_FILES = {
    'curved'  'slice_results_refined_latest.mat';
    'planar'  'slice_results_refined_latest_PLANAR.mat';
};

PATH_FILES = {
    %  cfg_name        path mat                                       expected slice
    'mine_stream',    'all_layers_paths_only_v3.mat',                'curved';
    'mine_offset',    'all_layers_paths_only_mine_offset.mat',       'curved';
    'planar_stream',  'all_layers_paths_only_planar_stream.mat',     'planar';
    'planar_offset',  'all_layers_paths_only_planar_offset.mat',     'planar';
};

%% ---------- 1) 检查切片文件 ----------
fprintf('\n[1] Slice files\n');
fprintf('%-8s %-50s %-8s %-10s %-12s\n', 'Mode', 'File', 'Layers', 'Size(MB)', 'ValidVoxels');
fprintf('%s\n', repmat('-', 1, 90));

slice_layers = struct();
for s = 1:size(SLICE_FILES, 1)
    mode = SLICE_FILES{s, 1};
    fn   = SLICE_FILES{s, 2};
    if exist(fn, 'file') ~= 2
        fprintf('%-8s %-50s %-8s %-10s %s\n', mode, fn, 'MISS', '--', '--');
        slice_layers.(mode) = -1;
        continue;
    end
    d = dir(fn);
    sz_mb = d.bytes / 1024 / 1024;
    try
        S = load(fn, 'slice_results');
        nL = length(S.slice_results.surface_layers);
        if isfield(S.slice_results, 'valid_grid_mask')
            nvox = sum(S.slice_results.valid_grid_mask(:));
        else
            nvox = -1;
        end
        slice_layers.(mode) = nL;
        fprintf('%-8s %-50s %-8d %-10.1f %-12d\n', mode, fn, nL, sz_mb, nvox);
    catch ME
        fprintf('%-8s %-50s LOAD-FAIL: %s\n', mode, fn, ME.message);
        slice_layers.(mode) = -1;
    end
end

%% ---------- 2) 检查路径文件 ----------
fprintf('\n[2] Path files\n');
fprintf('%-16s %-46s %-7s %-8s %-7s %-10s %-12s\n', ...
    'Config', 'File', 'Layers', 'Paths', 'EmptyL', 'TotalPts', 'TotalLen(mm)');
fprintf('%s\n', repmat('-', 1, 110));

path_stats = struct();
for c = 1:size(PATH_FILES, 1)
    cfg = PATH_FILES{c, 1};
    fn  = PATH_FILES{c, 2};
    slc = PATH_FILES{c, 3};
    if exist(fn, 'file') ~= 2
        fprintf('%-16s %-46s %s\n', cfg, fn, 'MISSING');
        path_stats.(cfg).ok = false;
        continue;
    end
    try
        S = load(fn, 'paths_only');
        po = S.paths_only;
        nL = length(po.layer_paths_3d);

        total_paths = 0;
        empty_layers = 0;
        total_pts = 0;
        total_len = 0.0;
        per_layer_path_count = zeros(nL, 1);
        per_layer_len = zeros(nL, 1);

        for L = 1:nL
            pL = po.layer_paths_3d{L};
            if isempty(pL)
                empty_layers = empty_layers + 1;
                continue;
            end
            per_layer_path_count(L) = length(pL);
            total_paths = total_paths + length(pL);
            for k = 1:length(pL)
                pts = pL{k};
                if isempty(pts) || size(pts, 1) < 2, continue; end
                total_pts = total_pts + size(pts, 1);
                seg = sqrt(sum(diff(pts, 1, 1).^2, 2));
                this_len = sum(seg);
                total_len = total_len + this_len;
                per_layer_len(L) = per_layer_len(L) + this_len;
            end
        end

        fprintf('%-16s %-46s %-7d %-8d %-7d %-10d %-12.1f\n', ...
            cfg, fn, nL, total_paths, empty_layers, total_pts, total_len);

        path_stats.(cfg).ok = true;
        path_stats.(cfg).num_layers = nL;
        path_stats.(cfg).total_paths = total_paths;
        path_stats.(cfg).empty_layers = empty_layers;
        path_stats.(cfg).total_pts = total_pts;
        path_stats.(cfg).total_len = total_len;
        path_stats.(cfg).per_layer_path_count = per_layer_path_count;
        path_stats.(cfg).per_layer_len = per_layer_len;
        path_stats.(cfg).expected_slice = slc;
    catch ME
        fprintf('%-16s %-46s LOAD-FAIL: %s\n', cfg, fn, ME.message);
        path_stats.(cfg).ok = false;
    end
end

%% ---------- 3) 一致性检查 ----------
fprintf('\n[3] Consistency check (paths layers == slice layers)\n');
fprintf('%-16s %-12s %-12s %-12s %s\n', ...
    'Config', 'PathLayers', 'SliceMode', 'SliceLayers', 'Match?');
fprintf('%s\n', repmat('-', 1, 70));

all_ok = true;
for c = 1:size(PATH_FILES, 1)
    cfg = PATH_FILES{c, 1};
    slc = PATH_FILES{c, 3};
    if ~isfield(path_stats, cfg) || ~path_stats.(cfg).ok
        fprintf('%-16s %-12s %-12s %-12s %s\n', cfg, '?', slc, '?', 'SKIP');
        all_ok = false;
        continue;
    end
    pnL = path_stats.(cfg).num_layers;
    if isfield(slice_layers, slc)
        snL = slice_layers.(slc);
    else
        snL = -1;
    end
    if snL == -1
        match_str = 'NO-SLICE';
        all_ok = false;
    elseif pnL == snL
        match_str = 'OK';
    else
        match_str = sprintf('MISMATCH (%d vs %d)', pnL, snL);
        all_ok = false;
    end
    fprintf('%-16s %-12d %-12s %-12d %s\n', cfg, pnL, slc, snL, match_str);
end

%% ---------- 4) Per-layer path count 对比 (横排表格) ----------
fprintf('\n[4] Per-layer path count (4 configs side-by-side)\n');
max_layers = 0;
for c = 1:size(PATH_FILES, 1)
    cfg = PATH_FILES{c, 1};
    if isfield(path_stats, cfg) && path_stats.(cfg).ok
        max_layers = max(max_layers, path_stats.(cfg).num_layers);
    end
end

if max_layers > 0
    fprintf('%-6s', 'Layer');
    for c = 1:size(PATH_FILES, 1)
        fprintf(' %-15s', PATH_FILES{c, 1});
    end
    fprintf('\n');
    fprintf('%s\n', repmat('-', 1, 6 + 16*4));

    for L = 1:max_layers
        fprintf('%-6d', L);
        for c = 1:size(PATH_FILES, 1)
            cfg = PATH_FILES{c, 1};
            if ~isfield(path_stats, cfg) || ~path_stats.(cfg).ok
                fprintf(' %-15s', '--');
                continue;
            end
            if L > path_stats.(cfg).num_layers
                fprintf(' %-15s', '(no layer)');
            else
                cnt = path_stats.(cfg).per_layer_path_count(L);
                lenmm = path_stats.(cfg).per_layer_len(L);
                fprintf(' %4d p / %5.0fmm', cnt, lenmm);
            end
        end
        fprintf('\n');
    end
end

%% ---------- 5) 总结 ----------
fprintf('\n[5] Summary\n');
fprintf('%s\n', repmat('=', 1, 70));
for c = 1:size(PATH_FILES, 1)
    cfg = PATH_FILES{c, 1};
    if isfield(path_stats, cfg) && path_stats.(cfg).ok
        ps = path_stats.(cfg);
        avg_pts_per_path = ps.total_pts / max(ps.total_paths, 1);
        avg_len_per_path = ps.total_len / max(ps.total_paths, 1);
        fprintf('  %-16s : %4d layers, %5d paths, %7d pts, %8.1f mm  ', ...
            cfg, ps.num_layers, ps.total_paths, ps.total_pts, ps.total_len);
        fprintf('(avg %.1f pts/path, %.2f mm/path)\n', avg_pts_per_path, avg_len_per_path);
    else
        fprintf('  %-16s : NOT GENERATED OR FAILED TO LOAD\n', cfg);
    end
end

fprintf('\n');
if all_ok
    fprintf('==> ALL CONSISTENT. 可以进 Stage 7 (export to FEA).\n');
else
    fprintf('==> Some issues found above. Review before exporting.\n');
end
fprintf('\n');
