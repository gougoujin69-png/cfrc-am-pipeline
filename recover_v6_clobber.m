%% recover_v6_clobber.m
%% ============================================================
%% 一次性恢复脚本: 用来修复 run_v6_with_slice 因 v6 顶部 `clear`
%% 把函数局部变量清空导致的崩溃后文件错位.
%%
%% 在 MATLAB 命令行 cd 到你的脚本目录然后:
%%   run('recover_v6_clobber.m')
%% 或直接复制粘贴进命令窗口.
%% ============================================================

fprintf('\n=== Recovery: undo v6-clobber ===\n');

% 1) 把被 v6 写出来的 all_layers_paths_only_v3.mat (现在内容其实是
%    planar_stream) 改名成 planar_stream 的正确文件名,
%    再把 mine_stream 的备份还原回 v3.mat
if exist('_bkp_v3_paths_master.mat', 'file') == 2
    if exist('all_layers_paths_only_v3.mat', 'file') == 2
        if exist('all_layers_paths_only_planar_stream.mat', 'file') == 2
            delete('all_layers_paths_only_planar_stream.mat');
        end
        movefile('all_layers_paths_only_v3.mat', 'all_layers_paths_only_planar_stream.mat');
        fprintf('  [1] all_layers_paths_only_v3.mat -> all_layers_paths_only_planar_stream.mat  (salvaged)\n');
    end
    movefile('_bkp_v3_paths_master.mat', 'all_layers_paths_only_v3.mat');
    fprintf('  [2] _bkp_v3_paths_master.mat -> all_layers_paths_only_v3.mat  (mine_stream restored)\n');
else
    fprintf('  [skip] _bkp_v3_paths_master.mat not found (already recovered?)\n');
end

% 2) 删除被覆盖的 slice_results_refined_latest.mat (现在是平面切片的副本),
%    再把曲面切片备份还原
if exist('_bkp_curved_slice_master.mat', 'file') == 2
    if exist('slice_results_refined_latest.mat', 'file') == 2
        delete('slice_results_refined_latest.mat');
        fprintf('  [3] Deleted clobbered slice_results_refined_latest.mat (was planar copy)\n');
    end
    movefile('_bkp_curved_slice_master.mat', 'slice_results_refined_latest.mat');
    fprintf('  [4] _bkp_curved_slice_master.mat -> slice_results_refined_latest.mat  (curved restored)\n');
else
    fprintf('  [skip] _bkp_curved_slice_master.mat not found (already recovered?)\n');
end

% 3) 报告最终状态
fprintf('\n--- Final state ---\n');
files = {
    'slice_results_refined_latest.mat',         '应为曲面切片';
    'slice_results_refined_latest_PLANAR.mat',  '应为平面切片';
    'all_layers_paths_only_v3.mat',             '应为 mine_stream 路径';
    'all_layers_paths_only_mine_offset.mat',    '应为 mine_offset 路径';
    'all_layers_paths_only_planar_stream.mat',  '应为 planar_stream 路径 (本次抢救出来)';
    'all_layers_paths_only_planar_offset.mat',  '应为 planar_offset 路径 (这次没生成, 下次跑会补)';
};
for k = 1:size(files, 1)
    if exist(files{k,1}, 'file') == 2
        d = dir(files{k,1});
        fprintf('  [OK]   %-50s %8.1f MB  -- %s\n', ...
            files{k,1}, d.bytes/1024/1024, files{k,2});
    else
        fprintf('  [miss] %-50s             -- %s\n', files{k,1}, files{k,2});
    end
end

fprintf('\n=== Recovery done. ===\n');
fprintf('Next: 用新版 run_full_comparison.m 覆盖旧的, 然后 run_full_comparison\n');
fprintf('      预期: 4 个 config 中只有 planar_offset 需要新跑.\n\n');
