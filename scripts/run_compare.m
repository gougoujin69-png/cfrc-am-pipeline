function run_compare(varargin)
% run_compare  在 C:\temp\cfrc_fea 启动 compare_fea_results
%
% 用法 (放在 C:\temp\cfrc_fea\run_compare.m, 在 MATLAB 任何位置调用):
%   >> cd C:\temp\cfrc_fea
%   >> run_compare              % 跑 compare_fea_results
%   >> run_compare('stats')     % 跑 compute_path_statistics
%   >> run_compare('all')       % 跑 stats + compare
%
% 工作原理:
%   1. 自动 addpath 到所有可能放原脚本的位置 (MATLAB 项目目录)
%   2. cd 到 C:\temp\cfrc_fea (FEA 数据目录, 所有 csv/mat 都在这)
%   3. 执行 compare_fea_results / compute_path_statistics
%
% 如果项目根目录变了, 改下面的 PROJECT_ROOT_FALLBACK (优先用 cfrc_layout 自动定位)

% ============================================================
% 1. 对比/统计脚本所在目录
%    compare_fea_results.m 与 compute_path_statistics.m 随本项目走, 位于
%    项目的 scripts/ (辅助函数在 functions/, 绘图在 viz/). 用项目布局函数
%    cfrc_layout() 自动定位 —— 与其它阶段"脚本/数据各归其位"的逻辑一致。
%    脱离 App 单独运行 (cfrc_layout 不在路径上) 时回退到项目根。
% ============================================================
PROJECT_ROOT_FALLBACK = 'E:\308\傅里叶\chair-leg\new';
script_dirs = {};
if exist('cfrc_layout', 'file') == 2
    try
        Lyt = cfrc_layout();
        script_dirs = {Lyt.scripts, Lyt.functions, Lyt.viz};
    catch
    end
end
if isempty(script_dirs)
    script_dirs = { ...
        fullfile(PROJECT_ROOT_FALLBACK, 'scripts'), ...
        fullfile(PROJECT_ROOT_FALLBACK, 'functions'), ...
        fullfile(PROJECT_ROOT_FALLBACK, 'viz') };
end

% ============================================================
% 2. FEA 数据目录 (输出 csv/mat 都在这)
% ============================================================
fea_dir = 'C:\temp\cfrc_fea';

% ============================================================
% 3. 解析参数: 默认跑 compare, 也可以指定 stats / all
% ============================================================
if nargin == 0
    mode = 'compare';
else
    mode = lower(varargin{1});
end
if ~ismember(mode, {'compare', 'stats', 'all'})
    error('run_compare:badArg', ...
        'Unknown mode: %s. Use ''compare'' / ''stats'' / ''all''.', mode);
end

% ============================================================
% 4. addpath 用户脚本目录 (静默失败, 后面再统一报错)
% ============================================================
fprintf('[run_compare] Searching for user MATLAB scripts...\n');
added_any = false;
for i = 1:numel(script_dirs)
    d = script_dirs{i};
    if exist(d, 'dir')
        addpath(d);
        fprintf('  [OK] addpath %s\n', d);
        added_any = true;
    else
        fprintf('  [skip] not exist: %s\n', d);
    end
end
if ~added_any
    error(['run_compare:noScriptDir\n', ...
           'No script directory found. 检查 cfrc_layout 是否在路径上, 或修正 PROJECT_ROOT_FALLBACK。']);
end

% ============================================================
% 5. 检查所需脚本可见
% ============================================================
required = {};
if ismember(mode, {'compare', 'all'})
    required{end+1} = 'compare_fea_results';
end
if ismember(mode, {'stats', 'all'})
    required{end+1} = 'compute_path_statistics';
end

missing = {};
for i = 1:numel(required)
    if ~exist(required{i}, 'file')
        missing{end+1} = required{i};  %#ok<AGROW>
    else
        which_path = which(required{i});
        fprintf('  [OK] %s -> %s\n', required{i}, which_path);
    end
end
if ~isempty(missing)
    error(['run_compare:missingScripts\n', ...
           'Cannot find: %s\n确认 compare_fea_results.m / compute_path_statistics.m 在项目 scripts/ 下 (或修正 PROJECT_ROOT_FALLBACK)。'], ...
           strjoin(missing, ', '));
end

% ============================================================
% 6. cd 到 FEA 数据目录
% ============================================================
if ~exist(fea_dir, 'dir')
    error('run_compare:noFeaDir', 'FEA dir does not exist: %s', fea_dir);
end
old_pwd = pwd;
cleanup = onCleanup(@() cd(old_pwd));
cd(fea_dir);
fprintf('[run_compare] Now in: %s\n', pwd);

% Show what FEA outputs we can see
csv_files = dir('*.csv');
mat_files = dir('*.mat');
odb_files = dir('*.odb');
fprintf('  CSV: %d   MAT: %d   ODB: %d\n', ...
    numel(csv_files), numel(mat_files), numel(odb_files));

% ============================================================
% 7. 跑用户脚本
% ============================================================
fprintf('\n');
switch mode
    case 'stats'
        fprintf('[run_compare] Running compute_path_statistics...\n');
        compute_path_statistics;
    case 'compare'
        fprintf('[run_compare] Running compare_fea_results...\n');
        compare_fea_results;
    case 'all'
        fprintf('[run_compare] Step 1/2: compute_path_statistics...\n');
        compute_path_statistics;
        fprintf('\n[run_compare] Step 2/2: compare_fea_results...\n');
        compare_fea_results;
end

fprintf('\n[run_compare] DONE\n');
end
