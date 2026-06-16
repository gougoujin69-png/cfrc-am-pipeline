function merge_carbon_resin_data(carbon_file, resin_data, output_file)
%% MERGE_CARBON_RESIN_DATA  把碳纤维原始数据 + 树脂 (Step 0 输出) 合并成 Step1_1 期望格式
%
% 输出文件结构 (顶层 all_layers_data, 每层字段):
%   paths_3d        cell of K×3   碳纤维 (从 carbon_file 来, 自动 3×K -> K×3)
%   paths_3dr       cell of K×3   树脂   (从 resin_data 来, 已经是 K×3)
%   paths_2d        cell of K×2   碳纤维 2D (Step1_1/Step1_2 会用)
%   paths_2dr       cell          空 cell (我们没生成树脂 2D, Step1_1 缩放空集仍是空)
%   pointCloud_data 结构 {X,Y,Z}  曲面
%   success         bool          继承原始
%
% 输入:
%   carbon_file  - 'all_layers_path_results_v3.mat' (顶层 results.all_layers_data)
%   resin_data   - Step 0 输出, struct array, 每个 .paths_3d 是 cell of K×3
%   output_file  - 'all_layers_path_carbon_resin.mat' (Step1_1 期望的文件名)

fprintf('[Merge] 加载碳纤维: %s\n', carbon_file);
S = load(carbon_file);
if isfield(S,'results') && isfield(S.results,'all_layers_data')
    ALD = S.results.all_layers_data;
elseif isfield(S,'all_layers_data')
    ALD = S.all_layers_data;
else
    error('在 %s 里找不到 all_layers_data', carbon_file);
end
nL = numel(ALD);
fprintf('  碳纤维层数: %d\n', nL);

nR = numel(resin_data);
fprintf('  树脂层间数: %d (resin_data(k) 表示层 k 之上 / 层 k+1 之下的填充)\n', nR);

%% 规范化 paths_3d / paths_2d 形状 (统一成 K×3 / K×2)
fprintf('  规范化路径形状 (3×K -> K×3, 2×L -> L×2)...\n');
for k = 1:nL
    ALD(k).paths_3d = normalize_cells(ALD(k).paths_3d, 3);
    if isfield(ALD, 'paths_2d')
        ALD(k).paths_2d = normalize_cells(ALD(k).paths_2d, 2);
    else
        ALD(k).paths_2d = {};
    end
end

%% 注入树脂字段
fprintf('  注入 paths_3dr (树脂) / paths_2dr (空)...\n');
n_carb_total = 0;
n_res_total  = 0;
for k = 1:nL
    if k <= nR && isfield(resin_data,'paths_3d') && ~isempty(resin_data(k).paths_3d)
        ALD(k).paths_3dr = resin_data(k).paths_3d;   % 已经 K×3
    else
        ALD(k).paths_3dr = {};
    end
    ALD(k).paths_2dr = {};   % Step1_1 会循环它的 length, 空 cell 是合法的
    n_carb_total = n_carb_total + numel(ALD(k).paths_3d);
    n_res_total  = n_res_total  + numel(ALD(k).paths_3dr);
end

%% 保存 (-v7.3 因为可能 > 2GB)
all_layers_data = ALD;
save(output_file, 'all_layers_data', '-v7.3');
fprintf('[Merge] 已保存: %s\n', output_file);
fprintf('  总碳纤维路径段: %d\n', n_carb_total);
fprintf('  总树脂路径段:   %d\n', n_res_total);

% 同时保存一份只读副本 (Step 1 会修改主文件, 副本用于"重新开始")
backup = strrep(output_file, '.mat', '_raw.mat');
copyfile(output_file, backup);
fprintf('  原始副本:       %s  (重跑流水线时从这里恢复)\n', backup);

end

function out = normalize_cells(c, dim)
% cell of (dim × K) or (K × dim) -> 统一成 K × dim
out = c;
if ~iscell(c), return; end
for i = 1:numel(c)
    seg = c{i};
    if ~isnumeric(seg) || isempty(seg), continue; end
    if size(seg,1)==dim && size(seg,2)~=dim
        out{i} = seg.';
    end
end
end
