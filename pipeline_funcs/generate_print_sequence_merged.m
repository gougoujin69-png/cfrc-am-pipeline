function manifest = generate_print_sequence_merged(merged_file, output_dir, options)
%% GENERATE_PRINT_SEQUENCE_MERGED  从"已处理合并数据"按打印时序输出指令文件
% =====================================================================
% 输入数据(每层 all_layers_data(k)):
%   connected_paths        碳纤维(已组装/变换/重采样)
%   resin_connected_paths  支撑(内部空腔填充, 来自 pipeline_step0_generate_support)
%   pointCloud_data        法向参考曲面
%
% 时序(从下到上, 碳/支撑逐层穿插):
%   L1 碳纤维(全部, 一次打完) -> L1 支撑(全部) ->
%   L2 碳纤维(全部)           -> L2 支撑(全部) -> ... -> Ln 碳纤维
%
% 满足的约束:
%   * 碳纤维同层一次打完: 先连续输出该层全部 connected_paths.
%   * 支撑在同层碳纤维之后: 该层碳纤维已完成, 支撑不再阻挡它.
%   * "被支撑的碳纤维"(空腔顶)在更高层 -> 排在其下方支撑之后, 即支撑先完成.
%   * 支撑随时可暂停: 支撑以独立路径段(独立文件)输出, 段间天然可暂停插碳纤维.
%
% 输出:
%   {output_dir}/print_NNNN_<carbon|support>_L<k>_P<i>.xlsx
%   {output_dir}/print_manifest.csv   (机械臂主程序按 seq 顺序读这个)
%
% Excel 8 列: [x y z v1 v2 v3 v4 v5]
%   v1 = atan2(-ny,-nz) (deg)   v2 = asin(-nx) (deg)
%   v3 v4 v5 = 信号 (碳=[0 14 111], 支撑=[0 23 222])
%
% 依赖: getPathNormals.m 在路径上.
%
% 用法:
%   generate_print_sequence_merged('Manufacturing_printing_path.mat','print_sequence');
% 注: 该文件须含 connected_paths 与 resin_connected_paths. 若你的 Step3.1 没把
%     resin_connected_paths 带过来, 改用 Step2 产物 'Modified_printing_path.mat'.
% =====================================================================

opt.carbon_field = 'connected_paths';
opt.resin_field  = 'resin_connected_paths';
opt.xyz_add      = [0 0 0];
opt.carbon_signal = [0 14 111];
opt.resin_signal  = [0 23 222];
opt.format       = 'xlsx';      % 'xlsx' 或 'csv' (csv 快很多)
opt.verbose      = true;
if nargin>=3 && ~isempty(options)
    fn = fieldnames(options); for i=1:numel(fn), opt.(fn{i}) = options.(fn{i}); end
end
if nargin<2 || isempty(output_dir), output_dir = 'print_sequence'; end
if ~exist(output_dir,'dir'), mkdir(output_dir); end

S = load(merged_file);
if isfield(S,'all_layers_data'), ALD = S.all_layers_data;
elseif isfield(S,'results') && isfield(S.results,'all_layers_data'), ALD = S.results.all_layers_data;
else, error('在 %s 找不到 all_layers_data', merged_file); end
nL = numel(ALD);
has_carbon = isfield(ALD, opt.carbon_field);
has_resin  = isfield(ALD, opt.resin_field);
if ~has_carbon, warning('无字段 %s, 碳纤维不会输出', opt.carbon_field); end
if ~has_resin,  warning('无字段 %s, 支撑不会输出',   opt.resin_field);  end

fprintf('[TimedSeq] 层数 %d  碳纤维=%s  支撑=%s  格式=%s\n', ...
        nL, opt.carbon_field, opt.resin_field, opt.format);

manifest = {}; seq = 0; tic;
for k = 1:nL
    pc = ALD(k).pointCloud_data;

    % (1) 碳纤维 L_k (先打, 同层一次性)
    if has_carbon
        paths = ALD(k).(opt.carbon_field);
        if iscell(paths)
            for i = 1:numel(paths)
                pth = normalize_path(paths{i});
                if size(pth,1)<2, continue; end
                seq = seq + 1;
                fn = sprintf('print_%04d_carbon_L%d_P%d.%s', seq, k, i, opt.format);
                write_one(fullfile(output_dir,fn), pth, pc, opt.xyz_add, opt.carbon_signal);
                manifest(end+1,:) = {seq, fn, 'carbon', k, size(pth,1), sig(opt.carbon_signal)}; %#ok<AGROW>
            end
        end
    end

    % (2) 支撑 L_k (后打)
    if has_resin
        rpaths = ALD(k).(opt.resin_field);
        if iscell(rpaths)
            for i = 1:numel(rpaths)
                pth = normalize_path(rpaths{i});
                if size(pth,1)<2, continue; end
                seq = seq + 1;
                fn = sprintf('print_%04d_support_L%d_P%d.%s', seq, k, i, opt.format);
                write_one(fullfile(output_dir,fn), pth, pc, opt.xyz_add, opt.resin_signal);
                manifest(end+1,:) = {seq, fn, 'support', k, size(pth,1), sig(opt.resin_signal)}; %#ok<AGROW>
            end
        end
    end

    if opt.verbose && (mod(k,5)==0 || k==nL)
        fprintf('  层 %3d/%-3d  累计文件 %5d  (%.1fs)\n', k, nL, seq, toc);
    end
end

T = cell2table(manifest, 'VariableNames', ...
    {'seq','filename','type','layer','n_points','signal_v3v4v5'});
mpath = fullfile(output_dir,'print_manifest.csv');
writetable(T, mpath);
fprintf('[TimedSeq] 完成! 文件 %d, 清单 %s (%.1fs)\n', seq, mpath, toc);
end

%% ================= 子函数 =================
function p = normalize_path(p)
    if isempty(p) || ~isnumeric(p), p = []; return; end
    if size(p,2)~=3 && size(p,1)==3, p = p.'; end
end

function write_one(fpath, xyz, pc, add, sigv)
    n  = getPathNormals(xyz, pc);
    v2 = rad2deg(asin(max(-1,min(1,-n(:,1)))));
    v1 = rad2deg(atan2(-n(:,2), -n(:,3)));
    out = [xyz + add, v1, v2, repmat(sigv(:).', size(xyz,1), 1)];
    writematrix(out, fpath);
end

function s = sig(v), s = sprintf('%g,%g,%g', v(1), v(2), v(3)); end
