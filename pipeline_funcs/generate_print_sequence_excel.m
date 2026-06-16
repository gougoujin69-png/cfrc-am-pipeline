function manifest = generate_print_sequence_excel(carbon_file, resin_input, output_dir, options)
%% GENERATE_PRINT_SEQUENCE_EXCEL  按打印时序输出顺序编号的机械臂指令文件
%
% 时序逻辑 (从下到上, 碳/树脂逐层穿插):
%   L1 碳纤维全部  ->  (L1, L2) 树脂全部  ->
%   L2 碳纤维全部  ->  (L2, L3) 树脂全部  ->
%   ...  ->  Ln 碳纤维
%
% 输入:
%   carbon_file  - 碳纤维 .mat 文件 (含 results.all_layers_data 或 all_layers_data)
%   resin_input  - 树脂数据: resin_data 结构体, 或 mat 文件路径
%   output_dir   - 输出目录, 默认 'print_sequence'
%   options      - 可选, 覆盖默认参数 (见下方 opt 结构)
%
% 输出文件:
%   {output_dir}/print_NNNN_<type>_L<k>[-L<k+1>]_P<i>.xlsx
%   {output_dir}/print_manifest.csv     -- 清单 (机械臂主程序读这个)
%   {output_dir}/README.md              -- 文件命名 + Excel 列定义
%
% Excel 8 列 (与你的 Step3_2_Robotic_*_path.m 一致):
%   [x, y, z, v1, v2, v3, v4, v5]
%     v1 = rad2deg(atan2(-ny, -nz))
%     v2 = rad2deg(asin(-nx))
%     v3, v4, v5 = 信号 (默认 碳=[0,14,111], 树脂=[0,23,222])
%
% 依赖: getPathNormals.m 必须在路径上.

%% ---------- 默认参数 ----------
opt.xyz_add        = [0 0 0];          % 全局坐标偏移
opt.format         = 'xlsx';           % 'xlsx' 或 'csv'  (csv 写入快 10-100 倍, 体积小)
opt.carbon_signal  = [0 14 111];       % 碳纤维 v3,v4,v5
opt.resin_signal   = [0 23 222];       % 树脂   v3,v4,v5
opt.carbon_field   = 'paths_3d';       % 用补偿后路径就改 'modified_paths'
opt.resin_field    = 'paths_3d';       % resin_data 字段就叫 paths_3d
opt.verbose        = true;
if nargin>=4 && ~isempty(options)
    fn=fieldnames(options); for i=1:numel(fn), opt.(fn{i})=options.(fn{i}); end
end
if nargin<3 || isempty(output_dir), output_dir = 'print_sequence'; end
if ~exist(output_dir,'dir'), mkdir(output_dir); end

%% ---------- 加载碳纤维 ----------
fprintf('[PrintSeq] 加载碳纤维: %s\n', carbon_file);
S = load(carbon_file);
if isfield(S,'results') && isfield(S.results,'all_layers_data')
    ALD = S.results.all_layers_data;
elseif isfield(S,'all_layers_data')
    ALD = S.all_layers_data;
else
    error('在 %s 找不到 all_layers_data', carbon_file);
end
nL = numel(ALD);
fprintf('  碳纤维层数: %d   (读取字段: %s)\n', nL, opt.carbon_field);

%% ---------- 加载树脂 ----------
if ischar(resin_input) || isstring(resin_input)
    fprintf('[PrintSeq] 加载树脂: %s\n', resin_input);
    R = load(resin_input,'resin_data');
    resin_data = R.resin_data;
else
    resin_data = resin_input;
end
nR = numel(resin_data);
fprintf('  树脂层间数: %d\n', nR);

%% ---------- 时序主循环 ----------
fprintf('\n[PrintSeq] 按时序生成文件 (L1碳 -> (L1,L2)树脂 -> L2碳 -> ...):\n');
manifest = {};   seq = 0;   tic;
for k = 1:nL
    pc = ALD(k).pointCloud_data;       % 法向参考曲面: 用层 k 的 pointCloud
                                        % (树脂垫在 L_k 之上, 用 L_k 曲面)

    %% (1) 碳纤维 L_k
    if isfield(ALD(k), opt.carbon_field)
        paths = ALD(k).(opt.carbon_field);
        if iscell(paths)
            for i = 1:numel(paths)
                p = normalize_path(paths{i});
                if size(p,1)<2, continue; end
                seq = seq + 1;
                fname = sprintf('print_%04d_carbon_L%d_P%d.%s', seq, k, i, opt.format);
                write_one(fullfile(output_dir,fname), p, pc, opt.xyz_add, opt.carbon_signal);
                manifest(end+1,:) = {seq, fname, 'carbon', k, k, size(p,1), sig_str(opt.carbon_signal)}; %#ok<AGROW>
            end
        end
    end

    %% (2) 树脂层间 (k, k+1)
    if k < nL && k <= nR
        rd = resin_data(k);
        if isfield(rd, opt.resin_field) && ~isempty(rd.(opt.resin_field))
            rpaths = rd.(opt.resin_field);
            for i = 1:numel(rpaths)
                p = normalize_path(rpaths{i});
                if size(p,1)<2, continue; end
                seq = seq + 1;
                fname = sprintf('print_%04d_resin_L%d-L%d_P%d.%s', seq, k, k+1, i, opt.format);
                write_one(fullfile(output_dir,fname), p, pc, opt.xyz_add, opt.resin_signal);
                manifest(end+1,:) = {seq, fname, 'resin', k, k+1, size(p,1), sig_str(opt.resin_signal)}; %#ok<AGROW>
            end
        end
    end

    if opt.verbose && (mod(k,5)==0 || k==nL)
        fprintf('  层 %3d/%-3d   累计文件 %5d   累计耗时 %.1fs\n', k, nL, seq, toc);
    end
end

%% ---------- 写 manifest + README ----------
T = cell2table(manifest, 'VariableNames', ...
    {'seq','filename','type','layer_a','layer_b','n_points','signal_v3v4v5'});
manifest_path = fullfile(output_dir,'print_manifest.csv');
writetable(T, manifest_path);
write_readme(output_dir, seq, nL, opt);

fprintf('\n[PrintSeq] 完成! 总文件: %d (用时 %.1fs)\n', seq, toc);
fprintf('  目录:   %s\n', output_dir);
fprintf('  清单:   %s\n', manifest_path);
fprintf('  说明:   %s\n', fullfile(output_dir,'README.md'));
end

%% ================= 子函数 =================
function p = normalize_path(p)
    if isempty(p) || ~isnumeric(p), p = []; return; end
    if size(p,2)~=3 && size(p,1)==3, p = p.'; end    % 3×K -> K×3
end

function write_one(fpath, path_xyz, pc, xyz_add, signal_vec)
    n = getPathNormals(path_xyz, pc);                % K × 3 法向
    v2 = rad2deg(asin(-n(:,1)));
    v1 = rad2deg(atan2(-n(:,2), -n(:,3)));
    mi = size(path_xyz, 1);
    out = [path_xyz + xyz_add, v1, v2, repmat(signal_vec(:).', mi, 1)];
    writematrix(out, fpath);
end

function s = sig_str(v), s = sprintf('%g,%g,%g', v(1), v(2), v(3)); end

function write_readme(out_dir, n_files, nL, opt)
    fid = fopen(fullfile(out_dir,'README.md'),'w');
    fprintf(fid, '# 打印时序文件清单\n\n');
    fprintf(fid, '总文件数: **%d** (碳纤维 %d 层 + 树脂层间)\n\n', n_files, nL);
    fprintf(fid, '## 打印时序\n\n```\n');
    fprintf(fid, 'L1 碳纤维全部 -> (L1, L2) 树脂全部 -> L2 碳纤维全部\n');
    fprintf(fid, '  -> (L2, L3) 树脂全部 -> ... -> L%d 碳纤维\n```\n\n', nL);
    fprintf(fid, '## 命名规则\n\n');
    fprintf(fid, '`print_<seq>_<type>_L<k>[-L<k+1>]_P<i>.%s`\n\n', opt.format);
    fprintf(fid, '- `<seq>` 4 位全局序号 (按打印顺序, 字典序与时序一致)\n');
    fprintf(fid, '- `<type>` `carbon` 或 `resin`\n');
    fprintf(fid, '- 碳: `L<k>_P<i>`     第 k 层第 i 条路径\n');
    fprintf(fid, '- 树: `L<k>-L<k+1>_P<i>`  层间 (k,k+1) 第 i 条路径\n\n');
    fprintf(fid, '## Excel 8 列定义\n\n');
    fprintf(fid, '| 列 | 名称 | 含义 |\n|---|---|---|\n');
    fprintf(fid, '| 1-3 | x, y, z | mm, 路径点坐标 (含 xyz_add 偏移) |\n');
    fprintf(fid, '| 4   | v1 | deg, `atan2(-ny,-nz)` 末端姿态 |\n');
    fprintf(fid, '| 5   | v2 | deg, `asin(-nx)` 末端姿态 |\n');
    fprintf(fid, '| 6   | v3 | 信号: 碳=%g, 树=%g |\n', opt.carbon_signal(1), opt.resin_signal(1));
    fprintf(fid, '| 7   | v4 | 信号: 碳=%g, 树=%g |\n', opt.carbon_signal(2), opt.resin_signal(2));
    fprintf(fid, '| 8   | v5 | 信号: 碳=%g, 树=%g |\n', opt.carbon_signal(3), opt.resin_signal(3));
    fprintf(fid, '\n## 机械臂主程序读法\n\n');
    fprintf(fid, '推荐读 `print_manifest.csv` (已按 seq 排序), 逐行执行:\n\n');
    fprintf(fid, '```\nfor row in manifest.csv:\n');
    fprintf(fid, '    load row.filename  -> Nx8 矩阵\n');
    fprintf(fid, '    for each point: send to ABB as (x,y,z, v1,v2) with signal (v3,v4,v5)\n');
    fprintf(fid, '    (碳/树脂喷头切换由 v4/v5 信号触发)\n```\n\n');
    fprintf(fid, '## 注意事项\n\n');
    fprintf(fid, '1. **未做 Step2_1 拖拽补偿**. 若需要:\n');
    fprintf(fid, '   - 对碳纤维先跑 Step2_1, 得到 `modified_paths` 字段\n');
    fprintf(fid, '   - 调用本脚本时 `options.carbon_field = ''modified_paths''`\n');
    fprintf(fid, '   - 树脂粘度不同, GPR 是否适用请自行判断\n\n');
    fprintf(fid, '2. **底座下方树脂未生成**. L1 以下到底座之间的悬空填充需要底座 STL + 单独处理\n\n');
    fprintf(fid, '3. **法向参考曲面**: 碳纤维用自己层的 pointCloud, 树脂用 L_k 的 pointCloud (因为树脂垫在 L_k 之上)\n\n');
    fprintf(fid, '4. **碰撞检测**: 序列输出后, 建议用 main_collision_detection.m 检查 cone 可达性\n');
    fclose(fid);
end
