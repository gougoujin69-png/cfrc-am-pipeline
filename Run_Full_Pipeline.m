%% Run_Full_Pipeline.m  v7  (模型驱动支撑 + 贴合底座)
% =====================================================================
% 与 v6 的差异(本次需求):
%   1) 支撑生成从"逐层悬空监测"换成"模型内部空腔分析":
%        pipeline_step0_generate_resin  ->  pipeline_step0_generate_support
%      只在【结构内部封闭空腔】生成支撑(20% 网格 + 临近碳纤维 0.6mm 实心).
%   2) 底座从"平面顶"换成"贴合结构底面的棱柱"(新增 generate_base_stl):
%        下表面 = Z0 投影外框, 上表面 = 结构下包络面.
%      它负责【底部间隙】与【开放悬垂】, 取代旧 Step1.4 的 Solid_Model.stl.
%   3) 支撑是直线/网格(非同心 loop) -> 建议 RUN.step2_3 = false(直接拷贝).
%   4) 末尾可选: generate_print_sequence_merged 产出"碳/支撑逐层穿插"的时序清单.
%
% 责任划分(关键): 内部封闭空腔 = 支撑(树脂); 底部/开放悬垂 = 底座 STL. 互不重叠.
%
% 打印时序保证(逐层穿插, 从下到上):
%   L1 碳(同层一次打完) -> L1 支撑 -> L2 碳 -> L2 支撑 -> ... -> Ln 碳
%   * 被支撑的碳纤维(空腔顶)在更高层 => 支撑总是先于它完成.
%   * 支撑在同层碳纤维之后 => 不挡碳纤维; 支撑随时可暂停插碳纤维.
%
% 目录结构(同 v6): pipeline_funcs/ 放全部 .m, 工作目录放主控 + 输入 mat.
%   新增放进 pipeline_funcs/ 的文件:
%     pipeline_step0_generate_support.m
%     generate_base_stl.m
%     generate_print_sequence_merged.m
% =====================================================================

clear; clc; close all;

%% ============== 路径设置 ==============
this_dir  = fileparts(mfilename('fullpath'));
funcs_dir = fullfile(this_dir, 'pipeline_funcs');
if ~exist(funcs_dir, 'dir')
    error('找不到 pipeline_funcs/ 子目录. 把全部 .m 放进 pipeline_funcs/');
end
addpath(funcs_dir);
fprintf('[Path] 已加 pipeline_funcs/ 到 MATLAB 路径\n');

required = {'pipeline_step0_generate_support','merge_carbon_resin_data', ...
            'generate_base_stl','run_isolated','run_step3_1_with_save', ...
            'getPathNormals', ...
            'Step1_1_Data_Preprocessing','Step2_1_add_drag_compensation', ...
            'Step3_2_Robotic_carbon_path'};
missing = required(~cellfun(@(n) exist(n,'file')==2, required));
if ~isempty(missing)
    error('pipeline_funcs/ 里缺这些文件: %s', strjoin(missing, ', '));
end

%% ============== 配置 ==============
CARBON_FILE = 'all_layers_path_results_v3.mat';

% --- 前置 ---
RUN.step0     = true;     % 模型内部空腔支撑
RUN.merge     = true;
RUN.viz_resin = true;
RUN.base_stl  = true;     % 贴合结构底面的底座 STL (取代旧 Step1.4)

% --- Step 1/2/3 ---
RUN.step1_1 = false;
RUN.step1_2 = false;
RUN.step1_3 = false;
RUN.step1_4 = false;      % 关掉旧的平面顶 STL; 用 generate_base_stl 代替
RUN.step2_1 = false;
RUN.step2_2 = false;
RUN.step2_3 = false;      % 支撑是直线/网格(非同心 loop) -> 直接拷贝 paths_3dr
RUN.step3_1 = false;
RUN.step3_2_carbon = false;
RUN.step3_2_resin  = false;
RUN.timed_sequence = false; % 末尾产出碳/支撑逐层穿插的时序清单

% --- 支撑参数(模型内部空腔) ---
support_params = struct( ...
    'INFILL_DENSITY', 0.20, ...   % 网格填充率
    'CONTACT_THICK',  0.6,  ...   % 临近碳纤维 0.6mm 实心
    'RESIN_LINE_WIDTH', 0.6, ...
    'MIN_CAVITY_AREA', 4.0, ...   % 小于此面积的孔当道间缝隙填实
    'FILL_BOTTOM_GAP', false);    % 底部交给底座 STL; 想用平面顶底座则设 true

% --- 底座变换(务必与你 Step1_2 的 缩放/旋转/平移 一致!) ---
% dz = TRANS(3) = 把结构抬高多少(决定底座高度).
BASE_TF  = struct('SCALE_FACTOR',1, 'ROT_Z_DEG',0, 'ROT_X_DEG',0, ...
                  'TRANS',[0 0 5], 'REVERSE_LAYERS',false);
BASE_OPT = struct('GRID_RES',0.5, 'Z_BASE',0, 'FILL_THROUGH_HOLES',true, ...
                  'USE_LAYER1_ONLY',false);

%% ============== Step 0: 模型内部空腔支撑 ==============
if RUN.step0
    fprintf('\n========== Step 0: 模型内部空腔支撑 ==========\n');
    support_data = pipeline_step0_generate_support(CARBON_FILE, support_params);
else
    fprintf('\n[Skip Step0] 加载 support_paths_step0.mat\n');
    R = load('support_paths_step0.mat','support_data');
    support_data = R.support_data;
end

%% ============== 合并 carbon + support ==============
if RUN.merge
    fprintf('\n========== 合并 carbon + support ==========\n');
    merge_carbon_resin_data(CARBON_FILE, support_data, 'all_layers_path_carbon_resin.mat');
end

%% ============== 支撑分布抽查 ==============
if RUN.viz_resin
    fprintf('\n========== 支撑分布抽查 ==========\n');
    try
        kk = find(arrayfun(@(s) ~isempty(s.paths_3d), support_data), 1, 'first');
        if ~isempty(kk), show_resin_paths(CARBON_FILE, support_data, kk); end
    catch ME
        fprintf('  可视化失败 (%s), 继续\n', ME.message);
    end
end

%% ============== 底座 STL (贴合结构底面的棱柱) ==============
if RUN.base_stl
    fprintf('\n========== 底座 STL 生成 ==========\n');
    % 在【原始】碳纤维文件上调用, 传入与 Step1 相同的变换 -> 底座对齐变换后的结构.
    generate_base_stl(CARBON_FILE, fullfile(this_dir,'Base_Model.stl'), BASE_TF, BASE_OPT);
end

%% ============== Step 1: 数据预处理 (workspace 接力) ==============
if RUN.step1_1
    fprintf('\n========== Step 1.1: 缩放 ==========\n');
    run('Step1_1_Data_Preprocessing');
end
if RUN.step1_2
    fprintf('\n========== Step 1.2: 旋转 + 平移 + 倒序 ==========\n');
    run('Step1_2_Data_Preprocessing');   % ⚠ 这里的变换要和上面 BASE_TF 一致
end
if RUN.step1_3
    fprintf('\n========== Step 1.3: 检查可视化 ==========\n');
    run('Step1_3_Data_Preprocessing');
end
if RUN.step1_4
    fprintf('\n========== Step 1.4: (旧)平面顶 STL ==========\n');
    run('Step1_4_Data_Preprocessing');
end

% 桥接: Step1 输出 _resin2.mat, Step2_1 读 _resin.mat
if (RUN.step1_1||RUN.step1_2) && exist('all_layers_path_carbon_resin2.mat','file')
    copyfile('all_layers_path_carbon_resin2.mat', 'all_layers_path_carbon_resin.mat');
    fprintf('\n[Bridge] all_layers_path_carbon_resin2.mat -> .mat\n');
end

%% ============== Step 2: 路径优化 ==============
if RUN.step2_1
    fprintf('\n========== Step 2.1: 拖拽补偿 ==========\n');
    run_isolated('Step2_1_add_drag_compensation');
end
if RUN.step2_2
    fprintf('\n========== Step 2.2: 碳纤维螺旋组装 ==========\n');
    run_isolated('Step2_2_assemble_all_layers');
end
if RUN.step2_3
    fprintf('\n========== Step 2.3: 树脂螺旋组装 ==========\n');
    run_isolated('Step2_3_resin_assemble_all_layers');
else
    fprintf('\n[Skip Step2_3] 直接 paths_3dr -> resin_connected_paths (支撑是直线/网格)\n');
    if exist('Modified_printing_path.mat','file')
        S = load('Modified_printing_path.mat','all_layers_data');
        ald = S.all_layers_data;
        for k = 1:numel(ald)
            ald(k).resin_connected_paths = ald(k).paths_3dr;
        end
        all_layers_data = ald; %#ok<NASGU>
        save('Modified_printing_path.mat','all_layers_data','-v7.3');
        clear all_layers_data ald S;
    end
end

%% ============== Step 3: 机械臂指令 ==============
if RUN.step3_1
    fprintf('\n========== Step 3.1: 重采样 (+ 补 save) ==========\n');
    run_step3_1_with_save();
end
if RUN.step3_2_carbon
    fprintf('\n========== Step 3.2 碳纤维: Excel ==========\n');
    run_isolated('Step3_2_Robotic_carbon_path');
end
if RUN.step3_2_resin
    fprintf('\n========== Step 3.2 支撑: Excel ==========\n');
    run_isolated('Step3_2_Robotic_Resin_path');
end

%% ============== 整理 Excel 到子目录 ==============
if RUN.step3_2_carbon || RUN.step3_2_resin
    excel_dir = fullfile(this_dir, 'excel_output');
    if ~exist(excel_dir,'dir'), mkdir(excel_dir); end
    moved = 0;
    for pat = {'carbon_path_*.xlsx', 'resin_path_*.xlsx'}
        fs = dir(fullfile(this_dir, pat{1}));
        for i = 1:numel(fs)
            movefile(fullfile(this_dir, fs(i).name), fullfile(excel_dir, fs(i).name));
            moved = moved + 1;
        end
    end
    fprintf('\n[Cleanup] %d 个 xlsx 文件已移到 %s\n', moved, excel_dir);
end

%% ============== (可选)碳/支撑逐层穿插时序清单 ==============
if RUN.timed_sequence
    fprintf('\n========== 打印时序 (碳/支撑逐层穿插) ==========\n');
    % 用最终处理后的合并数据(含 connected_paths + resin_connected_paths).
    % 若 Step3.1 未带 resin_connected_paths, 改成 'Modified_printing_path.mat'.
    seq_src = 'Manufacturing_printing_path.mat';
    try
        generate_print_sequence_merged(seq_src, fullfile(this_dir,'print_sequence'), ...
            struct('format','xlsx'));
    catch ME
        fprintf('  时序生成失败 (%s). 尝试 Modified_printing_path.mat ...\n', ME.message);
        try
            generate_print_sequence_merged('Modified_printing_path.mat', ...
                fullfile(this_dir,'print_sequence'), struct('format','xlsx'));
        catch ME2
            fprintf('  仍失败 (%s)\n', ME2.message);
        end
    end
end

%% ============== 总结 ==============
fprintf('\n========================================\n');
fprintf(' 流水线完成. 关键输出:\n');
fprintf('   - support_paths_step0.mat              Step0 内部空腔支撑\n');
fprintf('   - all_layers_path_carbon_resin.mat     合并后\n');
fprintf('   - Base_Model.stl                       贴合结构底面的底座\n');
fprintf('   - Modified_printing_path.mat           Step2 产物\n');
fprintf('   - Manufacturing_printing_path.mat      Step3.1 产物\n');
fprintf('   - excel_output/                        逐层机械臂 Excel\n');
fprintf('   - print_sequence/print_manifest.csv    碳/支撑时序清单(按 seq 打印)\n');
fprintf('========================================\n');
