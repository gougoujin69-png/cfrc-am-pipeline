%% Run_Full_Pipeline_v8.m
%  体素空腔支撑 + 底座曲率/界面分析 + 碰撞感知序列
%  Step1(缩放/旋转/平移/倒序)已内联到本文件, 参数见下方 CFG 块, 随便改.
%  不再依赖 Step1_1/1_2/1_3/1_4 文件. Step2/Step3 仍调你的脚本.
% =====================================================================
clear; clc; close all;

%% ====================== CFG: 所有可调参数都在这 ======================
% ---- 变换 (= 原 Step1_1 缩放 + Step1_2 旋转/平移/倒序, 完全等价) ----
CFG.SCALE_MM       = 1;      % 缩放因子 (原 Step1_1 的 mm)
CFG.ROT_Z_DEG      = 0;     % 绕 Z 旋转 (原 theta_z)
CFG.ROT_X_DEG      = 0;    % 绕 X 旋转 (原 theta_x)
CFG.TRANS_X        = 0;     % 平移 X (原 TransX)
CFG.TRANS_Y        = 0;     % 平移 Y
CFG.TRANS_Z        = 0;    % 平移 Z
CFG.REVERSE_LAYERS = false;   % 倒序层 (原 Step1_2 第三项)

% ---- Step0 体素空腔支撑 ----
CFG.STEP0_MODE     = 'voxel';% 'voxel'(体素封闭空腔) | 'overhang'(旧悬垂柱)
CFG.SUP_LINEW      = 0.6;    % 树脂道宽
CFG.SUP_CONTACT    = 0.6;    % 距实体<此->实心
CFG.SUP_INFILL     = 0.20;   % 网格填充率
CFG.SUP_GRIDRES    = 0.4;    % 填充XY分辨率

% ---- Step4 底座曲率/界面 ----
CFG.BASE_KAPPA     = 0.30;   % 曲率阈值(1/mm)
CFG.BASE_SLOPE     = 35;     % 坡度阈值(deg)
CFG.BASE_IFBAND    = 1.0;    % 顶部界面带厚(mm)
CFG.BASE_LAYERH    = 0.2;    % 底座支撑平Z层高(mm)

% ---- Step5 碰撞 ----
CFG.COLLISION_RETRY  = true; % 碰撞点搜索姿态(位置不动)
CFG.COLLISION_RADIUS = 120;  % 空间预筛半径(mm)

% ---- 各步开关 ----
RUN.step0       = true;
RUN.merge       = true;
RUN.transform   = true;   % 内联 Step1
RUN.step2_1     = true;   % 拖拽补偿 (需 GPRmodel.mat)
RUN.step2_2     = true;   % 碳纤维螺旋组装
RUN.step2_3     = true;   % 树脂螺旋组装
RUN.step3_1     = true;   % 重采样
RUN.step3_2_old = false;  % 你旧的逐段 Excel (我 Step5 已替代, 默认关)
RUN.base_analysis = true; % Step4
RUN.collision_seq = true; % Step5

% ---- 文件名 ----
CARBON_FILE = 'all_layers_path_results_v3.mat';
VOXEL_FILE  = 'voxel_refined_latest.mat';
MERGED_FILE = 'all_layers_path_carbon_resin.mat';
FINAL_FILE  = 'Manufacturing_printing_path.mat';
BASE_STL    = 'Base_Reduced.stl';
BASE_SUP    = 'base_support_paths.mat';
%% =====================================================================

%% ============== 路径 ==============
this_dir  = fileparts(mfilename('fullpath'));
funcs_dir = fullfile(this_dir, 'pipeline_funcs');
if ~exist(funcs_dir,'dir'), error('找不到 pipeline_funcs/'); end
addpath(funcs_dir);
collision_dir = fullfile(this_dir, 'colisiondection');   % 按实际改
if exist(collision_dir,'dir'), addpath(collision_dir); end
fprintf('[Path] pipeline_funcs/ (+ colisiondection/ 若存在) 已加载\n');

% 现在需要的文件 (Step1_* 已内联, 不再需要)
required = {'merge_carbon_resin_data','getPathNormals','run_isolated', ...
            'run_step3_1_with_save','Step2_1_add_drag_compensation', ...
            'Step2_2_assemble_all_layers','Step2_3_resin_assemble_all_layers', ...
            'Step3_1_Robotic_Preprocess', ...
            'pipeline_step0_generate_support_voxel','generate_base_and_interface_support', ...
            'sample_stl_surface','collision_check_sequence','assemble_print_sequence'};
missing = required(~cellfun(@(n) exist(n,'file')==2, required));
if ~isempty(missing), error('pipeline_funcs/ 缺: %s', strjoin(missing,', ')); end

resin_params = struct('RESIN_LINE_WIDTH',CFG.SUP_LINEW,'CONTACT_THICK',CFG.SUP_CONTACT, ...
                      'INFILL_DENSITY',CFG.SUP_INFILL,'GRID_RES',CFG.SUP_GRIDRES);
base_params  = struct('KAPPA_MAX',CFG.BASE_KAPPA,'SLOPE_MAX',CFG.BASE_SLOPE, ...
                      'INTERFACE_BAND',CFG.BASE_IFBAND,'BASE_LAYER_H',CFG.BASE_LAYERH);

%% ============== Step 0: 树脂/支撑 ==============
if RUN.step0
    if strcmpi(CFG.STEP0_MODE,'voxel')
        fprintf('\n========== Step 0: 体素封闭空腔支撑 ==========\n');
        resin_data = pipeline_step0_generate_support_voxel(VOXEL_FILE, CARBON_FILE, resin_params);
    else
        fprintf('\n========== Step 0: 悬垂柱支撑(旧) ==========\n');
        resin_data = pipeline_step0_generate_resin(CARBON_FILE, resin_params);
    end
else
    if exist('support_paths_step0.mat','file')
        R=load('support_paths_step0.mat','support_data'); resin_data=R.support_data;
    else, R=load('resin_paths_step0.mat','resin_data'); resin_data=R.resin_data; end
end

%% ============== 合并 ==============
if RUN.merge
    fprintf('\n========== 合并 carbon + resin ==========\n');
    merge_carbon_resin_data(CARBON_FILE, resin_data, MERGED_FILE);
end

%% ============== Step 1 (内联): 缩放 + 旋转 + 平移 + 倒序 ==============
% 等价于原 Step1_1 + Step1_2. 参数全来自上面 CFG.
if RUN.transform
    fprintf('\n========== Step 1: 变换 (缩放/旋转/平移/倒序) ==========\n');
    Sin = load(MERGED_FILE, 'all_layers_data'); ald = Sin.all_layers_data;
    nLz = numel(ald);
    tz = deg2rad(CFG.ROT_Z_DEG); tx = deg2rad(CFG.ROT_X_DEG);
    Rz = [cos(tz) -sin(tz) 0; sin(tz) cos(tz) 0; 0 0 1];
    Rxx= [1 0 0; 0 cos(tx) -sin(tx); 0 sin(tx) cos(tx)];
    Rtot = Rxx*Rz;  Tr = [CFG.TRANS_X CFG.TRANS_Y CFG.TRANS_Z];  mm = CFG.SCALE_MM;
    fprintf('  缩放 %.3f | Rz %g° Rx %g° | 平移 [%g %g %g] | 倒序 %d\n', ...
            mm, CFG.ROT_Z_DEG, CFG.ROT_X_DEG, Tr(1),Tr(2),Tr(3), CFG.REVERSE_LAYERS);
    for L = 1:nLz
        p3 = ald(L).paths_3d;
        for i=1:numel(p3), if ~isempty(p3{i}), p3{i}=(p3{i}*mm)*Rtot'+Tr; end, end
        ald(L).paths_3d = p3;
        if isfield(ald,'paths_3dr')
            pr = ald(L).paths_3dr;
            for i=1:numel(pr), if ~isempty(pr{i}), pr{i}=(pr{i}*mm)*Rtot'+Tr; end, end
            ald(L).paths_3dr = pr;
        end
        if isfield(ald,'paths_2d')
            q2 = ald(L).paths_2d;
            for i=1:numel(q2), if ~isempty(q2{i}), q2{i}=q2{i}*mm; end, end
            ald(L).paths_2d = q2;
        end
        if isfield(ald,'paths_2dr')
            q2r = ald(L).paths_2dr;
            for i=1:numel(q2r), if ~isempty(q2r{i}), q2r{i}=q2r{i}*mm; end, end
            ald(L).paths_2dr = q2r;
        end
        pc = ald(L).pointCloud_data;
        PP = [pc.X(:) pc.Y(:) pc.Z(:)]*mm*Rtot' + Tr;
        pc.X=reshape(PP(:,1),size(pc.X)); pc.Y=reshape(PP(:,2),size(pc.Y)); pc.Z=reshape(PP(:,3),size(pc.Z));
        ald(L).pointCloud_data = pc;
    end
    if CFG.REVERSE_LAYERS, ald = ald(end:-1:1); end
    all_layers_data = ald; %#ok<NASGU>
    save('all_layers_path_carbon_resin2.mat','all_layers_data','-v7.3');
    copyfile('all_layers_path_carbon_resin2.mat', MERGED_FILE);   % 桥接给 Step2_1
    clear Sin ald p3 pr q2 q2r pc PP L i all_layers_data;
    fprintf('  已写 %s (+ _resin2 副本)\n', MERGED_FILE);
end

%% ============== 自动: 树脂为空则跳过 Step2_3 ==============
% Step2_3 末尾可视化引用 all_layers_data(1).resin_connected_paths; 空树脂时该字段
% 从未被创建 -> "引用了不存在的字段". 故空树脂时跳过 Step2_3, 改用空字段占位.
if RUN.step2_3
    has_resin = false;
    if exist('resin_data','var') && isfield(resin_data,'paths_3d')
        for k=1:numel(resin_data)
            if ~isempty(resin_data(k).paths_3d), has_resin=true; break; end
        end
    end
    if ~has_resin
        RUN.step2_3 = false;
        fprintf('[自动] 树脂为空 -> 跳过 Step2_3(其可视化块会因缺字段报错), 用空字段占位\n');
    end
end

%% ============== Step 2: 螺旋组装 (你的脚本) ==============
if RUN.step2_1, fprintf('\n== Step 2.1 拖拽补偿 ==\n'); run_isolated('Step2_1_add_drag_compensation'); end
if RUN.step2_2, fprintf('\n== Step 2.2 碳纤维组装 ==\n'); run_isolated('Step2_2_assemble_all_layers'); end
if RUN.step2_3
    fprintf('\n== Step 2.3 树脂组装 ==\n'); run_isolated('Step2_3_resin_assemble_all_layers');
else
    fprintf('\n[Skip Step2_3] paths_3dr -> resin_connected_paths\n');
    if exist('Modified_printing_path.mat','file')
        S=load('Modified_printing_path.mat','all_layers_data'); a=S.all_layers_data;
        for k=1:numel(a)
            if isfield(a,'paths_3dr'), a(k).resin_connected_paths=a(k).paths_3dr;
            else, a(k).resin_connected_paths={}; end
        end
        all_layers_data=a; save('Modified_printing_path.mat','all_layers_data','-v7.3'); %#ok<NASGU>
        clear all_layers_data a S;
    end
end

%% ============== 保证 resin_connected_paths 字段存在 ==============
% 树脂为空时 Step2_3 每层都 continue, 不会创建该字段; 而 Step3_1 的可视化段
% 引用 all_layers_data(10).resin_connected_paths -> 否则报"非存在字段",
% 导致 Manufacturing_printing_path.mat 没保存、Step4/5 跑不到.
if exist('Modified_printing_path.mat','file')
    Sg=load('Modified_printing_path.mat','all_layers_data'); ag=Sg.all_layers_data;
    if ~isfield(ag,'resin_connected_paths')
        for k=1:numel(ag)
            if isfield(ag,'paths_3dr'), ag(k).resin_connected_paths=ag(k).paths_3dr;
            else, ag(k).resin_connected_paths={}; end
        end
        all_layers_data=ag; save('Modified_printing_path.mat','all_layers_data','-v7.3'); %#ok<NASGU>
        clear all_layers_data;
        fprintf('[Fix] Modified_printing_path.mat 补上空 resin_connected_paths 字段\n');
    end
    clear Sg ag;
end

%% ============== 保险: 确保 resin_connected_paths 字段存在 ==============
% 空树脂时 Step2_3 每层 continue 不建该字段, 而 Step3_1 末尾可视化块引用它 -> 报错.
if exist('Modified_printing_path.mat','file')
    Sg=load('Modified_printing_path.mat','all_layers_data'); ag=Sg.all_layers_data;
    if ~isfield(ag,'resin_connected_paths')
        for k=1:numel(ag), ag(k).resin_connected_paths={}; end
        all_layers_data=ag; save('Modified_printing_path.mat','all_layers_data','-v7.3'); %#ok<NASGU>
        fprintf('[修补] 补上空 resin_connected_paths 字段(空树脂, 防 Step3.1 可视化报错)\n');
    end
    clear Sg ag all_layers_data;
end

%% ============== Step 3.1: 重采样 ==============
if RUN.step3_1, fprintf('\n== Step 3.1 重采样 ==\n'); run_step3_1_with_save(); end

%% ============== Step 3.2 (旧式逐段 Excel, 可选) ==============
if RUN.step3_2_old
    fprintf('\n== Step 3.2 旧式 Excel ==\n');
    run_isolated('Step3_2_Robotic_carbon_path');
    run_isolated('Step3_2_Robotic_Resin_path');
    excel_dir = fullfile(this_dir,'excel_output'); if ~exist(excel_dir,'dir'), mkdir(excel_dir); end
    for pat = {'carbon_path_*.xlsx','resin_path_*.xlsx'}
        fs=dir(fullfile(this_dir,pat{1}));
        for i=1:numel(fs), movefile(fullfile(this_dir,fs(i).name),fullfile(excel_dir,fs(i).name)); end
    end
end

%% ============== Step 4: 底座曲率/界面分析 ==============
if RUN.base_analysis
    fprintf('\n========== Step 4: 底座曲率/界面分析 ==========\n');
    if ~exist(FINAL_FILE,'file'), error('缺 %s (需先跑 Step3.1)', FINAL_FILE); end
    P_id = struct('SCALE_FACTOR',1,'ROT_Z_DEG',0,'ROT_X_DEG',0,'TRANS',[0 0 0]);
    generate_base_and_interface_support(FINAL_FILE, BASE_STL, P_id, base_params);
end

%% ============== Step 5: 碰撞感知序列 ==============
if RUN.collision_seq
    fprintf('\n========== Step 5: 组装序列 + 碰撞校验 ==========\n');
    if exist('get_default_config','file')~=2
        warning('找不到 get_default_config (colisiondection 未在路径?), 跳过 Step5');
    else
        cfg = get_default_config();
        bsup=''; if exist(BASE_SUP,'file'), bsup=BASE_SUP; end
        bstl=''; if exist(BASE_STL,'file'), bstl=BASE_STL; end
        assemble_print_sequence(FINAL_FILE, bsup, bstl, cfg, ...
            struct('retry',CFG.COLLISION_RETRY,'radius_mm',CFG.COLLISION_RADIUS));
    end
end

%% ============== 总结 ==============
fprintf('\n========================================\n');
fprintf(' 完成. 关键输出:\n');
fprintf('   Manufacturing_printing_path.mat   变换+组装+重采样后\n');
fprintf('   Base_Reduced.stl                  降低/挖空底座 (Step4)\n');
fprintf('   base_support_paths.mat            底座界面/高曲率支撑 (Step4)\n');
fprintf('   print_sequence_collision/         有序+碰撞校验 Excel + 报告 (Step5)\n');
fprintf('========================================\n');