function visualize_pipeline_state(opt)
%% VISUALIZE_PIPELINE_STATE  诊断 + 可视化整条管线的当前状态
% =====================================================================
% 作用:
%   1) 扫工作目录里所有产物文件, 打印"跑到了哪一步" + 每个文件的关键信息.
%   2) 自动挑【最靠后的可用结果】把 碳纤维 / 树脂 / 底座支撑 / 降低底座STL
%      按统一坐标画出来, 看它们的相对位置和样式.
%
% 用法:
%   visualize_pipeline_state                 % 默认: 全部层, 自动选源
%   visualize_pipeline_state(struct('layers',1:20))      % 只看 1-20 层
%   visualize_pipeline_state(struct('source','Manufacturing_printing_path.mat'))
%   visualize_pipeline_state(struct('show_base',false,'show_pointcloud',true))
%
% opt 字段(都可选):
%   .layers          要画的层索引向量, []=全部(默认)
%   .max_layers      层太多时抽稀到这么多条画(默认 80)
%   .source          强制指定结构源文件名, ''=自动(默认)
%   .show_base       是否叠加 Base_Reduced.stl + 底座支撑(默认 true)
%   .show_pointcloud 是否画曲面点云底图(默认 false, 开了更慢)
%   .workdir         工作目录(默认 pwd)
% =====================================================================
if nargin<1, opt=struct(); end
D = struct('layers',[],'max_layers',80,'source','','show_base',true, ...
           'show_pointcloud',false,'workdir',pwd);
fn=fieldnames(D); for i=1:numel(fn), if ~isfield(opt,fn{i}), opt.(fn{i})=D.(fn{i}); end, end
wd = opt.workdir;

%% ============== 1) 进度扫描 ==============
fprintf('\n================ 管线进度扫描 (%s) ================\n', wd);
files = {
 'all_layers_path_results_v3.mat'      ,'碳纤维输入 (path-gen 产物)'        ,'原始'
 'support_paths_step0.mat'             ,'Step0 体素空腔支撑'                ,'原始'
 'resin_paths_step0.mat'               ,'Step0 悬垂柱支撑(旧模式)'          ,'原始'
 'all_layers_path_carbon_resin.mat'    ,'合并 carbon+resin / Step1后被覆盖' ,'?'
 'all_layers_path_carbon_resin_raw.mat','合并原始副本'                      ,'原始'
 'all_layers_path_carbon_resin2.mat'   ,'Step1 变换+倒序产物'               ,'打印'
 'Modified_printing_path.mat'          ,'Step2 组装产物'                    ,'打印'
 'Manufacturing_printing_path.mat'     ,'Step3.1 重采样产物 (最终)'         ,'打印'
 'Base_Reduced.stl'                    ,'Step4 降低/挖空底座'               ,'打印'
 'base_support_paths.mat'              ,'Step4 底座界面/高曲率支撑'         ,'打印'
};
stage = 0; stage_names = {};
for r = 1:size(files,1)
    f = fullfile(wd, files{r,1});
    if exist(f,'file')==2
        info = probe_file(f);
        fprintf(' [✓] %-38s %s\n', files{r,1}, files{r,3});
        fprintf('      %s | %s\n', files{r,2}, info);
        stage = r; stage_names{end+1} = files{r,1}; %#ok<AGROW>
    else
        fprintf(' [ ] %-38s (未生成)\n', files{r,1});
    end
end
% Step5 输出目录
seqdir = fullfile(wd, 'print_sequence_collision');
if exist(seqdir,'dir')
    xls = dir(fullfile(seqdir,'print_*.xlsx'));
    rep = fullfile(seqdir,'collision_report.txt');
    extra = '';
    if exist(rep,'file')
        t = fileread(rep); m = regexp(t,'碰撞\s*(\d+)','tokens','once');
        if ~isempty(m), extra = sprintf(', 碰撞报告: %s 个碰撞点', m{1}); end
    end
    fprintf(' [✓] %-38s 打印坐标系\n', 'print_sequence_collision/');
    fprintf('      Step5 有序+碰撞校验 Excel: %d 个%s\n', numel(xls), extra);
end
fprintf('--------------------------------------------------------\n');
if exist(fullfile(wd,'Manufacturing_printing_path.mat'),'file')==2
    fprintf(' 结论: 已跑到 Step3.1(最终路径已生成).');
    if exist(seqdir,'dir'), fprintf(' Step5 碰撞校验也已完成.'); end
    fprintf('\n');
elseif exist(fullfile(wd,'Modified_printing_path.mat'),'file')==2
    fprintf(' 结论: 跑到了 Step2(组装), 还没到 Step3.1 重采样.\n');
elseif exist(fullfile(wd,'all_layers_path_carbon_resin2.mat'),'file')==2
    fprintf(' 结论: 跑到了 Step1(变换), 还没到 Step2.\n');
elseif exist(fullfile(wd,'all_layers_path_carbon_resin.mat'),'file')==2
    fprintf(' 结论: 跑到了"合并", 还没(成功)跑 Step1.\n');
elseif exist(fullfile(wd,'support_paths_step0.mat'),'file')==2
    fprintf(' 结论: 只跑了 Step0(支撑生成).\n');
else
    fprintf(' 结论: 没找到任何中间产物.\n');
end
fprintf('========================================================\n\n');

%% ============== 2) 选结构源 ==============
% 优先最靠后, 并记录坐标系(决定能否叠加底座)
candidates = {
 'Manufacturing_printing_path.mat'   ,'connected_paths'      ,'resin_connected_paths' ,'打印'
 'Modified_printing_path.mat'        ,'connected_paths'      ,'resin_connected_paths' ,'打印'
 'all_layers_path_carbon_resin2.mat' ,'paths_3d'             ,'paths_3dr'             ,'打印'
 'all_layers_path_carbon_resin.mat'  ,'paths_3d'             ,'paths_3dr'             ,'?'
 'all_layers_path_results_v3.mat'    ,'paths_3d'             ,''                      ,'原始'
};
src=''; cF=''; rF=''; frame=''; 
if ~isempty(opt.source)
    k = find(strcmp(candidates(:,1), opt.source),1);
    if isempty(k), error('未知 source: %s', opt.source); end
    src=candidates{k,1}; cF=candidates{k,2}; rF=candidates{k,3}; frame=candidates{k,4};
else
    for k = 1:size(candidates,1)
        if exist(fullfile(wd,candidates{k,1}),'file')==2
            src=candidates{k,1}; cF=candidates{k,2}; rF=candidates{k,3}; frame=candidates{k,4};
            break;
        end
    end
end
if isempty(src), fprintf('没有可画的结构源文件.\n'); return; end
fprintf('[可视化] 结构源: %s  (碳=%s, 树脂=%s, 坐标系=%s)\n', src, cF, rF, frame);

S = load(fullfile(wd,src));
if isfield(S,'results') && isfield(S.results,'all_layers_data'), ALD = S.results.all_layers_data;
elseif isfield(S,'all_layers_data'), ALD = S.all_layers_data;
else, error('%s 里没有 all_layers_data', src); end
nL = numel(ALD);

% 树脂源: 若结构源没有树脂字段(碳纤维输入), 退回 support_paths_step0.mat
resin_src = ''; RSD = [];
if isempty(rF) || ~isfield(ALD, rF)
    if exist(fullfile(wd,'support_paths_step0.mat'),'file')==2
        Q = load(fullfile(wd,'support_paths_step0.mat'),'support_data');
        if isfield(Q,'support_data'), RSD = Q.support_data; resin_src='support_paths_step0.mat'; end
    end
end

% 选层
if isempty(opt.layers), layers = 1:nL; else, layers = opt.layers(opt.layers>=1 & opt.layers<=nL); end
if numel(layers) > opt.max_layers
    layers = layers(round(linspace(1, numel(layers), opt.max_layers)));
    fprintf('[可视化] 层数多, 抽稀到 %d 层画\n', numel(layers));
end

% 汇总线段
carbon = gather_field(ALD, cF, layers);
if ~isempty(rF) && isfield(ALD, rF)
    resin = gather_field(ALD, rF, layers);
elseif ~isempty(RSD)
    resin = gather_field(RSD, 'paths_3d', layers(layers<=numel(RSD)));
    fprintf('[可视化] 树脂取自 %s\n', resin_src);
else
    resin = zeros(0,3);
end

%% ============== 3) 画图 ==============
printed = strcmp(frame,'打印');
show_base = opt.show_base && printed;
if opt.show_base && ~printed
    fprintf('[可视化] 源是%s坐标系, 与打印坐标系的底座不同框, 不叠加底座.\n', frame);
end

figure('Color','w','Name',sprintf('管线状态: %s', src),'Position',[60 60 1280 560]);

% --- 左: 3D 全貌 ---
ax1 = subplot(1,2,1); hold(ax1,'on');
hleg = []; lname = {};
if opt.show_pointcloud
    for L = layers(1:max(1,round(numel(layers)/20)):end)
        if isfield(ALD(L),'pointCloud_data') && ~isempty(ALD(L).pointCloud_data)
            pc = ALD(L).pointCloud_data;
            if isfield(pc,'Z'), surf(ax1, pc.X, pc.Y, pc.Z, 'EdgeColor','none','FaceColor',[.85 .87 .9],'FaceAlpha',.15); end
        end
    end
end
if ~isempty(carbon)
    h=plot3(ax1, carbon(:,1),carbon(:,2),carbon(:,3),'-','Color',[.85 .12 .12],'LineWidth',.7);
    hleg(end+1)=h; lname{end+1}=sprintf('碳纤维 (%s)', cF);
end
if ~isempty(resin)
    h=plot3(ax1, resin(:,1),resin(:,2),resin(:,3),'-','Color',[.10 .35 .95],'LineWidth',.7);
    hleg(end+1)=h; lname{end+1}='树脂支撑';
end
if show_base
    bstl = fullfile(wd,'Base_Reduced.stl');
    if exist(bstl,'file')==2
        try
            TR = stlread(bstl);
            h=patch(ax1,'Faces',TR.ConnectivityList,'Vertices',TR.Points, ...
                  'FaceColor',[.55 .55 .58],'EdgeColor','none','FaceAlpha',.30);
            hleg(end+1)=h; lname{end+1}='降低底座 STL';
        catch ME, fprintf('  读 STL 失败: %s\n', ME.message); end
    end
    bsup = fullfile(wd,'base_support_paths.mat');
    if exist(bsup,'file')==2
        B = load(bsup,'base_support_data');
        if isfield(B,'base_support_data')
            [bs, bg] = gather_base_support(B.base_support_data);
            if ~isempty(bs), h=plot3(ax1,bs(:,1),bs(:,2),bs(:,3),'-','Color',[.10 .65 .20],'LineWidth',1.0); hleg(end+1)=h; lname{end+1}='底座支撑(实心)'; end
            if ~isempty(bg), h=plot3(ax1,bg(:,1),bg(:,2),bg(:,3),'-','Color',[.45 .85 .45],'LineWidth',.6); hleg(end+1)=h; lname{end+1}='底座支撑(网格)'; end
        end
    end
end
axis(ax1,'equal'); grid(ax1,'on'); view(ax1,40,22);
xlabel(ax1,'X (mm)'); ylabel(ax1,'Y (mm)'); zlabel(ax1,'Z (mm)');
title(ax1, sprintf('3D 全貌 (%d 层)', numel(layers)));
if ~isempty(hleg), legend(ax1, hleg, lname, 'Location','northeast'); end

% --- 右: 单层俯视(看样式/间距) ---
ax2 = subplot(1,2,2); hold(ax2,'on');
midL = layers(max(1,round(numel(layers)/2)));
cseg = gather_field(ALD, cF, midL);
if ~isempty(cseg), plot(ax2, cseg(:,1),cseg(:,2),'-','Color',[.85 .12 .12],'LineWidth',1.0); end
if ~isempty(rF) && isfield(ALD,rF)
    rseg = gather_field(ALD, rF, midL);
elseif ~isempty(RSD) && midL<=numel(RSD)
    rseg = gather_field(RSD,'paths_3d',midL);
else, rseg = zeros(0,3); end
if ~isempty(rseg), plot(ax2, rseg(:,1),rseg(:,2),'-','Color',[.10 .35 .95],'LineWidth',1.0); end
axis(ax2,'equal'); grid(ax2,'on');
xlabel(ax2,'X (mm)'); ylabel(ax2,'Y (mm)');
title(ax2, sprintf('第 %d 层俯视 (红=碳纤维, 蓝=树脂)', midL));

% --- 控制台小结 ---
nc = count_segs(ALD,cF,layers);
if ~isempty(rF)&&isfield(ALD,rF), nr=count_segs(ALD,rF,layers); elseif ~isempty(RSD), nr=count_segs(RSD,'paths_3d',layers(layers<=numel(RSD))); else, nr=0; end
fprintf('[可视化] 画了 碳纤维 %d 段, 树脂 %d 段.', nc, nr);
if nr==0, fprintf(' (树脂为空: 此件无内部封闭空腔, 见前面说明)'); end
fprintf('\n');
end

%% ================= 子函数 =================
function info = probe_file(f)
% 不全量加载, 用 whos + matfile 探一眼
    info = '';
    [~,~,ext] = fileparts(f);
    if strcmpi(ext,'.stl')
        try TR=stlread(f); info=sprintf('三角面 %d', size(TR.ConnectivityList,1)); catch, info='STL'; end
        return;
    end
    try W = whos('-file', f); catch, info='(无法读变量表)'; return; end
    vn = {W.name};
    if any(strcmp(vn,'all_layers_data'))
        ia = find(strcmp(vn,'all_layers_data'),1); nL = max(W(ia).size);
        present = '';
        try
            m = matfile(f); a1 = m.all_layers_data(1,1); fns = fieldnames(a1);
            pf = {'paths_3d','paths_3dr','connected_paths','resin_connected_paths','modified_paths'};
            for i=1:numel(pf)
                if ismember(pf{i},fns)
                    tag = pf{i}; if isempty(a1.(pf{i})), tag=[tag '(空)']; end
                    present = [present tag '  ']; %#ok<AGROW>
                end
            end
        catch, present='(字段需完整加载才能看)'; end
        info = sprintf('%d 层 | 字段: %s', nL, present);
    elseif any(strcmp(vn,'results'))
        info = '含 results.all_layers_data (碳纤维源)';
    elseif any(strcmp(vn,'support_data'))
        try m=matfile(f); sd=m.support_data; n=numel(sd);
            nseg=0; for k=1:n, nseg=nseg+numel(sd(k).paths_3d); end
            info=sprintf('support_data %d 层间, 共 %d 段', n, nseg);
        catch, info='含 support_data'; end
    elseif any(strcmp(vn,'base_support_data'))
        info = '含 base_support_data';
    else
        info = sprintf('变量: %s', strjoin(vn,', '));
    end
end

function out = gather_field(ALD, field, layers)
% 把指定层的某 cell 字段所有段拼成 [..;NaN;..] 便于一次 plot3
    acc = {};
    if isempty(field) || ~isfield(ALD, field), out=zeros(0,3); return; end
    for L = layers(:)'
        if L<1 || L>numel(ALD), continue; end
        c = ALD(L).(field);
        if ~iscell(c), continue; end
        for i = 1:numel(c)
            seg = c{i};
            if ~isnumeric(seg) || size(seg,1)<1, continue; end
            if size(seg,2)~=3 && size(seg,1)==3, seg=seg.'; end
            if size(seg,2)>=3, acc{end+1}=[seg(:,1:3); nan(1,3)]; end %#ok<AGROW>
        end
    end
    if isempty(acc), out=zeros(0,3); else, out=cat(1,acc{:}); end
end

function [solid, grid] = gather_base_support(BSD)
    as = {}; ag = {};
    for k = 1:numel(BSD)
        sd = BSD(k);
        if ~isfield(sd,'paths_3d'), continue; end
        for i = 1:numel(sd.paths_3d)
            c = sd.paths_3d{i};
            if size(c,1)<1, continue; end
            issolid = isfield(sd,'is_solid') && numel(sd.is_solid)>=i && ~isempty(sd.is_solid{i}) && sd.is_solid{i};
            if issolid, as{end+1}=[c; nan(1,3)]; else, ag{end+1}=[c; nan(1,3)]; end %#ok<AGROW>
        end
    end
    if isempty(as), solid=zeros(0,3); else, solid=cat(1,as{:}); end
    if isempty(ag), grid=zeros(0,3);  else, grid=cat(1,ag{:});  end
end

function n = count_segs(ALD, field, layers)
    n = 0;
    if isempty(field) || ~isfield(ALD, field), return; end
    for L = layers(:)'
        if L<1 || L>numel(ALD), continue; end
        c = ALD(L).(field); if iscell(c), n = n + numel(c); end
    end
end
