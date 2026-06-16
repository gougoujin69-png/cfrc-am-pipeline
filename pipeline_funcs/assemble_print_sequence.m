function [seq, R] = assemble_print_sequence(carbon_file, base_support_file, reduced_base_stl, config, opt)
%% ASSEMBLE_PRINT_SEQUENCE  v1  (第4点收尾: 组装有序序列 + 碰撞校验 + 输出)
% =====================================================================
% 打印顺序(自底向上, 碳/支撑逐层穿插):
%   降低底座 base_support(按 z_level 升序) -> 碳纤维 L1 -> 空腔支撑 L1 ->
%   碳纤维 L2 -> 空腔支撑 L2 -> ... -> 碳纤维 Ln
% 然后:
%   - 采样 reduced_base STL -> 静态障碍点云
%   - 调 collision_check_sequence 逐点校验(降低底座=静态障碍, 前序点=动态障碍)
%   - 写有序 8 列 Excel(姿态用重试修正后的) + 清单 + 碰撞报告
%
% 输入:
%   carbon_file       : 处理后的合并文件(每层含碳纤维 + 空腔支撑)
%                       碳纤维字段默认 connected_paths(回退 paths_3d)
%                       空腔支撑字段默认 resin_connected_paths(回退 paths_3dr)
%   base_support_file : generate_base_and_interface_support 产出的 base_support_paths.mat
%                       (含 base_support_data); 没有就传 '' 跳过
%   reduced_base_stl  : 'Base_Reduced.stl'; 没有就传 '' (则只查前序点自碰)
%   config            : get_default_config()
%   opt: .carbon_field .resin_field .base_spacing(默认1.0) .output_dir
%        .run_check(默认true) .retry(默认false) .write_excel(默认true) .format('xlsx'/'csv')
%        .carbon_signal([0 14 111]) .resin_signal([0 23 222]) .xyz_add([0 0 0])
%
% 依赖: getPathNormals.m, sample_stl_surface.m, collision_check_sequence.m 在路径上.
% =====================================================================
if nargin<4 || isempty(config), config = get_default_config(); end
if nargin<5, opt = struct(); end
D = struct('carbon_field','connected_paths','resin_field','resin_connected_paths', ...
           'base_spacing',1.0,'output_dir','print_sequence_collision', ...
           'run_check',true,'retry',false,'write_excel',true,'format','xlsx', ...
           'carbon_signal',[0 14 111],'resin_signal',[0 23 222],'xyz_add',[0 0 0], ...
           'radius_mm',120);   % B6: spatial pre-filter radius, forwarded to collision_check_sequence
fn=fieldnames(D); for i=1:numel(fn), if ~isfield(opt,fn{i}), opt.(fn{i})=D.(fn{i}); end, end
if ~exist(opt.output_dir,'dir'), mkdir(opt.output_dir); end

%% ---- 载入 ----
C = load(carbon_file);
if isfield(C,'results')&&isfield(C.results,'all_layers_data'), ALD=C.results.all_layers_data;
elseif isfield(C,'all_layers_data'), ALD=C.all_layers_data;
else, error('在 %s 找不到 all_layers_data', carbon_file); end
nL = numel(ALD);
cF = pick_field(ALD, opt.carbon_field, 'paths_3d');
rF = pick_field(ALD, opt.resin_field,  'paths_3dr');
fprintf('[assemble] 层数 %d, 碳纤维字段=%s, 空腔支撑字段=%s\n', nL, cF, rF);

BSD = [];
if ~isempty(base_support_file) && exist(base_support_file,'file')
    B = load(base_support_file,'base_support_data');
    if isfield(B,'base_support_data'), BSD = B.base_support_data; end
    fprintf('[assemble] 底座支撑层 %d\n', numel(BSD));
end

%% ---- 组装有序序列 ----
% segs{n} = struct(rows[ix0 ix1], signal, type, layer, name)
seq = zeros(0,7);   % [x y z v1 v2 pathid type]
segs = struct('r0',{},'r1',{},'signal',{},'type',{},'layer',{},'name',{});
pid = 0;

% (1) 底座支撑: 平 Z, 姿态竖直向下(v1=v2=0)
if ~isempty(BSD)
    zl = arrayfun(@(s) get0(s,'z_level',0), BSD);
    [~,ord] = sort(zl,'ascend');
    for ii = ord(:)'
        sd = BSD(ii);
        for j = 1:numel(sd.paths_3d)
            P = norm_path(sd.paths_3d{j}); if size(P,1)<2, continue; end
            pid = pid+1;
            rows = [P, zeros(size(P,1),1), zeros(size(P,1),1), ...
                    pid*ones(size(P,1),1), 2*ones(size(P,1),1)];
            segs(end+1) = mkseg(size(seq,1)+1, size(seq,1)+size(rows,1), ...
                opt.resin_signal, 2, 0, sprintf('baseSup_z%02d_P%d', ii, j)); %#ok<AGROW>
            seq = [seq; rows]; %#ok<AGROW>
        end
    end
end

% (2) 逐层: 碳纤维 -> 空腔支撑
for k = 1:nL
    pc = []; if isfield(ALD(k),'pointCloud_data'), pc = ALD(k).pointCloud_data; end
    % 碳纤维
    [seq, segs, pid] = add_paths2(seq, segs, pid, ALD(k), cF, pc, 1, opt.carbon_signal, k, 'carbon');
    % 空腔支撑
    [seq, segs, pid] = add_paths2(seq, segs, pid, ALD(k), rF, pc, 2, opt.resin_signal, k, 'cavSup');
end
N = size(seq,1);
fprintf('[assemble] 序列总点数 %d, 路径段 %d\n', N, numel(segs));
if N==0, error('序列为空: 检查字段名/数据'); end

%% ---- 静态障碍点云 ----
base_pts = [];
if ~isempty(reduced_base_stl) && exist(reduced_base_stl,'file')
    base_pts = sample_stl_surface(reduced_base_stl, opt.base_spacing);
end

%% ---- 碰撞校验 ----
if opt.run_check
    R = collision_check_sequence(seq, base_pts, config, struct('retry',opt.retry,'radius_mm',opt.radius_mm));
    v1f = R.v1_fix; v2f = R.v2_fix;     % 重试修正后的姿态
else
    R = []; v1f = seq(:,4); v2f = seq(:,5);
end

%% ---- 写 Excel + 清单 + 报告 ----
if opt.write_excel
    manifest = {}; sq = 0;
    for n = 1:numel(segs)
        s = segs(n); ra = s.r0:s.r1;
        xyz = seq(ra,1:3) + opt.xyz_add;
        out = [xyz, v1f(ra), v2f(ra), repmat(s.signal(:).', numel(ra), 1)];
        sq = sq+1;
        fname = sprintf('print_%04d_%s_L%d.%s', sq, s.name, s.layer, opt.format);
        writematrix(out, fullfile(opt.output_dir, fname));
        if ~isempty(R)
            st = R.status(ra); ncoll = sum(st==2); mx = max(st);
        else, ncoll=0; mx=0; end
        manifest(end+1,:) = {sq, fname, s.name, s.layer, numel(ra), mx, ncoll}; %#ok<AGROW>
    end
    T = cell2table(manifest, 'VariableNames', ...
        {'seq','filename','type','layer','n_points','max_status','n_collision_pts'});
    writetable(T, fullfile(opt.output_dir,'print_manifest.csv'));
    fprintf('[assemble] 写出 %d 个路径文件 + print_manifest.csv -> %s\n', sq, opt.output_dir);
    if ~isempty(R) && ~isempty(R.summary.collision_idx)
        write_collision_report(fullfile(opt.output_dir,'collision_report.txt'), R, seq, segs);
        fprintf('[assemble] !! 检出碰撞点 %d, 详见 collision_report.txt\n', numel(R.summary.collision_idx));
    end
end
fprintf('[assemble] 完成.\n');
end

%% ================= 子函数 =================
function f = pick_field(ALD, want, fallback)
    if isfield(ALD, want), f=want; elseif isfield(ALD, fallback), f=fallback;
    else, error('字段 %s / %s 都不存在', want, fallback); end
end
function s = mkseg(r0,r1,signal,type,layer,name)
    s=struct('r0',r0,'r1',r1,'signal',signal,'type',type,'layer',layer,'name',name);
end
function v = get0(s,f,d), if isfield(s,f)&&~isempty(s.(f)), v=s.(f); else, v=d; end, end
function P = norm_path(P)
    if isempty(P)||~isnumeric(P), P=zeros(0,3); return; end
    if size(P,2)~=3 && size(P,1)==3, P=P.'; end
end
function [seq, segs, pid] = add_paths2(seq, segs, pid, ld, field, pc, type, signal, layer, name)
    if ~isfield(ld, field) || isempty(ld.(field)), return; end
    paths = ld.(field); if ~iscell(paths), return; end
    for j = 1:numel(paths)
        P = norm_path(paths{j}); if size(P,1)<2, continue; end
        if type==1 || ~isempty(pc)    % 碳纤维/曲面支撑: 用曲面法向
            if ~isempty(pc)
                nrm = getPathNormals(P, pc);
                v2 = rad2deg(asin(max(-1,min(1,-nrm(:,1)))));
                v1 = rad2deg(atan2(-nrm(:,2), -nrm(:,3)));
            else
                v1 = zeros(size(P,1),1); v2 = v1;
            end
        else
            v1 = zeros(size(P,1),1); v2 = v1;
        end
        pid = pid+1;
        rows = [P, v1, v2, pid*ones(size(P,1),1), type*ones(size(P,1),1)];
        segs(end+1) = mkseg(size(seq,1)+1, size(seq,1)+size(rows,1), signal, type, layer, ...
                            sprintf('%s_P%d', name, j)); %#ok<AGROW>
        seq = [seq; rows]; %#ok<AGROW>
    end
end
function write_collision_report(fpath, R, seq, segs)
    fid = fopen(fpath,'w');
    fprintf(fid, '碰撞报告\n安全 %d | 风险 %d | 碰撞 %d\n\n', ...
        R.summary.n_safe, R.summary.n_risk, R.summary.n_collision);
    ci = R.summary.collision_idx;
    fprintf(fid, '碰撞点(全局点序号 / 所属段 / 触发源 / 最小间距mm):\n');
    seg_of = zeros(size(seq,1),1);
    for n=1:numel(segs), seg_of(segs(n).r0:segs(n).r1)=n; end
    for t = 1:numel(ci)
        i = ci(t); n = seg_of(i);
        nm = ''; if n>0, nm = segs(n).name; end
        fprintf(fid, '  #%d  段[%s]  源=%s  d=%.3f\n', i, nm, R.feature{i}, R.min_dist(i));
    end
    if ~isempty(R.summary.unresolved_idx)
        fprintf(fid, '\n姿态重试后仍未解(需改打印顺序/几何): %d 点\n', numel(R.summary.unresolved_idx));
    end
    fclose(fid);
end
