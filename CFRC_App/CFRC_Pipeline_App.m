classdef CFRC_Pipeline_App < handle
%CFRC_PIPELINE_APP  Interactive console that chains the whole CFRC-AM pipeline.
%
%   Launch with:  >> launch_CFRC
%
%   Left  : every pipeline stage with a colored status lamp + per-stage Run.
%   Centre: details of the selected stage (inputs/outputs, params, actions)
%           and a highlighted "NEXT STEP" card.
%   Bottom: live run log.
%
%   Folder split is owned by cfrc_layout(): scripts/ functions/ viz/ python/
%   are on the path; data/ is the working directory so every stage's bare
%   load('x.mat')/save('x.mat') resolves into data/. Outputs go to output/.
%
%   Status lamp colors:
%     grey  blocked (an input is missing)      amber  READY (this is next)
%     green done                                yellow stale (input newer)
%     red   last run errored                    violet manual (Abaqus) step

    properties
        L                 % folder layout struct (from cfrc_layout)
        UIFigure
        Stages            % struct array describing each stage
        Lamps             % gobjects, one status lamp per stage
        Sel       = 1     % selected stage index
        DetailPanel       % uipanel that gets rebuilt on selection
        NavPanel
        LogArea
        LogLines  = {}
        NextLabel         % "next step" text in the top bar
        StatusBar
        Err               % containers.Map: key -> last run errored?
        Vox               % struct of voxelize param widgets (built on demand)
        VoxState          % persistent voxelize params (survive detail rebuilds)
        VoxAx             % embedded 3D STL preview axes (stage 1)
        VoxInfo           % label under the preview (bbox / orientation info)
        PW                % containers.Map: stageKey -> struct of param widgets
        PState            % containers.Map: stageKey -> struct of persistent param values
    end

    methods
        function app = CFRC_Pipeline_App()
            app.L   = cfrc_layout();
            app.Err = containers.Map('KeyType','char','ValueType','logical');
            app.VoxState = struct('src','from-stl','in','','size',1.0,'thr',0.5,'rot','none');
            app.PW       = containers.Map('KeyType','char','ValueType','any');
            app.PState   = containers.Map('KeyType','char','ValueType','any');
            app.Stages = app.defineStages();
            app.buildUI();
            app.refreshStatus();
            app.selectStage(app.firstActionable());
        end
    end

    % ---------------------------------------------------------------- UI build
    methods (Access = private)
        function buildUI(app)
            scr = get(0,'ScreenSize');
            w = min(1320, scr(3)*0.92);  h = min(840, scr(4)*0.90);
            app.UIFigure = uifigure('Name','CFRC 增材制造管线控制台 · Pipeline Console', ...
                'Position',[max(40,(scr(3)-w)/2), max(40,(scr(4)-h)/2), w, h], ...
                'Color',[0.96 0.97 0.98], 'AutoResizeChildren','on');
            % Layout is made responsive by uigridlayout + scrollable panels below,
            % so no SizeChangedFcn is needed (it would conflict with AutoResizeChildren).

            g = uigridlayout(app.UIFigure,[3 1]);
            g.RowHeight   = {52,'1x',24};
            g.ColumnWidth = {'1x'};
            g.RowSpacing  = 6; g.Padding = [8 8 8 8];

            % ---- top bar ----
            top = uigridlayout(g,[2 6]); top.Layout.Row = 1;
            top.RowHeight = {'1x','fit'};
            top.ColumnWidth = {'1x',120,120,130,130,120};
            top.RowSpacing = 1; top.ColumnSpacing = 6; top.Padding = [2 2 2 2];
            t = uilabel(top,'Text','CFRC 管线控制台   Curved-layer CFRC-AM Pipeline', ...
                'FontSize',17,'FontWeight','bold','FontColor',[0.10 0.34 0.42]);
            t.Layout.Row = 1; t.Layout.Column = 1;
            app.NextLabel = uilabel(top,'Text','', 'FontSize',12,'FontColor',[0.72 0.42 0.04]);
            app.NextLabel.Layout.Row = 2; app.NextLabel.Layout.Column = [1 6];
            b1 = uibutton(top,'Text','▶ 自动串联 Run-all','BackgroundColor',[0.10 0.49 0.86], ...
                'FontColor','w','FontWeight','bold','ButtonPushedFcn',@(~,~)app.runAuto());
            b1.Layout.Row = 1; b1.Layout.Column = 2;
            b2 = uibutton(top,'Text','↻ 刷新状态','ButtonPushedFcn',@(~,~)app.refreshStatus());
            b2.Layout.Row = 1; b2.Layout.Column = 3;
            b3 = uibutton(top,'Text','📂 data 目录','ButtonPushedFcn',@(~,~)app.openFolder(app.L.data));
            b3.Layout.Row = 1; b3.Layout.Column = 4;
            b4 = uibutton(top,'Text','📂 output 目录','ButtonPushedFcn',@(~,~)app.openFolder(app.L.output));
            b4.Layout.Row = 1; b4.Layout.Column = 5;
            b5 = uibutton(top,'Text','❔ 帮助','ButtonPushedFcn',@(~,~)app.showHelp());
            b5.Layout.Row = 1; b5.Layout.Column = 6;

            % ---- body : nav | (detail / log) ----
            body = uigridlayout(g,[1 2]); body.Layout.Row = 2;
            body.ColumnWidth = {360,'1x'}; body.ColumnSpacing = 8; body.Padding = [0 0 0 0];

            app.NavPanel = uipanel(body,'Title','流程阶段 Pipeline Stages','FontWeight','bold');
            app.NavPanel.Layout.Column = 1;
            app.buildNav();

            right = uigridlayout(body,[2 1]); right.Layout.Column = 2;
            right.RowHeight = {'1x',210}; right.RowSpacing = 8; right.Padding = [0 0 0 0];

            app.DetailPanel = uipanel(right,'Title','阶段详情 Stage Detail','FontWeight','bold');
            app.DetailPanel.Layout.Row = 1;

            logp = uipanel(right,'Title','运行日志 Run Log','FontWeight','bold');
            logp.Layout.Row = 2;
            lg = uigridlayout(logp,[1 1]); lg.Padding = [6 6 6 6];
            app.LogArea = uitextarea(lg,'Editable','off','FontName','Consolas', ...
                'Value',{'就绪。Ready. 选择左侧阶段或点 “自动串联”。'});

            % ---- status bar ----
            app.StatusBar = uilabel(g,'Text',sprintf('工作目录 cwd = %s', app.L.data), ...
                'FontSize',11,'FontColor',[0.4 0.4 0.45]);
            app.StatusBar.Layout.Row = 3;
        end

        function buildNav(app)
            % flatten into header + stage rows
            items = {}; prev = '';
            for i = 1:numel(app.Stages)
                s = app.Stages(i);
                if ~strcmp(s.grp, prev)
                    items{end+1} = struct('type','header','text',s.grp,'idx',0); %#ok<AGROW>
                    prev = s.grp;
                end
                items{end+1} = struct('type','stage','text',s.name,'idx',i); %#ok<AGROW>
            end
            N = numel(items);
            RH = cell(1,N);
            for k = 1:N
                if strcmp(items{k}.type,'header'); RH{k} = 24; else; RH{k} = 38; end
            end
            nav = uigridlayout(app.NavPanel,[N 1]);
            nav.RowHeight = RH; nav.ColumnWidth = {'1x'};
            nav.Scrollable = 'on'; nav.RowSpacing = 3; nav.Padding = [4 4 4 4];

            app.Lamps = gobjects(1,numel(app.Stages));
            for k = 1:N
                it = items{k};
                if strcmp(it.type,'header')
                    hl = uilabel(nav,'Text',['  ' it.text],'FontWeight','bold', ...
                        'FontSize',11,'FontColor',[0.30 0.30 0.36], ...
                        'BackgroundColor',[0.90 0.92 0.94]);
                    hl.Layout.Row = k;
                else
                    i = it.idx;
                    row = uigridlayout(nav,[1 3]); row.Layout.Row = k;
                    row.ColumnWidth = {18,'1x',48}; row.Padding = [0 0 0 0]; row.ColumnSpacing = 4;
                    lp = uilabel(row,'Text',char(9679),'FontSize',15, ...
                        'FontColor',[0.6 0.6 0.6],'HorizontalAlignment','center');
                    lp.Layout.Column = 1;
                    bn = uibutton(row,'Text',it.text,'HorizontalAlignment','left', ...
                        'FontSize',12,'ButtonPushedFcn',@(~,~)app.selectStage(i));
                    bn.Layout.Column = 2;
                    rb = uibutton(row,'Text','Run','FontSize',11, ...
                        'ButtonPushedFcn',@(~,~)app.runStage(i));
                    rb.Layout.Column = 3;
                    app.Lamps(i) = lp;
                end
            end
        end

    end

    % ----------------------------------------------------------- stage detail
    methods (Access = private)
        function selectStage(app, i)
            if isempty(i) || i < 1; i = 1; end
            app.Sel = i;
            app.populateDetail();
            app.refreshStatus();   % so the selected lamp/next stays in sync
        end

        function populateDetail(app)
            delete(app.DetailPanel.Children);
            s = app.Stages(app.Sel);
            st = app.stageStatus(s);

            d = uigridlayout(app.DetailPanel,[8 1]);
            if strcmp(s.kind,'voxelize')
                d.RowHeight = {'fit','fit','fit','fit','fit','fit',330,'fit'};  % tall preview row
            else
                d.RowHeight = {'fit','fit','fit','fit','fit','fit','1x','fit'};
            end
            d.ColumnWidth = {'1x'}; d.RowSpacing = 6; d.Padding = [10 8 10 8];
            d.Scrollable = 'on';

            uilabel(d,'Text',s.name,'FontSize',16,'FontWeight','bold');
            uilabel(d,'Text',sprintf('分组 %s   |   类型 %s   |   状态 %s', ...
                s.grp, app.kindText(s.kind), app.statusText(st)), ...
                'FontColor',app.colorFor(st),'FontWeight','bold');

            uilabel(d,'Text',s.help,'WordWrap','on','FontColor',[0.25 0.25 0.3]);

            % inputs / outputs status
            uilabel(d,'Text',['输入 inputs:   '  app.ioLine(s.inputs, app.inDir(s))],'FontName','Consolas','FontSize',11);
            uilabel(d,'Text',['产物 outputs:  '  app.ioLine(s.outputs, s.iodir)],'FontName','Consolas','FontSize',11);

            % param area (voxelize only) or manual command text
            mid = uipanel(d,'BorderType','none'); mid.Layout.Row = 6;
            app.buildStageParams(mid, s);

            % row 7: 3D STL preview for voxelize; empty spacer otherwise
            if strcmp(s.kind,'voxelize')
                pv = uipanel(d,'Title','STL 预览（目标：Z 轴朝上）','FontWeight','bold');
                pv.Layout.Row = 7;
                app.buildVoxPreview(pv);
            end

            % row 8 = actions
            act = uigridlayout(d,[1 4]); act.Layout.Row = 8;
            act.ColumnWidth = {150,'1x','fit','fit'}; act.ColumnSpacing = 6; act.Padding=[0 0 0 0];
            rb = uibutton(act,'Text','▶ 运行此阶段 Run','BackgroundColor',[0.10 0.49 0.86], ...
                'FontColor','w','FontWeight','bold','ButtonPushedFcn',@(~,~)app.runStage(app.Sel));
            rb.Layout.Column = 1;
            ob = uibutton(act,'Text','📝 打开脚本','ButtonPushedFcn',@(~,~)app.openStageFile(s));
            ob.Layout.Column = 3;
            % viz buttons
            if ~isempty(s.viz)
                vb = uibutton(act,'Text',['👁 ' s.viz{1}{1}], ...
                    'ButtonPushedFcn',@(~,~)app.runViz(s.viz{1}{2}));
                vb.Layout.Column = 4;
            end
        end

        function buildStageParams(app, parent, s)
            if strcmp(s.kind,'voxelize')
                    if isempty(app.VoxState) || ~isfield(app.VoxState,'rot')
                        app.VoxState = struct('src','from-stl','in','','size',1.0,'thr',0.5,'rot','none');
                    end
                    vsd = app.VoxState;
                    gl = uigridlayout(parent,[3 4]);
                    gl.RowHeight = {'fit','fit','fit'};
                    gl.ColumnWidth = {120,'1x',120,'1x'}; gl.Padding=[0 0 0 0]; gl.RowSpacing=4; gl.ColumnSpacing=6;
                    l1=uilabel(gl,'Text','来源 source'); l1.Layout.Row=1; l1.Layout.Column=1;
                    app.Vox.src = uidropdown(gl,'Items',{'from-stl','from-topo'},'Value',vsd.src, ...
                        'ValueChangedFcn',@(~,~)app.onVoxParamChange());
                    app.Vox.src.Layout.Row=1; app.Vox.src.Layout.Column=2;
                    l2=uilabel(gl,'Text','体素尺寸 mm'); l2.Layout.Row=1; l2.Layout.Column=3;
                    app.Vox.size = uieditfield(gl,'numeric','Value',vsd.size,'Limits',[0.01 1000], ...
                        'ValueChangedFcn',@(~,~)app.syncVoxState());
                    app.Vox.size.Layout.Row=1; app.Vox.size.Layout.Column=4;
                    l3=uilabel(gl,'Text','旋转预设 rotate-axis'); l3.Layout.Row=2; l3.Layout.Column=1;
                    app.Vox.rot = uidropdown(gl,'Items', ...
                        {'none','y-to-z','z-to-y','x-to-z','z-to-x','x-to-y','y-to-x','flip-x','flip-y','flip-z'}, ...
                        'Value',vsd.rot,'ValueChangedFcn',@(~,~)app.onVoxParamChange());
                    app.Vox.rot.Layout.Row=2; app.Vox.rot.Layout.Column=2;
                    l4=uilabel(gl,'Text','密度阈值 (topo)'); l4.Layout.Row=2; l4.Layout.Column=3;
                    app.Vox.thr = uieditfield(gl,'numeric','Value',vsd.thr,'Limits',[0 1], ...
                        'ValueChangedFcn',@(~,~)app.syncVoxState());
                    app.Vox.thr.Layout.Row=2; app.Vox.thr.Layout.Column=4;
                    l5=uilabel(gl,'Text','输入文件 file'); l5.Layout.Row=3; l5.Layout.Column=1;
                    app.Vox.in = uieditfield(gl,'text','Value',vsd.in, ...
                        'ValueChangedFcn',@(~,~)app.onVoxParamChange());
                    app.Vox.in.Layout.Row=3; app.Vox.in.Layout.Column=[2 3];
                    bb = uibutton(gl,'Text','浏览…','ButtonPushedFcn',@(~,~)app.browseVoxInput());
                    bb.Layout.Row=3; bb.Layout.Column=4;
            elseif ~isempty(s.pspec)
                if strcmp(s.kind,'abaqus_b')
                    g2 = uigridlayout(parent,[2 1]); g2.RowHeight={'fit','1x'}; g2.Padding=[0 0 0 0]; g2.RowSpacing=6;
                    pp = uipanel(g2,'BorderType','none'); pp.Layout.Row=1; app.buildParamWidgets(pp, s);
                    ta = uitextarea(g2,'Editable','off','FontName','Consolas','FontSize',11); ta.Layout.Row=2;
                    ta.Value = app.wrapLines(s.cmd);
                else
                    app.buildParamWidgets(parent, s);
                end
            elseif strcmp(s.kind,'manual')
                gl = uigridlayout(parent,[1 1]); gl.Padding=[0 0 0 0];
                ta = uitextarea(gl,'Editable','off','FontName','Consolas','FontSize',11);
                ta.Value = app.wrapLines(s.cmd);
            else
                gl = uigridlayout(parent,[1 1]); gl.Padding=[0 0 0 0];
                uilabel(gl,'Text',['命令 command:  ' s.cmd],'FontName','Consolas', ...
                    'FontSize',11,'FontColor',[0.35 0.35 0.4],'WordWrap','on');
            end
        end
    end

    % ------------------------------------------------------------- run engine
    methods (Access = private)
        function runStage(app, i)
            % NOTE: do NOT rebuild the detail panel here -- that would reset the
            % param widgets (e.g. the voxelize file path). Persistent params live
            % in app.VoxState, so pressing Run never loses what you entered.
            app.Sel = i;
            s = app.Stages(i);
            app.setBusy(sprintf('运行中 Running: %s …', s.name));
            app.appendLog(sprintf('\n===== RUN  %s   [%s] =====', s.name, datestr(now,'HH:MM:SS')));
            ok = true; out = '';
            try
                switch s.kind
                    case 'func';     out = app.runMatlab(s.cmd, s.iodir);
                    case 'script'
                        if ~isempty(s.pspec)
                            out = app.runScriptParams(s.cmd, app.gatherParams(s.key), s.iodir);
                        else
                            out = app.runScript(s.cmd, s.iodir);
                        end
                    case 'voxelize'; out = app.runVoxelize();
                    case 'export4';  out = app.runExport4();
                    case 'extract';  out = app.runExtract();
                    case 'pathall';  out = app.runPathAll();
                    case 'abaqus_b'; out = app.runAbaqusExtract();
                    case 'manual'
                        app.appendLog('该阶段为 Abaqus 手动步骤，请按下方说明在 Abaqus 中执行：');
                        app.appendLog(s.cmd);
                        app.setBusy(''); app.refreshStatus(); app.selectStage(i); return;
                    otherwise;       out = app.runMatlab(s.cmd, s.iodir);
                end
            catch err
                ok = false;
                out = sprintf('!! ERROR: %s', err.message);
            end
            app.appendLog(out);
            app.Err(s.key) = ~ok;
            if ok; app.appendLog(sprintf('----- DONE  %s -----', s.name));
            else;  app.appendLog(sprintf('----- FAILED %s -----', s.name)); end
            app.setBusy('');
            app.refreshStatus();
            app.selectStage(i);
        end

        function runAuto(app)
            app.appendLog(sprintf('\n######## 自动串联 Run-all  [%s] ########', datestr(now,'HH:MM:SS')));
            for i = 1:numel(app.Stages)
                s  = app.Stages(i);
                st = app.stageStatus(s);
                if ismember(st, {'done'})
                    continue;
                end
                if ~ismember(s.kind, {'func','script'})
                    app.appendLog(sprintf('暂停：到达需手动/外部的阶段「%s」。请处理后再继续。', s.name));
                    app.NextLabel.Text = sprintf('下一步 NEXT ▸ %s', s.name);
                    app.selectStage(i); app.refreshStatus(); return;
                end
                if strcmp(st,'blocked')
                    app.appendLog(sprintf('暂停：阶段「%s」缺少输入。', s.name));
                    app.selectStage(i); app.refreshStatus(); return;
                end
                app.runStage(i);
                if isKey(app.Err, s.key) && app.Err(s.key)
                    app.appendLog(sprintf('暂停：阶段「%s」出错，停止自动串联。', s.name));
                    return;
                end
            end
            app.appendLog('######## 自动串联完成（MATLAB 段） ########');
        end

        function out = runMatlab(app, expr, where)
            d = app.dirFor(where);
            old = cd(d); cu = onCleanup(@() cd(old)); %#ok<NASGU>
            out = evalc(expr);
        end

        function out = runScript(app, base, where)
            % Run a top-level script that may start with `clear; clc; close all`.
            % Executed in the BASE workspace so its `clear` cannot wipe this
            % method frame; the app window is hidden from `close all`.
            d  = app.dirFor(where);
            sf = app.scriptFile(base);
            oldHV  = app.UIFigure.HandleVisibility;
            old    = cd(d);
            cu1 = onCleanup(@() app.restoreHV(oldHV)); %#ok<NASGU>
            cu2 = onCleanup(@() cd(old));              %#ok<NASGU>
            app.UIFigure.HandleVisibility = 'off';
            assignin('base','CFRC__sf__', sf);
            out = evalin('base', 'evalc(''run(CFRC__sf__)'')');
        end

        function out = runVoxelize(app)
            app.syncVoxState();           % pull any live widget edits into VoxState
            v = app.VoxState;
            if isempty(v); error('体素化参数未初始化。'); end
            src = v.src;  inf = strtrim(v.in);  vs = v.size;  thr = v.thr;
            if isempty(inf)
                error('请先选择输入文件（在「① 体素化」详情里点 “浏览…”）。');
            end
            py = app.pyFile('voxelize');
            if strcmp(src,'from-topo')
                cmd = sprintf('python "%s" from-topo "%s" -s %g -t %g', py, inf, vs, thr);
            else
                cmd = sprintf('python "%s" from-stl "%s" -s %g -r %s', py, inf, vs, v.rot);
            end
            out = app.runPython(cmd, 'data');
            if ~isempty(app.VoxInfo) && isvalid(app.VoxInfo)
                app.VoxInfo.Text = '体素化完成，详见下方日志（grid 尺寸 / bounds）。';
            end
        end

        function out = runExport4(app)
            pairs = { 'all_layers_paths_only_v3.mat','mine_stream'; ...
                      'all_layers_paths_only_mine_offset.mat','mine_offset'; ...
                      'all_layers_paths_only_planar_stream.mat','planar_stream'; ...
                      'all_layers_paths_only_planar_offset.mat','planar_offset' };
            d = app.L.data; old = cd(d); cu = onCleanup(@() cd(old)); %#ok<NASGU>
            if ~exist(app.L.fea,'dir'); mkdir(app.L.fea); end
            buf = sprintf('导出目标 FEA dir = %s\n', app.L.fea);
            n_ok = 0;
            for k = 1:size(pairs,1)
                if exist(fullfile(d,pairs{k,1}),'file')
                    expr = sprintf('export_paths_to_fea(''%s'',''%s'',''%s'')', ...
                        pairs{k,1}, pairs{k,2}, app.L.fea);
                    buf = [buf sprintf('\n-- %s --\n',pairs{k,2}) evalc(expr)]; %#ok<AGROW>
                    n_ok = n_ok + 1;
                else
                    buf = [buf sprintf('\n[skip] 缺少 %s\n', pairs{k,1})]; %#ok<AGROW>
                end
            end
            % --- 把 Abaqus / 对比 helper 脚本一并复制到 FEA dir, 让 temp 目录自成体系
            % --- (⑬ execfile / ⑭ extract / ⑮ run_compare 都在该目录内运行)。
            % --- 文件清单与 run_full_comparison 的 Stage 9 保持一致。
            helpers = { ...
                fullfile(app.L.python,  'abaqus_cfrc_compare.py'); ...
                fullfile(app.L.python,  'extract_fea_results.py'); ...
                fullfile(app.L.python,  'diagnose_loadpoint.py'); ...
                fullfile(app.L.python,  'grind_serial_runner.py'); ...
                fullfile(app.L.python,  'tools_query_odb_errsets.py'); ...
                fullfile(app.L.scripts, 'run_compare.m'); ...
                fullfile(app.L.scripts, 'compare_fea_results.m') };
            buf = [buf sprintf('\n-- copy helper scripts -> %s --\n', app.L.fea)];
            for k = 1:numel(helpers)
                src = helpers{k}; [~, nm, ext] = fileparts(src);
                if ~exist(src,'file')
                    buf = [buf sprintf('  [warn] 源缺失 %s\n', src)]; %#ok<AGROW>
                    continue;
                end
                try
                    copyfile(src, fullfile(app.L.fea, [nm ext]), 'f');
                    buf = [buf sprintf('  [ok]  %s%s\n', nm, ext)]; %#ok<AGROW>
                catch e
                    buf = [buf sprintf('  [fail] %s%s: %s\n', nm, ext, e.message)]; %#ok<AGROW>
                end
            end
            buf = [buf sprintf('\n导出完成: %d/4 config; 路径与 helper 均已写入 %s\n', n_ok, app.L.fea)];
            out = buf;
        end

        function out = runExtract(app)
            py = app.pyFile('extract_fea_results');
            cmd = sprintf('abaqus cae noGUI="%s"', py);
            out = app.runPython(cmd, 'fea');
        end

        function out = runPython(app, cmd, where)
            d = app.dirFor(where);
            old = cd(d); cu = onCleanup(@() cd(old)); %#ok<NASGU>
            app.appendLog(['$ ' cmd]);
            [status, out] = system(cmd);
            out = sprintf('(exit %d)\n%s', status, out);
        end
    end

    % --------------------------------------------------------------- status
    methods (Access = private)
        function refreshStatus(app)
            nReady = 0; firstNext = 0;
            for i = 1:numel(app.Stages)
                s  = app.Stages(i);
                st = app.stageStatus(s);
                if isgraphics(app.Lamps(i))
                    app.Lamps(i).FontColor = app.colorFor(st);
                    app.Lamps(i).Tooltip   = app.statusText(st);
                end
                if firstNext == 0 && ismember(st, {'ready','manual'})
                    firstNext = i;
                end
                if strcmp(st,'done'); nReady = nReady + 1; end
            end
            if firstNext > 0
                ns = app.Stages(firstNext);
                app.NextLabel.Text = sprintf('下一步 NEXT ▸  %s     —  %s', ns.name, ns.next);
            else
                app.NextLabel.Text = '全部阶段完成 ✓  All stages complete.';
            end
            app.StatusBar.Text = sprintf('已完成 %d / %d 阶段   |   cwd = %s', ...
                nReady, numel(app.Stages), app.L.data);
            drawnow limitrate;
        end

        function st = stageStatus(app, s)
            if isKey(app.Err, s.key) && app.Err(s.key); st = 'error'; return; end
            inOK  = isempty(s.inputs)  || app.haveAll(s.inputs,  app.inDir(s));
            outOK = ~isempty(s.outputs) && app.haveAll(s.outputs, s.iodir);
            if strcmp(s.kind,'manual')
                if outOK; st = 'done'; elseif inOK; st = 'manual'; else; st = 'blocked'; end
                return;
            end
            if ~inOK; st = 'blocked'; return; end
            if outOK
                if app.outputsStale(s); st = 'stale'; else; st = 'done'; end
            else
                st = 'ready';
            end
        end

        function tf = outputsStale(app, s)
            tf = false;
            ti = 0;
            for k = 1:numel(s.inputs)
                p = app.artPath(s.inputs{k}, app.inDir(s));
                dd = dir(p); if ~isempty(dd); ti = max(ti, dd(1).datenum); end
            end
            to = inf;
            for k = 1:numel(s.outputs)
                p = app.artPath(s.outputs{k}, s.iodir);
                dd = dir(p); if ~isempty(dd); to = min(to, dd(1).datenum); end
            end
            if ti > 0 && isfinite(to); tf = ti > to + 1/86400; end
        end

        function i0 = firstActionable(app)
            i0 = 1;
            for i = 1:numel(app.Stages)
                st = app.stageStatus(app.Stages(i));
                if ismember(st,{'ready','manual'}); i0 = i; return; end
            end
        end
    end

    % ----------------------------------------------------------- small helpers
    methods (Access = private)
        function d = dirFor(app, where)
            if nargin>=2 && strcmp(where,'fea'); d = app.L.fea; else; d = app.L.data; end
            if ~exist(d,'dir'); mkdir(d); end
        end
        function p = artPath(app, name, where)
            if strcmp(where,'fea'); p = fullfile(app.L.fea, name); else; p = fullfile(app.L.data, name); end
        end
        function w = inDir(~, s)
            % Directory to resolve a stage's INPUTS against. Defaults to its
            % iodir, but a stage whose inputs and outputs live in different
            % folders (e.g. ⑫ export: reads .mat from data/, writes to fea/)
            % sets s.indir explicitly so its status lamp isn't wrongly 'blocked'.
            if isfield(s,'indir') && ~isempty(s.indir); w = s.indir; else; w = s.iodir; end
        end
        function tf = haveAll(app, names, where)
            tf = true;
            for k = 1:numel(names)
                p = app.artPath(names{k}, where);
                if ~(exist(p,'file') || exist(p,'dir')); tf = false; return; end
            end
        end
        function s = ioLine(app, names, where)
            if isempty(names); s = '(无 none)'; return; end
            parts = cell(1,numel(names));
            for k = 1:numel(names)
                p = app.artPath(names{k}, where);
                mark = '✗'; if exist(p,'file')||exist(p,'dir'); mark = '✓'; end
                parts{k} = sprintf('%s %s', mark, names{k});
            end
            s = strjoin(parts, '   ');
        end
        function f = scriptFile(app, base)
            f = fullfile(app.L.scripts,[base '.m']);
            if ~exist(f,'file'); f2 = fullfile(app.L.viz,[base '.m']); if exist(f2,'file'); f = f2; end; end
            if ~exist(f,'file'); f3 = fullfile(app.L.functions,[base '.m']); if exist(f3,'file'); f = f3; end; end
        end
        function f = pyFile(app, base); f = fullfile(app.L.python,[base '.py']); end
        function restoreHV(app, hv); try; app.UIFigure.HandleVisibility = hv; catch; end; end

        function runViz(app, base)
            app.setBusy(sprintf('可视化 %s …', base));
            try
                out = app.runScript(base, 'data');
                app.appendLog(sprintf('\n[viz %s]\n%s', base, out));
            catch err
                app.appendLog(sprintf('[viz %s] ERROR: %s', base, err.message));
            end
            app.setBusy('');
        end

        function openStageFile(app, s)
            switch s.kind
                case 'voxelize'; f = app.pyFile('voxelize');
                case 'manual';   app.showText('Abaqus 手动步骤', app.wrapLines(s.cmd)); return;
                otherwise
                    base = regexp(s.cmd,'^[A-Za-z_]\w*','match','once');
                    if isempty(base); app.appendLog('无可打开脚本。'); return; end
                    f = app.scriptFile(base);
            end
            if exist(f,'file'); try; edit(f); catch; app.openFolder(fileparts(f)); end
            else; app.appendLog(['找不到文件: ' f]); end
        end

        function browseVoxInput(app)
            [fn,pp] = uigetfile({'*.stl;*.mat','STL / 拓扑 (*.stl,*.mat)';'*.*','所有文件'}, ...
                '选择体素化输入', app.L.data);
            if isequal(fn,0); return; end
            app.VoxState.in = fullfile(pp,fn);
            if ~isempty(app.Vox) && isfield(app.Vox,'in') && isvalid(app.Vox.in)
                app.Vox.in.Value = app.VoxState.in;
            end
            app.refreshVoxPreview();
        end

        function syncVoxState(app)
            % Copy live voxelize widget values into the persistent VoxState.
            % No-op when the voxelize detail isn't currently shown.
            if isempty(app.Vox); return; end
            try
                if isfield(app.Vox,'src')  && isvalid(app.Vox.src);  app.VoxState.src  = app.Vox.src.Value;        end
                if isfield(app.Vox,'in')   && isvalid(app.Vox.in);   app.VoxState.in   = strtrim(app.Vox.in.Value); end
                if isfield(app.Vox,'size') && isvalid(app.Vox.size); app.VoxState.size = app.Vox.size.Value;       end
                if isfield(app.Vox,'thr')  && isvalid(app.Vox.thr);  app.VoxState.thr  = app.Vox.thr.Value;        end
                if isfield(app.Vox,'rot')  && isvalid(app.Vox.rot);  app.VoxState.rot  = app.Vox.rot.Value;        end
            catch
            end
        end

        function onVoxParamChange(app)
            app.syncVoxState();
            app.refreshVoxPreview();
        end

        function buildVoxPreview(app, parent)
            g = uigridlayout(parent,[2 1]); g.RowHeight = {'1x','fit'}; g.RowSpacing = 4; g.Padding = [6 6 6 6];
            app.VoxAx = uiaxes(g); app.VoxAx.Layout.Row = 1;
            ctl = uigridlayout(g,[1 2]); ctl.Layout.Row = 2; ctl.ColumnWidth = {130,'1x'}; ctl.Padding = [0 0 0 0];
            rb = uibutton(ctl,'Text','↻ 刷新预览','ButtonPushedFcn',@(~,~)app.refreshVoxPreview()); rb.Layout.Column = 1;
            app.VoxInfo = uilabel(ctl,'Text','','FontSize',11,'FontColor',[0.3 0.3 0.4]); app.VoxInfo.Layout.Column = 2;
            app.refreshVoxPreview();
        end

        function refreshVoxPreview(app)
            if isempty(app.VoxAx) || ~isvalid(app.VoxAx); return; end
            ax = app.VoxAx; app.syncVoxState(); v = app.VoxState;
            cla(ax,'reset'); hold(ax,'on'); info = '';
            haveSTL = strcmpi(v.src,'from-stl') && ~isempty(v.in) && exist(v.in,'file') && endsWith(lower(v.in),'.stl');
            if haveSTL
                ok = true; V = []; F = [];
                try
                    TR = stlread(v.in); V = TR.Points; F = TR.ConnectivityList;
                catch
                    ok = false;
                end
                if ok && ~isempty(V)
                    Vr = V * app.rotPreset(v.rot)';
                    patch(ax,'Faces',F,'Vertices',Vr,'FaceColor',[0.62 0.74 0.90], ...
                        'EdgeColor','none','FaceAlpha',0.92);
                    try; camlight(ax,'headlight'); lighting(ax,'gouraud'); material(ax,'dull'); catch; end
                    mn = min(Vr,[],1); mx = max(Vr,[],1); dim = mx - mn;
                    app.drawTriad(ax, mn - 0.06*max(dim), 0.32*max(dim));
                    info = sprintf('bbox = %.1f × %.1f × %.1f mm   |   Z: %.1f → %.1f', dim(1),dim(2),dim(3), mn(3),mx(3));
                    title(ax, sprintf('rotate-axis = %s   (让承载/最长方向竖直, 底面落在 Z 最小处)', v.rot));
                else
                    app.drawTriad(ax,[0 0 0],1);
                    info = 'STL 读取失败：需 MATLAB R2018b+ 的 stlread，或文件无效。';
                    title(ax, info);
                end
            else
                app.drawTriad(ax,[0 0 0],1);
                if strcmpi(v.src,'from-topo')
                    title(ax,'from-topo（.mat 密度场）无 STL 几何可预览；坐标轴仅作朝向示意。');
                else
                    title(ax,'选择 from-stl 并“浏览…”一个 .stl 后，这里显示模型 + 坐标轴。');
                end
            end
            hold(ax,'off');
            xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z (up)');
            axis(ax,'equal'); grid(ax,'on'); view(ax,[-37.5 25]);
            try; rotate3d(ax,'on'); catch; end
            if ~isempty(app.VoxInfo) && isvalid(app.VoxInfo); app.VoxInfo.Text = info; end
        end

        function drawTriad(~, ax, org, L)
            if ~isfinite(L) || L <= 0; L = 1; end
            o = org(:)';
            quiver3(ax,o(1),o(2),o(3), L,0,0, 0,'Color',[0.86 0.16 0.16],'LineWidth',2.2,'MaxHeadSize',0.6);
            quiver3(ax,o(1),o(2),o(3), 0,L,0, 0,'Color',[0.16 0.62 0.24],'LineWidth',2.2,'MaxHeadSize',0.6);
            quiver3(ax,o(1),o(2),o(3), 0,0,L, 0,'Color',[0.16 0.36 0.86],'LineWidth',2.2,'MaxHeadSize',0.6);
            text(ax,o(1)+L*1.14,o(2),o(3),'X','Color',[0.86 0.16 0.16],'FontWeight','bold');
            text(ax,o(1),o(2)+L*1.14,o(3),'Y','Color',[0.16 0.62 0.24],'FontWeight','bold');
            text(ax,o(1),o(2),o(3)+L*1.14,'Z','Color',[0.16 0.36 0.86],'FontWeight','bold');
        end

        function R = rotPreset(~, name)
            % Match voxelize.py _ROTATION_PRESETS (trimesh rotation_matrix(angle, axis)).
            P = pi; c = @(a)cos(a); s = @(a)sin(a);
            switch name
                case 'y-to-z'; a=P/2;  R=[1 0 0;0 c(a) -s(a);0 s(a) c(a)];
                case 'z-to-y'; a=-P/2; R=[1 0 0;0 c(a) -s(a);0 s(a) c(a)];
                case 'x-to-z'; a=-P/2; R=[c(a) 0 s(a);0 1 0;-s(a) 0 c(a)];
                case 'z-to-x'; a=P/2;  R=[c(a) 0 s(a);0 1 0;-s(a) 0 c(a)];
                case 'x-to-y'; a=P/2;  R=[c(a) -s(a) 0;s(a) c(a) 0;0 0 1];
                case 'y-to-x'; a=-P/2; R=[c(a) -s(a) 0;s(a) c(a) 0;0 0 1];
                case 'flip-z'; a=P;    R=[1 0 0;0 c(a) -s(a);0 s(a) c(a)];
                case 'flip-x'; a=P;    R=[c(a) 0 s(a);0 1 0;-s(a) 0 c(a)];
                case 'flip-y'; a=P;    R=[c(a) -s(a) 0;s(a) c(a) 0;0 0 1];
                otherwise;     R=eye(3);
            end
        end

        % ---- generic editable-parameter system (stages with a .pspec) ----
        function buildParamWidgets(app, parent, s)
            ps = app.paramStateFor(s);
            spec = s.pspec; n = numel(spec); rows = max(1, ceil(n/2));
            gl = uigridlayout(parent,[rows 4]);
            gl.RowHeight = repmat({'fit'},1,rows);
            gl.ColumnWidth = {180,'1x',180,'1x'}; gl.Padding=[0 0 0 0]; gl.RowSpacing=4; gl.ColumnSpacing=6;
            w = struct();
            for k=1:n
                d = spec{k}; r = ceil(k/2); col0 = 1 + 2*mod(k-1,2);
                lb = uilabel(gl,'Text',d.label); lb.Layout.Row=r; lb.Layout.Column=col0;
                if isfield(d,'tip'); lb.Tooltip = d.tip; end
                val = ps.(d.name);
                switch d.type
                    case 'num'
                        wc = uieditfield(gl,'numeric','Value',val, ...
                            'ValueChangedFcn',@(~,~)app.syncParamState(s.key));
                    case 'bool'
                        wc = uidropdown(gl,'Items',{'true','false'},'Value',app.boolStr(val), ...
                            'ValueChangedFcn',@(~,~)app.syncParamState(s.key));
                    case 'enum'
                        wc = uidropdown(gl,'Items',d.choices,'Value',char(val), ...
                            'ValueChangedFcn',@(~,~)app.syncParamState(s.key));
                    otherwise
                        wc = uieditfield(gl,'text','Value',char(val), ...
                            'ValueChangedFcn',@(~,~)app.syncParamState(s.key));
                end
                wc.Layout.Row=r; wc.Layout.Column=col0+1;
                w.(d.name) = wc;
            end
            app.PW(s.key) = w;
        end

        function ps = paramStateFor(app, s)
            if isKey(app.PState, s.key); ps = app.PState(s.key); else; ps = struct(); end
            for k=1:numel(s.pspec)
                d = s.pspec{k};
                if ~isfield(ps, d.name); ps.(d.name) = d.default; end
            end
            app.PState(s.key) = ps;
        end

        function syncParamState(app, key)
            if ~isKey(app.PW,key) || ~isKey(app.PState,key); return; end
            w = app.PW(key); ps = app.PState(key); s = app.stageByKey(key);
            if isempty(s); return; end
            for k=1:numel(s.pspec)
                d = s.pspec{k}; nm = d.name;
                if ~isfield(w,nm) || ~isvalid(w.(nm)); continue; end
                switch d.type
                    case 'num';  ps.(nm) = w.(nm).Value;
                    case 'bool'; ps.(nm) = strcmp(w.(nm).Value,'true');
                    otherwise;   ps.(nm) = w.(nm).Value;
                end
            end
            app.PState(key) = ps;
        end

        function p = gatherParams(app, key)
            app.syncParamState(key);
            if isKey(app.PState,key); p = app.PState(key); else; p = struct(); end
        end

        function s = stageByKey(app, key)
            s = [];
            for i=1:numel(app.Stages)
                if strcmp(app.Stages(i).key, key); s = app.Stages(i); return; end
            end
        end

        function t = boolStr(~, v)
            if (islogical(v) && v) || (isnumeric(v) && any(v~=0)); t='true';
            elseif ischar(v); t=v;
            else; t='false'; end
        end

        function out = runScriptParams(app, scriptBase, params, where)
            % Byte-safe param injection: rewrite the param-assignment lines of a
            % clear-script into a temp copy (letter-leading name), then run it.
            d = app.dirFor(where);
            src = app.scriptFile(scriptBase);
            fid = fopen(src,'r'); if fid<0; error('找不到脚本: %s', src); end
            raw = fread(fid,'*uint8')'; fclose(fid); txt = char(raw);
            fns = fieldnames(params);
            for i=1:numel(fns); txt = app.replaceAssign(txt, fns{i}, params.(fns{i})); end
            tmp = fullfile(d, ['cfrc_run_' scriptBase '.m']);   % MUST start with a letter
            fid = fopen(tmp,'w'); fwrite(fid, uint8(txt)); fclose(fid);
            oldHV = app.UIFigure.HandleVisibility; old = cd(d);
            cu1 = onCleanup(@()app.restoreHV(oldHV)); %#ok<NASGU>
            cu2 = onCleanup(@()cd(old));               %#ok<NASGU>
            app.UIFigure.HandleVisibility = 'off';
            assignin('base','CFRC__sf__', tmp);
            out = evalin('base', 'evalc(''run(CFRC__sf__)'')');
        end

        function txt = replaceAssign(~, txt, name, val)
            if islogical(val); lit = mat2str(val);
            elseif isnumeric(val); lit = num2str(val,'%.10g');
            else; lit = ['''' char(val) '''']; end
            pat = ['(?m)^([ \t]*)' regexptranslate('escape',name) '[ \t]*=[ \t]*[^;\r\n]*;'];
            rep = ['$1' name ' = ' lit ';'];
            txt = regexprep(txt, pat, rep, 'once');
        end

        function out = runPathAll(app)
            d = app.L.data; old = cd(d); cu = onCleanup(@()cd(old)); %#ok<NASGU>
            jobs = { ...
              'all_layers_path_generation_v6(''slice_results_refined_latest.mat'',''all_layers_paths_only_v3.mat'',''all_layers_paths_only_v3_full.mat'')','mine_stream'; ...
              'path_generation_offset_only(''slice_results_refined_latest.mat'',''all_layers_paths_only_mine_offset.mat'')','mine_offset'; ...
              'all_layers_path_generation_v6(''slice_results_refined_latest_PLANAR.mat'',''all_layers_paths_only_planar_stream.mat'',''all_layers_paths_only_planar_stream_full.mat'')','planar_stream'; ...
              'path_generation_offset_only(''slice_results_refined_latest_PLANAR.mat'',''all_layers_paths_only_planar_offset.mat'')','planar_offset'};
            buf = '';
            for k = 1:size(jobs,1)
                app.setBusy(sprintf('C 一键 %d/4: %s …', k, jobs{k,2}));
                app.appendLog(sprintf('\n--- C %d/4: %s ---', k, jobs{k,2}));
                try
                    o = evalc(jobs{k,1});
                    buf = [buf sprintf('\n[%s]\n', jobs{k,2}) o]; %#ok<AGROW>
                    app.appendLog(o);
                catch e
                    buf = [buf sprintf('\n[%s] ERROR: %s\n', jobs{k,2}, e.message)]; %#ok<AGROW>
                    app.appendLog(sprintf('[%s] ERROR: %s', jobs{k,2}, e.message));
                end
                app.refreshStatus();
            end
            out = buf;
        end

        function out = runAbaqusExtract(app)
            p = app.gatherParams('abaqus_stress');
            py = app.pyFile('abaqus_odb_to_mat');
            cmd = sprintf('abaqus python "%s" --odb "%s" --npz "%s" --output "%s" --principal %s --frame %d', ...
                py, p.odb, p.npz, p.output, p.principal, round(p.frame));
            out = app.runPython(cmd, 'data');
        end

        function openFolder(app, p)
            if ~exist(p,'dir'); mkdir(p); end
            if ispc; winopen(p);
            else; app.appendLog(['打开目录: ' p]); end
        end

        function setBusy(app, msg)
            if isempty(msg); app.StatusBar.Text = sprintf('就绪 Ready   |   cwd = %s', app.L.data);
            else; app.StatusBar.Text = msg; end
            drawnow;
        end

        function appendLog(app, txt)
            if isempty(txt); return; end
            newl = strsplit(char(txt), newline);
            app.LogLines = [app.LogLines, newl];
            if numel(app.LogLines) > 600
                app.LogLines = app.LogLines(end-599:end);
            end
            app.LogArea.Value = app.LogLines;
            try; scroll(app.LogArea,'bottom'); catch; end
            % persist
            try
                if ~exist(app.L.logs,'dir'); mkdir(app.L.logs); end
                fid = fopen(fullfile(app.L.logs,'console.log'),'a');
                if fid>0; fprintf(fid,'%s\n', char(txt)); fclose(fid); end
            catch; end
            drawnow limitrate;
        end

        function showHelp(app)
            msg = {
              'CFRC 管线控制台 使用说明：'
              ''
              '• 左侧 = 全部流程阶段，圆点颜色为状态：'
              '    灰=缺输入  橙=可运行(下一步)  绿=完成  黄=输入更新需重跑  红=出错  紫=Abaqus手动'
              '• 点阶段名看详情；点 Run 运行单个阶段；顶部“自动串联”连续跑 MATLAB 段直到遇到手动步骤。'
              '• 顶部“下一步 NEXT”始终提示该做什么。'
              ''
              ['• 脚本: ' app.L.scripts]
              ['• 函数: ' app.L.functions]
              ['• 可视化: ' app.L.viz]
              ['• 数据: ' app.L.data '   (运行时的工作目录)']
              ['• 输出: ' app.L.output]
              ''
              '注意：Abaqus 应力分析与 4-way 作业为 CAE 内手动步骤，本控制台给出命令清单。'
            };
            app.showText('帮助 Help', msg);
        end

        function showText(app, ttl, lines)
            f = uifigure('Name',ttl,'Position',[100 100 720 460]);
            gl = uigridlayout(f,[1 1]); gl.Padding=[10 10 10 10];
            uitextarea(gl,'Editable','off','FontName','Consolas','Value',lines);
        end

        function v = wrapLines(~, s); v = strsplit(char(s), newline); end
    end

    % ------------------------------------------------------- presentation maps
    methods (Access = private, Static)
        function c = colorFor(st)
            switch st
                case 'done';    c = [0.18 0.65 0.33];
                case 'ready';   c = [0.95 0.61 0.07];
                case 'stale';   c = [0.85 0.72 0.10];
                case 'blocked'; c = [0.60 0.60 0.62];
                case 'error';   c = [0.86 0.21 0.27];
                case 'manual';  c = [0.49 0.36 0.80];
                otherwise;      c = [0.6 0.6 0.6];
            end
        end
        function t = statusText(st)
            m = struct('done','完成 done','ready','可运行 READY(下一步)', ...
                'stale','需重跑 stale','blocked','缺输入 blocked', ...
                'error','出错 error','manual','手动 Abaqus');
            if isfield(m,st); t = m.(st); else; t = st; end
        end
        function t = kindText(k)
            switch k
                case 'func';     t = 'MATLAB 函数';
                case 'script';   t = 'MATLAB 脚本';
                case 'voxelize'; t = 'Python (voxelize)';
                case 'export4';  t = 'MATLAB ×4 导出';
                case 'extract';  t = 'Abaqus 提取';
                case 'manual';   t = 'Abaqus 手动';
                otherwise;       t = k;
            end
        end
    end

    % --------------------------------------------------------- stage catalogue
    methods (Access = private)
        function S = defineStages(app)
            L = app.L;
            mk = @(key,name,grp,kind,cmd,inputs,outputs,iodir,next,help,viz) struct( ...
                'key',key,'name',name,'grp',grp,'kind',kind,'cmd',cmd, ...
                'inputs',{inputs},'outputs',{outputs},'iodir',iodir, ...
                'next',next,'help',help,'viz',{viz});

            S = mk('voxelize','① 体素化 Voxelize','A 前处理 / Pre-process','voxelize','', ...
                {}, {'voxel_grid.inp','voxel_grid.npz'}, 'data', ...
                '在 Abaqus 中导入 voxel_grid.inp 并加载边界条件', ...
                ['把 STL 或 SIMP 拓扑 .mat 转成 Abaqus 体素网格 (voxel_grid.inp + .npz)。' newline ...
                 '选择来源、输入文件与体素尺寸后点运行 (调用 python/voxelize.py)。'], {});

            S(end+1) = mk('abaqus_stress','② Abaqus 应力分析 + 提取','A 前处理 / Pre-process','abaqus_b', ...
                ['STEP B — 在 Abaqus/CAE 中手动完成：' newline ...
                 '1) Import  voxel_grid.inp' newline ...
                 '2) 施加 BC / Load / Step / Output' newline ...
                 '3) 提交 job，得到 job.odb' newline ...
                 '4) 回收应力 (cmd):' newline ...
                 '   abaqus python "' fullfile(L.python,'abaqus_odb_to_mat.py') '" \' newline ...
                 '        --odb job.odb --npz voxel_grid.npz --output topo_stress_result.mat'], ...
                {'voxel_grid.inp'}, {'topo_stress_result.mat'}, 'data', ...
                '体素细化 voxel_refinement_from_test', ...
                'Abaqus 线弹性分析得到每体素主应力方向，再用 abaqus_odb_to_mat.py 导出 topo_stress_result.mat。', {});

            S(end+1) = mk('voxel_refine','③ 体素细化 Refine','A 前处理 / Pre-process','script', ...
                'voxel_refinement_from_test', ...
                {'topo_stress_result.mat'}, {'voxel_refined_latest.mat'}, 'data', ...
                '参考曲面 generate_reference_surface', ...
                '把 topo_stress_result.mat 细化为 refined_data.grid_data 并预算方向向量 uu/vv/ww。', ...
                {{'细化场可视化','visualize_refinement_from_test_data'}});

            S(end+1) = mk('ref_surface','④ 参考曲面 Reference Surface','B 切片 / Slicing','script', ...
                'generate_reference_surface', ...
                {'voxel_refined_latest.mat'}, {'Pre_surface.mat'}, 'data', ...
                '曲面切片 slice_refined_model_v6', ...
                '基于中面(中面)用基函数+优化拟合参考曲面，写入 Pre_surface.mat。', ...
                {{'曲面诊断图','plot_reference_surface_diagram'}});

            S(end+1) = mk('slice_curved','⑤ 曲面切片 Curved slice','B 切片 / Slicing','func', ...
                'slice_refined_model_v6(''voxel_refined_latest.mat'',''slice_results_refined_latest.mat'')', ...
                {'Pre_surface.mat','voxel_refined_latest.mat'}, {'slice_results_refined_latest.mat'}, 'data', ...
                '平面切片(对照) generate_planar_slicing', ...
                '主力曲面切片器 v6：解析梯度 Z-only 偏移，无折叠。输出 slice_results_refined_latest.mat。', ...
                {{'切片结果可视化','visualize_slicing_results'}});

            S(end+1) = mk('slice_planar','⑥ 平面切片 Planar (baseline)','B 切片 / Slicing','func', ...
                'generate_planar_slicing(''voxel_refined_latest.mat'',''slice_results_refined_latest_PLANAR.mat'')', ...
                {'voxel_refined_latest.mat'}, {'slice_results_refined_latest_PLANAR.mat'}, 'data', ...
                '路径规划 (4 个 config)', ...
                '沿 Z 等高的平面切片，作为对照基线。', {});

            S(end+1) = mk('path_all','▶▶ 一键生成全部路径 (4 组)','C 路径 / Path planning','pathall','', ...
                {'slice_results_refined_latest.mat','slice_results_refined_latest_PLANAR.mat'}, ...
                {'all_layers_paths_only_v3.mat','all_layers_paths_only_mine_offset.mat', ...
                 'all_layers_paths_only_planar_stream.mat','all_layers_paths_only_planar_offset.mat'}, 'data', ...
                'C 全部完成 → 校验 verify', ...
                ['一次跑完 C 阶段全部 4 组路径：mine_stream / mine_offset / planar_stream / planar_offset。' newline ...
                 '需先完成 ⑤ 曲面切片 与 ⑥ 平面切片。'], {});

            S(end+1) = mk('path_ms','⑦ 路径 mine_stream','C 路径 / Path planning','func', ...
                'all_layers_path_generation_v6(''slice_results_refined_latest.mat'',''all_layers_paths_only_v3.mat'',''all_layers_paths_only_v3_full.mat'')', ...
                {'slice_results_refined_latest.mat'}, {'all_layers_paths_only_v3.mat'}, 'data', ...
                'mine_offset 路径', ...
                '曲面切片 + 应力主方向流线 (stream) 路径。', ...
                {{'碳纤维路径','Path_show_carbon'}});

            S(end+1) = mk('path_mo','⑧ 路径 mine_offset','C 路径 / Path planning','func', ...
                'path_generation_offset_only(''slice_results_refined_latest.mat'',''all_layers_paths_only_mine_offset.mat'')', ...
                {'slice_results_refined_latest.mat'}, {'all_layers_paths_only_mine_offset.mat'}, 'data', ...
                'planar_stream 路径', ...
                '曲面切片 + 轮廓等距偏移 (offset) 路径。', {});

            S(end+1) = mk('path_ps','⑨ 路径 planar_stream','C 路径 / Path planning','func', ...
                'all_layers_path_generation_v6(''slice_results_refined_latest_PLANAR.mat'',''all_layers_paths_only_planar_stream.mat'',''all_layers_paths_only_planar_stream_full.mat'')', ...
                {'slice_results_refined_latest_PLANAR.mat'}, {'all_layers_paths_only_planar_stream.mat'}, 'data', ...
                'planar_offset 路径', ...
                '平面切片 + stream 路径 (对照)。', {});

            S(end+1) = mk('path_po','⑩ 路径 planar_offset','C 路径 / Path planning','func', ...
                'path_generation_offset_only(''slice_results_refined_latest_PLANAR.mat'',''all_layers_paths_only_planar_offset.mat'')', ...
                {'slice_results_refined_latest_PLANAR.mat'}, {'all_layers_paths_only_planar_offset.mat'}, 'data', ...
                '校验 verify_outputs', ...
                '平面切片 + offset 路径 (基线)。', {});

            S(end+1) = mk('verify','⑪ 校验 4 组产物 Verify','D FEA 对比 / Comparison','script', ...
                'verify_outputs', ...
                {'all_layers_paths_only_v3.mat','all_layers_paths_only_mine_offset.mat', ...
                 'all_layers_paths_only_planar_stream.mat','all_layers_paths_only_planar_offset.mat'}, ...
                {}, 'data', '导出到 FEA export_paths_to_fea', ...
                '检查 4 个路径 .mat 是否齐全、层数一致。', {});

            S(end+1) = mk('export','⑫ 导出到 FEA Export (×4)','D FEA 对比 / Comparison','export4','', ...
                {'all_layers_paths_only_v3.mat'}, ...
                {'mine_stream/beam_paths','planar_offset/beam_paths'}, 'fea', ...
                'Abaqus 4-way 作业 (手动)', ...
                ['把 4 组路径 + host 网格导出到 ' L.fea '\<cfg>\beam_paths\，' newline ...
                 '并把 Abaqus/对比 helper 脚本一并复制到该目录 (temp 路径为 ASCII, 供 Abaqus 使用)。' newline ...
                 '逐个调用 export_paths_to_fea。'], {});

            S(end+1) = mk('abaqus_jobs','⑬ Abaqus 4-way 作业 (手动)','D FEA 对比 / Comparison','manual', ...
                ['STEP — 在 Abaqus/CAE Python 命令行：' newline ...
                 '>>> execfile(''' fullfile(L.fea,'abaqus_cfrc_compare.py') ''')' newline ...
                 '>>> step1_build_template()      % 首次：手动加 Cload/BC/Step 后存 template.cae' newline ...
                 '>>> run_with_auto_retry([''mine_stream'',''mine_offset'',''planar_stream'',''planar_offset''])' newline ...
                 '>>> dump_blacklist()' newline ...
                 ['（abaqus_cfrc_compare.py 已由 ⑫ 自动复制到 ' L.fea '；若没有, 请从 python/ 手动复制过去）']], ...
                {}, {}, 'fea', '提取 ODB extract_fea_results', ...
                '在 Abaqus 内跑 4 个嵌入梁作业并自动重试，得到 4 个 .odb。', {});

            S(end+1) = mk('extract','⑭ 提取 ODB → CSV','D FEA 对比 / Comparison','extract', ...
                'abaqus cae noGUI=extract_fea_results.py', ...
                {}, {'results/mine_stream_time_history.csv'}, 'fea', ...
                '对比 run_compare', ...
                ['在 ' L.fea ' 下运行 abaqus cae noGUI=extract_fea_results.py，' newline ...
                 '把每个 config 的 ODB 提取成 results/<cfg>_time_history.csv。需 abaqus 在 PATH。'], {});

            S(end+1) = mk('compare','⑮ 刚度对比 Compare K','D FEA 对比 / Comparison','func', ...
                'run_compare(''all'')', ...
                {'results/mine_stream_time_history.csv'}, {'results/comparison_table.md'}, 'fea', ...
                '完成 / 路径统计', ...
                '4-way 刚度 K、F-U 曲线、雷达图 (compare_fea_results)。', {});

            S(end+1) = mk('pathstats','⑯ 路径几何统计 Path stats','D FEA 对比 / Comparison','func', ...
                sprintf('compute_path_statistics(''stress_file'',''voxel_refined_latest.mat'',''output_dir'',''%s'')', ...
                        fullfile(L.figures,'path_stats')), ...
                {'all_layers_paths_only_v3.mat','voxel_refined_latest.mat'}, {}, 'data', ...
                '全部完成 ✓', ...
                '路径长度 / 平滑段 / 路径-应力与曲面-应力对齐度统计图，输出到 output/figures/path_stats/。', {});

            % ---- input-dir overrides (stage whose inputs live elsewhere than its iodir) ----
            % ⑫ export reads the path .mat from data/ but writes outputs into fea/.
            % Without this its input-existence check would look in fea/ and the lamp
            % would wrongly show 'blocked'.
            [S.indir] = deal('');
            ke = find(strcmp({S.key},'export'),1); if ~isempty(ke); S(ke).indir = 'data'; end

            % ---- editable parameter specs (attached by key; injected at run time) ----
            [S.pspec] = deal({});
            refSpec = { ...
                struct('name','ELEM_SIZE','label','体素边长 ELEM_SIZE (mm)','type','num','default',2,'tip','体素物理边长, 放大整体尺寸'), ...
                struct('name','REFINE_FACTOR','label','细化倍数 REFINE_FACTOR','type','num','default',2,'tip','应力场采样加密倍数 (不改物理尺寸)'), ...
                struct('name','LAYER_HEIGHT_MODE','label','层高模式 LAYER_HEIGHT_MODE','type','enum','choices',{{'decouple','bind'}},'default','decouple','tip','decouple=固定层高(推荐); bind=随网格'), ...
                struct('name','TARGET_LAYER_HEIGHT','label','目标层高 TARGET_LAYER_HEIGHT (mm)','type','num','default',0.25,'tip','仅 decouple 模式生效'), ...
                struct('name','INTERP_METHOD','label','插值 INTERP_METHOD','type','enum','choices',{{'linear','cubic'}},'default','linear','tip','体素插值方法'), ...
                struct('name','USE_SMOOTHING','label','3D 平滑 USE_SMOOTHING','type','bool','default',true,'tip','是否做 3D 高斯平滑'), ...
                struct('name','SMOOTH_SIGMA','label','平滑 sigma SMOOTH_SIGMA','type','num','default',0.5,'tip','高斯平滑标准差 (体素单位)'), ...
                struct('name','DENSITY_THRESHOLD','label','密度阈值 DENSITY_THRESHOLD','type','num','default',0.5,'tip','低密度体素截断 [0,1]') };
            bSpec = { ...
                struct('name','odb','label','ODB 文件 --odb','type','text','default','job.odb','tip','Abaqus 求解得到的 .odb'), ...
                struct('name','npz','label','NPZ 文件 --npz','type','text','default','voxel_grid.npz','tip','体素化产生的 .npz'), ...
                struct('name','output','label','输出 --output','type','text','default','topo_stress_result.mat','tip','导出的应力 .mat (落在 data/)'), ...
                struct('name','principal','label','主应力 --principal','type','enum','choices',{{'max','min'}},'default','max','tip','取最大/最小主应力方向'), ...
                struct('name','frame','label','帧 --frame','type','num','default',-1,'tip','ODB 帧号, -1 = 最后一帧') };
            ki = find(strcmp({S.key},'voxel_refine'),1);  if ~isempty(ki); S(ki).pspec = refSpec; end
            kb = find(strcmp({S.key},'abaqus_stress'),1); if ~isempty(kb); S(kb).pspec = bSpec;   end
        end
    end
end
