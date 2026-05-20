function bc_selector_gui()
%% bc_selector_gui  -  Interactive BC picker for CFRC reference surface
%
%   用于在 voxel_refined_latest.mat 体素模型上交互式选取
%   固定约束点 BC_FIXED 和载荷点 BC_LOAD。
%
%   用法：
%       >> bc_selector_gui
%
%   功能：
%       1. 加载 .mat 文件，3D 可视化所有有效体素（按密度着色）
%       2. 密度阈值滑块实时过滤显示
%       3. 两种选择方式：
%           - 框选(brush)  : 拖动鼠标框选屏幕投影内的多个点
%           - 点选(pick)   : 点击视图，自动捕捉射线最近的体素
%       4. 两种选择类型：Fixed / Load （可切换）
%       5. 撤销 / 清除 / 视角快捷
%       6. 导出：复制可粘贴的代码到剪贴板，或保存 BC_picked.mat
%
%   说明：
%       依赖与 generate_reference_surface.m 相同的数据结构
%       refined_data.grid_data(i,j,k).{x,y,z,xPhys}
%       refined_data.valid_grid_mask
%       refined_data.grid_size.{nelx,nely,nelz}

% ============================================================
%  Shared variables (nested-function scope)
% ============================================================
coords_all  = [];
density_all = [];
ijk_all     = [];
eff_idx     = [];
fixed_idx   = [];
load_idx    = [];
boost_radius= 5.0;          % 与主脚本 BC_BOOST_RADIUS 同步显示
threshold   = 0.5;
mode_str    = 'fixed';
method_str  = 'box';
history     = {};
h_scatter   = [];
h_fixed_pts = [];
h_load_pts  = [];
h_boost     = [];

% ============================================================
%  Figure + Axes
% ============================================================
fig = figure('Name','BC Selector  -  CFRC Reference Surface', ...
    'Position',[80 80 1450 820], ...
    'NumberTitle','off','Color',[0.96 0.96 0.97], ...
    'MenuBar','figure','ToolBar','figure');

ax = axes('Parent',fig, ...
    'Units','pixels','Position',[50 70 920 720], ...
    'Color','w');
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
grid(ax,'on'); axis(ax,'equal'); view(ax,45,30);
rotate3d(ax,'on');
title(ax,'请先点击右侧"加载 .mat 数据"');

% ============================================================
%  Control Panel
% ============================================================
panel = uipanel(fig,'Title','控制面板','FontSize',11,'FontWeight','bold', ...
    'Units','pixels','Position',[990 70 440 720], ...
    'BackgroundColor',[0.98 0.98 0.99]);

%% -- Row 1: load data
uicontrol(panel,'Style','pushbutton','String','加载 .mat 数据', ...
    'Position',[15 660 150 32],'FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.85 0.92 1], 'Callback',@(~,~) load_data());

h_file = uicontrol(panel,'Style','text','String','(尚未加载)', ...
    'Position',[175 660 245 30],'FontSize',9, ...
    'HorizontalAlignment','left','ForegroundColor',[0.4 0.4 0.4], ...
    'BackgroundColor',[0.98 0.98 0.99]);

%% -- Row 2: density threshold slider
uicontrol(panel,'Style','text','String','密度阈值:', ...
    'Position',[15 620 80 22],'FontSize',10,'HorizontalAlignment','left', ...
    'BackgroundColor',[0.98 0.98 0.99]);
h_thresh = uicontrol(panel,'Style','slider','Min',0,'Max',1,'Value',0.5, ...
    'Position',[100 622 250 20], 'SliderStep',[0.02 0.1], ...
    'Callback',@(~,~) update_threshold());
h_thresh_val = uicontrol(panel,'Style','text','String','0.50', ...
    'Position',[360 620 60 22],'FontSize',10, ...
    'HorizontalAlignment','left','BackgroundColor',[0.98 0.98 0.99]);

%% -- Row 3: BC boost radius (display + sphere visualization)
uicontrol(panel,'Style','text','String','Boost 半径:', ...
    'Position',[15 585 80 22],'FontSize',10,'HorizontalAlignment','left', ...
    'BackgroundColor',[0.98 0.98 0.99]);
h_boost_edit = uicontrol(panel,'Style','edit','String','5.0', ...
    'Position',[100 587 60 22],'FontSize',10, ...
    'Callback',@(~,~) update_boost_radius());
uicontrol(panel,'Style','text','String','mm  (用于显示影响半球)', ...
    'Position',[165 585 250 22],'FontSize',9,'HorizontalAlignment','left', ...
    'BackgroundColor',[0.98 0.98 0.99]);

%% -- Row 4: Mode (Fixed / Load)
uicontrol(panel,'Style','text','String','选择类型:', ...
    'Position',[15 545 80 22],'FontSize',10,'FontWeight','bold', ...
    'HorizontalAlignment','left','BackgroundColor',[0.98 0.98 0.99]);
bg_mode = uibuttongroup(panel,'Units','pixels', ...
    'Position',[100 538 320 32],'BorderType','none', ...
    'BackgroundColor',[0.98 0.98 0.99], ...
    'SelectionChangedFcn',@(~,e) update_mode(e));
uicontrol(bg_mode,'Style','radiobutton','String','Fixed (固定 / 红色)', ...
    'Position',[5 5 160 22],'Tag','fixed','FontSize',10, ...
    'BackgroundColor',[0.98 0.98 0.99]);
uicontrol(bg_mode,'Style','radiobutton','String','Load (载荷 / 蓝色)', ...
    'Position',[170 5 150 22],'Tag','load','FontSize',10, ...
    'BackgroundColor',[0.98 0.98 0.99]);

%% -- Row 5: Method (Brush / Pick)
uicontrol(panel,'Style','text','String','选择方式:', ...
    'Position',[15 510 80 22],'FontSize',10,'FontWeight','bold', ...
    'HorizontalAlignment','left','BackgroundColor',[0.98 0.98 0.99]);
bg_method = uibuttongroup(panel,'Units','pixels', ...
    'Position',[100 502 320 32],'BorderType','none', ...
    'BackgroundColor',[0.98 0.98 0.99], ...
    'SelectionChangedFcn',@(~,e) update_method(e));
uicontrol(bg_method,'Style','radiobutton','String','框选 (brush)', ...
    'Position',[5 5 130 22],'Tag','box','FontSize',10, ...
    'BackgroundColor',[0.98 0.98 0.99]);
uicontrol(bg_method,'Style','radiobutton','String','点选 (最近射线)', ...
    'Position',[170 5 150 22],'Tag','pick','FontSize',10, ...
    'BackgroundColor',[0.98 0.98 0.99]);

%% -- Row 6: Start + confirm + undo
uicontrol(panel,'Style','pushbutton', ...
    'String','>> 开始选择 (按当前类型+方式)', ...
    'Position',[15 455 405 38],'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.82 0.95 0.82], ...
    'Callback',@(~,~) start_selection());

uicontrol(panel,'Style','pushbutton','String','[确认] 提交框选', ...
    'Position',[15 410 195 36],'FontSize',10, ...
    'BackgroundColor',[0.7 0.92 0.7], ...
    'Callback',@(~,~) confirm_brush());
uicontrol(panel,'Style','pushbutton','String','[撤销] 上一步', ...
    'Position',[225 410 195 36],'FontSize',10, ...
    'BackgroundColor',[1.0 0.92 0.7], ...
    'Callback',@(~,~) undo_last());

%% -- Row 7: Clear / View
uicontrol(panel,'Style','pushbutton','String','清除全部 Fixed', ...
    'Position',[15 367 195 32],'FontSize',10, ...
    'BackgroundColor',[1.0 0.82 0.82], ...
    'Callback',@(~,~) clear_set('fixed'));
uicontrol(panel,'Style','pushbutton','String','清除全部 Load', ...
    'Position',[225 367 195 32],'FontSize',10, ...
    'BackgroundColor',[0.82 0.86 1.0], ...
    'Callback',@(~,~) clear_set('load'));

uicontrol(panel,'Style','text','String','视角:', ...
    'Position',[15 330 40 22],'FontSize',9, ...
    'HorizontalAlignment','left','BackgroundColor',[0.98 0.98 0.99]);
uicontrol(panel,'Style','pushbutton','String','3D','Position',[60 327 60 26], ...
    'FontSize',9,'Callback',@(~,~) view(ax,45,30));
uicontrol(panel,'Style','pushbutton','String','+X','Position',[125 327 50 26], ...
    'FontSize',9,'Callback',@(~,~) view(ax,90,0));
uicontrol(panel,'Style','pushbutton','String','+Y','Position',[180 327 50 26], ...
    'FontSize',9,'Callback',@(~,~) view(ax,0,0));
uicontrol(panel,'Style','pushbutton','String','+Z (top)','Position',[235 327 80 26], ...
    'FontSize',9,'Callback',@(~,~) view(ax,0,90));
uicontrol(panel,'Style','pushbutton','String','重置','Position',[320 327 100 26], ...
    'FontSize',9,'Callback',@(~,~) {view(ax,45,30); axis(ax,'equal'); axis(ax,'auto')});

%% -- Lists
uicontrol(panel,'Style','text','String','已选 Fixed (双击列表项删除):', ...
    'Position',[15 295 250 22],'FontSize',10,'FontWeight','bold', ...
    'ForegroundColor',[0.7 0 0], ...
    'HorizontalAlignment','left','BackgroundColor',[0.98 0.98 0.99]);
h_list_fixed = uicontrol(panel,'Style','listbox','String',{}, ...
    'Position',[15 200 405 92],'FontSize',9,'Max',1, ...
    'Callback',@(s,~) on_list_click(s,'fixed'));

uicontrol(panel,'Style','text','String','已选 Load:', ...
    'Position',[15 175 250 22],'FontSize',10,'FontWeight','bold', ...
    'ForegroundColor',[0 0 0.7], ...
    'HorizontalAlignment','left','BackgroundColor',[0.98 0.98 0.99]);
h_list_load = uicontrol(panel,'Style','listbox','String',{}, ...
    'Position',[15 80 405 92],'FontSize',9,'Max',1, ...
    'Callback',@(s,~) on_list_click(s,'load'));

%% -- Export
uicontrol(panel,'Style','pushbutton','String','复制为脚本代码 (剪贴板)', ...
    'Position',[15 30 195 38],'FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.82 0.9 1.0], ...
    'Callback',@(~,~) export_clipboard());
uicontrol(panel,'Style','pushbutton','String','保存 BC_picked.mat', ...
    'Position',[225 30 195 38],'FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.92 0.85 1.0], ...
    'Callback',@(~,~) export_mat());

% ============================================================
%  Status bar
% ============================================================
h_status = uicontrol(fig,'Style','text','String','就绪 - 请加载体素数据', ...
    'Position',[50 30 1380 28],'FontSize',10, ...
    'HorizontalAlignment','left','BackgroundColor',[0.96 0.96 0.97], ...
    'ForegroundColor',[0.15 0.15 0.55],'FontWeight','bold');

% ============================================================
%  Try auto-load 'voxel_refined_latest.mat' from pwd
% ============================================================
if exist('voxel_refined_latest.mat','file') == 2
    load_from_path(fullfile(pwd,'voxel_refined_latest.mat'));
end

% ============================================================
% ====================  NESTED FUNCTIONS  ====================
% ============================================================
    function load_data()
        [fname,pname] = uigetfile({'*.mat','MAT files (*.mat)'}, ...
            '选择体素数据文件','voxel_refined_latest.mat');
        if isequal(fname,0), return; end
        load_from_path(fullfile(pname,fname));
    end

    function load_from_path(full)
        set(h_status,'String',['加载中: ',full,' ...']); drawnow;
        try
            S = load(full);
            if ~isfield(S,'refined_data')
                error('文件中缺少 refined_data 字段');
            end
            R = S.refined_data;
            gd = R.grid_data; vm = R.valid_grid_mask;
            nelx = R.grid_size.nelx;
            nely = R.grid_size.nely;
            nelz = R.grid_size.nelz;
            n_all = sum(vm(:));
            coords_all  = zeros(n_all,3);
            density_all = zeros(n_all,1);
            ijk_all     = zeros(n_all,3);
            idx = 0;
            for i = 1:nelx
                for j = 1:nely
                    for k = 1:nelz
                        if vm(i,j,k)
                            idx = idx+1;
                            g = gd(i,j,k);
                            coords_all(idx,:)  = [g.x, g.y, g.z];
                            density_all(idx)   = g.xPhys;
                            ijk_all(idx,:)     = [i,j,k];
                        end
                    end
                end
            end
            fixed_idx = []; load_idx = []; history = {};
            [~,fn,ext] = fileparts(full);
            set(h_file,'String',[fn,ext]);
            set(h_status,'String',sprintf( ...
                '已加载 %d 个有效体素，密度范围 [%.2f, %.2f]', ...
                n_all, min(density_all), max(density_all)));
            refresh_plot(); refresh_lists();
        catch ME
            set(h_status,'String',['加载失败: ',ME.message]);
        end
    end

    function refresh_plot()
        if isempty(coords_all), return; end
        % 记录视角
        try, [az,el] = view(ax); catch, az=45; el=30; end
        cla(ax); hold(ax,'on');
        eff = density_all >= threshold;
        eff_idx = find(eff);
        coords = coords_all(eff,:);
        dens   = density_all(eff);
        if ~isempty(coords)
            h_scatter = scatter3(ax, coords(:,1),coords(:,2),coords(:,3), ...
                22, dens, 'filled', ...
                'MarkerEdgeColor','none','MarkerFaceAlpha',0.45);
            colormap(ax,parula); 
            cb = colorbar(ax); cb.Label.String = 'Density (xPhys)';
            caxis(ax,[0 1]);
        end
        % Fixed
        if ~isempty(fixed_idx)
            fc = coords_all(fixed_idx,:);
            h_fixed_pts = scatter3(ax, fc(:,1),fc(:,2),fc(:,3), ...
                160,'r','filled','MarkerEdgeColor','k','LineWidth',1.5);
        end
        % Load
        if ~isempty(load_idx)
            lc = coords_all(load_idx,:);
            h_load_pts = scatter3(ax, lc(:,1),lc(:,2),lc(:,3), ...
                160,'b','filled','MarkerEdgeColor','k','LineWidth',1.5);
        end
        % Boost spheres (semi-transparent)
        draw_boost_spheres();
        xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
        grid(ax,'on'); axis(ax,'equal');
        title(ax,sprintf( ...
            '体素模型  |  密度阈值 >= %.2f  |  显示 %d / %d  |  Fixed=%d, Load=%d', ...
            threshold, size(coords,1), size(coords_all,1), ...
            numel(fixed_idx), numel(load_idx)));
        view(ax,az,el);
        rotate3d(ax,'on');
    end

    function draw_boost_spheres()
        if isempty(coords_all), return; end
        all_bc = [coords_all(fixed_idx,:); coords_all(load_idx,:)];
        if isempty(all_bc), return; end
        [Xs,Ys,Zs] = sphere(20);
        for i = 1:size(all_bc,1)
            c = all_bc(i,:);
            surf(ax, Xs*boost_radius+c(1), Ys*boost_radius+c(2), ...
                Zs*boost_radius+c(3), ...
                'FaceAlpha',0.07,'FaceColor',[0.5 0.5 0.5], ...
                'EdgeColor','none','HandleVisibility','off');
        end
    end

    function refresh_lists()
        sf = cell(numel(fixed_idx),1);
        for i = 1:numel(fixed_idx)
            c = coords_all(fixed_idx(i),:);
            sf{i} = sprintf('%2d :   X=%.3f   Y=%.3f   Z=%.3f', ...
                i, c(1), c(2), c(3));
        end
        set(h_list_fixed,'String',sf,'Value',max(1,min(get(h_list_fixed,'Value'),numel(sf))));
        if isempty(sf), set(h_list_fixed,'Value',1); end

        sl = cell(numel(load_idx),1);
        for i = 1:numel(load_idx)
            c = coords_all(load_idx(i),:);
            sl{i} = sprintf('%2d :   X=%.3f   Y=%.3f   Z=%.3f', ...
                i, c(1), c(2), c(3));
        end
        set(h_list_load,'String',sl,'Value',max(1,min(get(h_list_load,'Value'),numel(sl))));
        if isempty(sl), set(h_list_load,'Value',1); end
    end

    function update_threshold()
        threshold = get(h_thresh,'Value');
        set(h_thresh_val,'String',sprintf('%.2f',threshold));
        refresh_plot();
    end

    function update_boost_radius()
        v = str2double(get(h_boost_edit,'String'));
        if isnan(v) || v < 0
            set(h_boost_edit,'String',sprintf('%.1f',boost_radius));
            return;
        end
        boost_radius = v;
        refresh_plot();
    end

    function update_mode(e)
        mode_str = get(e.NewValue,'Tag');
        set(h_status,'String',['当前选择类型: ',mode_str]);
    end

    function update_method(e)
        method_str = get(e.NewValue,'Tag');
        set(h_status,'String',['当前选择方式: ',method_str]);
    end

    function start_selection()
        if isempty(coords_all)
            set(h_status,'String','请先加载数据!'); return;
        end
        if strcmp(method_str,'box')
            % brush 模式
            try, brush(fig,'off'); end
            % 关掉 rotate3d 否则会和 brush 抢事件
            try, rotate3d(ax,'off'); end
            brush(fig,'on');
            set(h_status,'String', ...
                ['【框选】 拖动鼠标在视图中拉出矩形框选体素 (按屏幕投影), ', ...
                 '完成后点击下方"[确认] 提交框选"按钮。  当前类型: ', mode_str]);
        else
            % 点选模式
            try, brush(fig,'off'); end
            try, rotate3d(ax,'off'); end
            set(fig,'WindowButtonDownFcn',@(~,~) pick_point());
            set(h_status,'String', ...
                ['【点选】 单击视图，自动捕捉射线最近的体素加入 ', mode_str, ...
                 '。  完成后请切换至"框选"或按 Esc。']);
            % 允许 Esc 退出
            set(fig,'KeyPressFcn',@on_key);
        end
    end

    function on_key(~,evt)
        if strcmp(evt.Key,'escape')
            set(fig,'WindowButtonDownFcn','');
            set(fig,'KeyPressFcn','');
            rotate3d(ax,'on');
            set(h_status,'String','已退出点选模式');
        end
    end

    function confirm_brush()
        if isempty(h_scatter) || ~isvalid(h_scatter)
            set(h_status,'String','没有可用的散点图'); return;
        end
        brushed = get(h_scatter,'BrushData');
        if isempty(brushed) || ~any(brushed)
            set(h_status,'String','没有框选任何点 - 请先拖动鼠标框选'); return;
        end
        sel_local = find(brushed);            % 在当前显示点中的位置
        global_new = eff_idx(sel_local);      % 在 coords_all 中的位置
        % 与已有的去重（保留新加入的实际增量）
        if strcmp(mode_str,'fixed')
            existing = fixed_idx;
        else
            existing = load_idx;
        end
        added = setdiff(global_new, existing);
        if isempty(added)
            set(h_status,'String','框选的点已全部存在于当前类型 - 未添加');
            brush(fig,'off'); return;
        end
        history{end+1} = struct('mode',mode_str,'added',added(:)); %#ok<NASGU>
        if strcmp(mode_str,'fixed')
            fixed_idx = [fixed_idx(:); added(:)];
        else
            load_idx = [load_idx(:); added(:)];
        end
        brush(fig,'off');
        rotate3d(ax,'on');
        refresh_plot(); refresh_lists();
        set(h_status,'String',sprintf( ...
            '已添加 %d 个点到 %s （总计 Fixed=%d, Load=%d）', ...
            numel(added), mode_str, numel(fixed_idx), numel(load_idx)));
    end

    function pick_point()
        if isempty(coords_all), return; end
        cp = get(ax,'CurrentPoint');     % 2x3
        % 鼠标点必须落在 axes 区域
        pt_fig = get(fig,'CurrentPoint');
        ax_pos = get(ax,'Position');
        if pt_fig(1) < ax_pos(1) || pt_fig(1) > ax_pos(1)+ax_pos(3) || ...
           pt_fig(2) < ax_pos(2) || pt_fig(2) > ax_pos(2)+ax_pos(4)
            return;
        end
        ray0 = cp(1,:); ray1 = cp(2,:);
        rdir = ray1 - ray0; rdir = rdir/max(norm(rdir),1e-12);
        pts = coords_all(eff_idx,:);
        v = pts - ray0;
        cr = cross(v, repmat(rdir, size(v,1),1), 2);
        dist = sqrt(sum(cr.^2,2));
        [dmin, mi] = min(dist);
        gi = eff_idx(mi);
        if strcmp(mode_str,'fixed')
            if ismember(gi, fixed_idx)
                set(h_status,'String','该点已在 Fixed 列表中'); return;
            end
            history{end+1} = struct('mode','fixed','added',gi);
            fixed_idx = [fixed_idx; gi];
        else
            if ismember(gi, load_idx)
                set(h_status,'String','该点已在 Load 列表中'); return;
            end
            history{end+1} = struct('mode','load','added',gi);
            load_idx = [load_idx; gi];
        end
        c = coords_all(gi,:);
        refresh_plot(); refresh_lists();
        set(h_status,'String',sprintf( ...
            '点选 -> %s :  (%.3f, %.3f, %.3f)  射线距离=%.2f mm', ...
            mode_str, c(1),c(2),c(3), dmin));
    end

    function undo_last()
        if isempty(history)
            set(h_status,'String','没有可撤销的操作'); return;
        end
        last = history{end}; history(end) = [];
        if strcmp(last.mode,'fixed')
            fixed_idx = setdiff(fixed_idx, last.added, 'stable');
        else
            load_idx = setdiff(load_idx, last.added, 'stable');
        end
        refresh_plot(); refresh_lists();
        set(h_status,'String',sprintf('已撤销 %s 选择 %d 个点', ...
            last.mode, numel(last.added)));
    end

    function clear_set(which)
        if strcmp(which,'fixed')
            fixed_idx = [];
        else
            load_idx = [];
        end
        history = {};
        refresh_plot(); refresh_lists();
        set(h_status,'String',['已清除全部 ',which,' 点']);
    end

    function on_list_click(s,which)
        % 双击 -> 删除
        if strcmp(get(fig,'SelectionType'),'open')
            v = get(s,'Value');
            if strcmp(which,'fixed')
                if v < 1 || v > numel(fixed_idx), return; end
                fixed_idx(v) = [];
            else
                if v < 1 || v > numel(load_idx), return; end
                load_idx(v) = [];
            end
            refresh_plot(); refresh_lists();
            set(h_status,'String',['已从 ',which,' 列表删除第 ',num2str(v),' 项']);
        end
    end

    function export_clipboard()
        fc = coords_all(fixed_idx,:);
        lc = coords_all(load_idx,:);
        s = sprintf('%%%% ===== BC picked via bc_selector_gui (%s) =====\n', ...
            datestr(now,'yyyy-mm-dd HH:MM:SS'));
        s = [s, fmt_matrix('BC_FIXED', fc)];
        s = [s, fmt_matrix('BC_LOAD ', lc)];
        s = [s, sprintf('BC_BOOST_RADIUS = %.2f;\n', boost_radius)];
        try
            clipboard('copy', s);
        catch
            % 无显示环境，跳过
        end
        fprintf('\n%s\n', s);
        set(h_status,'String',sprintf( ...
            '已复制并打印至命令行：BC_FIXED=%d 行，BC_LOAD=%d 行', ...
            size(fc,1), size(lc,1)));
    end

    function out = fmt_matrix(name, M)
        if isempty(M)
            out = sprintf('%s = [];   %% (空 - 未选择)\n', strtrim(name));
            return;
        end
        out = sprintf('%s = [', strtrim(name));
        pad  = repmat(' ',1,numel(strtrim(name))+4);
        for r = 1:size(M,1)
            if r == 1
                out = [out, sprintf('%.4f, %.4f, %.4f', M(r,1),M(r,2),M(r,3))]; %#ok<AGROW>
            else
                out = [out, sprintf('; ...\n%s%.4f, %.4f, %.4f', ...
                    pad, M(r,1),M(r,2),M(r,3))]; %#ok<AGROW>
            end
        end
        out = [out, sprintf('];\n')];
    end

    function export_mat()
        BC_FIXED         = coords_all(fixed_idx,:); %#ok<NASGU>
        BC_LOAD          = coords_all(load_idx,:);  %#ok<NASGU>
        BC_BOOST_RADIUS  = boost_radius;            %#ok<NASGU>
        FIXED_IDX_GLOBAL = fixed_idx;               %#ok<NASGU>
        LOAD_IDX_GLOBAL  = load_idx;                %#ok<NASGU>
        [fn,pn] = uiputfile('BC_picked.mat','保存 BC 文件为');
        if isequal(fn,0), return; end
        full = fullfile(pn,fn);
        save(full, 'BC_FIXED','BC_LOAD','BC_BOOST_RADIUS', ...
                   'FIXED_IDX_GLOBAL','LOAD_IDX_GLOBAL');
        set(h_status,'String',['已保存到: ',full]);
    end
end
