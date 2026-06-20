function interactive_path_planning_v28()
%% ========================================
%% 交互式路径规划 V28
%% ========================================
% V28 vs V27:
%   [HOTFIX] V27 报错 "无法识别的字段名称 x_min_raw":
%     V14 时 compute_contours_streamlines 把 grid_bbox 替换为 prep.effective_bbox
%     (= grid_bbox ∩ layer_bbox). intersect_bboxes 输出的 bbox 不含 x_min_raw/y_min_raw
%     (只有 x_min/y_min/vox_dx 等). V27 在该位置访问 grid_bbox.x_min_raw 就崩了.
%     现在: 保留 orig_grid_bbox 副本, V27 的 bnd 坐标转换用 orig_grid_bbox.x_min_raw.
%     effective_bbox 仍用于其他裁剪 (xy 范围更紧).
%
% 使用方法:
%   >> interactive_path_planning_v28

clc;
warning('off', 'MATLAB:polyshape:repairedBySimplify');
warning('off', 'MATLAB:polyshape:boundary3Points');

fprintf('\n================================================================\n');
fprintf('  交互式路径规划 V28 - 启动\n');
fprintf('================================================================\n\n');

% ---------- 加载切片数据 ----------
slice_file = 'slice_results_refined_latest.mat';
if ~exist(slice_file, 'file')
    errordlg(sprintf('找不到 %s', slice_file), '数据缺失');
    return;
end
fprintf('[INIT] 加载切片数据...\n');
S = load(slice_file);
sr = S.slice_results;

% ---------- 初始化应用状态 ----------
app                  = struct();
app.sr               = sr;
app.surface_layers   = sr.surface_layers;
app.grid_data        = sr.grid_data;
app.valid_grid_mask  = sr.valid_grid_mask;
app.num_layers       = sr.statistics.num_layers;
app.cur              = 1;
fprintf('[INIT] 层数: %d\n', app.num_layers);

fprintf('[INIT] 构建结构遮罩...\n');
app.mask_poly = build_structure_mask(app.grid_data, app.valid_grid_mask);
app.has_mask  = area(app.mask_poly) > 0;
if app.has_mask
    fprintf('[INIT] 遮罩面积: %.2f\n', area(app.mask_poly));
end

if isfield(sr, 'z_height_field')
    app.zhf     = sr.z_height_field;
    app.has_zhf = true;
    fprintf('[INIT] Z 高度场: %dx%d\n', length(app.zhf.xx), length(app.zhf.yy));
else
    app.zhf     = [];
    app.has_zhf = false;
end
app.has_img = ~isempty(ver('images'));

% [V12] 计算完整网格 XYZ bbox (硬约束边界)
fprintf('[INIT] 计算完整网格 XYZ bbox...\n');
app.grid_bbox = compute_grid_bbox(app.grid_data);
fprintf('[INIT] grid_bbox: x[%.2f, %.2f]  y[%.2f, %.2f]  z[%.2f, %.2f]\n', ...
    app.grid_bbox.x_min, app.grid_bbox.x_max, ...
    app.grid_bbox.y_min, app.grid_bbox.y_max, ...
    app.grid_bbox.z_min, app.grid_bbox.z_max);
fprintf('[INIT] margin (半体素): dx=%.3f dy=%.3f dz=%.3f\n', ...
    app.grid_bbox.margin_x, app.grid_bbox.margin_y, app.grid_bbox.margin_z);

% [V24] 加载时只裁 XY (Z 不裁), 避免 F_z 插值丢点导致路径 z 跳跃尖刺
%       Z 维度的视觉裁剪在 mesh 渲染时单独做
fprintf('[INIT] 裁剪曲面数据到 grid_bbox + 2 体素 margin (仅 XY)...\n');
n_pts_before = 0; n_pts_after = 0;
for li_init = 1:app.num_layers
    ld = app.surface_layers{li_init};
    if isempty(ld.X_surf), continue; end
    n_pts_before = n_pts_before + numel(ld.X_surf);
    [Xc, Yc, Zc] = crop_surface_to_bbox(ld.X_surf, ld.Y_surf, ld.Z_surf, ...
        app.grid_bbox, 2, false);   % [V24] also_z = false
    if ~isempty(Xc)
        app.surface_layers{li_init}.X_surf = Xc;
        app.surface_layers{li_init}.Y_surf = Yc;
        app.surface_layers{li_init}.Z_surf = Zc;
    end
    n_pts_after = n_pts_after + numel(app.surface_layers{li_init}.X_surf);
end
if n_pts_before > 0
    fprintf('[INIT] 曲面顶点: %d -> %d (减少 %.1f%%)\n', ...
        n_pts_before, n_pts_after, 100*(1 - n_pts_after/n_pts_before));
end

% 每层独立参数 + 每层手动选择数据 + 每层中间结果 + 接受状态
defp                  = default_params();
% [集中化] 线宽默认值跟随管线: 从切片 slice_results.parameters.LINE_WIDTH 取,
%   与 all_layers_path_generation_v6 / path_generation_offset_only 同源 (回退 0.4).
%   用户仍可在 GUI 里手动改每层线宽; 这里只设"初始默认", 让交互工具与对比管线一致.
if isfield(sr, 'parameters') && isfield(sr.parameters, 'LINE_WIDTH')
    defp.offset_distance = sr.parameters.LINE_WIDTH;
    fprintf('[INIT] 线宽默认值继承自切片 LINE_WIDTH = %.3f mm\n', defp.offset_distance);
else
    fprintf('[INIT] 切片无 LINE_WIDTH, 线宽默认值回退 %.3f mm (建议重跑切片)\n', defp.offset_distance);
end
app.params_per_layer  = repmat({defp}, app.num_layers, 1);
app.manual_per_layer  = cell(app.num_layers, 1);   % 手动选择数据
app.layer_mode        = repmat({'auto'}, app.num_layers, 1);  % 'auto' or 'manual'
app.results_per_layer = cell(app.num_layers, 1);
app.accepted          = false(app.num_layers, 1);
% [V9] 每层 prep 缓存 (与参数无关的中间数据)
app.layer_cache       = cell(app.num_layers, 1);

% UI 句柄 + 3D 预览窗口句柄
app.h        = struct();
app.preview3d_fig = [];

% ---------- 创建 UI ----------
fig = uifigure('Name', 'Interactive Path Planning V28', ...
    'Position', [60 50 1620 920], ...
    'CloseRequestFcn', @(s,e) on_close(s));
fig.UserData = app;

build_ui(fig);

% 默认计算第一层
on_recompute(fig);

fprintf('[INIT] UI 就绪.\n\n');

end


%% ================================================================
%% 默认参数
%% ================================================================
function p = default_params()
    p = struct();
    p.offset_distance          = 0.4;   % [默认/回退] 线宽; 启动时由切片
                                         %   slice_results.parameters.LINE_WIDTH 覆盖 (见 INIT)
    p.max_iterations           = 30;
    p.min_path_length          = 2;
    p.volfrac                  = 0.5;
    p.do_streamlines           = false;   % [V9] 默认不算流线 (慢)
    p.filter_radius            = 5;
    p.filter_iterations        = 2;
    p.min_region_area          = 0.05;
    p.adaptive_outer_factor    = 0.05;
    p.min_contour_length_inner = 4;
    p.contour_dilate_pixels    = 3;
    p.contour_expand_ratio     = 0.6;
    p.use_z_burn               = true;
    p.z_margin_factor          = 0.5;
end


%% ================================================================
%% UI 构造
%% ================================================================
function build_ui(fig)
    app = fig.UserData;

    main = uigridlayout(fig, [1, 2]);
    main.ColumnWidth   = {360, '1x'};
    main.Padding       = [8 8 8 8];
    main.ColumnSpacing = 8;

    % -------- 左侧控制面板 (Scrollable) --------
    left = uipanel(main, 'Title', '阶段一: 逐层调参', ...
        'Scrollable', 'on');
    left.Layout.Column = 1;

    lg = uigridlayout(left, [12, 1]);
    lg.RowHeight = {40, 70, 50, 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 30, 32, 36};
    lg.Padding   = [8 8 8 8];
    lg.RowSpacing = 6;

    % --- Row 1: 当前层 ---
    p_nav = uipanel(lg, 'Title', sprintf('当前层 (共 %d 层)', app.num_layers));
    p_nav.Layout.Row = 1;
    nav_grid = uigridlayout(p_nav, [1, 2]);
    nav_grid.ColumnWidth = {'1x', 80};
    nav_grid.Padding = [4 4 4 4];

    app.h.layer_slider = uislider(nav_grid, ...
        'Limits', [1, app.num_layers], 'Value', 1, ...
        'MajorTicks', linspace(1, app.num_layers, min(6, app.num_layers)), ...
        'ValueChangingFcn', @(s,e) on_slider_changing(fig, e), ...
        'ValueChangedFcn',  @(s,e) on_slider_changed(fig));
    app.h.layer_spinner = uispinner(nav_grid, ...
        'Limits', [1, app.num_layers], 'Value', 1, 'Step', 1, ...
        'ValueChangedFcn', @(s,e) on_spinner_changed(fig));

    % --- Row 2: 进度条 ---
    p_prog = uipanel(lg, 'Title', '层进度 (绿=已接受 红=失败 蓝=待确认 灰=未处理)');
    p_prog.Layout.Row = 2;
    pg = uigridlayout(p_prog, [1, 1]);
    pg.Padding = [4 4 4 4];
    app.h.progress_ax = uiaxes(pg);
    app.h.progress_ax.Toolbar.Visible = 'off';
    app.h.progress_ax.XTick = []; app.h.progress_ax.YTick = [];
    app.h.progress_ax.XColor = 'none'; app.h.progress_ax.YColor = 'none';
    disableDefaultInteractivity(app.h.progress_ax);
    app.h.progress_ax.ButtonDownFcn = @(s,e) on_progress_click(fig, e);

    % --- Row 3: 模式切换 ---
    p_mode = uipanel(lg, 'Title', '本层模式');
    p_mode.Layout.Row = 3;
    mg = uigridlayout(p_mode, [1, 2]);
    mg.Padding = [4 4 4 4];

    app.h.btn_mode_auto = uibutton(mg, 'state', 'Text', '[ 自动 ] 算法识别轮廓', ...
        'Value', true, 'BackgroundColor', [0.7 0.85 1], ...
        'ValueChangedFcn', @(s,e) on_mode_change(fig, 'auto'));
    app.h.btn_mode_manual = uibutton(mg, 'state', 'Text', '[ 手动 ] 框选点定义轮廓', ...
        'Value', false, 'BackgroundColor', [0.95 0.95 0.95], ...
        'ValueChangedFcn', @(s,e) on_mode_change(fig, 'manual'));

    % --- Row 4: 参数面板 - 双模式切换 ---
    p_param_holder = uipanel(lg, 'Title', '轮廓识别 / 手动选择', 'BorderType', 'none');
    p_param_holder.Layout.Row = 4;
    ph_grid = uigridlayout(p_param_holder, [2, 1]);
    ph_grid.RowHeight = {'fit', 0};   % 默认: auto 可见, manual 折叠
    ph_grid.Padding = [0 0 0 0];
    app.h.ph_grid = ph_grid;

    % -- 4a: 自动模式参数面板 --
    app.h.panel_auto = uipanel(ph_grid, 'Title', '[ 自动 ] 轮廓识别参数', ...
        'BackgroundColor', [0.94 0.96 1]);
    app.h.panel_auto.Layout.Row = 1;
    cg = uigridlayout(app.h.panel_auto, [5, 2]);
    cg.ColumnWidth = {'1x', 90};
    cg.Padding = [6 6 6 6]; cg.RowSpacing = 4;
    uilabel(cg, 'Text', '膨胀像素 (dilate)');
    app.h.p_dilate = uispinner(cg, 'Limits', [0 15], 'Value', 3, 'Step', 1, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    uilabel(cg, 'Text', '外扩比 (expand_ratio)');
    app.h.p_expand = uispinner(cg, 'Limits', [0 3], 'Value', 0.6, 'Step', 0.1, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    uilabel(cg, 'Text', '内轮廓最小周长');
    app.h.p_min_inner = uispinner(cg, 'Limits', [0 100], 'Value', 4, 'Step', 1, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    app.h.p_use_zburn = uicheckbox(cg, 'Text', '启用 Z 越界烧入', 'Value', true, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    uilabel(cg, 'Text', '');
    uilabel(cg, 'Text', 'Z 余量因子');
    app.h.p_zmargin = uispinner(cg, 'Limits', [0 3], 'Value', 0.5, 'Step', 0.1, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));

    % -- 4b: 手动模式选择面板 --
    app.h.panel_manual = uipanel(ph_grid, 'Title', '[ 手动 ] 框选点定义轮廓', ...
        'BackgroundColor', [1 0.97 0.92], 'Visible', 'off');
    app.h.panel_manual.Layout.Row = 2;
    mng = uigridlayout(app.h.panel_manual, [11, 2]);
    mng.ColumnWidth = {'1x', '1x'};
    mng.RowHeight = {32, 32, 32, 32, 32, 32, 32, 'fit', 32, 32, 32};
    mng.Padding = [6 6 6 6]; mng.RowSpacing = 4;

    % Row1: 紧凑度参数
    uilabel(mng, 'Text', '紧凑度 (0=凸包松, 1=紧贴点)');
    app.h.p_shrink = uispinner(mng, 'Limits', [0 1], 'Value', 0.5, 'Step', 0.1, ...
        'Tooltip', '控制 boundary() 函数的 shrink factor');

    % Row1b: [V13] 聚类阈值倍数
    uilabel(mng, 'Text', '聚类距离倍数 (×体素间距)');
    app.h.p_cluster_factor = uispinner(mng, 'Limits', [0.5 100], ...
        'Value', 1.5, 'Step', 0.1, ...
        'Tooltip', ['框内点聚成连通簇的距离阈值 (倍体素间距). ' ...
                    '默认 1.5 让相邻栅格连通; 设很大 (如 100) 等效不聚类']);

    % Row2: 新建外/内轮廓
    uibutton(mng, 'Text', '[+ 新建外轮廓]', ...
        'BackgroundColor', [0.7 0.85 1], ...
        'ButtonPushedFcn', @(s,e) on_new_outer(fig));
    uibutton(mng, 'Text', '[+ 新建内轮廓]', ...
        'BackgroundColor', [1 0.8 0.8], ...
        'ButtonPushedFcn', @(s,e) on_new_inner(fig));

    % Row2b: [V12] 区域+孔双重角色
    btn_both = uibutton(mng, 'Text', '[+ 新建 区域+孔 (both)]', ...
        'BackgroundColor', [0.9 0.85 1], ...
        'Tooltip', ['此组既作独立小区域的外轮廓, 又作大区域的内轮廓 (挖孔). ' ...
                    '常用于"大区域中嵌套独立填充小区域"场景'], ...
        'ButtonPushedFcn', @(s,e) on_new_both(fig));
    btn_both.Layout.Column = [1 2];

    % Row3: 删除 / 清空
    uibutton(mng, 'Text', '[ 删除选中组 ]', ...
        'ButtonPushedFcn', @(s,e) on_delete_group(fig));
    uibutton(mng, 'Text', '[ 清空本层手动 ]', ...
        'ButtonPushedFcn', @(s,e) on_clear_manual(fig));

    % Row4: 重算所有组轮廓 (调紧凑度后用)
    btn_reshrink = uibutton(mng, 'Text', '[ 用当前紧凑度重算所有组 ]', ...
        'BackgroundColor', [1 0.95 0.7], ...
        'ButtonPushedFcn', @(s,e) on_reshrink_all(fig));
    btn_reshrink.Layout.Column = [1 2];

    % Row5: 列表
    uilabel(mng, 'Text', '已定义的轮廓组:');
    uilabel(mng, 'Text', '');

    app.h.list_groups = uilistbox(mng, 'Items', {}, ...
        'FontName', 'Consolas', 'FontSize', 10);
    app.h.list_groups.Layout.Column = [1 2];

    % Row7: 应用
    btn_apply = uibutton(mng, 'Text', '[ 应用手动轮廓 -> 重算 ]', ...
        'BackgroundColor', [0.7 0.95 0.7], 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @(s,e) on_apply_manual(fig));
    btn_apply.Layout.Column = [1 2];

    % Row8: 3D
    uibutton(mng, 'Text', '[ 打开 3D 预览 ]', ...
        'BackgroundColor', [0.9 0.85 1], ...
        'ButtonPushedFcn', @(s,e) on_open_3d_preview(fig));
    uibutton(mng, 'Text', '[ 同步 3D 预览 ]', ...
        'ButtonPushedFcn', @(s,e) refresh_3d_preview(fig));

    app.h.lbl_manual_status = uilabel(mng, 'Text', '未选择', ...
        'FontColor', [0.4 0.4 0.4]);
    % 状态标签放在 listbox 旁边

    % --- Row 5: 区域筛选 ---
    p_r = uipanel(lg, 'Title', '区域筛选 (大小区域取舍)');
    p_r.Layout.Row = 5;
    rg = uigridlayout(p_r, [2, 2]);
    rg.ColumnWidth = {'1x', 90};
    rg.Padding = [6 6 6 6]; rg.RowSpacing = 4;
    uilabel(rg, 'Text', '最小区域面积');
    app.h.p_min_area = uispinner(rg, 'Limits', [0 50], 'Value', 0.05, 'Step', 0.05, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    uilabel(rg, 'Text', '外轮廓自适应因子');
    app.h.p_outer_fac = uispinner(rg, 'Limits', [0 0.5], 'Value', 0.05, 'Step', 0.01, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));

    % --- Row 6: 方向场滤波 ---
    p_f = uipanel(lg, 'Title', '方向场滤波');
    p_f.Layout.Row = 6;
    fg = uigridlayout(p_f, [2, 2]);
    fg.ColumnWidth = {'1x', 90};
    fg.Padding = [6 6 6 6]; fg.RowSpacing = 4;
    uilabel(fg, 'Text', '滤波半径');
    app.h.p_filt_r = uispinner(fg, 'Limits', [0 30], 'Value', 5, 'Step', 1, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    uilabel(fg, 'Text', '滤波迭代次数');
    app.h.p_filt_it = uispinner(fg, 'Limits', [0 10], 'Value', 2, 'Step', 1, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));

    % --- Row 7: 流线 ---
    p_s = uipanel(lg, 'Title', '流线 (拓扑优化, 速度瓶颈)');
    p_s.Layout.Row = 7;
    sg = uigridlayout(p_s, [3, 2]);
    sg.ColumnWidth = {'1x', 90};
    sg.RowHeight = {28, 24, 24};
    sg.Padding = [6 6 6 6]; sg.RowSpacing = 3;

    app.h.p_do_stream = uicheckbox(sg, ...
        'Text', '计算拓扑流线 (默认关闭, 加速调参)', ...
        'Value', false, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    app.h.p_do_stream.Layout.Column = [1 2];

    uilabel(sg, 'Text', 'volfrac');
    app.h.p_volfrac = uispinner(sg, 'Limits', [0.05 0.95], 'Value', 0.5, 'Step', 0.05, ...
        'ValueChangedFcn', @(s,e) mark_dirty(fig));
    uilabel(sg, 'Text', '说明: 不算流线时多个轮廓内不切分', ...
        'FontColor', [0.5 0.5 0.5], 'FontSize', 9);
    uilabel(sg, 'Text', '');

    % --- Row 8: 路径偏置 (阶段二) ---
    p_o = uipanel(lg, 'Title', '路径偏置 (阶段二全局参数)');
    p_o.Layout.Row = 8;
    og = uigridlayout(p_o, [3, 2]);
    og.ColumnWidth = {'1x', 90};
    og.Padding = [6 6 6 6]; og.RowSpacing = 4;
    uilabel(og, 'Text', '偏置距离 (mm)');
    % [集中化] 初始值取每层默认线宽 (已继承自切片 LINE_WIDTH), 不再硬编码 0.3
    app.h.p_off_dist = uispinner(og, 'Limits', [0.05 5], ...
        'Value', app.params_per_layer{app.cur}.offset_distance, 'Step', 0.05);
    uilabel(og, 'Text', '最大偏置层数');
    app.h.p_max_iter = uispinner(og, 'Limits', [1 200], 'Value', 30, 'Step', 1);
    uilabel(og, 'Text', '最小路径长度');
    app.h.p_min_path = uispinner(og, 'Limits', [0 50], 'Value', 2, 'Step', 0.5);

    % --- Row 9: 操作按钮 ---
    p_a = uipanel(lg, 'Title', '操作');
    p_a.Layout.Row = 9;
    ag = uigridlayout(p_a, [1, 4]);
    ag.Padding = [4 4 4 4];

    uibutton(ag, 'Text', '[ 重算本层 ]', ...
        'BackgroundColor', [0.7 0.85 1], 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @(s,e) on_recompute(fig));
    % [V12] 单独按钮: 算路径预览 (不接受)
    uibutton(ag, 'Text', '[ 预览路径 ]', ...
        'BackgroundColor', [1 0.85 0.6], 'FontWeight', 'bold', ...
        'Tooltip', '基于当前轮廓 / 区域算偏置路径并显示在主视图. 不会接受本层.', ...
        'ButtonPushedFcn', @(s,e) on_preview_paths(fig));
    uibutton(ag, 'Text', '[ 接受 + 下一层 ]', ...
        'BackgroundColor', [0.55 0.85 0.55], 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @(s,e) on_accept_next(fig));
    uibutton(ag, 'Text', '[ 重置参数 ]', ...
        'ButtonPushedFcn', @(s,e) on_reset_params(fig));

    % --- Row 10: (空，为按钮高度) - 改为放可视化复选框 ---
    p_vis = uipanel(lg, 'BorderType', 'none');
    p_vis.Layout.Row = 10;
    vg = uigridlayout(p_vis, [1, 2]);
    vg.Padding = [2 2 2 2];
    app.h.chk_per_feas = uicheckbox(vg, ...
        'Text', '显示每组 bbox+feasible (青/品红)', ...
        'Value', true, ...
        'Tooltip', '关闭可减少视觉干扰; 重算/切层后自动刷新', ...
        'ValueChangedFcn', @(s,e) on_toggle_vis(fig));
    uilabel(vg, 'Text', '');

    % --- Row 11: 会话 + 3D ---
    p_b = uipanel(lg, 'BorderType', 'none');
    p_b.Layout.Row = 11;
    bg = uigridlayout(p_b, [1, 3]);
    bg.Padding = [2 2 2 2];

    uibutton(bg, 'Text', '[ 保存会话 ]', ...
        'ButtonPushedFcn', @(s,e) on_save_session(fig));
    uibutton(bg, 'Text', '[ 加载会话 ]', ...
        'ButtonPushedFcn', @(s,e) on_load_session(fig));
    uibutton(bg, 'Text', '[ 3D 预览 ]', ...
        'BackgroundColor', [0.85 0.8 1], ...
        'ButtonPushedFcn', @(s,e) on_open_3d_preview(fig));

    % --- Row 12: 阶段二 ---
    app.h.btn_gen = uibutton(lg, 'Text', '====  阶段二: 生成所有路径  ====', ...
        'BackgroundColor', [1 0.7 0.3], 'FontWeight', 'bold', 'FontSize', 13, ...
        'ButtonPushedFcn', @(s,e) on_generate_paths(fig));
    app.h.btn_gen.Layout.Row = 12;

    % -------- 右侧可视化区 --------
    right = uigridlayout(main, [3, 1]);
    right.RowHeight = {'1x', 90, 30};
    right.Layout.Column = 2;
    right.RowSpacing = 6;

    plot_panel = uipanel(right, 'Title', '层预览');
    plot_panel.Layout.Row = 1;
    pp_grid = uigridlayout(plot_panel, [1, 1]);
    pp_grid.Padding = [5 5 5 5];
    app.h.ax = uiaxes(pp_grid);
    axis(app.h.ax, 'equal');
    xlabel(app.h.ax, 'X (mm)'); ylabel(app.h.ax, 'Y (mm)');
    title(app.h.ax, '');

    stats_panel = uipanel(right, 'Title', '统计 & 状态');
    stats_panel.Layout.Row = 2;
    sp_grid = uigridlayout(stats_panel, [1, 2]);
    sp_grid.ColumnWidth = {'1x', 220};
    sp_grid.Padding = [6 6 6 6];

    app.h.stats_lbl = uilabel(sp_grid, ...
        'Text', '尚未计算', 'FontName', 'Consolas', 'FontSize', 11);
    app.h.status_lbl = uilabel(sp_grid, ...
        'Text', '状态: 等待', ...
        'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', 'FontSize', 13, ...
        'BackgroundColor', [0.9 0.9 0.9]);

    hint = uilabel(right, ...
        'Text', '提示: 自动模式调参后点[重算本层]; 手动模式点[+新建外/内轮廓]在主视图框选, 完成后点[应用手动轮廓->重算]; 满意了点[接受+下一层]; 全部接受后点[阶段二: 生成所有路径]', ...
        'HorizontalAlignment', 'center', 'FontColor', [0.4 0.4 0.4]);
    hint.Layout.Row = 3;

    fig.UserData = app;
    draw_progress(fig);
    update_status(fig, 'idle');
    update_mode_visual(fig);
end


%% ================================================================
%% UI 回调 - 层导航
%% ================================================================
function on_slider_changing(fig, evt)
    app = fig.UserData;
    app.h.layer_spinner.Value = round(evt.Value);
end

function on_slider_changed(fig)
    app = fig.UserData;
    new_layer = round(app.h.layer_slider.Value);
    app.h.layer_spinner.Value = new_layer;
    switch_to_layer(fig, new_layer);
end

function on_spinner_changed(fig)
    app = fig.UserData;
    new_layer = app.h.layer_spinner.Value;
    app.h.layer_slider.Value = new_layer;
    switch_to_layer(fig, new_layer);
end

function on_progress_click(fig, evt)
    app = fig.UserData;
    if isfield(evt, 'IntersectionPoint')
        target = round(evt.IntersectionPoint(1));
        target = max(1, min(app.num_layers, target));
        app.h.layer_slider.Value = target;
        app.h.layer_spinner.Value = target;
        switch_to_layer(fig, target);
    end
end

function switch_to_layer(fig, new_idx)
    app = fig.UserData;
    % 保存当前层参数
    app.params_per_layer{app.cur} = collect_params_from_ui(app);
    app.cur = new_idx;
    fig.UserData = app;
    % 把目标层参数推回 UI
    push_params_to_ui(fig, app.params_per_layer{new_idx});
    % 模式切换
    app = fig.UserData;
    app.h.btn_mode_auto.Value   = strcmp(app.layer_mode{new_idx}, 'auto');
    app.h.btn_mode_manual.Value = strcmp(app.layer_mode{new_idx}, 'manual');
    fig.UserData = app;
    update_mode_visual(fig);
    update_groups_listbox(fig);
    % 渲染
    if ~isempty(app.results_per_layer{new_idx})
        render_layer(fig, app.results_per_layer{new_idx});
        update_status(fig, ternary(app.accepted(new_idx), 'accepted', 'computed'));
    else
        if strcmp(app.layer_mode{new_idx}, 'manual')
            render_manual_canvas(fig);
        else
            cla(app.h.ax);
        end
        app.h.stats_lbl.Text = sprintf('Layer %d / %d: 未计算, 点[重算本层]', ...
            new_idx, app.num_layers);
        update_status(fig, 'pending');
    end
    draw_progress(fig);
    % [V9] 移除自动 3D 同步: 切层不需要立即刷新 3D, 由用户显式 [同步 3D 预览]
end

function mark_dirty(fig)
    update_status(fig, 'dirty');
end


%% ================================================================
%% UI 回调 - 模式切换
%% ================================================================
function on_mode_change(fig, target_mode)
    app = fig.UserData;
    % 互斥
    if strcmp(target_mode, 'auto')
        app.h.btn_mode_auto.Value   = true;
        app.h.btn_mode_manual.Value = false;
        app.h.btn_mode_auto.BackgroundColor = [0.7 0.85 1];
        app.h.btn_mode_manual.BackgroundColor = [0.95 0.95 0.95];
    else
        app.h.btn_mode_auto.Value   = false;
        app.h.btn_mode_manual.Value = true;
        app.h.btn_mode_auto.BackgroundColor = [0.95 0.95 0.95];
        app.h.btn_mode_manual.BackgroundColor = [1 0.85 0.55];
    end
    app.layer_mode{app.cur} = target_mode;
    fig.UserData = app;
    update_mode_visual(fig);
    update_groups_listbox(fig);
    update_status(fig, 'dirty');
    % 切到手动模式时立即显示活动栅格点
    if strcmp(target_mode, 'manual')
        render_manual_canvas(fig);
    end
    draw_progress(fig);  % 更新手动模式标记
end

function update_mode_visual(fig)
    app = fig.UserData;
    is_auto = strcmp(app.layer_mode{app.cur}, 'auto');
    if is_auto
        app.h.panel_auto.Visible   = 'on';
        app.h.panel_manual.Visible = 'off';
        if isfield(app.h, 'ph_grid')
            app.h.ph_grid.RowHeight = {'fit', 0};
        end
    else
        app.h.panel_auto.Visible   = 'off';
        app.h.panel_manual.Visible = 'on';
        if isfield(app.h, 'ph_grid')
            app.h.ph_grid.RowHeight = {0, 'fit'};
        end
    end
end


%% ================================================================
%% UI 回调 - 手动选择
%% ================================================================
function on_new_outer(fig)
    pick_polygon_for(fig, 'outer');
end

function on_new_inner(fig)
    pick_polygon_for(fig, 'inner');
end

% [V12] 既作外轮廓又作内轮廓 (区域 + 孔)
function on_new_both(fig)
    pick_polygon_for(fig, 'both');
end

function pick_polygon_for(fig, group_type)
    app = fig.UserData;
    li = app.cur;
    ld = app.surface_layers{li};
    if isempty(ld.final_grids)
        uialert(fig, '本层没有活动栅格', '无数据');
        return;
    end
    render_manual_canvas(fig);
    update_status(fig, 'picking');
    drawnow;
    % 根据 group_type 决定 drawpolygon 颜色
    switch group_type
        case 'outer', roi_color = [0 0.4 0.9];
        case 'inner', roi_color = [0.9 0.1 0.1];
        case 'both',  roi_color = [0.6 0.3 0.7];
    end
    try
        roi = drawpolygon(app.h.ax, ...
            'Color', roi_color, 'LineWidth', 2, 'FaceAlpha', 0.15);
    catch ME
        uialert(fig, ['drawpolygon 失败: ' ME.message], '错误');
        update_status(fig, 'idle');
        return;
    end
    if ~isvalid(roi) || size(roi.Position, 1) < 3
        update_status(fig, 'idle');
        return;
    end
    poly_xy = roi.Position;
    delete(roi);

    % [V16] 不再预裁剪用户选区, 也不用 bbox 过滤 z
    %       bbox 约束在后续 (compute_contours_streamlines) 通过 feasible_region 实现:
    %       用户轮廓 ∩ feasible_region -> 自动用 bbox 边界封闭超出部分
    %       prep 不在时按需触发 prepare_layer (供后续聚类阈值 / 缓存使用)
    if isempty(app.layer_cache{li}) || ~isfield(app.layer_cache{li}, 'prep') ...
            || isempty(app.layer_cache{li}.prep)
        try
            prep_now = prepare_layer(app.surface_layers{li}, app.grid_data, app.grid_bbox);
            ce = struct(); ce.prep = prep_now;
            app.layer_cache{li} = ce;
            fig.UserData = app;
        catch
            prep_now = [];
        end
    else
        prep_now = app.layer_cache{li}.prep;
    end

    % 检测哪些活动栅格点在多边形内 (仅 xy, 不再用 bbox 过滤)
    grids = ld.final_grids;
    gx = arrayfun(@(g) g.x, grids);
    gy = arrayfun(@(g) g.y, grids);
    gz = arrayfun(@(g) g.z, grids);
    in = inpolygon(gx, gy, poly_xy(:,1), poly_xy(:,2));
    pt_idx = find(in);

    if length(pt_idx) < 3
        uialert(fig, ...
            sprintf('框内只有 %d 个激活点 (至少需要 3 个) - 取消', ...
            length(pt_idx)), '点数不足');
        update_status(fig, 'idle');
        return;
    end

    % [V13] 聚类: 框内点可能是不连通的多个簇, 每簇独立生成轮廓
    cluster_factor = app.h.p_cluster_factor.Value;
    % [V16] 直接用 grid_bbox 的体素间距 (因为 V16 不再有 use_bbox)
    if isstruct(app.grid_bbox)
        voxel_spacing = mean([app.grid_bbox.vox_dx, app.grid_bbox.vox_dy]);
    else
        voxel_spacing = 1.0;
    end
    threshold = cluster_factor * voxel_spacing;
    clusters = cluster_points_by_distance(gx(pt_idx), gy(pt_idx), threshold);
    n_clusters = max(clusters);
    fprintf('[CLUSTER] %d 点 -> %d 簇 (阈值=%.3f mm)\n', ...
        length(pt_idx), n_clusters, threshold);

    % 过滤掉点数太少的簇 (< 3)
    valid_cluster_ids = [];
    for c = 1:n_clusters
        if sum(clusters == c) >= 3
            valid_cluster_ids(end+1) = c; %#ok<AGROW>
        end
    end
    if isempty(valid_cluster_ids)
        uialert(fig, '没有任何簇含足够多的点 (≥3) - 取消', '点数不足');
        update_status(fig, 'idle');
        return;
    end

    % 若多簇, 提示用户确认
    if length(valid_cluster_ids) > 1
        n_drop = n_clusters - length(valid_cluster_ids);
        msg = sprintf(['框内含 %d 个不连通点簇 (将创建 %d 个独立"%s"组).\n' ...
                       '%s\n继续?'], ...
            n_clusters, length(valid_cluster_ids), ...
            group_type_label(group_type), ...
            ternary(n_drop>0, sprintf('其中 %d 个簇点数<3 已忽略', n_drop), ''));
        choice = uiconfirm(fig, msg, '检测到多簇', ...
            'Options', {'分簇创建', '合并为一组', '取消'}, ...
            'DefaultOption', 1, 'CancelOption', 3);
        switch choice
            case '取消'
                update_status(fig, 'idle'); return;
            case '合并为一组'
                % 退回 V12 行为: 不聚类, 所有点合为一组
                clusters = ones(length(pt_idx), 1);
                valid_cluster_ids = 1;
        end
    end

    shrink = app.h.p_shrink.Value;
    md = app.manual_per_layer{li};
    if isempty(md), md = init_manual_data(); end
    label_zh = group_type_label(group_type);
    n_added  = 0;
    last_id  = 0;

    % [V17] 计算 per-contour feasible 的"曲面基底"(在本次 selection_poly 内, 曲面 z 有效的 XY 区域)
    % 后续每簇用 selection_poly ∩ 这个基底 -> 该轮廓的 feasible (实际等同于 selection_poly 内的 global_feasible)
    global_feasible = polyshape();
    if ~isempty(prep_now) && isstruct(prep_now) && isfield(prep_now, 'feasible_region') ...
            && ~isempty(prep_now.feasible_region) && area(prep_now.feasible_region) > 1e-9
        global_feasible = prep_now.feasible_region;
    end
    sel_polyshape = polyshape();
    try
        sel_polyshape = polyshape(poly_xy(:,1), poly_xy(:,2));
    catch
    end

    % 每个有效簇创建一个独立 group
    for ck = 1:length(valid_cluster_ids)
        c = valid_cluster_ids(ck);
        local_idx = find(clusters == c);
        sub_pt_idx = pt_idx(local_idx);
        sub_gx = gx(sub_pt_idx);
        sub_gy = gy(sub_pt_idx);
        sub_gz = gz(sub_pt_idx);

        [c_xy_raw, c_pt_local] = compute_boundary_from_points(sub_gx, sub_gy, shrink);
        if isempty(c_xy_raw)
            fprintf(2, '[WARN] 簇 %d boundary 失败 - 跳过\n', c);
            continue;
        end

        % [V20] per-contour 也用 bbox 投影到曲面 (跟 grid_bbox / layer_bbox 同构):
        %   pc_bbox = 该轮廓激活点的最小 axis-aligned 长方体
        %   pc_feasible = 曲面在 pc_bbox 内的 XY 投影 (polyshape)
        per_contour_feasible = polyshape();
        pc_bbox = [];
        try
            pc_bbox = make_per_contour_bbox(sub_gx, sub_gy, sub_gz, app.grid_bbox);
            per_contour_feasible = compute_per_contour_feasible(...
                sub_gx, sub_gy, sub_gz, ld.X_surf, ld.Y_surf, ld.Z_surf, app.grid_bbox);
        catch ME_pcf
            fprintf(2, '[WARN] per-contour bbox/feasible 计算失败: %s\n', ME_pcf.message);
        end
        if area(per_contour_feasible) < 1e-9
            % 兜底: 用 pc_bbox xy 矩形 ∩ global_feasible
            if ~isempty(pc_bbox) && area(global_feasible) > 1e-9
                try
                    per_contour_feasible = intersect(pc_bbox.poly_xy, global_feasible);
                catch
                end
            end
            if area(per_contour_feasible) < 1e-9 && ~isempty(pc_bbox)
                % 最后兜底: 用 pc_bbox xy 矩形本身
                per_contour_feasible = pc_bbox.poly_xy;
            end
        end

        % 用 per_contour_feasible 裁剪反算轮廓 (核心: 超出有效区域内缩到交线)
        c_xy = c_xy_raw;
        clipped_boundary_pt_idx = sub_pt_idx(c_pt_local);
        if area(per_contour_feasible) > 1e-9
            try
                c_raw_poly = polyshape(c_xy_raw(:,1), c_xy_raw(:,2));
                c_clipped  = intersect(c_raw_poly, per_contour_feasible);
                if area(c_clipped) > 1e-9
                    [bx, by] = boundary(c_clipped);
                    ni = find(isnan(bx));
                    if isempty(ni)
                        c_xy = [bx, by];
                    else
                        segs = {}; prev = 1;
                        for si = [ni(:)', length(bx)+1]
                            sg = [bx(prev:si-1), by(prev:si-1)];
                            if size(sg,1) >= 3, segs{end+1} = sg; end %#ok<AGROW>
                            prev = si + 1;
                        end
                        if ~isempty(segs)
                            % 取最大段
                            [~, mi] = max(cellfun(@(s) size(s,1), segs));
                            c_xy = segs{mi};
                        end
                    end
                    % 裁剪后, boundary_pt_idx 不再精确对应; 简单地保留所有活动点
                    clipped_boundary_pt_idx = sub_pt_idx;
                else
                    fprintf(2, '[WARN] 簇 %d 轮廓与 feasible 交集为空 - 跳过\n', c);
                    continue;
                end
            catch ME_clip
                fprintf(2, '[WARN] 簇 %d feasible 裁剪失败 (%s) - 用原始轮廓\n', ...
                    c, ME_clip.message);
            end
        end

        grp = struct();
        grp.type            = group_type;
        grp.role            = group_type;
        grp.selection_poly  = poly_xy;
        grp.pt_idx          = sub_pt_idx(:);
        grp.contour_xy      = c_xy;                 % [V17] 已裁剪到 feasible
        grp.contour_xy_raw  = c_xy_raw;             % [V17] 裁剪前的原始 boundary
        grp.boundary_pt_idx = clipped_boundary_pt_idx;
        grp.shrink          = shrink;
        grp.cluster_id      = c;
        grp.cluster_factor  = cluster_factor;
        % [V17] 保存 per-contour feasible region (用于显示和后续约束)
        grp.feasible_region = per_contour_feasible;
        % [V20] per-contour bbox (axis-aligned 长方体, 与 grid/layer_bbox 同构)
        if ~isempty(pc_bbox)
            grp.contour_bbox = pc_bbox;
        end

        if strcmp(group_type, 'inner')
            grp.id = numel(md.inner_groups) + 1;
            md.inner_groups{end+1} = grp;
        else
            grp.id = numel(md.outer_groups) + 1;
            md.outer_groups{end+1} = grp;
        end
        n_added = n_added + 1;
        last_id = grp.id;
    end

    if n_added == 0
        uialert(fig, '没有任何簇成功生成轮廓 - 取消', '失败');
        update_status(fig, 'idle');
        return;
    end

    app.manual_per_layer{li} = md;
    fig.UserData = app;

    render_manual_canvas(fig);
    update_groups_listbox(fig);
    if n_added == 1
        app.h.lbl_manual_status.Text = sprintf('已添加 %s#%d', label_zh, last_id);
    else
        app.h.lbl_manual_status.Text = sprintf('已添加 %d 个 %s 簇', n_added, label_zh);
    end
    update_status(fig, 'dirty');
end

%% [V13] 简单工具: group_type 中文名
function s = group_type_label(t)
    switch t
        case 'outer', s = '外';
        case 'inner', s = '内';
        case 'both',  s = '区+孔';
        otherwise,    s = t;
    end
end

function on_delete_group(fig)
    app = fig.UserData;
    li = app.cur;
    md = app.manual_per_layer{li};
    if isempty(md), return; end
    sel_str = app.h.list_groups.Value;
    if isempty(sel_str)
        uialert(fig, '请先在列表中选择一组', '未选择');
        return;
    end
    [type, id] = parse_group_label(sel_str);
    if strcmp(type, 'outer')
        if id >= 1 && id <= numel(md.outer_groups)
            md.outer_groups(id) = [];
            for k = 1:numel(md.outer_groups)
                md.outer_groups{k}.id = k;
            end
        end
    else
        if id >= 1 && id <= numel(md.inner_groups)
            md.inner_groups(id) = [];
            for k = 1:numel(md.inner_groups)
                md.inner_groups{k}.id = k;
            end
        end
    end
    app.manual_per_layer{li} = md;
    fig.UserData = app;
    render_manual_canvas(fig);
    update_groups_listbox(fig);
    update_status(fig, 'dirty');
    % [V9] 不自动 3D
end

function on_clear_manual(fig)
    app = fig.UserData;
    li = app.cur;
    choice = uiconfirm(fig, '确定清空本层所有手动选择吗?', '确认', ...
        'Options', {'清空', '取消'}, 'DefaultOption', 2, 'CancelOption', 2);
    if strcmp(choice, '取消'), return; end
    app.manual_per_layer{li} = [];
    fig.UserData = app;
    render_manual_canvas(fig);
    update_groups_listbox(fig);
    update_status(fig, 'dirty');
    % [V9] 不自动 3D
end

function on_apply_manual(fig)
    % 把手动选择应用到计算（重算本层，强制使用 manual contours）
    on_recompute(fig);
end

%% [V12] 用当前紧凑度批量重算所有组的轮廓 (不重新框选)
function on_reshrink_all(fig)
    app = fig.UserData;
    li  = app.cur;
    md  = app.manual_per_layer{li};
    if isempty(md) || (isempty(md.outer_groups) && isempty(md.inner_groups))
        uialert(fig, '本层没有手动定义的轮廓组', '无组');
        return;
    end
    new_shrink = app.h.p_shrink.Value;
    ld = app.surface_layers{li};
    grids = ld.final_grids;
    gx = arrayfun(@(g) g.x, grids);
    gy = arrayfun(@(g) g.y, grids);

    % [V20] 顺便重算 per-contour bbox + feasible (升级老会话到 V20 算法)
    gz = arrayfun(@(g) g.z, grids);

    n_updated = 0;
    for k = 1:numel(md.outer_groups)
        pi = md.outer_groups{k}.pt_idx;
        if length(pi) < 3, continue; end
        try
            new_pc_bbox = make_per_contour_bbox(gx(pi), gy(pi), gz(pi), app.grid_bbox);
            new_per_feas = compute_per_contour_feasible(...
                gx(pi), gy(pi), gz(pi), ld.X_surf, ld.Y_surf, ld.Z_surf, app.grid_bbox);
            md.outer_groups{k}.contour_bbox    = new_pc_bbox;
            if area(new_per_feas) > 1e-9
                md.outer_groups{k}.feasible_region = new_per_feas;
            end
        catch
        end
        [c, ci] = compute_boundary_from_points(gx(pi), gy(pi), new_shrink);
        if ~isempty(c)
            c = clip_contour_with_feasible(c, ...
                get_field_or(md.outer_groups{k}, 'feasible_region', []));
            md.outer_groups{k}.contour_xy      = c;
            md.outer_groups{k}.contour_xy_raw  = c;
            md.outer_groups{k}.boundary_pt_idx = pi(ci);
            md.outer_groups{k}.shrink          = new_shrink;
            n_updated = n_updated + 1;
        end
    end
    for k = 1:numel(md.inner_groups)
        pi = md.inner_groups{k}.pt_idx;
        if length(pi) < 3, continue; end
        try
            new_pc_bbox = make_per_contour_bbox(gx(pi), gy(pi), gz(pi), app.grid_bbox);
            new_per_feas = compute_per_contour_feasible(...
                gx(pi), gy(pi), gz(pi), ld.X_surf, ld.Y_surf, ld.Z_surf, app.grid_bbox);
            md.inner_groups{k}.contour_bbox    = new_pc_bbox;
            if area(new_per_feas) > 1e-9
                md.inner_groups{k}.feasible_region = new_per_feas;
            end
        catch
        end
        [c, ci] = compute_boundary_from_points(gx(pi), gy(pi), new_shrink);
        if ~isempty(c)
            c = clip_contour_with_feasible(c, ...
                get_field_or(md.inner_groups{k}, 'feasible_region', []));
            md.inner_groups{k}.contour_xy      = c;
            md.inner_groups{k}.contour_xy_raw  = c;
            md.inner_groups{k}.boundary_pt_idx = pi(ci);
            md.inner_groups{k}.shrink          = new_shrink;
            n_updated = n_updated + 1;
        end
    end
    app.manual_per_layer{li} = md;
    fig.UserData = app;
    render_manual_canvas(fig);
    update_groups_listbox(fig);
    update_status(fig, 'dirty');
    app.h.lbl_manual_status.Text = sprintf('已用紧凑度 %.1f 重算 %d 组', ...
        new_shrink, n_updated);
end

%% [V12] 从一组点反算紧贴边界
function [contour_xy, k_idx] = compute_boundary_from_points(xs, ys, shrink)
% 输入: 点集 xs, ys + shrink 因子 (0=凸包, 1=最紧凑凹包)
% 输出: contour_xy (N×2 闭合轮廓), k_idx (轮廓点在 xs/ys 中的索引)
    contour_xy = [];
    k_idx = [];
    xs = xs(:); ys = ys(:);
    if length(xs) < 3, return; end
    % 去重 (boundary 对重复点敏感)
    [uniq_xy, ~, ic] = unique([xs, ys], 'rows', 'stable');
    if size(uniq_xy, 1) < 3, return; end
    try
        kk = boundary(uniq_xy(:,1), uniq_xy(:,2), shrink);
    catch
        try
            kk = convhull(uniq_xy(:,1), uniq_xy(:,2));
        catch
            return;
        end
    end
    if isempty(kk) || length(kk) < 3, return; end
    contour_xy = uniq_xy(kk, :);
    % 把 unique 后的索引 kk 映射回原始 xs/ys 的索引
    % uniq_xy(j,:) 对应原始里第一个出现该点的位置
    first_orig_idx = zeros(size(uniq_xy, 1), 1);
    for j = 1:size(uniq_xy, 1)
        first_orig_idx(j) = find(ic == j, 1, 'first');
    end
    k_idx = first_orig_idx(kk);
end

function md = init_manual_data()
    md = struct();
    md.outer_groups = {};
    md.inner_groups = {};
end

function update_groups_listbox(fig)
    app = fig.UserData;
    li = app.cur;
    md = app.manual_per_layer{li};
    if isempty(md)
        app.h.list_groups.Items = {};
        app.h.lbl_manual_status.Text = '未选择';
        return;
    end
    items = {};
    for k = 1:numel(md.outer_groups)
        g = md.outer_groups{k};
        sh = get_field_or(g, 'shrink', NaN);
        n_bnd = numel(get_field_or(g, 'boundary_pt_idx', []));
        role = get_field_or(g, 'role', 'outer');
        prefix = ternary(strcmp(role,'both'), '区+孔', '外');
        items{end+1} = sprintf('%s #%d  内%d点  轮%d点  s=%.1f', ...
            prefix, g.id, numel(g.pt_idx), n_bnd, sh); %#ok<AGROW>
    end
    for k = 1:numel(md.inner_groups)
        g = md.inner_groups{k};
        sh = get_field_or(g, 'shrink', NaN);
        n_bnd = numel(get_field_or(g, 'boundary_pt_idx', []));
        items{end+1} = sprintf('内 #%d  内%d点  轮%d点  s=%.1f', ...
            g.id, numel(g.pt_idx), n_bnd, sh); %#ok<AGROW>
    end
    app.h.list_groups.Items = items;
    n_out  = sum(cellfun(@(g) ~strcmp(get_field_or(g,'role','outer'),'both'), md.outer_groups));
    n_both = sum(cellfun(@(g)  strcmp(get_field_or(g,'role','outer'),'both'), md.outer_groups));
    n_in   = numel(md.inner_groups);
    app.h.lbl_manual_status.Text = sprintf('外%d 区+孔%d 内%d', n_out, n_both, n_in);
end

function [type, id] = parse_group_label(s)
    % e.g. '外 #2  内12点  ...'      -> type='outer', id=2
    %      '区+孔 #1  内12点  ...'  -> type='outer', id=1 (因为 both 存在 outer_groups)
    %      '内 #3  内5点  ...'      -> type='inner', id=3
    type = 'outer'; id = 1;
    if startsWith(s, '内 ')
        type = 'inner';
    else
        % '外 ' 或 '区+孔 ' 都视为 outer (both 也在 outer_groups)
        type = 'outer';
    end
    tok = regexp(s, '#(\d+)', 'tokens', 'once');
    if ~isempty(tok)
        id = str2double(tok{1});
    end
end


%% ================================================================
%% UI 回调 - 重算 / 接受 / 重置
%% ================================================================
function on_recompute(fig)
    app = fig.UserData;
    li  = app.cur;
    update_status(fig, 'computing');
    drawnow;

    p = collect_params_from_ui(app);
    app.params_per_layer{li} = p;

    is_manual = strcmp(app.layer_mode{li}, 'manual');
    manual_data = [];
    if is_manual
        manual_data = app.manual_per_layer{li};
        if isempty(manual_data) || ...
                (isempty(manual_data.outer_groups) && isempty(manual_data.inner_groups))
            uialert(fig, '手动模式但尚未定义任何轮廓组, 请先 [+ 新建外轮廓]', '无手动数据');
            update_status(fig, 'pending');
            return;
        end
    end

    try
        t0 = tic;

        % [V9] 确保 prep 缓存就绪
        if isempty(app.layer_cache{li}) || ~isfield(app.layer_cache{li}, 'prep') ...
                || isempty(app.layer_cache{li}.prep)
            fprintf('[CACHE] Layer %d prep miss, computing...\n', li);
            prep = prepare_layer(app.surface_layers{li}, app.grid_data, app.grid_bbox);
            cache_entry = struct();
            cache_entry.prep = prep;
            app.layer_cache{li} = cache_entry;
            fig.UserData = app;
        else
            fprintf('[CACHE] Layer %d prep HIT\n', li);
        end
        prep = app.layer_cache{li}.prep;

        if isempty(prep) || ~isstruct(prep)
            error('prepare_layer 失败 (例如活动栅格不足)');
        end

        result = compute_contours_streamlines(li, prep, app.surface_layers{li}, ...
            p, app.has_img, app.has_mask, app.mask_poly, ...
            app.has_zhf, app.zhf, is_manual, manual_data, app.grid_bbox);
        result.processing_time = toc(t0);
        result.mode = ternary(is_manual, 'manual', 'auto');
        app.results_per_layer{li} = result;
        if app.accepted(li)
            app.accepted(li) = false;
        end
        fig.UserData = app;
        render_layer(fig, result);
        update_status(fig, 'computed');
    catch ME
        fig.UserData = app;
        update_status(fig, 'failed');
        cla(app.h.ax);
        title(app.h.ax, sprintf('Layer %d 计算失败', li));
        app.h.stats_lbl.Text = sprintf('错误: %s\n  位置: %s (line %d)', ...
            ME.message, ME.stack(1).name, ME.stack(1).line);
        fprintf(2, '[ERROR] Layer %d: %s\n', li, ME.message);
    end
    draw_progress(fig);
    % [V9] 不自动 3D, 用户手动 [同步 3D 预览]
end

%% [V12] 单独路径预览 - 不接受, 仅在主视图显示偏置路径
function on_preview_paths(fig)
    app = fig.UserData;
    li  = app.cur;
    if isempty(app.results_per_layer{li})
        uialert(fig, '请先 [重算本层] 得到轮廓和区域', '无结果');
        return;
    end
    R = app.results_per_layer{li};
    if ~R.success
        uialert(fig, '本层 [重算] 不成功, 无法预览路径', '无结果');
        return;
    end
    if isempty(R.regions)
        uialert(fig, '本层没有任何区域, 无法生成路径', '无区域');
        return;
    end
    update_status(fig, 'computing');
    drawnow;

    p = collect_params_from_ui(app);
    global_params = struct(...
        'offset_distance', p.offset_distance, ...
        'max_iterations',  p.max_iterations, ...
        'min_path_length', p.min_path_length);
    try
        t0 = tic;
        % [V23] 手动模式不用 mask 裁剪路径
        is_manual_layer = strcmp(app.layer_mode{li}, 'manual');
        eff_has_mask = app.has_mask && ~is_manual_layer;
        paths_result = compute_paths_only(R, global_params, ...
            eff_has_mask, app.mask_poly);
        dt = toc(t0);
        % 把路径附加到 result, 供 render_layer 使用
        R.paths_2d = paths_result.paths_2d;
        R.paths_3d = paths_result.paths_3d;
        R.statistics = struct( ...
            'num_outer',       length(R.outer_contours), ...
            'num_inner',       length(R.inner_contours), ...
            'num_streamlines', length(R.streamlines), ...
            'num_regions',     length(R.regions), ...
            'num_paths_2d',    length(paths_result.paths_2d), ...
            'num_paths_3d',    length(paths_result.paths_3d));
        R.paths_processing_time = dt;
        app.results_per_layer{li} = R;
        fig.UserData = app;
        render_layer(fig, R);
        update_status(fig, 'computed');
        app.h.stats_lbl.Text = sprintf([...
            'Layer %d / %d   预览路径   2D 路径:%d   3D 路径:%d   耗时:%.2fs\n' ...
            '提示: 满意了点 [接受+下一层], 不满意调参后重新 [预览路径]'], ...
            li, app.num_layers, length(R.paths_2d), length(R.paths_3d), dt);
    catch ME
        update_status(fig, 'failed');
        uialert(fig, sprintf('路径生成失败: %s', ME.message), '错误');
        fprintf(2, '[ERROR] Preview Layer %d paths: %s\n', li, ME.message);
    end
end

function on_accept_next(fig)
    app = fig.UserData;
    li  = app.cur;
    if isempty(app.results_per_layer{li})
        uialert(fig, '请先点[重算本层]', '未计算');
        return;
    end
    if ~app.results_per_layer{li}.success
        choice = uiconfirm(fig, ...
            sprintf('Layer %d 计算未成功. 仍要接受吗?', li), ...
            '层失败', 'Options', {'仍然接受', '跳到下一层', '取消'}, ...
            'DefaultOption', 2, 'CancelOption', 3);
        switch choice
            case '取消',     return;
            case '跳到下一层'
                advance_layer(fig); return;
        end
    end
    app.accepted(li) = true;
    fig.UserData = app;
    advance_layer(fig);
end

function advance_layer(fig)
    app = fig.UserData;
    if app.cur < app.num_layers
        next_idx = app.cur + 1;
        app.h.layer_slider.Value = next_idx;
        app.h.layer_spinner.Value = next_idx;
        switch_to_layer(fig, next_idx);
        on_recompute(fig);
    else
        update_status(fig, 'all_done');
        draw_progress(fig);
        uialert(fig, ...
            sprintf('已到达最后一层. 已接受 %d/%d.\n准备好之后请点[阶段二: 生成所有路径].', ...
            sum(app.accepted), app.num_layers), '完成阶段一', 'Icon', 'success');
    end
end

function on_reset_params(fig)
    app = fig.UserData;
    li  = app.cur;
    app.params_per_layer{li} = default_params();
    fig.UserData = app;
    push_params_to_ui(fig, app.params_per_layer{li});
    update_status(fig, 'dirty');
end

%% [V18] 可视化开关切换 -> 重渲染
function on_toggle_vis(fig)
    app = fig.UserData;
    li = app.cur;
    R = app.results_per_layer{li};
    is_manual = strcmp(app.layer_mode{li}, 'manual');
    if ~isempty(R) && isstruct(R)
        render_layer(fig, R);
    elseif is_manual
        render_manual_canvas(fig);
    end
end


%% ================================================================
%% UI 回调 - 会话
%% ================================================================
function on_save_session(fig)
    app = fig.UserData;
    app.params_per_layer{app.cur} = collect_params_from_ui(app);
    fig.UserData = app;
    [fn, fp] = uiputfile('*.mat', '保存会话', 'path_planning_session_v8.mat');
    if isequal(fn, 0), return; end
    sess = struct();
    sess.params_per_layer  = app.params_per_layer;
    sess.manual_per_layer  = app.manual_per_layer;
    sess.layer_mode        = app.layer_mode;
    sess.results_per_layer = app.results_per_layer;
    sess.accepted          = app.accepted;
    sess.num_layers        = app.num_layers;
    sess.cur               = app.cur;
    save(fullfile(fp, fn), 'sess', '-v7.3');
    uialert(fig, sprintf('已保存: %s', fn), '保存成功', 'Icon', 'success');
end

function on_load_session(fig)
    app = fig.UserData;
    [fn, fp] = uigetfile('*.mat', '加载会话');
    if isequal(fn, 0), return; end
    L = load(fullfile(fp, fn), 'sess');
    if ~isfield(L, 'sess')
        uialert(fig, '文件中没有 sess 变量', '加载失败'); return;
    end
    s = L.sess;
    if s.num_layers ~= app.num_layers
        uialert(fig, sprintf('会话层数 (%d) 与当前切片 (%d) 不匹配', ...
            s.num_layers, app.num_layers), '不匹配');
        return;
    end
    app.params_per_layer  = s.params_per_layer;
    app.results_per_layer = s.results_per_layer;
    app.accepted          = s.accepted;
    if isfield(s, 'manual_per_layer'), app.manual_per_layer = s.manual_per_layer; end
    if isfield(s, 'layer_mode'),       app.layer_mode       = s.layer_mode;       end
    app.cur = s.cur;
    fig.UserData = app;
    app.h.layer_slider.Value  = app.cur;
    app.h.layer_spinner.Value = app.cur;
    push_params_to_ui(fig, app.params_per_layer{app.cur});
    % 模式按钮
    app = fig.UserData;
    app.h.btn_mode_auto.Value   = strcmp(app.layer_mode{app.cur}, 'auto');
    app.h.btn_mode_manual.Value = strcmp(app.layer_mode{app.cur}, 'manual');
    fig.UserData = app;
    update_mode_visual(fig);
    update_groups_listbox(fig);
    if ~isempty(app.results_per_layer{app.cur})
        render_layer(fig, app.results_per_layer{app.cur});
        update_status(fig, ternary(app.accepted(app.cur), 'accepted', 'computed'));
    end
    draw_progress(fig);
    uialert(fig, sprintf('已加载: %s\n已接受 %d/%d 层', ...
        fn, sum(app.accepted), app.num_layers), '加载成功', 'Icon', 'success');
end


%% ================================================================
%% UI 回调 - 阶段二
%% ================================================================
function on_generate_paths(fig)
    app = fig.UserData;
    app.params_per_layer{app.cur} = collect_params_from_ui(app);
    fig.UserData = app;

    n_acc = sum(app.accepted);
    if n_acc == 0
        uialert(fig, '还没有任何已接受的层', '无可用层'); return;
    end

    msg = sprintf('将基于已接受的 %d / %d 层生成路径. 继续?', n_acc, app.num_layers);
    choice = uiconfirm(fig, msg, '阶段二', 'Options', {'继续', '取消'}, ...
        'DefaultOption', 1, 'CancelOption', 2);
    if strcmp(choice, '取消'), return; end

    global_params = struct();
    global_params.offset_distance  = app.h.p_off_dist.Value;
    global_params.max_iterations   = app.h.p_max_iter.Value;
    global_params.min_path_length  = app.h.p_min_path.Value;

    dlg = uiprogressdlg(fig, 'Title', '阶段二: 生成路径', ...
        'Message', '准备...', 'Indeterminate', 'off');

    num = app.num_layers;
    all_layers_data = init_layer_results_struct(num);
    success_count = 0;
    failed_layers = [];
    t_total = tic;

    for li = 1:num
        if ~app.accepted(li)
            all_layers_data(li) = stage_a_to_layer_result(li, ...
                app.surface_layers{li}.offset, []);
            continue;
        end
        if dlg.CancelRequested, break; end
        dlg.Value = li / num;
        dlg.Message = sprintf('Layer %d / %d (已接受 %d)', li, num, sum(app.accepted));

        stage_a = app.results_per_layer{li};
        try
            t_p = tic;
            % [V23] 该层是手动模式则不用 mask
            is_manual_layer = strcmp(app.layer_mode{li}, 'manual');
            eff_has_mask = app.has_mask && ~is_manual_layer;
            paths_result = compute_paths_only(stage_a, global_params, ...
                eff_has_mask, app.mask_poly);
            paths_result.processing_time = stage_a.processing_time + toc(t_p);
            all_layers_data(li) = merge_stage_a_and_paths(li, ...
                app.surface_layers{li}.offset, stage_a, paths_result);
            success_count = success_count + 1;
        catch ME
            fprintf(2, '[ERROR] Path gen Layer %d: %s\n', li, ME.message);
            failed_layers(end+1) = li; %#ok<AGROW>
            all_layers_data(li) = stage_a_to_layer_result(li, ...
                app.surface_layers{li}.offset, stage_a);
            all_layers_data(li).success = false;
            all_layers_data(li).error_message = ME.message;
        end
    end

    total_time = toc(t_total);
    close(dlg);

    save_final_results(fig, all_layers_data, global_params, total_time, ...
        success_count, failed_layers);
end


%% ================================================================
%% 参数 UI <-> struct 同步
%% ================================================================
function p = collect_params_from_ui(app)
    p = struct();
    p.contour_dilate_pixels    = app.h.p_dilate.Value;
    p.contour_expand_ratio     = app.h.p_expand.Value;
    p.min_contour_length_inner = app.h.p_min_inner.Value;
    p.use_z_burn               = app.h.p_use_zburn.Value;
    p.z_margin_factor          = app.h.p_zmargin.Value;
    p.min_region_area          = app.h.p_min_area.Value;
    p.adaptive_outer_factor    = app.h.p_outer_fac.Value;
    p.filter_radius            = app.h.p_filt_r.Value;
    p.filter_iterations        = app.h.p_filt_it.Value;
    p.volfrac                  = app.h.p_volfrac.Value;
    p.do_streamlines           = app.h.p_do_stream.Value;   % [V9]
    p.offset_distance          = app.h.p_off_dist.Value;
    p.max_iterations           = app.h.p_max_iter.Value;
    p.min_path_length          = app.h.p_min_path.Value;
end

function push_params_to_ui(fig, p)
    app = fig.UserData;
    app.h.p_dilate.Value     = p.contour_dilate_pixels;
    app.h.p_expand.Value     = p.contour_expand_ratio;
    app.h.p_min_inner.Value  = p.min_contour_length_inner;
    app.h.p_use_zburn.Value  = p.use_z_burn;
    app.h.p_zmargin.Value    = p.z_margin_factor;
    app.h.p_min_area.Value   = p.min_region_area;
    app.h.p_outer_fac.Value  = p.adaptive_outer_factor;
    app.h.p_filt_r.Value     = p.filter_radius;
    app.h.p_filt_it.Value    = p.filter_iterations;
    app.h.p_volfrac.Value    = p.volfrac;
    if isfield(p, 'do_streamlines')   % [V9] 向后兼容老会话
        app.h.p_do_stream.Value = p.do_streamlines;
    end
end


%% ================================================================
%% 状态指示 + 进度条
%% ================================================================
function update_status(fig, state)
    app = fig.UserData;
    lbl = app.h.status_lbl;
    switch state
        case 'idle',      lbl.Text = '等待';           lbl.BackgroundColor = [0.9 0.9 0.9];
        case 'pending',   lbl.Text = '未计算';         lbl.BackgroundColor = [0.85 0.85 0.85];
        case 'dirty',     lbl.Text = '参数已变, 需要重算'; lbl.BackgroundColor = [1 0.85 0.4];
        case 'computing', lbl.Text = '计算中...';      lbl.BackgroundColor = [0.5 0.8 1];
        case 'picking',   lbl.Text = '请在主视图绘制多边形'; lbl.BackgroundColor = [1 0.8 0.4];
        case 'computed',  lbl.Text = '已计算 (待确认)'; lbl.BackgroundColor = [0.6 0.85 1];
        case 'accepted',  lbl.Text = '已接受';         lbl.BackgroundColor = [0.5 0.85 0.5];
        case 'failed',    lbl.Text = '失败';           lbl.BackgroundColor = [1 0.5 0.5];
        case 'all_done',  lbl.Text = '阶段一完成';     lbl.BackgroundColor = [0.4 0.85 0.4];
    end
end

function draw_progress(fig)
    app = fig.UserData;
    ax  = app.h.progress_ax;
    cla(ax); hold(ax, 'on');
    n = app.num_layers;
    for li = 1:n
        if app.accepted(li)
            c = [0.4 0.85 0.4];
        elseif ~isempty(app.results_per_layer{li}) && ~app.results_per_layer{li}.success
            c = [1 0.5 0.5];
        elseif ~isempty(app.results_per_layer{li})
            c = [0.7 0.8 1];
        else
            c = [0.85 0.85 0.85];
        end
        rectangle(ax, 'Position', [li-0.5, 0, 1, 1], 'FaceColor', c, ...
            'EdgeColor', 'w', 'LineWidth', 0.5, ...
            'PickableParts', 'none', 'HitTest', 'off');
        % 手动模式标记
        if strcmp(app.layer_mode{li}, 'manual')
            rectangle(ax, 'Position', [li-0.5, 0.7, 1, 0.3], ...
                'FaceColor', [1 0.5 0], 'EdgeColor', 'none', ...
                'PickableParts', 'none', 'HitTest', 'off');
        end
    end
    rectangle(ax, 'Position', [app.cur-0.5, 0, 1, 1], ...
        'EdgeColor', [0 0 0], 'LineWidth', 2, ...
        'PickableParts', 'none', 'HitTest', 'off');
    xlim(ax, [0.5, n+0.5]); ylim(ax, [0 1]);
    hold(ax, 'off');
end


%% ================================================================
%% 关闭确认
%% ================================================================
function on_close(fig)
    app = fig.UserData;
    % 关闭 3D 预览
    if ~isempty(app.preview3d_fig) && isvalid(app.preview3d_fig)
        delete(app.preview3d_fig);
    end
    n_acc = sum(app.accepted);
    if n_acc > 0
        choice = uiconfirm(fig, ...
            sprintf('已接受 %d 层未保存会话. 确定关闭?', n_acc), ...
            '确认关闭', 'Options', {'保存并关闭', '直接关闭', '取消'}, ...
            'DefaultOption', 1, 'CancelOption', 3);
        switch choice
            case '保存并关闭'
                on_save_session(fig); delete(fig);
            case '直接关闭'
                delete(fig);
        end
    else
        delete(fig);
    end
end


%% ================================================================
%% 工具: 三元
%% ================================================================
function v = ternary(cond, a, b)
    if cond, v = a; else, v = b; end
end

%% [V14] 工具: 在 2D 轴上画 bbox 矩形 (虚线)
function draw_bbox_rect_2d(ax, bb, color, lw)
    if ~isstruct(bb), return; end
    plot(ax, ...
        [bb.x_min, bb.x_max, bb.x_max, bb.x_min, bb.x_min], ...
        [bb.y_min, bb.y_min, bb.y_max, bb.y_max, bb.y_min], ...
        '--', 'Color', color, 'LineWidth', lw);
end

%% [V14] 工具: 取该层的 layer_bbox (按需从 prep 取, 没有就 lazy 计算)
function bb = get_layer_bbox(app, li)
    bb = [];
    if li < 1 || li > app.num_layers, return; end
    if ~isempty(app.layer_cache{li}) && isfield(app.layer_cache{li}, 'prep') ...
            && ~isempty(app.layer_cache{li}.prep) ...
            && isfield(app.layer_cache{li}.prep, 'layer_bbox')
        bb = app.layer_cache{li}.prep.layer_bbox;
    end
end

%% [V14] 工具: 取该层的 effective_bbox (按需从 prep 取)
function bb = get_effective_bbox(app, li)
    bb = app.grid_bbox;   % 兜底
    if li < 1 || li > app.num_layers, return; end
    if ~isempty(app.layer_cache{li}) && isfield(app.layer_cache{li}, 'prep') ...
            && ~isempty(app.layer_cache{li}.prep) ...
            && isfield(app.layer_cache{li}.prep, 'effective_bbox') ...
            && ~isempty(app.layer_cache{li}.prep.effective_bbox)
        bb = app.layer_cache{li}.prep.effective_bbox;
    end
end

%% [V16] 工具: 在 2D 轴上画 feasible_region 边界 (绿虚线)
function draw_feasible_region_2d(ax, app, li)
    if li < 1 || li > app.num_layers, return; end
    if isempty(app.layer_cache{li}) || ~isfield(app.layer_cache{li}, 'prep') ...
            || isempty(app.layer_cache{li}.prep)
        return;
    end
    prep = app.layer_cache{li}.prep;
    if ~isfield(prep, 'feasible_region') || isempty(prep.feasible_region) ...
            || area(prep.feasible_region) < 1e-9
        return;
    end
    try
        [fbx, fby] = boundary(prep.feasible_region);
        plot(ax, fbx, fby, '--', 'Color', [0.1 0.65 0.2], 'LineWidth', 1.8);
    catch
    end
end

%% [V18] 工具: 在 2D 轴上画每组 per-contour feasible 边界 (青色 / 品红 点划线)
function draw_per_contour_feasible_2d(ax, app, li)
    if li < 1 || li > app.num_layers, return; end
    if isempty(app.manual_per_layer{li}), return; end
    if isfield(app.h, 'chk_per_feas') && isvalid(app.h.chk_per_feas) ...
            && app.h.chk_per_feas.Value == 0
        return;
    end
    md = app.manual_per_layer{li};
    % [V20] 配色:
    %   per-contour bbox  (axis-aligned 矩形): 实线
    %   per-contour feasible (投影 polyshape): 点划线
    %   颜色: 青色 (外/区+孔), 品红 (内)
    col_outer = [0.0 0.65 0.75];     % 青色
    col_inner = [0.75 0.2 0.55];     % 品红
    % outer / both 组
    for k = 1:numel(md.outer_groups)
        g = md.outer_groups{k};
        % bbox 矩形 (实线)
        cb = get_field_or(g, 'contour_bbox', []);
        if ~isempty(cb) && isstruct(cb)
            plot(ax, ...
                [cb.x_min, cb.x_max, cb.x_max, cb.x_min, cb.x_min], ...
                [cb.y_min, cb.y_min, cb.y_max, cb.y_max, cb.y_min], ...
                '-', 'Color', col_outer, 'LineWidth', 1.2);
        end
        % feasible 投影 (点划线)
        per_feas = get_field_or(g, 'feasible_region', []);
        if ~isempty(per_feas) && isa(per_feas, 'polyshape') && area(per_feas) > 1e-9
            try
                [fbx, fby] = boundary(per_feas);
                plot(ax, fbx, fby, '-.', 'Color', col_outer, 'LineWidth', 1.0);
            catch
            end
        end
    end
    % inner 组
    for k = 1:numel(md.inner_groups)
        g = md.inner_groups{k};
        cb = get_field_or(g, 'contour_bbox', []);
        if ~isempty(cb) && isstruct(cb)
            plot(ax, ...
                [cb.x_min, cb.x_max, cb.x_max, cb.x_min, cb.x_min], ...
                [cb.y_min, cb.y_min, cb.y_max, cb.y_max, cb.y_min], ...
                '-', 'Color', col_inner, 'LineWidth', 1.2);
        end
        per_feas = get_field_or(g, 'feasible_region', []);
        if ~isempty(per_feas) && isa(per_feas, 'polyshape') && area(per_feas) > 1e-9
            try
                [fbx, fby] = boundary(per_feas);
                plot(ax, fbx, fby, '-.', 'Color', col_inner, 'LineWidth', 1.0);
            catch
            end
        end
    end
end


%% ================================================================
%% 渲染单层结果 (计算完成后)
%% ================================================================
function render_layer(fig, R)
    app = fig.UserData;
    ax  = app.h.ax;
    cla(ax); hold(ax, 'on');

    % [V14] 双 bbox: 红=grid (全局), 蓝=layer (本层激活栅格)
    draw_bbox_rect_2d(ax, app.grid_bbox, [0.8 0.2 0.2], 1.2);
    layer_bbox = get_layer_bbox(app, R.layer_idx);
    draw_bbox_rect_2d(ax, layer_bbox, [0.15 0.4 0.85], 1.5);

    % [V16] feasible_region 边界 (绿色虚线 - 该层可行 XY 区域)
    draw_feasible_region_2d(ax, app, R.layer_idx);

    % [V18] per-contour feasible (手动模式每组的 feasible 边界)
    if isfield(R, 'mode') && strcmp(R.mode, 'manual')
        draw_per_contour_feasible_2d(ax, app, R.layer_idx);
    end

    if app.has_mask
        [mx, my] = boundary(app.mask_poly);
        plot(ax, mx, my, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
    end

    if R.success
        for i = 1:length(R.outer_contours)
            c = R.outer_contours{i};
            fill(ax, c(:,1), c(:,2), [0.78 0.85 0.95], ...
                'EdgeColor', [0 0.3 0.8], 'LineWidth', 2, 'FaceAlpha', 0.6);
        end
        for i = 1:length(R.inner_contours)
            c = R.inner_contours{i};
            fill(ax, c(:,1), c(:,2), [1 1 1], ...
                'EdgeColor', [0.85 0.1 0.1], 'LineWidth', 2);
        end
        for i = 1:length(R.regions)
            try
                plot(ax, R.regions{i}, 'FaceColor', [1 0.95 0.6], ...
                    'EdgeColor', [0.8 0.6 0], 'FaceAlpha', 0.25, 'LineWidth', 0.8);
            catch
            end
        end
        for i = 1:length(R.streamlines)
            sl = R.streamlines{i};
            if ~isempty(sl)
                plot(ax, sl(:,1), sl(:,2), '-', ...
                    'Color', [0.1 0.6 0.2], 'LineWidth', 1.8);
            end
        end
        if isfield(R, 'activated_xy') && ~isempty(R.activated_xy)
            scatter(ax, R.activated_xy(:,1), R.activated_xy(:,2), 4, ...
                [0.8 0.2 0.2], '.');
        end
        % [V12] 2D 偏置路径 (若已预览过)
        % 注: paths_2d 里点格式是 (y, x), 来自 generate_offset_path2
        n_paths = 0;
        if isfield(R, 'paths_2d') && ~isempty(R.paths_2d)
            n_paths = length(R.paths_2d);
            cmap_p = jet(max(n_paths, 2));
            for i = 1:n_paths
                p = R.paths_2d{i};
                if isempty(p) || size(p,1) < 2, continue; end
                plot(ax, p(:,2), p(:,1), '-', ...
                    'Color', cmap_p(i,:), 'LineWidth', 1);
            end
        end
        mode_str = ternary(isfield(R,'mode') && strcmp(R.mode,'manual'),'[手动]','[自动]');
        path_str = '';
        if n_paths > 0, path_str = sprintf('  路径:%d', n_paths); end
        title(ax, sprintf('Layer %d / %d  %s  |  外:%d  内:%d  流:%d  区:%d%s', ...
            R.layer_idx, app.num_layers, mode_str, ...
            length(R.outer_contours), length(R.inner_contours), ...
            length(R.streamlines), length(R.regions), path_str));
    else
        title(ax, sprintf('Layer %d 失败: %s', R.layer_idx, R.error_message));
    end

    hold(ax, 'off');
    axis(ax, 'equal');

    if R.success
        burn_bbox = 0;
        if isfield(R, 'burn_bbox'), burn_bbox = R.burn_bbox; end
        app.h.stats_lbl.Text = sprintf([...
            'Layer %d / %d   offset=%.3f   模式=%s\n' ...
            '外轮廓:%2d  内轮廓:%2d  流线:%2d  区域:%2d\n' ...
            'Burn(XY):%d  Burn(Z):%d  Burn(bbox):%d   耗时:%.2fs'], ...
            R.layer_idx, app.num_layers, R.offset, ...
            ternary(isfield(R,'mode') && strcmp(R.mode,'manual'),'手动','自动'), ...
            length(R.outer_contours), length(R.inner_contours), ...
            length(R.streamlines), length(R.regions), ...
            R.burn_xy, R.burn_z, burn_bbox, R.processing_time);
    end
end


%% ================================================================
%% 渲染手动画布 (drawpolygon 之前显示活动栅格 + 已有选择组)
%% ================================================================
function render_manual_canvas(fig)
    app = fig.UserData;
    ax  = app.h.ax;
    li  = app.cur;
    ld  = app.surface_layers{li};

    cla(ax); hold(ax, 'on');

    % [V14] 双 bbox: 红=grid (全局), 蓝=layer (本层激活栅格)
    draw_bbox_rect_2d(ax, app.grid_bbox, [0.8 0.2 0.2], 1.2);
    layer_bbox = get_layer_bbox(app, li);
    draw_bbox_rect_2d(ax, layer_bbox, [0.15 0.4 0.85], 1.5);

    % [V16] feasible_region 边界 (绿色虚线 - 该层路径合法可走范围)
    draw_feasible_region_2d(ax, app, li);

    % [V18] per-contour feasible 边界 (青色/品红点划线 - 每组独立)
    draw_per_contour_feasible_2d(ax, app, li);

    % 结构遮罩
    if app.has_mask
        [mx, my] = boundary(app.mask_poly);
        plot(ax, mx, my, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
    end

    % 所有活动栅格点 (灰色基准)
    grids = ld.final_grids;
    if isempty(grids), hold(ax,'off'); return; end
    gx = arrayfun(@(g) g.x, grids);
    gy = arrayfun(@(g) g.y, grids);
    scatter(ax, gx, gy, 20, [0.6 0.6 0.6], 'filled', ...
        'MarkerEdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.3);
    % [V16] 不再用 bbox 过滤标记点 (bbox 现在通过 feasible_region 软封闭轮廓)

    % 已有的 manual 组
    md = app.manual_per_layer{li};
    if ~isempty(md)
        % 外轮廓 / both 组 (在 outer_groups 中, role 决定颜色)
        for k = 1:numel(md.outer_groups)
            g = md.outer_groups{k};
            pi = g.pt_idx;
            role = get_field_or(g, 'role', 'outer');
            if strcmp(role, 'both')
                col_solid = [0.5 0.2 0.7];     % 紫色
                col_dash  = [0.7 0.5 0.85];
                label_txt = sprintf('区+孔%d', g.id);
            else
                col_solid = [0 0.3 0.8];
                col_dash  = [0.4 0.6 0.95];
                label_txt = sprintf('外%d', g.id);
            end
            scatter(ax, gx(pi), gy(pi), 50, col_solid, 'filled', ...
                'MarkerEdgeColor', 'k', 'LineWidth', 0.3, 'MarkerFaceAlpha', 0.5);
            sel = get_field_or(g, 'selection_poly', get_field_or(g, 'poly_xy', []));
            if ~isempty(sel)
                plot(ax, [sel(:,1); sel(1,1)], [sel(:,2); sel(1,2)], ':', ...
                    'Color', col_dash, 'LineWidth', 1);
            end
            % [V18] per-contour feasible 已由 draw_per_contour_feasible_2d 统一绘制
            cxy = get_field_or(g, 'contour_xy', sel);
            if ~isempty(cxy)
                plot(ax, [cxy(:,1); cxy(1,1)], [cxy(:,2); cxy(1,2)], '-', ...
                    'Color', col_solid, 'LineWidth', 2.2);
                bpi = get_field_or(g, 'boundary_pt_idx', []);
                if ~isempty(bpi)
                    scatter(ax, gx(bpi), gy(bpi), 90, col_solid, 'LineWidth', 1.5);
                end
            end
            cx = mean(gx(pi)); cy = mean(gy(pi));
            text(ax, cx, cy, label_txt, ...
                'FontWeight', 'bold', 'Color', col_solid, 'FontSize', 11, ...
                'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.5]);
        end
        % 内轮廓组 (红色调)
        for k = 1:numel(md.inner_groups)
            g = md.inner_groups{k};
            pi = g.pt_idx;
            scatter(ax, gx(pi), gy(pi), 50, [0.95 0.3 0.3], 'filled', ...
                'MarkerEdgeColor', 'k', 'LineWidth', 0.3, 'MarkerFaceAlpha', 0.5);
            sel = get_field_or(g, 'selection_poly', get_field_or(g, 'poly_xy', []));
            if ~isempty(sel)
                plot(ax, [sel(:,1); sel(1,1)], [sel(:,2); sel(1,2)], ':', ...
                    'Color', [0.95 0.5 0.5], 'LineWidth', 1);
            end
            % [V18] per-contour feasible 已由 draw_per_contour_feasible_2d 统一绘制
            cxy = get_field_or(g, 'contour_xy', sel);
            if ~isempty(cxy)
                plot(ax, [cxy(:,1); cxy(1,1)], [cxy(:,2); cxy(1,2)], '-', ...
                    'Color', [0.85 0.1 0.1], 'LineWidth', 2.2);
                bpi = get_field_or(g, 'boundary_pt_idx', []);
                if ~isempty(bpi)
                    scatter(ax, gx(bpi), gy(bpi), 90, [0.85 0.1 0.1], ...
                        'LineWidth', 1.5);
                end
            end
            cx = mean(gx(pi)); cy = mean(gy(pi));
            text(ax, cx, cy, sprintf('内%d', g.id), ...
                'FontWeight', 'bold', 'Color', [0.7 0 0], 'FontSize', 11, ...
                'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.5]);
        end
    end

    title(ax, sprintf(['Layer %d / %d  [手动模式]   ' ...
        '红=grid 蓝=layer 绿=层 feasible 青/品红 实=每组 bbox 点划=feasible'], ...
        li, app.num_layers));
    xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)');
    axis(ax, 'equal');
    hold(ax, 'off');
end

% [V12] 工具: 安全取字段
function v = get_field_or(s, name, default)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end


%% ================================================================
%% 3D 预览 (独立 classical figure)
%% ================================================================
function on_open_3d_preview(fig)
    app = fig.UserData;
    if ~isempty(app.preview3d_fig) && isvalid(app.preview3d_fig)
        figure(app.preview3d_fig);  % 提升到前台
    else
        % 新建独立窗口 (classical figure, 支持 rotate3d)
        f3 = figure('Name', '3D 预览 - Path Planning V8', ...
            'NumberTitle', 'off', ...
            'Position', [200 100 1100 800], ...
            'Color', 'w', ...
            'CloseRequestFcn', @(s,e) on_close_3d(fig, s));
        ax3 = axes('Parent', f3);
        xlabel(ax3, 'X (mm)'); ylabel(ax3, 'Y (mm)'); zlabel(ax3, 'Z (mm)');
        grid(ax3, 'on'); axis(ax3, 'equal'); view(ax3, 45, 30);
        rotate3d(ax3, 'on');

        % 控制条 - 两行布局
        % 上排 (Y=0.055): 层数 + mesh 控件
        uicontrol(f3, 'Style', 'text', 'String', '上下层数显示:', ...
            'Units', 'normalized', 'Position', [0.02 0.055 0.10 0.03], ...
            'BackgroundColor', 'w');
        h_neigh = uicontrol(f3, 'Style', 'slider', ...
            'Min', 0, 'Max', 10, 'Value', 0, 'SliderStep', [0.1 0.3], ...
            'Units', 'normalized', 'Position', [0.12 0.055 0.16 0.03], ...
            'Callback', @(s,e) refresh_3d_preview(fig));
        h_neigh_val = uicontrol(f3, 'Style', 'text', 'String', '+/- 0', ...
            'Units', 'normalized', 'Position', [0.285 0.055 0.04 0.03], ...
            'BackgroundColor', 'w');

        uicontrol(f3, 'Style', 'checkbox', 'String', '显示曲面 mesh', ...
            'Value', 0, 'Units', 'normalized', ...
            'Position', [0.34 0.055 0.10 0.03], 'BackgroundColor', 'w', ...
            'Callback', @(s,e) refresh_3d_preview(fig), 'Tag', 'chk_surf');
        % mesh 稀疏度
        uicontrol(f3, 'Style', 'text', 'String', '稀疏度:', ...
            'Units', 'normalized', 'Position', [0.45 0.055 0.04 0.03], ...
            'BackgroundColor', 'w');
        h_mesh_step = uicontrol(f3, 'Style', 'slider', ...
            'Min', 1, 'Max', 20, 'Value', 4, 'SliderStep', [0.05 0.25], ...
            'Units', 'normalized', 'Position', [0.49 0.055 0.10 0.03], ...
            'Callback', @(s,e) refresh_3d_preview(fig), 'Tag', 'slider_mesh_step');
        % [V15] mesh 范围 margin (倍体素)
        uicontrol(f3, 'Style', 'text', 'String', 'mesh 范围:', ...
            'Units', 'normalized', 'Position', [0.60 0.055 0.06 0.03], ...
            'BackgroundColor', 'w');
        h_mesh_margin = uicontrol(f3, 'Style', 'slider', ...
            'Min', 0.5, 'Max', 20, 'Value', 2, 'SliderStep', [0.025 0.1], ...
            'Units', 'normalized', 'Position', [0.66 0.055 0.10 0.03], ...
            'Callback', @(s,e) refresh_3d_preview(fig), 'Tag', 'slider_mesh_margin');
        h_mesh_margin_val = uicontrol(f3, 'Style', 'text', 'String', '×2.0', ...
            'Units', 'normalized', 'Position', [0.77 0.055 0.04 0.03], ...
            'BackgroundColor', 'w', 'Tag', 'lbl_mesh_margin_val');
        uicontrol(f3, 'Style', 'checkbox', 'String', '遮罩', ...
            'Value', 1, 'Units', 'normalized', ...
            'Position', [0.82 0.055 0.05 0.03], 'BackgroundColor', 'w', ...
            'Callback', @(s,e) refresh_3d_preview(fig), 'Tag', 'chk_mask');

        % 下排 (Y=0.02): 视角按钮
        uicontrol(f3, 'Style', 'pushbutton', 'String', '重置视角', ...
            'Units', 'normalized', 'Position', [0.02 0.02 0.08 0.03], ...
            'Callback', @(s,e) view(ax3,45,30));
        uicontrol(f3, 'Style', 'pushbutton', 'String', '俯视 (Z)', ...
            'Units', 'normalized', 'Position', [0.11 0.02 0.07 0.03], ...
            'Callback', @(s,e) view(ax3,0,90));
        uicontrol(f3, 'Style', 'pushbutton', 'String', '+X', ...
            'Units', 'normalized', 'Position', [0.19 0.02 0.04 0.03], ...
            'Callback', @(s,e) view(ax3,90,0));
        uicontrol(f3, 'Style', 'pushbutton', 'String', '+Y', ...
            'Units', 'normalized', 'Position', [0.24 0.02 0.04 0.03], ...
            'Callback', @(s,e) view(ax3,0,0));

        % 存到 fig.UserData 共享
        f3.UserData = struct('ax', ax3, 'h_neigh', h_neigh, ...
            'h_neigh_val', h_neigh_val, 'h_mesh_step', h_mesh_step, ...
            'h_mesh_margin', h_mesh_margin, ...
            'h_mesh_margin_val', h_mesh_margin_val);
        app.preview3d_fig = f3;
        fig.UserData = app;
    end
    refresh_3d_preview(fig);
end

function on_close_3d(main_fig, f3)
    delete(f3);
    if isvalid(main_fig)
        app = main_fig.UserData;
        app.preview3d_fig = [];
        main_fig.UserData = app;
    end
end

function refresh_3d_preview(fig)
    app = fig.UserData;
    f3 = app.preview3d_fig;
    if isempty(f3) || ~isvalid(f3), return; end
    ud = f3.UserData;
    ax3 = ud.ax;
    n_neigh = round(ud.h_neigh.Value);
    ud.h_neigh_val.String = sprintf('+/- %d', n_neigh);

    % 视角保留
    try, [az, el] = view(ax3); catch, az=45; el=30; end
    cla(ax3); hold(ax3, 'on');

    li = app.cur;
    layer_range = max(1,li-n_neigh):min(app.num_layers,li+n_neigh);
    n_show = length(layer_range);
    colors = parula(max(n_show,2));

    chk_surf = findobj(f3, 'Tag', 'chk_surf');
    chk_mask = findobj(f3, 'Tag', 'chk_mask');
    slider_step = findobj(f3, 'Tag', 'slider_mesh_step');
    show_surf = ~isempty(chk_surf) && chk_surf.Value == 1;
    show_mask = ~isempty(chk_mask) && chk_mask.Value == 1;
    mesh_step = 4;
    if ~isempty(slider_step), mesh_step = max(1, round(slider_step.Value)); end

    % 邻近层曲面 (淡色, 稀疏化 + [V15] bbox 裁剪)
    % 读 UI 的 mesh margin 参数 (默认 2)
    slider_margin = findobj(f3, 'Tag', 'slider_mesh_margin');
    if ~isempty(slider_margin)
        mesh_margin = slider_margin.Value;
    else
        mesh_margin = 2;
    end
    lbl_margin = findobj(f3, 'Tag', 'lbl_mesh_margin_val');
    if ~isempty(lbl_margin)
        lbl_margin.String = sprintf('×%.1f', mesh_margin);
    end
    for ii = 1:n_show
        li_show = layer_range(ii);
        ld = app.surface_layers{li_show};
        if isempty(ld.X_surf), continue; end
        if show_surf
            alpha = ternary(li_show==li, 0.55, 0.18);
            % 稀疏化
            [Xs, Ys, Zs] = downsample_surface(ld.X_surf, ld.Y_surf, ld.Z_surf, mesh_step);
            % [V24] mesh 渲染时启用 z 裁剪 (also_z=true), 仅影响显示不影响数据
            [Xs, Ys, Zs] = crop_surface_to_bbox(Xs, Ys, Zs, ...
                app.grid_bbox, mesh_margin, true);
            if isempty(Xs), continue; end
            mesh(ax3, Xs, Ys, Zs, ...
                'FaceColor', colors(ii,:), 'EdgeColor', [0.6 0.6 0.6], ...
                'FaceAlpha', alpha, 'EdgeAlpha', 0.3);
        end
    end

    % 当前层活动栅格点
    ld = app.surface_layers{li};
    grids = ld.final_grids;
    if ~isempty(grids)
        gx = arrayfun(@(g) g.x, grids);
        gy = arrayfun(@(g) g.y, grids);
        gz = arrayfun(@(g) g.z, grids);
        scatter3(ax3, gx, gy, gz, 25, [0.5 0.5 0.5], 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 0.3, ...
            'MarkerFaceAlpha', 0.6);

        % 手动选择组高亮
        md = app.manual_per_layer{li};
        if ~isempty(md)
            for k = 1:numel(md.outer_groups)
                g = md.outer_groups{k};
                pi = g.pt_idx;
                % 框内的所有点 (淡色)
                scatter3(ax3, gx(pi), gy(pi), gz(pi), 50, ...
                    [0 0.4 0.9], 'filled', 'MarkerEdgeColor', 'k', ...
                    'LineWidth', 0.3, 'MarkerFaceAlpha', 0.5);
                % [V12] 选择范围 (虚线, 仅参考)
                sel = get_field_or(g, 'selection_poly', get_field_or(g, 'poly_xy', []));
                if ~isempty(sel)
                    z_avg = mean(gz(pi));
                    plot3(ax3, [sel(:,1); sel(1,1)], [sel(:,2); sel(1,2)], ...
                              repmat(z_avg, size(sel,1)+1, 1), ...
                        ':', 'Color', [0.4 0.6 0.95], 'LineWidth', 1);
                end
                % [V12] 实际紧贴轮廓 - 用边界点真实 z
                cxy = get_field_or(g, 'contour_xy', []);
                bpi = get_field_or(g, 'boundary_pt_idx', []);
                if ~isempty(cxy)
                    if ~isempty(bpi) && length(bpi) == size(cxy,1)
                        cz = gz(bpi);
                    else
                        cz = repmat(mean(gz(pi)), size(cxy,1), 1);
                    end
                    plot3(ax3, [cxy(:,1); cxy(1,1)], ...
                              [cxy(:,2); cxy(1,2)], ...
                              [cz(:); cz(1)], ...
                        '-', 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
                    % 高亮轮廓上的点
                    if ~isempty(bpi)
                        scatter3(ax3, gx(bpi), gy(bpi), gz(bpi), 130, ...
                            [0 0.3 0.8], 'LineWidth', 1.5);
                    end
                end
            end
            for k = 1:numel(md.inner_groups)
                g = md.inner_groups{k};
                pi = g.pt_idx;
                scatter3(ax3, gx(pi), gy(pi), gz(pi), 50, ...
                    [0.95 0.1 0.1], 'filled', 'MarkerEdgeColor', 'k', ...
                    'LineWidth', 0.3, 'MarkerFaceAlpha', 0.5);
                sel = get_field_or(g, 'selection_poly', get_field_or(g, 'poly_xy', []));
                if ~isempty(sel)
                    z_avg = mean(gz(pi));
                    plot3(ax3, [sel(:,1); sel(1,1)], [sel(:,2); sel(1,2)], ...
                              repmat(z_avg, size(sel,1)+1, 1), ...
                        ':', 'Color', [0.95 0.5 0.5], 'LineWidth', 1);
                end
                cxy = get_field_or(g, 'contour_xy', []);
                bpi = get_field_or(g, 'boundary_pt_idx', []);
                if ~isempty(cxy)
                    if ~isempty(bpi) && length(bpi) == size(cxy,1)
                        cz = gz(bpi);
                    else
                        cz = repmat(mean(gz(pi)), size(cxy,1), 1);
                    end
                    plot3(ax3, [cxy(:,1); cxy(1,1)], ...
                              [cxy(:,2); cxy(1,2)], ...
                              [cz(:); cz(1)], ...
                        '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 2.5);
                    if ~isempty(bpi)
                        scatter3(ax3, gx(bpi), gy(bpi), gz(bpi), 130, ...
                            [0.85 0.1 0.1], 'LineWidth', 1.5);
                    end
                end
            end
        end
    end

    % 当前层已计算的轮廓 (3D)
    R = app.results_per_layer{li};
    if ~isempty(R) && R.success
        % 用 F_z 把轮廓抬到 Z
        F_z = [];
        if isfield(R, 'F_z_inputs') && ~isempty(R.F_z_inputs) ...
                && isfield(R.F_z_inputs,'X') && ~isempty(R.F_z_inputs.X)
            try
                F_z = scatteredInterpolant(R.F_z_inputs.X, R.F_z_inputs.Y, ...
                    R.F_z_inputs.Z, 'linear', 'nearest');
            catch
            end
        end
        for i = 1:length(R.outer_contours)
            c = R.outer_contours{i};
            z = lift_to_z(c, F_z, ld);
            plot3(ax3, c(:,1), c(:,2), z, '-', ...
                'Color', [0 0.3 0.8], 'LineWidth', 2.5);
        end
        for i = 1:length(R.inner_contours)
            c = R.inner_contours{i};
            z = lift_to_z(c, F_z, ld);
            plot3(ax3, c(:,1), c(:,2), z, '-', ...
                'Color', [0.85 0.1 0.1], 'LineWidth', 2.5);
        end
        for i = 1:length(R.streamlines)
            sl = R.streamlines{i};
            if isempty(sl), continue; end
            z = lift_to_z(sl, F_z, ld);
            plot3(ax3, sl(:,1), sl(:,2), z, '-', ...
                'Color', [0.1 0.6 0.2], 'LineWidth', 1.5);
        end
        % [V12] 3D 路径 (若已预览)
        if isfield(R, 'paths_3d') && ~isempty(R.paths_3d)
            cmap_p = jet(max(length(R.paths_3d), 2));
            for i = 1:length(R.paths_3d)
                p = R.paths_3d{i};
                if isempty(p) || size(p,1) < 2, continue; end
                plot3(ax3, p(:,1), p(:,2), p(:,3), '-', ...
                    'Color', cmap_p(i,:), 'LineWidth', 0.8);
            end
        end
    end

    % 结构遮罩边界 (Z=0)
    if show_mask && app.has_mask
        [mx, my] = boundary(app.mask_poly);
        if ~isempty(grids)
            z_base = min(arrayfun(@(g) g.z, grids));
        else
            z_base = 0;
        end
        plot3(ax3, mx, my, repmat(z_base, length(mx),1), '--', ...
            'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
    end

    % [V14] 双 bbox: 红=grid (全局), 蓝=layer (本层激活栅格)
    if isstruct(app.grid_bbox)
        draw_bbox_wireframe(ax3, app.grid_bbox, [0.8 0.2 0.2]);
    end
    layer_bbox = get_layer_bbox(app, li);
    if isstruct(layer_bbox)
        draw_bbox_wireframe(ax3, layer_bbox, [0.15 0.4 0.85]);
    end

    % [V19] feasible 在 3D 显示 (抬升到曲面)
    draw_feasible_3d(ax3, app, li);

    title(ax3, sprintf(['3D 预览  |  当前层=%d (高亮)  |  显示 %d 层 (+/- %d)'], ...
        li, n_show, n_neigh));
    grid(ax3, 'on'); axis(ax3, 'equal');
    view(ax3, az, el);
    hold(ax3, 'off');
end

%% [V12] 画 bbox 立方体线框
function draw_bbox_wireframe(ax, bb, color)
    x = [bb.x_min, bb.x_max];
    y = [bb.y_min, bb.y_max];
    z = [bb.z_min, bb.z_max];
    % 8 个顶点
    pts = [
        x(1), y(1), z(1); x(2), y(1), z(1);
        x(2), y(2), z(1); x(1), y(2), z(1);
        x(1), y(1), z(2); x(2), y(1), z(2);
        x(2), y(2), z(2); x(1), y(2), z(2)];
    % 12 条棱
    edges = [1 2; 2 3; 3 4; 4 1;
             5 6; 6 7; 7 8; 8 5;
             1 5; 2 6; 3 7; 4 8];
    for e = 1:size(edges,1)
        p1 = pts(edges(e,1), :);
        p2 = pts(edges(e,2), :);
        plot3(ax, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
            '--', 'Color', color, 'LineWidth', 0.8);
    end
end

%% [V19] 3D 视图: 全层 + per-contour feasible 投影到曲面
function draw_feasible_3d(ax3, app, li)
    if li < 1 || li > app.num_layers, return; end
    ld = app.surface_layers{li};

    % 取 F_z (从 prep 缓存)
    F_z = [];
    if ~isempty(app.layer_cache{li}) && isfield(app.layer_cache{li}, 'prep') ...
            && ~isempty(app.layer_cache{li}.prep)
        prep = app.layer_cache{li}.prep;
        if isfield(prep, 'F_z_inputs') && ~isempty(prep.F_z_inputs) ...
                && isfield(prep.F_z_inputs, 'X') && ~isempty(prep.F_z_inputs.X)
            try
                F_z = scatteredInterpolant(prep.F_z_inputs.X, ...
                    prep.F_z_inputs.Y, prep.F_z_inputs.Z, 'linear', 'nearest');
            catch
            end
        end

        % 1) 全层 feasible (深绿虚线)
        if isfield(prep, 'feasible_region') && ~isempty(prep.feasible_region) ...
                && area(prep.feasible_region) > 1e-9
            plot_polyshape_on_surface_3d(ax3, prep.feasible_region, F_z, ld, ...
                '--', [0.1 0.65 0.2], 1.8);
        end
    end

    % 2) per-contour bbox 立方体 (青/品红实线) + feasible 投影 (点划线)
    if isfield(app.h, 'chk_per_feas') && isvalid(app.h.chk_per_feas) ...
            && app.h.chk_per_feas.Value == 0
        return;
    end
    col_outer_3d = [0.0 0.65 0.75];
    col_inner_3d = [0.75 0.2 0.55];
    if ~isempty(app.manual_per_layer{li})
        md = app.manual_per_layer{li};
        for k = 1:numel(md.outer_groups)
            % [V20] per-contour bbox 立方体
            cb = get_field_or(md.outer_groups{k}, 'contour_bbox', []);
            if ~isempty(cb) && isstruct(cb)
                draw_bbox_wireframe(ax3, cb, col_outer_3d);
            end
            % feasible 投影
            per_feas = get_field_or(md.outer_groups{k}, 'feasible_region', []);
            if ~isempty(per_feas) && isa(per_feas, 'polyshape') && area(per_feas) > 1e-9
                plot_polyshape_on_surface_3d(ax3, per_feas, F_z, ld, ...
                    '-.', col_outer_3d, 1.0);
            end
        end
        for k = 1:numel(md.inner_groups)
            cb = get_field_or(md.inner_groups{k}, 'contour_bbox', []);
            if ~isempty(cb) && isstruct(cb)
                draw_bbox_wireframe(ax3, cb, col_inner_3d);
            end
            per_feas = get_field_or(md.inner_groups{k}, 'feasible_region', []);
            if ~isempty(per_feas) && isa(per_feas, 'polyshape') && area(per_feas) > 1e-9
                plot_polyshape_on_surface_3d(ax3, per_feas, F_z, ld, ...
                    '-.', col_inner_3d, 1.0);
            end
        end
    end
end

%% [V19] 把 polyshape 边界用 F_z 抬升到 3D 曲面
function plot_polyshape_on_surface_3d(ax3, ps, F_z, ld, linestyle, color, lw)
    if isempty(ps) || ~isa(ps, 'polyshape') || area(ps) < 1e-9, return; end
    try
        [bx, by] = boundary(ps);
        % 拆 NaN 分隔的多段, 每段独立画
        ni = find(isnan(bx));
        if isempty(ni)
            bz = lift_to_z([bx, by], F_z, ld);
            plot3(ax3, bx, by, bz, linestyle, 'Color', color, 'LineWidth', lw);
        else
            prev = 1;
            for si = [ni(:)', length(bx)+1]
                seg_x = bx(prev:si-1);
                seg_y = by(prev:si-1);
                if length(seg_x) >= 2
                    seg_z = lift_to_z([seg_x, seg_y], F_z, ld);
                    plot3(ax3, seg_x, seg_y, seg_z, linestyle, ...
                        'Color', color, 'LineWidth', lw);
                end
                prev = si + 1;
            end
        end
    catch
    end
end

function z = lift_to_z(xy, F_z, layer_data)
    % 把 2D 点抬到 3D (使用 F_z 插值器或最近点法)
    n = size(xy,1);
    z = nan(n,1);
    if ~isempty(F_z)
        try
            z = F_z(xy(:,1), xy(:,2));
        catch
        end
    end
    % 兜底: 用 layer_data.Z_surf 最近点
    bad = ~isfinite(z);
    if any(bad) && ~isempty(layer_data.X_surf)
        Xf = layer_data.X_surf(:);
        Yf = layer_data.Y_surf(:);
        Zf = layer_data.Z_surf(:);
        ok = isfinite(Xf) & isfinite(Yf) & isfinite(Zf);
        Xf=Xf(ok); Yf=Yf(ok); Zf=Zf(ok);
        for k = find(bad(:))'
            d = (Xf-xy(k,1)).^2 + (Yf-xy(k,2)).^2;
            [~, idx] = min(d);
            z(k) = Zf(idx);
        end
    end
    z(~isfinite(z)) = layer_data.offset;  % 最终兜底
end

%% [V12] 曲面网格稀疏化
function [Xs, Ys, Zs] = downsample_surface(X, Y, Z, step)
    if step <= 1
        Xs = X; Ys = Y; Zs = Z;
        return;
    end
    [r, c] = size(X);
    ri = unique([1:step:r, r]);   % 保留首尾边界
    ci = unique([1:step:c, c]);
    Xs = X(ri, ci);
    Ys = Y(ri, ci);
    Zs = Z(ri, ci);
end

%% [V15] 按 bbox 裁剪曲面 mesh, 减少渲染顶点数
function [Xc, Yc, Zc] = crop_surface_to_bbox(X, Y, Z, bbox, margin_factor, also_z)
% 按 bbox 裁剪曲面: 完全在 bbox+margin 外的边缘行/列删除, 子矩阵剩余越界点设 NaN.
% margin_factor: bbox 之外保留几倍体素的 margin (默认 2)
% also_z (默认 false): 是否也对 Z 维度裁剪
%   false -> 只裁 XY, Z 保留完整 (用于加载时的数据预处理, 避免 F_z 插值丢点)
%   true  -> XYZ 都裁 (用于 mesh 显示, 让超出 bbox.z 的曲面凹陷不显示)
    Xc = X; Yc = Y; Zc = Z;
    if ~isstruct(bbox) || isempty(X)
        return;
    end
    if nargin < 5 || isempty(margin_factor)
        margin_factor = 2;
    end
    if nargin < 6 || isempty(also_z)
        also_z = false;
    end

    ex = margin_factor * bbox.vox_dx;
    ey = margin_factor * bbox.vox_dy;
    xmin = bbox.x_min - ex; xmax = bbox.x_max + ex;
    ymin = bbox.y_min - ey; ymax = bbox.y_max + ey;

    if also_z
        ez = margin_factor * bbox.vox_dz;
        zmin = bbox.z_min - ez; zmax = bbox.z_max + ez;
        in = (X >= xmin) & (X <= xmax) & ...
             (Y >= ymin) & (Y <= ymax) & ...
             (Z >= zmin) & (Z <= zmax);
    else
        in = (X >= xmin) & (X <= xmax) & ...
             (Y >= ymin) & (Y <= ymax);
    end

    % 找出至少包含一个 in 点的行/列
    row_has = any(in, 2);
    col_has = any(in, 1);
    if ~any(row_has) || ~any(col_has)
        Xc = []; Yc = []; Zc = []; return;
    end

    r_first = find(row_has, 1, 'first');
    r_last  = find(row_has, 1, 'last');
    c_first = find(col_has, 1, 'first');
    c_last  = find(col_has, 1, 'last');

    % 边界外侧加 1 行/列 (避免锯齿)
    r_first = max(1, r_first - 1);
    r_last  = min(size(X,1), r_last + 1);
    c_first = max(1, c_first - 1);
    c_last  = min(size(X,2), c_last + 1);

    Xc = X(r_first:r_last, c_first:c_last);
    Yc = Y(r_first:r_last, c_first:c_last);
    Zc = Z(r_first:r_last, c_first:c_last);

    % 子矩阵里剩余越界点设为 NaN
    if also_z
        sub_in = (Xc >= xmin) & (Xc <= xmax) & ...
                 (Yc >= ymin) & (Yc <= ymax) & ...
                 (Zc >= zmin) & (Zc <= zmax);
    else
        sub_in = (Xc >= xmin) & (Xc <= xmax) & ...
                 (Yc >= ymin) & (Yc <= ymax);
    end
    Xc(~sub_in) = NaN;
    Yc(~sub_in) = NaN;
    Zc(~sub_in) = NaN;
end


%% [V13] 距离阈值聚类 (连通图 + BFS, 不依赖 Toolbox)
function clusters = cluster_points_by_distance(xs, ys, threshold)
% 输入: xs, ys (N×1 点坐标), threshold (距离阈值)
% 输出: clusters (N×1, 每个点的簇标号 1..K)
% 算法: 距离 < threshold 的两点视为相邻, 连通分量即为一簇.
% threshold = inf 时退化为单一簇 (所有点连通)
% 复杂度: O(N^2) 距离矩阵, 适合 N < 5000
    xs = xs(:); ys = ys(:);
    n = numel(xs);
    if n == 0, clusters = zeros(0,1); return; end
    if n == 1, clusters = 1; return; end
    if ~isfinite(threshold) || threshold <= 0
        % 单一簇
        clusters = ones(n, 1);
        return;
    end
    % 两两距离 (避免依赖 pdist2)
    dx = xs - xs.';
    dy = ys - ys.';
    D = sqrt(dx.^2 + dy.^2);
    adj = (D < threshold) & (D > 0);   % 邻接 (排除自连接)

    % BFS 找连通分量
    clusters = zeros(n, 1);
    cid = 0;
    for i = 1:n
        if clusters(i) > 0, continue; end
        cid = cid + 1;
        queue = i;
        clusters(i) = cid;
        while ~isempty(queue)
            cur = queue(1);
            queue(1) = [];
            % 找未访问的邻居
            nbrs = find(adj(cur, :)' & clusters == 0);
            clusters(nbrs) = cid;
            queue = [queue; nbrs]; %#ok<AGROW>
        end
    end
end


%% [V16] 该层可行 XY 区域 = bbox 矩形 ∩ alphaShape(曲面上 z 在 bbox.z 范围内的点)
function feasible = compute_feasible_region(X_surf, Y_surf, Z_surf, grid_bbox)
% 输出: polyshape (可能为空) - 该层路径合法可走的最大 XY 范围
% 实际可行区域 = bbox 的 xy 矩形 与 该层曲面在 z ∈ bbox.z 范围内的 (x,y) 投影集合 的交集
    feasible = polyshape();
    if ~isstruct(grid_bbox) || isempty(X_surf)
        return;
    end

    Xf = X_surf(:); Yf = Y_surf(:); Zf = Z_surf(:);
    valid = isfinite(Xf) & isfinite(Yf) & isfinite(Zf) & ...
            Zf >= grid_bbox.z_min & Zf <= grid_bbox.z_max & ...
            Xf >= grid_bbox.x_min & Xf <= grid_bbox.x_max & ...
            Yf >= grid_bbox.y_min & Yf <= grid_bbox.y_max;
    if sum(valid) < 3
        return;
    end
    vx = Xf(valid); vy = Yf(valid);

    % alphaShape 算可行 XY 边界
    surface_xy = polyshape();
    try
        alpha_r = max([grid_bbox.vox_dx, grid_bbox.vox_dy]) * 2.5;
        shp = alphaShape(vx, vy, alpha_r);
        if isempty(shp.Points)
            % alphaShape 失败时用 convex hull 兜底
            k = convhull(vx, vy);
            surface_xy = polyshape(vx(k), vy(k));
        else
            [bf, bv] = boundaryFacets(shp);
            if isempty(bf)
                k = convhull(vx, vy);
                surface_xy = polyshape(vx(k), vy(k));
            else
                % 把 alphaShape 边界拼成 polyshape (可能多个环)
                surface_xy = alphashape_facets_to_polyshape(bf, bv);
                if area(surface_xy) < 1e-9
                    k = convhull(vx, vy);
                    surface_xy = polyshape(vx(k), vy(k));
                end
            end
        end
    catch
        try
            k = convhull(vx, vy);
            surface_xy = polyshape(vx(k), vy(k));
        catch
            return;
        end
    end

    % 与 bbox xy 矩形取交集
    try
        feasible = intersect(surface_xy, grid_bbox.poly_xy);
    catch
        feasible = surface_xy;
    end
end

%% [V19] per-contour feasible: 用激活点的 alphaShape (适度膨胀) ∩ global_feasible
%% [V20] per-contour bbox: 该轮廓激活点的最小 axis-aligned 长方体 (+ margin)
function pc_bbox = make_per_contour_bbox(pt_xs, pt_ys, pt_zs, grid_bbox)
% 输入: 该轮廓选定激活点 + 全局 bbox (用其 margin / vox 间距)
% 输出: pc_bbox struct (与 grid_bbox / layer_bbox 同构)
    pc_bbox = struct();
    if isstruct(grid_bbox)
        mx = grid_bbox.margin_x;
        my = grid_bbox.margin_y;
        mz = grid_bbox.margin_z;
        vx = grid_bbox.vox_dx;
        vy = grid_bbox.vox_dy;
        vz = grid_bbox.vox_dz;
    else
        mx = 0.5; my = 0.5; mz = 0.5;
        vx = 1.0; vy = 1.0; vz = 1.0;
    end
    pc_bbox.x_min_raw = min(pt_xs); pc_bbox.x_max_raw = max(pt_xs);
    pc_bbox.y_min_raw = min(pt_ys); pc_bbox.y_max_raw = max(pt_ys);
    pc_bbox.z_min_raw = min(pt_zs); pc_bbox.z_max_raw = max(pt_zs);
    pc_bbox.margin_x = mx; pc_bbox.margin_y = my; pc_bbox.margin_z = mz;
    pc_bbox.vox_dx = vx;   pc_bbox.vox_dy = vy;   pc_bbox.vox_dz = vz;
    % xy 维度: 激活点的最小 bbox (局部紧)
    pc_bbox.x_min = pc_bbox.x_min_raw - mx;
    pc_bbox.x_max = pc_bbox.x_max_raw + mx;
    pc_bbox.y_min = pc_bbox.y_min_raw - my;
    pc_bbox.y_max = pc_bbox.y_max_raw + my;
    % [V25] z 维度: 用 grid_bbox.z (全局) 替代激活点 z 范围
    %   激活点 z = 栅格中心 z, 同层跨度通常很窄, 加半体素 margin 仍不够覆盖曲面真实 z 范围;
    %   会导致 compute_feasible_region 用 pc_bbox 时把曲面 XY 投影切碎.
    %   xy 维度的硬约束已经够用, z 维度交给 grid_bbox 全局即可.
    if isstruct(grid_bbox)
        pc_bbox.z_min = grid_bbox.z_min;
        pc_bbox.z_max = grid_bbox.z_max;
    else
        pc_bbox.z_min = pc_bbox.z_min_raw - mz;
        pc_bbox.z_max = pc_bbox.z_max_raw + mz;
    end
    pc_bbox.poly_xy = polyshape(...
        [pc_bbox.x_min, pc_bbox.x_max, pc_bbox.x_max, pc_bbox.x_min], ...
        [pc_bbox.y_min, pc_bbox.y_min, pc_bbox.y_max, pc_bbox.y_max]);
end

%% [V20] per-contour feasible: pc_bbox 投影到曲面
%   = 曲面在 pc_bbox 内的点的 XY 投影区域 (polyshape)
%   逻辑等同 compute_feasible_region(全曲面, grid_bbox), 但 bbox 换成 pc_bbox
function poly = compute_per_contour_feasible(pt_xs, pt_ys, pt_zs, ...
        X_surf, Y_surf, Z_surf, grid_bbox)
    poly = polyshape();
    pt_xs = pt_xs(:); pt_ys = pt_ys(:); pt_zs = pt_zs(:);
    if numel(pt_xs) < 3, return; end
    pc_bbox = make_per_contour_bbox(pt_xs, pt_ys, pt_zs, grid_bbox);
    % 复用 global_feasible 的算法, 只是 bbox 不同
    try
        poly = compute_feasible_region(X_surf, Y_surf, Z_surf, pc_bbox);
    catch
        poly = polyshape();
    end
end

%% [V16] alphaShape boundaryFacets 拼成 polyshape (按连通环组装)
function poly = alphashape_facets_to_polyshape(facets, vertices)
% facets: M×2 (每条边的两个顶点索引), vertices: N×2 (xy 坐标)
% 输出: polyshape (可能含多个环)
    poly = polyshape();
    if isempty(facets) || isempty(vertices), return; end

    % 把 facets 看成图, 找连通环
    nv = size(vertices, 1);
    adj = cell(nv, 1);
    for k = 1:size(facets, 1)
        a = facets(k, 1); b = facets(k, 2);
        adj{a}(end+1) = b;
        adj{b}(end+1) = a;
    end

    visited = false(nv, 1);
    rings = {};
    for start = 1:nv
        if visited(start) || isempty(adj{start}), continue; end
        % 沿邻居走环
        ring = start;
        visited(start) = true;
        cur = start; prev = -1;
        guard = 0;
        while guard < nv + 5
            guard = guard + 1;
            nbrs = adj{cur};
            nbrs = nbrs(nbrs ~= prev);
            if isempty(nbrs), break; end
            nxt = nbrs(1);
            if nxt == start, break; end   % 闭环
            ring(end+1) = nxt; %#ok<AGROW>
            visited(nxt) = true;
            prev = cur; cur = nxt;
        end
        if length(ring) >= 3
            rings{end+1} = ring(:); %#ok<AGROW>
        end
    end

    if isempty(rings), return; end
    % 用 NaN 分隔多个环传给 polyshape
    xs = []; ys = [];
    for k = 1:length(rings)
        r = rings{k};
        xs = [xs; vertices(r, 1); NaN]; %#ok<AGROW>
        ys = [ys; vertices(r, 2); NaN]; %#ok<AGROW>
    end
    if ~isempty(xs)
        xs(end) = []; ys(end) = [];  % 去掉末尾 NaN
    end
    try
        poly = polyshape(xs, ys, 'Simplify', true);
    catch
        try
            % fallback: 只用最大环
            [~, mi] = max(cellfun(@length, rings));
            r = rings{mi};
            poly = polyshape(vertices(r, 1), vertices(r, 2));
        catch
            poly = polyshape();
        end
    end
end

%% [V17] 通用裁剪: 单轮廓 (N×2) 与 feasible polyshape 取交, 返回最大段
function c_out = clip_contour_with_feasible(c_in, feasible)
    c_out = c_in;
    if isempty(c_in) || size(c_in, 1) < 3, return; end
    if isempty(feasible) || ~isa(feasible, 'polyshape') || area(feasible) < 1e-9
        return;
    end
    try
        pp = polyshape(c_in(:,1), c_in(:,2));
        cc = intersect(pp, feasible);
        if area(cc) < 1e-9, return; end
        [bx, by] = boundary(cc);
        ni = find(isnan(bx));
        if isempty(ni)
            c_out = [bx, by];
        else
            segs = {}; prev = 1;
            for si = [ni(:)', length(bx)+1]
                sg = [bx(prev:si-1), by(prev:si-1)];
                if size(sg,1) >= 3, segs{end+1} = sg; end %#ok<AGROW>
                prev = si + 1;
            end
            if ~isempty(segs)
                [~, mi] = max(cellfun(@(s) size(s,1), segs));
                c_out = segs{mi};
            end
        end
    catch
    end
end

%% ================================================================
%% [V12] XYZ bbox 工具
%% ================================================================
function bbox = compute_grid_bbox(grid_data)
% 计算完整网格的 XYZ bbox + 半体素 margin
% 输入: grid_data 是 nelx_g x nely_g x nelz_g 的 struct 数组
% 输出: bbox struct 含 x_min/max, y_min/max, z_min/max, margin_x/y/z, poly_xy

    sz = size(grid_data);
    n = prod(sz);
    xs = zeros(n,1); ys = zeros(n,1); zs = zeros(n,1);
    idx = 0;
    for k = 1:sz(3)
        for j = 1:sz(2)
            for i = 1:sz(1)
                idx = idx + 1;
                g = grid_data(i,j,k);
                xs(idx) = g.x; ys(idx) = g.y; zs(idx) = g.z;
            end
        end
    end

    bbox = struct();
    bbox.x_min_raw = min(xs); bbox.x_max_raw = max(xs);
    bbox.y_min_raw = min(ys); bbox.y_max_raw = max(ys);
    bbox.z_min_raw = min(zs); bbox.z_max_raw = max(zs);

    % 体素间距 (相邻去重排序后取中位)
    dx_u = diff(sort(unique(xs))); dx_u = dx_u(dx_u > 1e-6);
    dy_u = diff(sort(unique(ys))); dy_u = dy_u(dy_u > 1e-6);
    dz_u = diff(sort(unique(zs))); dz_u = dz_u(dz_u > 1e-6);
    vox_dx = ternary(isempty(dx_u), 1.0, median(dx_u));
    vox_dy = ternary(isempty(dy_u), 1.0, median(dy_u));
    vox_dz = ternary(isempty(dz_u), 1.0, median(dz_u));

    bbox.margin_x = 0.5 * vox_dx;
    bbox.margin_y = 0.5 * vox_dy;
    bbox.margin_z = 0.5 * vox_dz;

    % [V13] 暴露体素间距 (供聚类用)
    bbox.vox_dx = vox_dx;
    bbox.vox_dy = vox_dy;
    bbox.vox_dz = vox_dz;

    bbox.x_min = bbox.x_min_raw - bbox.margin_x;
    bbox.x_max = bbox.x_max_raw + bbox.margin_x;
    bbox.y_min = bbox.y_min_raw - bbox.margin_y;
    bbox.y_max = bbox.y_max_raw + bbox.margin_y;
    bbox.z_min = bbox.z_min_raw - bbox.margin_z;
    bbox.z_max = bbox.z_max_raw + bbox.margin_z;

    % xy 矩形 polyshape (用于裁剪)
    bbox.poly_xy = polyshape(...
        [bbox.x_min, bbox.x_max, bbox.x_max, bbox.x_min], ...
        [bbox.y_min, bbox.y_min, bbox.y_max, bbox.y_max]);
end

function segs_out = clip_contour_to_xyz_bbox(contour, bbox, F_z, strict_z)
% 把 2D 轮廓裁剪到 xyz bbox 内 (xy: polyshape ∩; z: F_z 采样)
% strict_z: true=z 超界整段丢弃 (内轮廓用); false=拆开重组 (外轮廓用)
% 返回: cell of N_k × 2 子段
    segs_out = {};
    if isempty(contour) || size(contour, 1) < 3, return; end

    try
        c_poly = polyshape(contour(:,1), contour(:,2));
    catch
        return;
    end
    try
        clipped = intersect(c_poly, bbox.poly_xy);
    catch
        return;
    end
    if area(clipped) < 1e-9, return; end

    [bx, by] = boundary(clipped);
    ni = find(isnan(bx));
    candidates = {};
    if isempty(ni)
        candidates{1} = [bx, by];
    else
        prev = 1;
        for si = [ni(:)', length(bx)+1]
            sg = [bx(prev:si-1), by(prev:si-1)];
            if size(sg,1) >= 3
                candidates{end+1} = sg; %#ok<AGROW>
            end
            prev = si + 1;
        end
    end

    if isempty(F_z)
        segs_out = candidates;
        return;
    end

    for k = 1:length(candidates)
        seg = candidates{k};
        try
            zs = F_z(seg(:,1), seg(:,2));
        catch
            zs = nan(size(seg,1),1);
        end
        ok = isfinite(zs) & zs >= bbox.z_min & zs <= bbox.z_max;
        if all(ok)
            segs_out{end+1} = seg; %#ok<AGROW>
        elseif ~any(ok)
            continue;
        elseif strict_z
            continue;
        else
            sub = split_by_mask(seg, ok);
            for s = 1:length(sub)
                if size(sub{s}, 1) >= 3
                    segs_out{end+1} = sub{s}; %#ok<AGROW>
                end
            end
        end
    end
end

function subs = split_by_mask(seg, mask)
% 按 mask=true 的连续区间切分
    subs = {};
    n = numel(mask);
    if n == 0, return; end
    start = 0;
    for i = 1:n
        if mask(i) && start == 0
            start = i;
        elseif ~mask(i) && start > 0
            if i - start >= 2
                subs{end+1} = seg(start:i-1, :); %#ok<AGROW>
            end
            start = 0;
        end
    end
    if start > 0 && n - start + 1 >= 2
        subs{end+1} = seg(start:end, :); %#ok<AGROW>
    end
end

function reg_out = clip_polyshape_to_bbox(reg, bbox, F_z)
% 把 polyshape 区域裁剪到 xyz bbox
    try
        reg_out = intersect(reg, bbox.poly_xy);
    catch
        reg_out = polyshape.empty;
        return;
    end
    if isempty(reg_out) || area(reg_out) < 1e-9
        reg_out = polyshape.empty;
        return;
    end
    if isempty(F_z)
        return;
    end
    parts = regions(reg_out);
    good = polyshape.empty;
    for p = 1:length(parts)
        try
            [cx, cy] = centroid(parts(p));
            z = F_z(cx, cy);
            if isfinite(z) && z >= bbox.z_min && z <= bbox.z_max
                good(end+1) = parts(p); %#ok<AGROW>
            end
        catch
        end
    end
    if isempty(good)
        reg_out = polyshape.empty;
    else
        reg_out = good(1);
        for p = 2:length(good)
            try, reg_out = union(reg_out, good(p)); catch, end
        end
    end
end

function [contours_out, src_out] = apply_bbox_clip(contours_in, src_in, bbox, F_z, strict_z)
% 批量裁剪轮廓 cell 数组到 bbox, 同步更新 source_id 数组
% strict_z: true=内轮廓(z 超界整段丢弃); false=外轮廓(可拆段)
    contours_out = {};
    src_out      = {};
    for k = 1:length(contours_in)
        c = contours_in{k};
        sid = '';
        if k <= length(src_in), sid = src_in{k}; end
        segs = clip_contour_to_xyz_bbox(c, bbox, F_z, strict_z);
        for s = 1:length(segs)
            contours_out{end+1} = segs{s}; %#ok<AGROW>
            % 多段共享同一 source_id (但加后缀避免完全相同)
            if length(segs) > 1
                src_out{end+1} = sprintf('%s_p%d', sid, s); %#ok<AGROW>
            else
                src_out{end+1} = sid; %#ok<AGROW>
            end
        end
    end
end

%% [V16] 批量用 feasible_region 封闭轮廓 (替代 apply_bbox_clip 用于轮廓阶段)
function [contours_out, src_out] = apply_feasible_clip(contours_in, src_in, feasible, is_inner)
% 用 polyshape intersect 把轮廓裁剪到 feasible_region 内
%   - 原轮廓在 feasible 内的部分保留
%   - 超出部分被 feasible 边界替代 -> 自动闭合
%   - 多段时:
%     - 外轮廓 (is_inner=false): 全部保留
%     - 内轮廓 (is_inner=true):  只取最大段 (孔不能破碎)
    contours_out = {};
    src_out      = {};
    for k = 1:length(contours_in)
        c = contours_in{k};
        sid = '';
        if k <= length(src_in), sid = src_in{k}; end
        if isempty(c) || size(c,1) < 3
            continue;
        end
        try
            c_poly = polyshape(c(:,1), c(:,2));
        catch
            % 退回保留原轮廓
            contours_out{end+1} = c; %#ok<AGROW>
            src_out{end+1}      = sid; %#ok<AGROW>
            continue;
        end
        try
            clipped = intersect(c_poly, feasible);
        catch
            contours_out{end+1} = c; %#ok<AGROW>
            src_out{end+1}      = sid; %#ok<AGROW>
            continue;
        end
        if area(clipped) < 1e-9
            continue;   % 完全在 feasible 外, 丢弃
        end

        [bx, by] = boundary(clipped);
        ni = find(isnan(bx));
        if isempty(ni)
            % 单段 - 直接保留
            contours_out{end+1} = [bx, by]; %#ok<AGROW>
            src_out{end+1}      = sid; %#ok<AGROW>
        else
            % 多段 - 拆开
            segs = {}; prev = 1;
            for si = [ni(:)', length(bx)+1]
                sg = [bx(prev:si-1), by(prev:si-1)];
                if size(sg,1) >= 3, segs{end+1} = sg; end %#ok<AGROW>
                prev = si + 1;
            end
            if isempty(segs), continue; end
            if is_inner
                % 内轮廓: 只取最大段
                [~, mi] = max(cellfun(@(s) size(s,1), segs));
                contours_out{end+1} = segs{mi}; %#ok<AGROW>
                src_out{end+1}      = sid; %#ok<AGROW>
            else
                % 外轮廓: 全保留, source_id 加后缀
                for s = 1:length(segs)
                    contours_out{end+1} = segs{s}; %#ok<AGROW>
                    if length(segs) > 1
                        src_out{end+1} = sprintf('%s_p%d', sid, s); %#ok<AGROW>
                    else
                        src_out{end+1} = sid; %#ok<AGROW>
                    end
                end
            end
        end
    end
end


%% ================================================================
%% [V9] 阶段一预备: 与参数无关的中间数据 (缓存复用)
%% ================================================================
function prep = prepare_layer(layer_data, grid_data, grid_bbox)
% 输入: layer_data (surface_layers{li}) + grid_data (全局体素网格) + grid_bbox (全局 bbox)
% 输出: prep struct, 含 pointCloud, F_z 输入点, 2D 投影 xold/t, 坐标映射,
%       layer_bbox (该层激活栅格 bbox), effective_bbox (grid ∩ layer 交集)
% 这部分计算与所有可调参数都无关. 切片数据不变, prep 永远不需要重算.

    prep = [];
    activated_grids = layer_data.final_grids;
    X_surf = layer_data.X_surf;
    Y_surf = layer_data.Y_surf;
    Z_surf = layer_data.Z_surf;

    if isempty(activated_grids) || length(activated_grids) < 3
        return;
    end

    p = struct();

    % 点云
    p.pointCloud_data = create_pointcloud_from_surface(X_surf, Y_surf, Z_surf);

    % F_z 输入点
    X_flat = X_surf(:); Y_flat = Y_surf(:); Z_flat = Z_surf(:);
    valid_pts = all(isfinite([X_flat, Y_flat, Z_flat]), 2);
    p.F_z_inputs = struct();
    if sum(valid_pts) >= 3
        p.F_z_inputs.X = X_flat(valid_pts);
        p.F_z_inputs.Y = Y_flat(valid_pts);
        p.F_z_inputs.Z = Z_flat(valid_pts);
    end

    % 二维投影
    [p.xold, p.t, p.nelx, p.nely, ~] = extract_layer_2d_projection(activated_grids, grid_data);

    % 坐标映射 (含 z 范围)
    num_g = length(activated_grids);
    ax_arr = zeros(num_g,1); ay_arr = zeros(num_g,1); az_arr = zeros(num_g,1);
    for i = 1:num_g
        ax_arr(i) = activated_grids(i).x;
        ay_arr(i) = activated_grids(i).y;
        az_arr(i) = activated_grids(i).z;   % [V14] 也取 z
    end
    p.actual_x_min = min(ax_arr); p.actual_x_max = max(ax_arr);
    p.actual_y_min = min(ay_arr); p.actual_y_max = max(ay_arr);
    p.actual_z_min = min(az_arr); p.actual_z_max = max(az_arr);   % [V14]
    p.actual_range = max(p.actual_x_max - p.actual_x_min, ...
                        p.actual_y_max - p.actual_y_min);
    p.scale_x  = (p.actual_x_max - p.actual_x_min) / max(p.nelx - 1, 1);
    p.scale_y  = (p.actual_y_max - p.actual_y_min) / max(p.nely - 1, 1);
    p.activated_xy = [ax_arr, ay_arr];

    % [V27] 诊断: 判断 nelx/nely 是激活子集尺寸还是 grid_data 全集尺寸
    %   - 如果 nelx 接近激活子集尺寸: V6 原行为 (actual_x_min + scale_x) 正确
    %   - 如果 nelx 接近 grid_data 全集尺寸: 必须用 grid_bbox 物理坐标系
    p.bnd_coord_uses_grid_bbox = false;   % 默认 V6 行为
    if nargin >= 3 && isstruct(grid_bbox) && isfield(grid_bbox, 'vox_dx') ...
            && isfield(grid_bbox, 'x_min_raw') && isfield(grid_bbox, 'x_max_raw')
        n_grid_x = round((grid_bbox.x_max_raw - grid_bbox.x_min_raw) / grid_bbox.vox_dx) + 1;
        n_grid_y = round((grid_bbox.y_max_raw - grid_bbox.y_min_raw) / grid_bbox.vox_dy) + 1;
        n_active_x = round((p.actual_x_max - p.actual_x_min) / grid_bbox.vox_dx) + 1;
        n_active_y = round((p.actual_y_max - p.actual_y_min) / grid_bbox.vox_dy) + 1;
        % 判断 nelx 更接近哪个
        if abs(p.nelx - n_grid_x) < abs(p.nelx - n_active_x) || ...
                abs(p.nely - n_grid_y) < abs(p.nely - n_active_y)
            p.bnd_coord_uses_grid_bbox = true;
            fprintf('[PREP] nelx=%d nely=%d  grid_data=%dx%d  active=%dx%d  -> 用 grid_bbox 坐标系\n', ...
                p.nelx, p.nely, n_grid_x, n_grid_y, n_active_x, n_active_y);
        else
            fprintf('[PREP] nelx=%d nely=%d  grid_data=%dx%d  active=%dx%d  -> 用激活子集坐标系 (V6)\n', ...
                p.nelx, p.nely, n_grid_x, n_grid_y, n_active_x, n_active_y);
        end
    end

    % [V14] 构造 layer_bbox 和 effective_bbox
    if nargin >= 3 && isstruct(grid_bbox)
        p.layer_bbox     = make_layer_bbox(p, grid_bbox);
        p.effective_bbox = intersect_bboxes(grid_bbox, p.layer_bbox);
        % [V16] 计算可行 XY 区域 (该层路径合法可走的最大 polyshape)
        try
            p.feasible_region = compute_feasible_region(X_surf, Y_surf, Z_surf, grid_bbox);
        catch ME_fr
            fprintf(2, '[WARN] compute_feasible_region failed: %s\n', ME_fr.message);
            p.feasible_region = polyshape();
        end
    else
        p.layer_bbox      = [];
        p.effective_bbox  = [];
        p.feasible_region = polyshape();
    end

    prep = p;
end

%% [V14] 从 prep 构造 layer_bbox - 用 grid_bbox 的体素间距作 margin
function bbox = make_layer_bbox(prep, grid_bbox)
    mx = grid_bbox.margin_x;
    my = grid_bbox.margin_y;
    mz = grid_bbox.margin_z;
    bbox = struct();
    bbox.x_min_raw = prep.actual_x_min;
    bbox.x_max_raw = prep.actual_x_max;
    bbox.y_min_raw = prep.actual_y_min;
    bbox.y_max_raw = prep.actual_y_max;
    bbox.z_min_raw = prep.actual_z_min;
    bbox.z_max_raw = prep.actual_z_max;
    bbox.margin_x = mx;
    bbox.margin_y = my;
    bbox.margin_z = mz;
    bbox.vox_dx = grid_bbox.vox_dx;
    bbox.vox_dy = grid_bbox.vox_dy;
    bbox.vox_dz = grid_bbox.vox_dz;
    bbox.x_min = bbox.x_min_raw - mx;
    bbox.x_max = bbox.x_max_raw + mx;
    bbox.y_min = bbox.y_min_raw - my;
    bbox.y_max = bbox.y_max_raw + my;
    bbox.z_min = bbox.z_min_raw - mz;
    bbox.z_max = bbox.z_max_raw + mz;
    bbox.poly_xy = polyshape(...
        [bbox.x_min, bbox.x_max, bbox.x_max, bbox.x_min], ...
        [bbox.y_min, bbox.y_min, bbox.y_max, bbox.y_max]);
end

%% [V14] 两个 bbox 取交集 (字段同构)
function bbox = intersect_bboxes(b1, b2)
    bbox = struct();
    bbox.x_min = max(b1.x_min, b2.x_min);
    bbox.x_max = min(b1.x_max, b2.x_max);
    bbox.y_min = max(b1.y_min, b2.y_min);
    bbox.y_max = min(b1.y_max, b2.y_max);
    bbox.z_min = max(b1.z_min, b2.z_min);
    bbox.z_max = min(b1.z_max, b2.z_max);
    % 退化检查
    if bbox.x_min >= bbox.x_max || bbox.y_min >= bbox.y_max
        bbox.x_min = b2.x_min; bbox.x_max = b2.x_max;
        bbox.y_min = b2.y_min; bbox.y_max = b2.y_max;
    end
    if bbox.z_min >= bbox.z_max
        bbox.z_min = b2.z_min; bbox.z_max = b2.z_max;
    end
    bbox.margin_x = min(b1.margin_x, b2.margin_x);
    bbox.margin_y = min(b1.margin_y, b2.margin_y);
    bbox.margin_z = min(b1.margin_z, b2.margin_z);
    bbox.vox_dx = min(b1.vox_dx, b2.vox_dx);
    bbox.vox_dy = min(b1.vox_dy, b2.vox_dy);
    bbox.vox_dz = min(b1.vox_dz, b2.vox_dz);
    bbox.poly_xy = polyshape(...
        [bbox.x_min, bbox.x_max, bbox.x_max, bbox.x_min], ...
        [bbox.y_min, bbox.y_min, bbox.y_max, bbox.y_max]);
end


%% ================================================================
%% 阶段一核心: 计算轮廓 + 流线 + 区域
%% (支持自动/手动两种模式)
%% ================================================================
function R = compute_contours_streamlines(layer_idx, prep, layer_data, ...
        params, has_img, has_mask, mask_poly, has_zhf, zhf_data, ...
        is_manual, manual_data, grid_bbox)
% [V9]  prep 已由 prepare_layer 预计算并缓存
% [V12] grid_bbox: 完整网格的 xyz bbox
% [V14] 函数内部使用 effective_bbox = grid_bbox ∩ layer_bbox (来自 prep)
%       作为实际硬约束 (更紧)

    % [V14] 优先用 prep.effective_bbox (= grid_bbox ∩ layer_bbox)
    % [V28] 但保留 orig_grid_bbox (含 x_min_raw/y_min_raw) 给 V27 bnd 坐标转换用
    orig_grid_bbox = grid_bbox;
    if isstruct(prep) && isfield(prep, 'effective_bbox') && ...
            ~isempty(prep.effective_bbox)
        grid_bbox = prep.effective_bbox;
    end

    R = struct();
    R.layer_idx       = layer_idx;
    R.offset          = layer_data.offset;
    R.success         = false;
    R.error_message   = '';
    R.outer_contours  = {};
    R.inner_contours  = {};
    R.streamlines     = {};
    R.medial_axis     = [];
    R.regions         = {};
    R.activated_xy    = [];
    R.burn_xy         = 0;
    R.burn_z          = 0;
    R.processing_time = 0;
    R.pointCloud_data = [];
    R.F_z_inputs      = struct();
    R.scale_xy        = struct();
    R.adaptive_min_outer = 0;
    R.expand_dist     = 0;

    try
        % ----- 从 prep 取出预计算结果 -----
        if isempty(prep) || ~isstruct(prep)
            R.error_message = 'prep cache is empty';
            return;
        end
        pointCloud_data = prep.pointCloud_data;
        F_z_struct      = prep.F_z_inputs;
        xold            = prep.xold;
        t               = prep.t;
        nelx            = prep.nelx;
        nely            = prep.nely;
        actual_x_min    = prep.actual_x_min;
        actual_x_max    = prep.actual_x_max;
        actual_y_min    = prep.actual_y_min;
        actual_y_max    = prep.actual_y_max;
        actual_range    = prep.actual_range;
        scale_x         = prep.scale_x;
        scale_y         = prep.scale_y;
        activated_xy    = prep.activated_xy;

        % [V28] 用 orig_grid_bbox 取 x_min_raw 等字段 (effective_bbox 不含 _raw 字段)
        if isfield(prep, 'bnd_coord_uses_grid_bbox') && prep.bnd_coord_uses_grid_bbox ...
                && isstruct(orig_grid_bbox) && isfield(orig_grid_bbox, 'x_min_raw')
            bnd_x_origin = orig_grid_bbox.x_min_raw;
            bnd_y_origin = orig_grid_bbox.y_min_raw;
            bnd_scale_x  = orig_grid_bbox.vox_dx;
            bnd_scale_y  = orig_grid_bbox.vox_dy;
        else
            bnd_x_origin = actual_x_min;
            bnd_y_origin = actual_y_min;
            bnd_scale_x  = scale_x;
            bnd_scale_y  = scale_y;
        end

        R.pointCloud_data    = pointCloud_data;
        R.F_z_inputs         = F_z_struct;
        R.activated_xy       = activated_xy;

        % 参数相关的派生量 (每次重算都要刷)
        adaptive_min_outer = max(actual_range * params.adaptive_outer_factor, 1.5);
        expand_dist        = max(scale_x, scale_y) * params.contour_expand_ratio;

        R.scale_xy = struct('xmin', actual_x_min, 'ymin', actual_y_min, ...
            'sx', scale_x, 'sy', scale_y, 'nelx', nelx, 'nely', nely);
        R.adaptive_min_outer = adaptive_min_outer;
        R.expand_dist        = expand_dist;

        % F_z 插值器: prep 里只存输入点, 这里重建 (轻量)
        if ~isempty(F_z_struct) && isfield(F_z_struct, 'X') && ~isempty(F_z_struct.X)
            F_z = scatteredInterpolant(F_z_struct.X, F_z_struct.Y, F_z_struct.Z, ...
                'linear', 'nearest');
        else
            F_z = [];
        end

        % 方向场滤波 (依赖参数, 但通常很快)
        t_filtered = filter_orientation_simple(t, xold, ...
            params.filter_radius, params.filter_iterations);

        %% =============================================================
        %%  分支: 自动 vs 手动 - 轮廓来源
        %% =============================================================
        if is_manual
            % --- 手动模式: [V12] 使用从框内点反算出的紧贴轮廓 ---
            outer_contours   = {};
            inner_contours   = {};
            outer_source_id  = {};   % [V12] 跟踪每个轮廓的来源, 避免同源挖空自己
            inner_source_id  = {};
            md = manual_data;
            for k = 1:numel(md.outer_groups)
                g = md.outer_groups{k};
                if isfield(g, 'contour_xy') && ~isempty(g.contour_xy)
                    poly = g.contour_xy;
                else
                    poly = g.poly_xy;   % 兼容老会话
                end
                if ~isequal(poly(1,:), poly(end,:))
                    poly(end+1,:) = poly(1,:); %#ok<AGROW>
                end
                % [V17] 手动模式跳过 expand_contour (pick 时 boundary 已紧贴点 + feasible 裁剪)
                %       如果撑大会跑出 feasible
                %       自动模式仍需 expand 因为 bwboundaries 紧贴像素中心

                % [V17] per-contour feasible 兜底裁剪 (处理老会话或边缘情况)
                per_feas = get_field_or(g, 'feasible_region', []);
                if ~isempty(per_feas) && isa(per_feas, 'polyshape') && area(per_feas) > 1e-9
                    try
                        pp = polyshape(poly(:,1), poly(:,2));
                        cc = intersect(pp, per_feas);
                        if area(cc) > 1e-9
                            [bx, by] = boundary(cc);
                            ni = find(isnan(bx));
                            if isempty(ni)
                                poly = [bx, by];
                            else
                                segs = {}; prev = 1;
                                for si = [ni(:)', length(bx)+1]
                                    sg = [bx(prev:si-1), by(prev:si-1)];
                                    if size(sg,1) >= 3, segs{end+1} = sg; end %#ok<AGROW>
                                    prev = si + 1;
                                end
                                if ~isempty(segs)
                                    [~, mi] = max(cellfun(@(s) size(s,1), segs));
                                    poly = segs{mi};
                                end
                            end
                        end
                    catch
                    end
                end

                role = get_field_or(g, 'role', 'outer');
                src  = sprintf('out_%d', g.id);
                outer_contours{end+1} = poly;       %#ok<AGROW>
                outer_source_id{end+1} = src;       %#ok<AGROW>
                if strcmp(role, 'both')
                    inner_contours{end+1} = poly;   %#ok<AGROW>
                    inner_source_id{end+1} = src;   %#ok<AGROW>
                end
            end
            for k = 1:numel(md.inner_groups)
                g = md.inner_groups{k};
                if isfield(g, 'contour_xy') && ~isempty(g.contour_xy)
                    poly = g.contour_xy;
                else
                    poly = g.poly_xy;
                end
                if ~isequal(poly(1,:), poly(end,:))
                    poly(end+1,:) = poly(1,:); %#ok<AGROW>
                end
                % [V17] inner_groups 也走 per-contour feasible 裁剪
                per_feas = get_field_or(g, 'feasible_region', []);
                if ~isempty(per_feas) && isa(per_feas, 'polyshape') && area(per_feas) > 1e-9
                    try
                        pp = polyshape(poly(:,1), poly(:,2));
                        cc = intersect(pp, per_feas);
                        if area(cc) > 1e-9
                            [bx, by] = boundary(cc);
                            ni = find(isnan(bx));
                            if isempty(ni)
                                poly = [bx, by];
                            else
                                segs = {}; prev = 1;
                                for si = [ni(:)', length(bx)+1]
                                    sg = [bx(prev:si-1), by(prev:si-1)];
                                    if size(sg,1) >= 3, segs{end+1} = sg; end %#ok<AGROW>
                                    prev = si + 1;
                                end
                                if ~isempty(segs)
                                    [~, mi] = max(cellfun(@(s) size(s,1), segs));
                                    poly = segs{mi};
                                end
                            end
                        end
                    catch
                    end
                end
                inner_contours{end+1} = poly;       %#ok<AGROW>
                inner_source_id{end+1} = sprintf('in_%d', g.id);  %#ok<AGROW>
            end
        else
            % --- 自动模式: bwboundaries 提取 ---
            outer_source_id = {};
            inner_source_id = {};
            x_filter = zeros(nely, nelx);
            x_filter(xold > 0) = 1;
            if has_img && params.contour_dilate_pixels > 0
                se = strel('disk', params.contour_dilate_pixels);
                x_filter = imdilate(x_filter, se);
            end

            n_burned_z = 0; n_burned_xy = 0;
            n_burned_bbox = 0;   % [V16] 保留字段, 实际不再烧入 (feasible_region 取代)
            F_zmin_local = []; F_zmax_local = [];
            if params.use_z_burn && has_zhf && ~isempty(zhf_data)
                try
                    F_zmin_local = griddedInterpolant({zhf_data.yy, zhf_data.xx}, ...
                        zhf_data.Z_min_map, 'linear', 'nearest');
                    F_zmax_local = griddedInterpolant({zhf_data.yy, zhf_data.xx}, ...
                        zhf_data.Z_max_map, 'linear', 'nearest');
                catch
                end
            end

            % [V16] 仅在 mask / zhf 存在时跑循环, 不再做 bbox 烧入
            %       bbox 约束已迁移到 apply_feasible_clip (轮廓阶段)
            need_loop = ~isempty(F_z) && (~isempty(F_zmin_local) || has_mask);
            if need_loop
                z_margin = max(scale_x, scale_y) * params.z_margin_factor;
                for row = 1:nely
                    for col = 1:nelx
                        if x_filter(row, col) == 0, continue; end
                        px = bnd_x_origin + (col - 1) * bnd_scale_x;   % [V27]
                        py = bnd_y_origin + (row - 1) * bnd_scale_y;   % [V27]
                        if has_mask
                            if ~isinterior(mask_poly, px, py)
                                x_filter(row, col) = 0;
                                n_burned_xy = n_burned_xy + 1;
                                continue;
                            end
                        end
                        pz = F_z(px, py);
                        if isnan(pz)
                            x_filter(row, col) = 0;
                            n_burned_xy = n_burned_xy + 1;
                            continue;
                        end
                        if ~isempty(F_zmin_local)
                            z_lo = F_zmin_local(py, px);
                            z_hi = F_zmax_local(py, px);
                            if pz < z_lo - z_margin || pz > z_hi + z_margin
                                x_filter(row, col) = 0;
                                n_burned_z = n_burned_z + 1;
                            end
                        end
                    end
                end
                if has_img && (n_burned_z + n_burned_xy) > 0
                    x_filter = imerode(x_filter, ones(3));
                    x_filter = imdilate(x_filter, ones(3));
                end
            end
            R.burn_xy = n_burned_xy;
            R.burn_z  = n_burned_z;
            R.burn_bbox = n_burned_bbox;   % [V16] 始终为 0

            [B, ~, N, ~] = bwboundaries(x_filter, 'holes');
            outer_raw = {}; inner_raw = {};
            for k = 1:length(B)
                if k <= N, outer_raw{end+1} = B{k}; %#ok<AGROW>
                else,       inner_raw{end+1} = B{k}; %#ok<AGROW>
                end
            end

            outer_contours = {};
            for k = 1:length(outer_raw)
                bnd = outer_raw{k};
                if isempty(bnd) || size(bnd,1) < 3, continue; end
                if size(bnd,1) > 200
                    step = ceil(size(bnd,1)/200);
                    bnd = bnd(1:step:end,:);
                end
                if ~isequal(bnd(1,:), bnd(end,:)), bnd(end+1,:) = bnd(1,:); end %#ok<AGROW>
                if size(bnd,1) > 6
                    bnd(:,1) = smooth(bnd(:,1),3);
                    bnd(:,2) = smooth(bnd(:,2),3);
                end
                xa = bnd_x_origin + (bnd(:,2)-1)*bnd_scale_x;   % [V27]
                ya = bnd_y_origin + (bnd(:,1)-1)*bnd_scale_y;   % [V27]
                pts = [xa(:), ya(:)];
                pts = expand_contour(pts, expand_dist);
                clen = sum(sqrt(sum(diff(pts).^2,2)));
                if clen >= adaptive_min_outer
                    outer_contours{end+1} = pts; %#ok<AGROW>
                    outer_source_id{end+1} = sprintf('auto_o%d', k); %#ok<AGROW>
                end
            end

            inner_contours = {};
            for k = 1:length(inner_raw)
                bnd = inner_raw{k};
                if isempty(bnd) || size(bnd,1) < 3, continue; end
                if size(bnd,1) > 200
                    step = ceil(size(bnd,1)/200);
                    bnd = bnd(1:step:end,:);
                end
                if ~isequal(bnd(1,:), bnd(end,:)), bnd(end+1,:) = bnd(1,:); end %#ok<AGROW>
                if size(bnd,1) > 6
                    bnd(:,1) = smooth(bnd(:,1),3);
                    bnd(:,2) = smooth(bnd(:,2),3);
                end
                xa = bnd_x_origin + (bnd(:,2)-1)*bnd_scale_x;   % [V27]
                ya = bnd_y_origin + (bnd(:,1)-1)*bnd_scale_y;   % [V27]
                pts = [xa(:), ya(:)];
                clen = sum(sqrt(sum(diff(pts).^2,2)));
                if clen >= params.min_contour_length_inner
                    inner_contours{end+1} = pts; %#ok<AGROW>
                    inner_source_id{end+1} = sprintf('auto_i%d', k); %#ok<AGROW>
                end
            end
        end

        %% =============================================================
        %%  [V16] 用 feasible_region 封闭轮廓 (取代 V12 的 apply_bbox_clip)
        %%  - 原轮廓在 feasible_region 内的部分保留
        %%  - 超出 feasible_region 的部分被 bbox 与曲面交线替代
        %%  - 整体上 XY 投影面仍是封闭多边形
        %% =============================================================
        feasible = polyshape();
        if isstruct(prep) && isfield(prep, 'feasible_region') && ...
                ~isempty(prep.feasible_region) && area(prep.feasible_region) > 1e-6
            feasible = prep.feasible_region;
        end
        if area(feasible) > 1e-6
            [outer_contours, outer_source_id] = apply_feasible_clip(...
                outer_contours, outer_source_id, feasible, false);
            [inner_contours, inner_source_id] = apply_feasible_clip(...
                inner_contours, inner_source_id, feasible, true);
        end

        %% =============================================================
        %%  通用: 中轴线 / 流线 / 区域分割 / 遮罩裁剪
        %% =============================================================
        % 中轴线 (仅自动模式需要; 手动模式可跳过)
        if ~is_manual && has_img
            x_filter_tmp = zeros(nely, nelx);
            x_filter_tmp(xold > 0) = 1;
            sk = bwmorph(x_filter_tmp, 'skel', Inf);
            sk = bwmorph(sk, 'spur', 3);
            [ys, xs] = find(sk);
            if ~isempty(xs)
                R.medial_axis = [actual_x_min + (xs-1)*scale_x, ...
                                 actual_y_min + (ys-1)*scale_y];
            end
        end

        % 流线 (始终用 t_filtered)
        % [V9] 拓扑流线是性能瓶颈, 只在 params.do_streamlines=true 时计算
        do_streamlines = isfield(params, 'do_streamlines') && params.do_streamlines;
        if do_streamlines
            [~, ~, sl_raw, ~, ~, ~] = plotTopologyWithMedialAxis(...
                xold, t_filtered, nelx, nely, params.volfrac);
            streamlines = {};
            for k = 1:length(sl_raw)
                sl = sl_raw{k};
                if isempty(sl) || size(sl,1) < 3, continue; end
                xa = actual_x_min + (sl(:,1)-1)*scale_x;
                ya = actual_y_min + (sl(:,2)-1)*scale_y;
                streamlines{end+1} = [xa(:), ya(:)]; %#ok<AGROW>
            end
            if ~isempty(streamlines) && ~isempty(outer_contours)
                [fs, ~] = filterStreamlinesInsideContours(streamlines, ...
                    outer_contours, inner_contours);
                fs = fs(~cellfun(@isempty, fs));
                if ~isempty(fs)
                    ext = extendStreamlinesToContour(fs, outer_contours, inner_contours);
                    ext = ext(~cellfun(@isempty, ext));
                else
                    ext = {};
                end
            else
                ext = {};
            end
        else
            % 跳过拓扑优化, 流线为空 -> 区域分割不切分, 每个外轮廓 = 一个区域
            ext = {};
        end

        % 区域分割 [V12] 用 source_id 跳过同源 inner (避免 both 把自己挖空)
        all_regions = {};
        for ci = 1:length(outer_contours)
            cur_o   = outer_contours(ci);
            this_src = '';
            if ci <= length(outer_source_id)
                this_src = outer_source_id{ci};
            end
            cur_i = {};
            try
                mpt = polyshape(cur_o{1}(:,1), cur_o{1}(:,2));
                for h = 1:length(inner_contours)
                    if isempty(inner_contours{h}), continue; end
                    % [V12] 跳过自己的副本
                    if h <= length(inner_source_id) && ...
                            strcmp(inner_source_id{h}, this_src)
                        continue;
                    end
                    ic = mean(inner_contours{h},1);
                    if isinterior(mpt, ic(1), ic(2))
                        cur_i{end+1} = inner_contours{h}; %#ok<AGROW>
                    end
                end
            catch
                mpt = polyshape();
            end

            cur_s = {};
            for s = 1:length(ext)
                sl = ext{s};
                if isempty(sl), continue; end
                mid = sl(round(size(sl,1)/2), :);
                in_o = inpolygon(mid(1), mid(2), cur_o{1}(:,1), cur_o{1}(:,2));
                in_i = false;
                for h = 1:length(cur_i)
                    if inpolygon(mid(1), mid(2), cur_i{h}(:,1), cur_i{h}(:,2))
                        in_i = true; break;
                    end
                end
                if in_o && ~in_i, cur_s{end+1} = sl; end %#ok<AGROW>
            end

            try
                if ~isempty(cur_s)
                    [~, regions_poly] = split_region_points_improved(...
                        cur_o, cur_i, cur_s, ...
                        'point_tol', 1e-6, 'dist_tol', 1e-6, ...
                        'area_tol', 1e-12, 'debug', false);
                else
                    regions_poly = mpt;
                    for h = 1:length(cur_i)
                        regions_poly = subtract(regions_poly, ...
                            polyshape(cur_i{h}(:,1), cur_i{h}(:,2)));
                    end
                end
            catch
                regions_poly = polyshape.empty;
            end

            for i = 1:length(regions_poly)
                try
                    if length(regions_poly) == 1
                        tmp = regions_poly;
                    else
                        tmp = regions_poly(i);
                    end
                    for k = 1:length(cur_i)
                        tmp = subtract(tmp, polyshape(cur_i{k}(:,1), cur_i{k}(:,2)));
                    end
                    if area(tmp) > params.min_region_area
                        all_regions{end+1} = tmp; %#ok<AGROW>
                    end
                catch
                end
            end
        end

        n_init_regions = length(all_regions);
        if n_init_regions > 0
            fprintf('    [REGION] 初始 %d region (来自 %d outer)\n', ...
                n_init_regions, length(outer_contours));
        end

        % 遮罩裁剪 ([V23] 手动模式跳过, 用户已自主决定轮廓范围)
        if has_mask && ~is_manual && ~isempty(all_regions)
            clipped = {};
            for ri = 1:length(all_regions)
                try
                    c = intersect(all_regions{ri}, mask_poly);
                    if area(c) > params.min_region_area
                        parts = regions(c);
                        for cp = 1:length(parts)
                            if area(parts(cp)) > params.min_region_area
                                clipped{end+1} = parts(cp); %#ok<AGROW>
                            end
                        end
                    end
                catch
                    if area(all_regions{ri}) > params.min_region_area
                        clipped{end+1} = all_regions{ri}; %#ok<AGROW>
                    end
                end
            end
            n_after_mask = length(clipped);
            if n_after_mask ~= n_init_regions
                fprintf('    [REGION] mask 裁剪后 %d -> %d\n', n_init_regions, n_after_mask);
            end
            all_regions = clipped;
        end

        % [V26] XYZ bbox 裁剪 regions: 只裁 xy, 不再检查 z
        %   理由: outer_contours 已经过 apply_bbox_clip(strict_z) 裁剪, regions 不会跑出 xy 范围;
        %   z 检查 (F_z(重心) 在 grid_bbox.z 范围内) 在 V24 修复 F_z 数据完整后会误杀,
        %   因为曲面真实 z 常比栅格中心 z 偏出几个体素 (栅格中心在曲面下方/附近).
        if isstruct(grid_bbox) && ~isempty(all_regions)
            n_before_bbox = length(all_regions);
            clipped = {};
            for ri = 1:length(all_regions)
                try
                    c = clip_polyshape_to_bbox(all_regions{ri}, grid_bbox, []);  % [V26] F_z=[] 跳过 z 检查
                    if ~isempty(c) && area(c) > params.min_region_area
                        parts = regions(c);
                        for cp = 1:length(parts)
                            if area(parts(cp)) > params.min_region_area
                                clipped{end+1} = parts(cp); %#ok<AGROW>
                            end
                        end
                    end
                catch
                end
            end
            if length(clipped) ~= n_before_bbox
                fprintf('    [REGION] bbox 裁剪后 %d -> %d\n', n_before_bbox, length(clipped));
            end
            all_regions = clipped;
        end

        if isempty(all_regions) && n_init_regions > 0
            fprintf(2, '    [REGION] WARN: 初始有 %d region 但全被裁剪掉, 检查 mask/bbox 是否合理\n', n_init_regions);
        end

        if has_mask && ~is_manual  % 手动模式不裁剪外轮廓 (用户已经手动确定)
            outer_contours = clip_contours_to_mask(outer_contours, mask_poly);
        end

        R.outer_contours = outer_contours;
        R.inner_contours = inner_contours;
        R.streamlines    = ext;
        R.regions        = all_regions;
        R.success        = true;

    catch ME
        R.success = false;
        R.error_message = ME.message;
        fprintf(2, '[ERROR] Layer %d: %s\n', layer_idx, ME.message);
    end
end


%% ================================================================
%% 阶段二核心: 基于轮廓+区域生成偏置路径
%% ================================================================
function PR = compute_paths_only(stage_a, params, has_mask, mask_poly)
    PR = struct();
    PR.paths_2d = {};
    PR.paths_3d = {};
    if ~stage_a.success || isempty(stage_a.regions)
        if isempty(stage_a.regions)
            fprintf(2, '    [PATH] regions 为空, 没有路径可生成\n');
        end
        return;
    end

    if ~isempty(stage_a.F_z_inputs) && isfield(stage_a.F_z_inputs, 'X') ...
            && ~isempty(stage_a.F_z_inputs.X)
        F_z = scatteredInterpolant(stage_a.F_z_inputs.X, ...
            stage_a.F_z_inputs.Y, stage_a.F_z_inputs.Z, ...
            'linear', 'nearest');
    else
        F_z = [];
    end
    pointCloud_data = stage_a.pointCloud_data;

    n_regions = length(stage_a.regions);
    n_path_success = 0;
    n_path_skip = 0;
    n_path_fail = 0;
    fprintf('    [PATH] 开始生成路径, region 数 = %d\n', n_regions);
    for i = 1:n_regions
        region_result = stage_a.regions{i};
        r_area = area(region_result);
        if r_area < 1e-6
            fprintf(2, '    [PATH] Region %d: area=%.4g 太小, 跳过\n', i, r_area);
            n_path_skip = n_path_skip + 1;
            continue;
        end
        n_paths_this = 0;
        try
            [outer_cell, inner_cell] = polyshape_to_cell(region_result);
            if isempty(outer_cell)
                fprintf(2, '    [PATH] Region %d: polyshape_to_cell 返回空 outer_cell (面积=%.3f), 跳过\n', ...
                    i, r_area);
                n_path_skip = n_path_skip + 1;
                continue;
            end
            % 多边形顶点数检查
            n_verts_outer = 0;
            for oc = 1:length(outer_cell)
                if ~isempty(outer_cell{oc})
                    n_verts_outer = n_verts_outer + size(outer_cell{oc}, 1);
                end
            end
            rings_results = generate_offset_path2(outer_cell, inner_cell, ...
                params.offset_distance, params.max_iterations, ...
                params.min_path_length, pointCloud_data);
            if isempty(rings_results)
                fprintf(2, '    [PATH] Region %d: generate_offset_path2 返回空 (面积=%.3f, outer 顶点数=%d, offset=%.2f)\n', ...
                    i, r_area, n_verts_outer, params.offset_distance);
                n_path_skip = n_path_skip + 1;
                continue;
            end
            for j = 1:length(rings_results)
                rings_iter = rings_results{j};
                if isempty(rings_iter), continue; end
                for k = 1:length(rings_iter)
                    pts_2d = rings_iter{k};
                    if isempty(pts_2d) || size(pts_2d,1) < 2, continue; end
                    PR.paths_2d{end+1} = pts_2d; %#ok<AGROW>

                    pts_3d = zeros(size(pts_2d,1), 3);
                    for pp = 1:size(pts_2d,1)
                        x = pts_2d(pp,2);
                        y = pts_2d(pp,1);
                        if ~isempty(F_z)
                            z = F_z(x, y);
                        else
                            z = nan;
                        end
                        if isnan(z) || ~isfinite(z)
                            d = (pointCloud_data.X(:)-x).^2 + ...
                                (pointCloud_data.Y(:)-y).^2;
                            [~, idx] = min(d);
                            z = pointCloud_data.Z(idx);
                        end
                        pts_3d(pp,:) = [x, y, z];
                    end
                    if has_mask
                        c3 = clip_path_to_mask(pts_3d, mask_poly);
                        for ss = 1:length(c3)
                            PR.paths_3d{end+1} = c3{ss}; %#ok<AGROW>
                        end
                    else
                        PR.paths_3d{end+1} = pts_3d; %#ok<AGROW>
                    end
                    n_paths_this = n_paths_this + 1;
                end
            end
            if n_paths_this == 0
                fprintf(2, '    [PATH] Region %d: rings_results 非空但所有 ring 都被过滤 (可能 min_path_length=%.2f 太严)\n', ...
                    i, params.min_path_length);
                n_path_skip = n_path_skip + 1;
            else
                fprintf('    [PATH] Region %d: 生成 %d 条路径 (面积=%.3f)\n', ...
                    i, n_paths_this, r_area);
                n_path_success = n_path_success + 1;
            end
        catch ME
            fprintf(2, '    [PATH] Region %d 失败: %s\n', i, ME.message);
            if ~isempty(ME.stack)
                fprintf(2, '    [PATH]   位置: %s (line %d)\n', ...
                    ME.stack(1).name, ME.stack(1).line);
            end
            n_path_fail = n_path_fail + 1;
        end
    end
    fprintf('    [PATH] 完成: 成功 %d, 跳过 %d, 失败 %d / %d region; 总路径 %d 条\n', ...
        n_path_success, n_path_skip, n_path_fail, n_regions, length(PR.paths_2d));
end


%% ================================================================
%% 最终保存
%% ================================================================
function save_final_results(fig, all_layers_data, global_params, total_time, ...
        success_count, failed_layers)
    app = fig.UserData;
    num = app.num_layers;

    total_outer = 0; total_inner = 0; total_streamlines = 0;
    total_regions = 0; total_paths_2d = 0; total_paths_3d = 0;
    for i = 1:num
        if all_layers_data(i).success
            total_outer       = total_outer + all_layers_data(i).statistics.num_outer;
            total_inner       = total_inner + all_layers_data(i).statistics.num_inner;
            total_streamlines = total_streamlines + all_layers_data(i).statistics.num_streamlines;
            total_regions     = total_regions + all_layers_data(i).statistics.num_regions;
            total_paths_2d    = total_paths_2d + all_layers_data(i).statistics.num_paths_2d;
            total_paths_3d    = total_paths_3d + all_layers_data(i).statistics.num_paths_3d;
        end
    end

    results = struct();
    results.params              = global_params;
    results.params_per_layer    = app.params_per_layer;
    results.manual_per_layer    = app.manual_per_layer;
    results.layer_mode          = app.layer_mode;
    results.accepted            = app.accepted;
    results.num_layers          = num;
    results.all_layers_data     = all_layers_data;
    results.structure_mask_poly = app.mask_poly;
    results.total_statistics = struct(...
        'total_outer', total_outer, 'total_inner', total_inner, ...
        'total_streamlines', total_streamlines, 'total_regions', total_regions, ...
        'total_paths_2d', total_paths_2d, 'total_paths_3d', total_paths_3d, ...
        'success_count', success_count, ...
        'failed_count', length(failed_layers), ...
        'failed_layers', failed_layers, ...
        'total_time', total_time);
    results.version = 'v8_interactive_dual_mode';
    save('all_layers_path_results_v8.mat', 'results', '-v7.3');

    paths_only = struct();
    paths_only.num_layers = num;
    paths_only.layer_paths_2d = cell(num,1);
    paths_only.layer_paths_3d = cell(num,1);
    paths_only.layer_offsets  = zeros(num,1);
    paths_only.layer_outer_contours = cell(num,1);
    paths_only.layer_inner_contours = cell(num,1);
    for i = 1:num
        if all_layers_data(i).success
            paths_only.layer_paths_2d{i} = all_layers_data(i).paths_2d;
            paths_only.layer_paths_3d{i} = all_layers_data(i).paths_3d;
            paths_only.layer_offsets(i)  = all_layers_data(i).offset;
            paths_only.layer_outer_contours{i} = all_layers_data(i).outer_contours;
            paths_only.layer_inner_contours{i} = all_layers_data(i).inner_contours;
        else
            paths_only.layer_paths_2d{i} = {};
            paths_only.layer_paths_3d{i} = {};
            paths_only.layer_offsets(i) = NaN;
            paths_only.layer_outer_contours{i} = {};
            paths_only.layer_inner_contours{i} = {};
        end
    end
    save('all_layers_paths_only_v8.mat', 'paths_only', '-v7.3');

    fig3d = figure('Name', '3D Paths V8', 'Position', [120 120 1100 800], ...
        'Visible', 'off');
    hold on;
    layer_colors = jet(num);
    for li = 1:num
        if ~all_layers_data(li).success, continue; end
        for i = 1:length(all_layers_data(li).paths_3d)
            pts = all_layers_data(li).paths_3d{i};
            if ~isempty(pts) && size(pts,1) >= 2
                plot3(pts(:,1), pts(:,2), pts(:,3), ...
                    'Color', layer_colors(li,:), 'LineWidth', 1);
            end
        end
    end
    hold off; axis equal; view(-37.5,30); grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('All Layers 3D Paths (V8)');
    saveas(fig3d, 'all_layers_3d_v8.png');
    close(fig3d);

    msg = sprintf([...
        '路径生成完成!\n\n' ...
        '已接受层: %d / %d\n' ...
        '成功生成: %d\n' ...
        '失败: %d\n' ...
        '2D 路径: %d\n' ...
        '3D 路径: %d\n' ...
        '耗时: %.1f 秒\n\n' ...
        '已保存:\n' ...
        '  all_layers_path_results_v8.mat\n' ...
        '  all_layers_paths_only_v8.mat\n' ...
        '  all_layers_3d_v8.png'], ...
        sum(app.accepted), num, success_count, length(failed_layers), ...
        total_paths_2d, total_paths_3d, total_time);
    uialert(fig, msg, '阶段二完成', 'Icon', 'success');

    if ~isempty(failed_layers)
        fprintf('[WARN] 失败层: %s\n', mat2str(failed_layers));
    end
end


%% ================================================================
%% 辅助: layer_result 结构
%% ================================================================
function S = init_layer_results_struct(n)
    S = struct(...
        'layer_idx', cell(n,1), 'offset', cell(n,1), ...
        'outer_contours', cell(n,1), 'inner_contours', cell(n,1), ...
        'streamlines', cell(n,1), 'medial_axis', cell(n,1), ...
        'regions', cell(n,1), 'paths_2d', cell(n,1), ...
        'paths_3d', cell(n,1), 'pointCloud_data', cell(n,1), ...
        'statistics', cell(n,1), 'processing_time', cell(n,1), ...
        'success', cell(n,1), 'error_message', cell(n,1));
end

function r = stage_a_to_layer_result(layer_idx, offset, stage_a)
    r = struct();
    r.layer_idx = layer_idx;
    r.offset    = offset;
    if isempty(stage_a)
        r.outer_contours = {};
        r.inner_contours = {};
        r.streamlines    = {};
        r.medial_axis    = [];
        r.regions        = {};
        r.pointCloud_data = [];
    else
        r.outer_contours  = stage_a.outer_contours;
        r.inner_contours  = stage_a.inner_contours;
        r.streamlines     = stage_a.streamlines;
        r.medial_axis     = stage_a.medial_axis;
        r.regions         = stage_a.regions;
        r.pointCloud_data = stage_a.pointCloud_data;
    end
    r.paths_2d = {};
    r.paths_3d = {};
    r.statistics = struct('num_outer',0,'num_inner',0,'num_streamlines',0, ...
        'num_regions',0,'num_paths_2d',0,'num_paths_3d',0);
    r.processing_time = 0;
    r.success = false;
    r.error_message = 'skipped (not accepted)';
end

function r = merge_stage_a_and_paths(layer_idx, offset, stage_a, paths_result)
    r = struct();
    r.layer_idx       = layer_idx;
    r.offset          = offset;
    r.outer_contours  = stage_a.outer_contours;
    r.inner_contours  = stage_a.inner_contours;
    r.streamlines     = stage_a.streamlines;
    r.medial_axis     = stage_a.medial_axis;
    r.regions         = stage_a.regions;
    r.pointCloud_data = stage_a.pointCloud_data;
    r.paths_2d        = paths_result.paths_2d;
    r.paths_3d        = paths_result.paths_3d;
    r.statistics = struct(...
        'num_outer',       length(stage_a.outer_contours), ...
        'num_inner',       length(stage_a.inner_contours), ...
        'num_streamlines', length(stage_a.streamlines), ...
        'num_regions',     length(stage_a.regions), ...
        'num_paths_2d',    length(paths_result.paths_2d), ...
        'num_paths_3d',    length(paths_result.paths_3d));
    r.processing_time = paths_result.processing_time;
    r.success = true;
    r.error_message = '';
end


%% ================================================================
%% V6 辅助函数
%% ================================================================
function mask_poly = build_structure_mask(grid_data, valid_grid_mask)
    [nelx_g, nely_g, nelz_g] = size(valid_grid_mask);
    valid_xy = [];
    for i = 1:nelx_g
        for j = 1:nely_g
            for k = 1:nelz_g
                if valid_grid_mask(i,j,k)
                    gc = grid_data(i,j,k);
                    valid_xy = [valid_xy; gc.x, gc.y]; %#ok<AGROW>
                end
            end
        end
    end
    valid_xy = unique(valid_xy, 'rows');
    if size(valid_xy,1) < 3
        mask_poly = polyshape();
        return;
    end
    dx_v = diff(sort(unique(valid_xy(:,1))));
    dx_v = dx_v(dx_v > 0.01);
    vox_sp = median(dx_v);
    if isempty(vox_sp) || isnan(vox_sp), vox_sp = 1.0; end
    try
        alpha_r = vox_sp * 2.0;
        shp = alphaShape(valid_xy(:,1), valid_xy(:,2), alpha_r);
        [bf, bv] = boundaryFacets(shp);
        n_v = size(bv,1);
        adj = cell(n_v,1);
        for ei = 1:size(bf,1)
            adj{bf(ei,1)}(end+1) = bf(ei,2);
            adj{bf(ei,2)}(end+1) = bf(ei,1);
        end
        visited = false(n_v,1);
        loops = {};
        for sv = 1:n_v
            if visited(sv) || isempty(adj{sv}), continue; end
            loop = sv; visited(sv) = true;
            cur = sv; prv = 0;
            while true
                nb = adj{cur}; nxt = 0;
                for ni = 1:length(nb)
                    if nb(ni) ~= prv && ~visited(nb(ni))
                        nxt = nb(ni); break;
                    end
                end
                if nxt == 0, break; end
                visited(nxt) = true;
                loop(end+1) = nxt; %#ok<AGROW>
                prv = cur; cur = nxt;
            end
            if length(loop) >= 3, loops{end+1} = loop; end %#ok<AGROW>
        end
        if ~isempty(loops)
            lsizes = cellfun(@length, loops);
            [~, mi] = max(lsizes);
            ml = loops{mi};
            mask_poly = polyshape(bv(ml,1), bv(ml,2));
        else
            kh = convhull(valid_xy(:,1), valid_xy(:,2));
            mask_poly = polyshape(valid_xy(kh,1), valid_xy(kh,2));
        end
    catch
        kh = convhull(valid_xy(:,1), valid_xy(:,2));
        mask_poly = polyshape(valid_xy(kh,1), valid_xy(kh,2));
    end
    try
        mask_poly = polybuffer(mask_poly, vox_sp * 0.6);
    catch
    end
end

function pts = expand_contour(pts, ed)
    if ed <= 0, return; end
    try
        ps = polyshape(pts(:,1), pts(:,2));
        pe = polybuffer(ps, ed);
        if area(pe) > area(ps)
            [bx, by] = boundary(pe);
            if length(bx) >= 3
                pts = [bx(:), by(:)];
                if ~isequal(pts(1,:), pts(end,:))
                    pts(end+1,:) = pts(1,:);
                end
            end
        end
    catch
        c = mean(pts,1);
        d = pts - c;
        n = sqrt(sum(d.^2,2));
        n(n < 1e-10) = 1e-10;
        u = d ./ n;
        pts = pts + u*ed;
    end
end

function co = clip_contours_to_mask(ci, mp)
    co = ci;
    for oi = 1:length(ci)
        try
            oc = ci{oi};
            ocp = polyshape(oc(:,1), oc(:,2));
            ocl = intersect(ocp, mp);
            if area(ocl) > 0
                [bx, by] = boundary(ocl);
                ni = find(isnan(bx));
                if isempty(ni)
                    co{oi} = [bx, by];
                else
                    segs = {};
                    prev = 1;
                    for si = [ni(:)', length(bx)+1]
                        sg = [bx(prev:si-1), by(prev:si-1)];
                        if size(sg,1) >= 3, segs{end+1} = sg; end %#ok<AGROW>
                        prev = si + 1;
                    end
                    if ~isempty(segs)
                        sl = cellfun(@(s) sum(sqrt(sum(diff(s).^2,2))), segs);
                        [~, best] = max(sl);
                        co{oi} = segs{best};
                    end
                end
            end
        catch
        end
    end
end

function cl = clip_path_to_mask(pts, mp)
    im = isinterior(mp, pts(:,1), pts(:,2));
    if all(im), cl = {pts}; return; end
    cl = {};
    ss = 0;
    np = size(pts,1);
    for pp = 1:np
        if im(pp) && ss == 0
            ss = pp;
        elseif ~im(pp) && ss > 0
            if pp - ss >= 2
                cl{end+1} = pts(ss:pp-1,:); %#ok<AGROW>
            end
            ss = 0;
        end
    end
    if ss > 0 && np - ss + 1 >= 2
        cl{end+1} = pts(ss:end,:); %#ok<AGROW>
    end
    if isempty(cl) && size(pts,1) >= 2
        cl = {pts};
    end
end