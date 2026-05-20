%% ========================================
%% 曲面层交互式预览工具 v2
%% ========================================
% 功能:
%   - 滑块/按钮逐层切换曲面
%   - [checkbox] 显示当前层激活的体素网格
%   - [checkbox] 显示当前层对应的纤维路径
%   - [checkbox] 显示完整有效结构 (半透明)
%   - 3D + 俯视双视图, 降采样, 导出
%
% 数据源:
%   - slice_results_refined_latest.mat  (切片结果, 必须)
%   - all_layers_path_results_v3.mat    (路径数据, 可选)

clear; clc; close all;

fprintf('====================================================\n');
fprintf('  Surface Layer Viewer v2\n');
fprintf('====================================================\n\n');

%% ========== 加载切片结果 ==========
fprintf('[1] Loading slice results...\n');
if exist('slice_results_refined_latest.mat', 'file')
    load('slice_results_refined_latest.mat');
    fprintf('  Loaded: slice_results_refined_latest.mat\n');
else
    error('Cannot find slice_results_refined_latest.mat');
end

num_layers = slice_results.statistics.num_layers;
grid_data = slice_results.grid_data;
valid_grid_mask = slice_results.valid_grid_mask;
[nelx, nely, nelz] = size(valid_grid_mask);
fprintf('  Layers: %d, Grid: %dx%dx%d\n', num_layers, nelx, nely, nelz);

% 预计算有效网格中心坐标 (供"完整结构"显示)
fprintf('  Pre-computing valid grid centers...\n');
valid_idx = find(valid_grid_mask);
n_valid = length(valid_idx);
valid_xyz = zeros(n_valid, 3);
for m = 1:n_valid
    [ii, jj, kk] = ind2sub([nelx, nely, nelz], valid_idx(m));
    gc = grid_data(ii, jj, kk);
    valid_xyz(m,:) = [gc.x, gc.y, gc.z];
end
fprintf('  Valid grids: %d\n', n_valid);

% 估算体素尺寸 (从相邻网格间距推断)
voxel_size = 1.0;
if n_valid > 1
    diffs = diff(sort(unique(valid_xyz(:,1))));
    diffs = diffs(diffs > 0.01);
    if ~isempty(diffs)
        voxel_size = median(diffs);
    end
end
fprintf('  Estimated voxel size: %.3f\n', voxel_size);

%% ========== 加载路径数据 (可选) ==========
fprintf('[2] Loading path data...\n');
path_data = [];
has_paths = false;

path_candidates = {'all_layers_path_results_v3.mat', ...
                   'all_layers_path_results.mat'};
path_file = '';
for c = 1:length(path_candidates)
    if exist(path_candidates{c}, 'file')
        path_file = path_candidates{c};
        break;
    end
end

if ~isempty(path_file)
    try
        tmp = load(path_file);
        if isfield(tmp, 'results')
            path_data = tmp.results.all_layers_data;
            has_paths = true;
            fprintf('  Loaded: %s (%d layers)\n', path_file, length(path_data));
        end
    catch ME
        fprintf('  [WARN] Failed to load path data: %s\n', ME.message);
    end
else
    fprintf('  No path file found (optional, will disable Paths checkbox)\n');
end

%% ========== 创建主窗口 ==========
fprintf('[3] Creating viewer...\n');

fig = figure('Name', 'Surface Layer Viewer v2', ...
    'NumberTitle', 'off', ...
    'Position', [30, 30, 1500, 850], ...
    'Color', [0.94 0.94 0.94], ...
    'CloseRequestFcn', @close_viewer);

% -------- 存储状态 --------
vd = struct();
vd.current_layer = 1;
vd.num_layers = num_layers;
vd.slice_results = slice_results;
vd.grid_data = grid_data;
vd.valid_grid_mask = valid_grid_mask;
vd.valid_xyz = valid_xyz;
vd.voxel_size = voxel_size;
vd.nelx = nelx; vd.nely = nely; vd.nelz = nelz;
vd.path_data = path_data;
vd.has_paths = has_paths;
vd.az = -37.5;
vd.el = 30;
vd.show_grids = false;
vd.show_paths = false;
vd.show_structure = false;

% -------- 坐标轴 --------
ax3d = axes('Parent', fig, 'Position', [0.02 0.14 0.55 0.78]);
vd.ax3d = ax3d;

ax2d = axes('Parent', fig, 'Position', [0.60 0.14 0.38 0.78]);
vd.ax2d = ax2d;

% -------- 信息面板 --------
vd.info_text = uicontrol('Style', 'text', 'Parent', fig, ...
    'Position', [20, 815, 1200, 25], ...
    'String', '', 'FontSize', 11, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', ...
    'BackgroundColor', [0.94 0.94 0.94]);

% ======== 底部行1: 滑块 ========
uicontrol('Style', 'text', 'Parent', fig, ...
    'Position', [20, 82, 55, 20], 'String', 'Layer:', ...
    'FontSize', 10, 'BackgroundColor', [0.94 0.94 0.94]);

vd.slider = uicontrol('Style', 'slider', 'Parent', fig, ...
    'Position', [78, 82, 800, 22], ...
    'Min', 1, 'Max', max(num_layers, 2), 'Value', 1, ...
    'SliderStep', [1/max(num_layers-1,1), 5/max(num_layers-1,1)], ...
    'Callback', @slider_cb);

vd.layer_label = uicontrol('Style', 'text', 'Parent', fig, ...
    'Position', [885, 82, 90, 20], ...
    'String', sprintf('1 / %d', num_layers), ...
    'FontSize', 10, 'BackgroundColor', [0.94 0.94 0.94]);

% ======== 底部行2: 按钮 + checkbox ========
y2 = 42;

uicontrol('Style', 'pushbutton', 'Parent', fig, ...
    'Position', [20, y2, 70, 30], 'String', '<< Prev', ...
    'FontSize', 9, 'Callback', @prev_cb);

uicontrol('Style', 'pushbutton', 'Parent', fig, ...
    'Position', [95, y2, 70, 30], 'String', 'Next >>', ...
    'FontSize', 9, 'Callback', @next_cb);

uicontrol('Style', 'pushbutton', 'Parent', fig, ...
    'Position', [172, y2, 85, 30], 'String', 'Reset View', ...
    'FontSize', 9, 'Callback', @reset_view_cb);

uicontrol('Style', 'pushbutton', 'Parent', fig, ...
    'Position', [262, y2, 85, 30], 'String', 'All Layers', ...
    'FontSize', 9, 'Callback', @show_all_cb);

% 降采样
uicontrol('Style', 'text', 'Parent', fig, ...
    'Position', [358, y2+5, 50, 18], 'String', 'Down:', ...
    'FontSize', 9, 'BackgroundColor', [0.94 0.94 0.94]);

vd.ds_popup = uicontrol('Style', 'popupmenu', 'Parent', fig, ...
    'Position', [408, y2+2, 55, 25], ...
    'String', {'1x','2x','4x','8x','16x'}, 'Value', 3, ...
    'FontSize', 9, 'Callback', @redraw_cb);

% ---- 分隔 ----
x0 = 485;

% checkbox 1: 激活网格
vd.chk_grids = uicontrol('Style', 'checkbox', 'Parent', fig, ...
    'Position', [x0, y2+5, 145, 22], ...
    'String', 'Activated Grids', ...
    'FontSize', 9, 'Value', 0, ...
    'BackgroundColor', [0.94 0.94 0.94], ...
    'ForegroundColor', [0.8 0.15 0.1], ...
    'Callback', @chk_grids_cb);

% checkbox 2: 路径
if has_paths
    path_enable = 'on';
else
    path_enable = 'off';
end
vd.chk_paths = uicontrol('Style', 'checkbox', 'Parent', fig, ...
    'Position', [x0+150, y2+5, 130, 22], ...
    'String', 'Fiber Paths', ...
    'FontSize', 9, 'Value', 0, ...
    'Enable', path_enable, ...
    'BackgroundColor', [0.94 0.94 0.94], ...
    'ForegroundColor', [0.0 0.55 0.15], ...
    'Callback', @chk_paths_cb);

% checkbox 3: 完整结构
vd.chk_structure = uicontrol('Style', 'checkbox', 'Parent', fig, ...
    'Position', [x0+285, y2+5, 145, 22], ...
    'String', 'Full Structure', ...
    'FontSize', 9, 'Value', 0, ...
    'BackgroundColor', [0.94 0.94 0.94], ...
    'ForegroundColor', [0.4 0.4 0.4], ...
    'Callback', @chk_structure_cb);

% 透明度滑块
uicontrol('Style', 'text', 'Parent', fig, ...
    'Position', [x0+285, y2-18, 50, 16], 'String', 'Alpha:', ...
    'FontSize', 8, 'BackgroundColor', [0.94 0.94 0.94]);

vd.alpha_slider = uicontrol('Style', 'slider', 'Parent', fig, ...
    'Position', [x0+335, y2-16, 95, 16], ...
    'Min', 0.01, 'Max', 0.30, 'Value', 0.06, ...
    'Callback', @redraw_cb);

% 导出
uicontrol('Style', 'pushbutton', 'Parent', fig, ...
    'Position', [x0+450, y2, 90, 30], 'String', 'Export PNG', ...
    'FontSize', 9, 'Callback', @export_cb);

% ---- 保存 ----
guidata(fig, vd);

% 初始显示
update_display(fig);

fprintf('\n  Viewer ready.\n');
fprintf('  - Slider / Prev / Next to navigate layers\n');
fprintf('  - Checkboxes: [Activated Grids] [Fiber Paths] [Full Structure]\n');
fprintf('  - Alpha slider controls structure transparency\n');
fprintf('====================================================\n');

%% ================================================================
%% 回调函数
%% ================================================================

function slider_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    vd.current_layer = max(1, min(round(get(src, 'Value')), vd.num_layers));
    guidata(fig, vd);
    update_display(fig);
end

function prev_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    if vd.current_layer > 1
        vd.current_layer = vd.current_layer - 1;
        set(vd.slider, 'Value', vd.current_layer);
        guidata(fig, vd);
        update_display(fig);
    end
end

function next_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    if vd.current_layer < vd.num_layers
        vd.current_layer = vd.current_layer + 1;
        set(vd.slider, 'Value', vd.current_layer);
        guidata(fig, vd);
        update_display(fig);
    end
end

function reset_view_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    vd.az = -37.5; vd.el = 30;
    guidata(fig, vd);
    update_display(fig);
end

function redraw_cb(src, ~)
    fig = ancestor(src, 'figure');
    update_display(fig);
end

function chk_grids_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    vd.show_grids = get(src, 'Value');
    guidata(fig, vd);
    update_display(fig);
end

function chk_paths_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    vd.show_paths = get(src, 'Value');
    guidata(fig, vd);
    update_display(fig);
end

function chk_structure_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    vd.show_structure = get(src, 'Value');
    guidata(fig, vd);
    update_display(fig);
end

function export_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    fname = sprintf('layer_%02d_offset_%.1f.png', vd.current_layer, ...
        vd.slice_results.surface_layers{vd.current_layer}.offset);
    exportgraphics(fig, fname, 'Resolution', 200);
    fprintf('Exported: %s\n', fname);
end

function show_all_cb(src, ~)
    fig = ancestor(src, 'figure');
    vd = guidata(fig);
    
    fig2 = figure('Name', 'All Layers Overview', ...
        'Position', [100, 100, 1200, 800]);
    ax_all = axes('Parent', fig2);
    hold(ax_all, 'on'); grid(ax_all, 'on');
    
    cmap = jet(vd.num_layers);
    ds = 8;
    for L = 1:vd.num_layers
        layer = vd.slice_results.surface_layers{L};
        Xs = layer.X_surf(1:ds:end, 1:ds:end);
        Ys = layer.Y_surf(1:ds:end, 1:ds:end);
        Zs = layer.Z_surf(1:ds:end, 1:ds:end);
        vv = isfinite(Xs) & isfinite(Ys) & isfinite(Zs);
        scatter3(ax_all, Xs(vv), Ys(vv), Zs(vv), 1, cmap(L,:), '.');
    end
    
    xlabel(ax_all, 'X'); ylabel(ax_all, 'Y'); zlabel(ax_all, 'Z');
    title(ax_all, sprintf('All %d layers', vd.num_layers));
    colormap(ax_all, jet(vd.num_layers));
    cb = colorbar(ax_all); caxis(ax_all, [1 vd.num_layers]);
    ylabel(cb, 'Layer');
    axis(ax_all, 'equal');
    view(ax_all, -37.5, 30);
    rotate3d(fig2, 'on');
end

function close_viewer(src, ~)
    delete(src);
end

%% ================================================================
%% 核心绘制
%% ================================================================
function update_display(fig)
    vd = guidata(fig);
    L = vd.current_layer;
    layer = vd.slice_results.surface_layers{L};
    
    % 保存当前3D视角 (切层时保持)
    try
        [az_now, el_now] = view(vd.ax3d);
        if ~isnan(az_now)
            vd.az = az_now; vd.el = el_now;
            guidata(fig, vd);
        end
    catch
    end
    
    % 降采样
    ds_opts = [1, 2, 4, 8, 16];
    ds = ds_opts(get(vd.ds_popup, 'Value'));
    
    Xs = layer.X_surf; Ys = layer.Y_surf; Zs = layer.Z_surf;
    Xs_d = Xs(1:ds:end, 1:ds:end);
    Ys_d = Ys(1:ds:end, 1:ds:end);
    Zs_d = Zs(1:ds:end, 1:ds:end);
    
    % --- 信息字符串 ---
    fix_str = '';
    if isfield(layer, 'fix_stats')
        fs = layer.fix_stats;
        parts = {};
        fnames = fieldnames(fs);
        for fi = 1:length(fnames)
            val = fs.(fnames{fi});
            if isnumeric(val) && val > 0
                parts{end+1} = sprintf('%s=%d', fnames{fi}, val);
            end
        end
        if ~isempty(parts)
            fix_str = [' | ' strjoin(parts, ', ')];
        end
    end
    
    overlay_parts = {};
    if vd.show_grids, overlay_parts{end+1} = 'Grids'; end
    if vd.show_paths, overlay_parts{end+1} = 'Paths'; end
    if vd.show_structure, overlay_parts{end+1} = 'Structure'; end
    overlay_str = '';
    if ~isempty(overlay_parts)
        overlay_str = [' | Show: ' strjoin(overlay_parts, '+')];
    end
    
    info_str = sprintf('Layer %d/%d | offset=%.2f | %s | act=%d | new=%d%s%s', ...
        L, vd.num_layers, layer.offset, layer.direction, ...
        layer.total_activated, layer.total_newly_activated, ...
        fix_str, overlay_str);
    set(vd.info_text, 'String', info_str);
    set(vd.layer_label, 'String', sprintf('%d / %d', L, vd.num_layers));
    
    % ============================================================
    % 3D 视图
    % ============================================================
    cla(vd.ax3d);
    hold(vd.ax3d, 'on');
    
    % (a) 完整结构 — 最底层先画
    if vd.show_structure
        alpha_val = get(vd.alpha_slider, 'Value');
        draw_structure(vd.ax3d, vd.valid_xyz, alpha_val);
    end
    
    % (b) 曲面
    valid_s = isfinite(Xs_d) & isfinite(Ys_d) & isfinite(Zs_d);
    if any(valid_s(:))
        Zs_plot = Zs_d; Zs_plot(~valid_s) = NaN;
        surf(vd.ax3d, Xs_d, Ys_d, Zs_plot, ...
            'EdgeColor', 'none', 'FaceAlpha', 0.75, ...
            'FaceColor', 'interp');
    end
    
    % (c) 激活网格
    if vd.show_grids && isfield(layer, 'final_grids') && ~isempty(layer.final_grids)
        draw_activated_grids(vd.ax3d, layer.final_grids, vd.voxel_size);
    end
    
    % (d) 纤维路径
    if vd.show_paths && vd.has_paths
        draw_paths_3d(vd.ax3d, vd.path_data, L);
    end
    
    % 轴设置
    grid(vd.ax3d, 'on'); box(vd.ax3d, 'on');
    xlabel(vd.ax3d, 'X'); ylabel(vd.ax3d, 'Y'); zlabel(vd.ax3d, 'Z');
    title(vd.ax3d, sprintf('Layer %d  (offset = %.2f)', L, layer.offset));
    colormap(vd.ax3d, 'parula');
    axis(vd.ax3d, 'equal');
    view(vd.ax3d, vd.az, vd.el);
    rotate3d(vd.ax3d, 'on');
    
    % 光照
    delete(findobj(vd.ax3d, 'Type', 'light'));
    light('Parent', vd.ax3d, 'Position', [1, 1, 1], 'Style', 'infinite');
    light('Parent', vd.ax3d, 'Position', [-1, -1, 0.5], 'Style', 'infinite');
    lighting(vd.ax3d, 'gouraud');
    
    % ============================================================
    % 俯视图
    % ============================================================
    cla(vd.ax2d);
    hold(vd.ax2d, 'on');
    
    if any(valid_s(:))
        Zs_plot2 = Zs_d; Zs_plot2(~valid_s) = NaN;
        pcolor(vd.ax2d, Xs_d, Ys_d, Zs_plot2);
        shading(vd.ax2d, 'interp');
    end
    
    % 叠加激活网格 2D
    if vd.show_grids && isfield(layer, 'final_grids') && ~isempty(layer.final_grids)
        grids = layer.final_grids;
        gx = arrayfun(@(g) g.x, grids);
        gy = arrayfun(@(g) g.y, grids);
        plot(vd.ax2d, gx, gy, 's', ...
            'Color', [1 0.2 0.15], 'MarkerSize', 3, ...
            'MarkerFaceColor', [1 0.35 0.25]);
    end
    
    % 叠加路径 2D
    if vd.show_paths && vd.has_paths
        draw_paths_2d(vd.ax2d, vd.path_data, L);
    end
    
    grid(vd.ax2d, 'on');
    xlabel(vd.ax2d, 'X'); ylabel(vd.ax2d, 'Y');
    if any(valid_s(:))
        title(vd.ax2d, sprintf('Top View  Z:[%.2f, %.2f]', ...
            min(Zs_d(valid_s)), max(Zs_d(valid_s))));
    else
        title(vd.ax2d, 'Top View');
    end
    colormap(vd.ax2d, 'parula');
    colorbar(vd.ax2d);
    axis(vd.ax2d, 'equal');
    view(vd.ax2d, 0, 90);
    
    drawnow;
end

%% ================================================================
%% 绘制子函数
%% ================================================================

function draw_structure(ax, xyz, alpha_val)
% 完整有效结构: scatter3 (比逐个 patch 快)
    % 自动降采样: 最多 ~8000 点
    n = size(xyz, 1);
    ds = max(1, round(n / 8000));
    idx = 1:ds:n;
    scatter3(ax, xyz(idx,1), xyz(idx,2), xyz(idx,3), ...
        14, [0.5 0.5 0.5], 'filled', ...
        'MarkerFaceAlpha', alpha_val, ...
        'MarkerEdgeColor', 'none');
end

function draw_activated_grids(ax, grids, vs)
% 当前层激活的体素: 合并 patch
    n = length(grids);
    if n == 0, return; end
    
    hs = vs * 0.5;
    cube_f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    
    max_draw = 3000;
    if n > max_draw
        sel = round(linspace(1, n, max_draw));
    else
        sel = 1:n;
    end
    nd = length(sel);
    
    all_v = zeros(nd*8, 3);
    all_f = zeros(nd*6, 4);
    
    for m = 1:nd
        g = grids(sel(m));
        cx = g.x; cy = g.y; cz = g.z;
        verts = [cx-hs,cy-hs,cz-hs; cx+hs,cy-hs,cz-hs;
                 cx+hs,cy+hs,cz-hs; cx-hs,cy+hs,cz-hs;
                 cx-hs,cy-hs,cz+hs; cx+hs,cy-hs,cz+hs;
                 cx+hs,cy+hs,cz+hs; cx-hs,cy+hs,cz+hs];
        vo = (m-1)*8;
        fo = (m-1)*6;
        all_v(vo+1:vo+8, :) = verts;
        all_f(fo+1:fo+6, :) = cube_f + vo;
    end
    
    patch(ax, 'Faces', all_f, 'Vertices', all_v, ...
        'FaceColor', [1 0.35 0.25], ...
        'EdgeColor', [0.7 0.15 0.1], ...
        'LineWidth', 0.3, ...
        'FaceAlpha', 0.35);
end

function draw_paths_3d(ax, path_data, layer_idx)
% 3D 纤维路径
    if layer_idx > length(path_data), return; end
    ld = path_data(layer_idx);
    if ~isfield(ld, 'success') || ~ld.success, return; end
    if ~isfield(ld, 'paths_3d'), return; end
    
    for pi = 1:length(ld.paths_3d)
        pts = ld.paths_3d{pi};
        if isempty(pts) || size(pts,1) < 2, continue; end
        plot3(ax, pts(:,1), pts(:,2), pts(:,3), ...
            '-', 'Color', [0.05 0.75 0.2], 'LineWidth', 1.5);
    end
end

function draw_paths_2d(ax, path_data, layer_idx)
% 俯视路径 (XY投影)
    if layer_idx > length(path_data), return; end
    ld = path_data(layer_idx);
    if ~isfield(ld, 'success') || ~ld.success, return; end
    if ~isfield(ld, 'paths_3d'), return; end
    
    for pi = 1:length(ld.paths_3d)
        pts = ld.paths_3d{pi};
        if isempty(pts) || size(pts,1) < 2, continue; end
        plot(ax, pts(:,1), pts(:,2), '-', ...
            'Color', [0.05 0.7 0.15], 'LineWidth', 1.0);
    end
end