function display_fiber_tubes_interactive
%% =====================================================================
%% display_fiber_tubes_interactive.m
%% ---------------------------------------------------------------------
%% Interactive viewer for 4 path-planning results, sharing one voxel
%% background and identical BC markers. Sliders for layer range, tube
%% radius, Z-spacing, view angles. "SAVE all 4" exports every file at
%% the exact same camera view for fair cross-method comparison.
%%
%%   File expected formats (auto-detected per file):
%%     (A) results-style:    fields all_layers_data + num_layers
%%     (B) paths_only-style: fields layer_paths_3d  + num_layers
%%
%% Run:  >> display_fiber_tubes_interactive
%% =====================================================================

%% ===== Configuration =====
config.path_mat_list = { ...
    'all_layers_paths_only_v3.mat', ...
    'all_layers_paths_only_mine_offset.mat', ...
    'all_layers_paths_only_planar_offset.mat', ...
    'all_layers_paths_only_planar_stream.mat'};
config.file_labels = {'v3 (mine)', 'mine\_offset', 'planar\_offset', 'planar\_stream'};
config.voxel_mat   = 'voxel_refined_latest.mat';

config.bc_fixed_centers = [ ...
    5.0,  23.0, 0.0;
    14.4, 50.8, 0.0;
    29.0,  5.5, 0.0;
    44.0, 51.0, 0.0;
    53.0, 23.0, 0.0];
config.bc_load_center = [29.3, 30.4, 20.0];

config.voxel_color     = [0.55 0.55 0.58];
config.voxel_alpha     = 0.05;
config.voxel_half_size = 0.5;
config.tube_nsides     = 12;

%% ===== State (nested-function scope) =====
cache        = cell(1, length(config.path_mat_list));
voxel_data   = [];
SHOW_VOXEL   = true;

current_file_idx = 1;
layer_start  = 3;
layer_count  = 5;
tube_radius  = 0.10;
z_offset     = 0.30;
az           = -60;
el           = 22;

tube_handles = gobjects(0);

%% ===== Load voxel (once) =====
if exist(config.voxel_mat,'file')==2
    fprintf('Loading voxel %s ...\n', config.voxel_mat);
    vm = load(config.voxel_mat);
    if isfield(vm, 'refined_data')
        rd = vm.refined_data;
        voxel_data.grid_data       = rd.grid_data;
        voxel_data.valid_grid_mask = rd.valid_grid_mask;
        voxel_data.nelx = rd.grid_size.nelx;
        voxel_data.nely = rd.grid_size.nely;
        voxel_data.nelz = rd.grid_size.nelz;
        fprintf('  grid %d x %d x %d, valid = %d\n', ...
            voxel_data.nelx, voxel_data.nely, voxel_data.nelz, ...
            sum(voxel_data.valid_grid_mask(:)));
    else
        SHOW_VOXEL = false;
        fprintf('  refined_data missing, skip voxel.\n');
    end
else
    SHOW_VOXEL = false;
    fprintf('Voxel file %s not found, skip voxel.\n', config.voxel_mat);
end

%% ===== Build UI =====
fig_w   = 1500;
fig_h   = 950;
panel_w = 300;

fig = figure('Name', 'CFRC Path Inspector (interactive)', ...
    'NumberTitle','off', ...
    'Position', [50 50 fig_w fig_h], ...
    'Color', 'w', ...
    'Renderer', 'opengl', ...
    'MenuBar', 'none', ...
    'ToolBar', 'figure');

panel = uipanel('Parent', fig, ...
    'Units','pixels', ...
    'Position', [10 10 panel_w fig_h-20], ...
    'BackgroundColor', [0.95 0.95 0.95]);

ax = axes('Parent', fig, ...
    'Units','pixels', ...
    'Position', [panel_w+40 60 fig_w-panel_w-60 fig_h-100]);
hold(ax, 'on');

% --- Lay widgets top-down. y is the top of the next widget. ---
y = 915;

% Title
uicontrol(panel, 'Style','text', 'String','CFRC Path Inspector', ...
    'Position', [10 y-22 280 22], 'FontWeight','bold','FontSize',13, ...
    'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
y = y - 32;

% File selector label
uicontrol(panel, 'Style','text', 'String','Path data file:', ...
    'Position', [10 y-18 280 18], 'HorizontalAlignment','left', ...
    'FontWeight','bold', 'BackgroundColor',[0.95 0.95 0.95]);
y = y - 22;

% Radio button group
bg = uibuttongroup('Parent', panel, ...
    'Units','pixels', 'Position', [10 y-130 280 130], ...
    'BackgroundColor',[0.95 0.95 0.95], 'BorderType','none', ...
    'SelectionChangedFcn', @on_file_change);
btn_handles = gobjects(1, length(config.path_mat_list));
for k = 1:length(config.path_mat_list)
    btn_handles(k) = uicontrol(bg, 'Style','radiobutton', ...
        'String', sprintf('[%d] %s', k, config.file_labels{k}), ...
        'Position', [5, 130-25 - (k-1)*30, 270, 25], ...
        'BackgroundColor', [0.95 0.95 0.95], ...
        'Tag', num2str(k));
end
y = y - 140;

% Auto-center button
uicontrol(panel, 'Style','pushbutton', ...
    'String','Auto-center on richest layer', ...
    'Position', [10 y-25 280 25], 'Callback', @on_auto_center);
y = y - 35;

% --- Path/layer sliders ---
[slider_ls, txt_ls] = add_slider(panel, y, ...
    'Layer start', 1, 20, layer_start, 1, '%d', @on_layer_start_change, false);
y = y - 49;

[slider_lc, txt_lc] = add_slider(panel, y, ...
    'Layer count', 1, 15, layer_count, 1, '%d', @on_layer_count_change, false);
y = y - 49;

[slider_tr, txt_tr] = add_slider(panel, y, ...
    'Tube radius (mm)', 0.05, 0.50, tube_radius, 0.01, '%.2f', @on_tube_radius_change, false);
y = y - 49;

[slider_zo, txt_zo] = add_slider(panel, y, ...
    'Z-offset / layer (mm)', 0.0, 2.0, z_offset, 0.05, '%.2f', @on_z_offset_change, false);
y = y - 49;

% View-angle section label
uicontrol(panel, 'Style','text', 'String','View angles:', ...
    'Position', [10 y-18 280 18], 'HorizontalAlignment','left', ...
    'FontWeight','bold', 'BackgroundColor',[0.95 0.95 0.95]);
y = y - 22;

[slider_az, txt_az] = add_slider(panel, y, ...
    'Azimuth (deg)', -180, 180, az, 1, '%d', @on_az_change, true);  % continuous
y = y - 49;

[slider_el_h, txt_el] = add_slider(panel, y, ...
    'Elevation (deg)', -90, 90, el, 1, '%d', @on_el_change, true);  % continuous
y = y - 49;

uicontrol(panel, 'Style','pushbutton', ...
    'String','Read current view -> sliders', ...
    'Position', [10 y-25 280 25], 'Callback', @on_read_view);
y = y - 35;

% Save buttons
uicontrol(panel, 'Style','pushbutton', ...
    'String','SAVE current view (PNG)', ...
    'Position', [10 y-35 280 35], 'FontWeight','bold', ...
    'BackgroundColor', [0.85 0.95 0.85], 'Callback', @on_save);
y = y - 45;

uicontrol(panel, 'Style','pushbutton', ...
    'String','SAVE all 4 files (same view)', ...
    'Position', [10 y-35 280 35], 'FontWeight','bold', ...
    'BackgroundColor', [0.85 0.85 0.95], 'Callback', @on_save_all);

%% ===== Initial rendering =====
draw_voxel_and_bc();
view(ax, az, el);
rotate3d(fig, 'on');

% Load first file and render its tubes
load_and_redraw(current_file_idx);

%% ====================================================================
%%                          Nested callbacks
%% ====================================================================
    function on_file_change(~, evt)
        idx = str2double(get(evt.NewValue, 'Tag'));
        current_file_idx = idx;
        load_and_redraw(idx);
    end

    function on_layer_start_change(src, ~)
        v = round(src.Value);
        v = max(get(slider_ls,'Min'), min(get(slider_ls,'Max'), v));
        layer_start = v;
        set(src, 'Value', v);
        set(txt_ls, 'String', sprintf('= %d', v));
        redraw_tubes();
    end

    function on_layer_count_change(src, ~)
        v = round(src.Value);
        v = max(1, min(get(slider_lc,'Max'), v));
        layer_count = v;
        set(src, 'Value', v);
        set(txt_lc, 'String', sprintf('= %d', v));
        redraw_tubes();
    end

    function on_tube_radius_change(src, ~)
        tube_radius = src.Value;
        set(txt_tr, 'String', sprintf('= %.2f', tube_radius));
        redraw_tubes();
    end

    function on_z_offset_change(src, ~)
        z_offset = src.Value;
        set(txt_zo, 'String', sprintf('= %.2f', z_offset));
        redraw_tubes();
    end

    function on_az_change(src, ~)
        az = src.Value;
        set(txt_az, 'String', sprintf('= %d', round(az)));
        view(ax, az, el);
    end

    function on_el_change(src, ~)
        el = src.Value;
        set(txt_el, 'String', sprintf('= %d', round(el)));
        view(ax, az, el);
    end

    function on_read_view(~, ~)
        [a, e] = view(ax);
        a = max(-180, min(180, a));
        e = max(-90, min(90, e));
        az = a; el = e;
        set(slider_az,   'Value', az);
        set(slider_el_h, 'Value', el);
        set(txt_az, 'String', sprintf('= %d', round(az)));
        set(txt_el, 'String', sprintf('= %d', round(el)));
    end

    function on_auto_center(~, ~)
        if isempty(cache{current_file_idx}), return; end
        pd = cache{current_file_idx};
        [~, peak] = max(pd.layer_richness);
        nL = pd.num_layers;
        new_start = peak - floor((layer_count - 1)/2);
        new_start = max(1, min(max(1, nL - layer_count + 1), new_start));
        layer_start = new_start;
        set(slider_ls, 'Value', layer_start);
        set(txt_ls, 'String', sprintf('= %d', layer_start));
        redraw_tubes();
    end

    function on_save(~, ~)
        save_current(current_file_idx);
    end

    function on_save_all(~, ~)
        orig_idx = current_file_idx;
        for k = 1:length(config.path_mat_list)
            current_file_idx = k;
            set(btn_handles(k), 'Value', 1);
            drawnow;
            load_and_redraw(k);
            drawnow;
            save_current(k);
        end
        current_file_idx = orig_idx;
        set(btn_handles(orig_idx), 'Value', 1);
        load_and_redraw(orig_idx);
    end

%% ====================================================================
%%                       Nested helpers
%% ====================================================================
    function save_current(idx)
        [~, base, ~] = fileparts(config.path_mat_list{idx});
        a_str = sprintf('az%+04d_el%+03d', round(az), round(el));
        out_png = sprintf('fiber_tubes_%s_%s.png', base, a_str);
        try
            exportgraphics(ax, out_png, 'Resolution', 250, 'BackgroundColor', 'white');
        catch
            saveas(fig, out_png);   % older MATLAB fallback
        end
        fprintf('Saved: %s\n', out_png);
    end

    function load_and_redraw(idx)
        if isempty(cache{idx})
            try
                cache{idx} = load_path_data(config.path_mat_list{idx});
            catch ME
                warndlg(sprintf('Failed to load %s:\n%s', ...
                    config.path_mat_list{idx}, ME.message), 'Load error');
                return;
            end
        end
        nL = cache{idx}.num_layers;
        % Update layer_start slider range
        set(slider_ls, 'Max', nL);
        if nL > 1
            set(slider_ls, 'SliderStep', [1/(nL-1), 5/(nL-1)]);
        end
        if layer_start > nL
            layer_start = nL;
            set(slider_ls, 'Value', layer_start);
        end
        set(txt_ls, 'String', sprintf('= %d', layer_start));
        set(fig, 'Name', sprintf('CFRC Path Inspector - [%s]  (num_layers=%d)', ...
            config.file_labels{idx}, nL));
        redraw_tubes();
    end

    function redraw_tubes()
        if current_file_idx < 1 || current_file_idx > length(cache) || ...
                isempty(cache{current_file_idx})
            return;
        end
        pd = cache{current_file_idx};
        nL = pd.num_layers;

        % Delete old tubes
        if ~isempty(tube_handles)
            valid = isvalid(tube_handles);
            delete(tube_handles(valid));
        end
        tube_handles = gobjects(0);

        % Shown layer range
        le = min(nL, layer_start + layer_count - 1);
        shown = layer_start:le;
        if isempty(shown), drawnow; return; end

        N = length(shown);
        center_idx = (N + 1) / 2;
        dz_per = ((1:N)' - center_idx) * z_offset;

        layer_colors = jet(nL);
        for ii = 1:N
            li = shown(ii);
            if li < 1 || li > nL, continue; end
            if ~pd.all_layers_data(li).success, continue; end
            dz = dz_per(ii);
            c  = layer_colors(li, :);
            P  = pd.all_layers_data(li).paths_3d;
            for pi = 1:length(P)
                pts = P{pi};
                if isempty(pts) || size(pts,1) < 2, continue; end
                pts_s = pts;
                pts_s(:,3) = pts(:,3) + dz;
                pts_rs = resample_path_uniform(pts_s, max(0.2, tube_radius*0.7));
                [Xs, Ys, Zs] = tube_along_path(pts_rs, tube_radius, config.tube_nsides);
                h_tube = surf(ax, Xs, Ys, Zs, ...
                    'FaceColor', c, 'EdgeColor', 'none', ...
                    'FaceLighting','gouraud','AmbientStrength',0.50, ...
                    'DiffuseStrength',0.65,'SpecularStrength',0.20);
                tube_handles(end+1) = h_tube; %#ok<AGROW>
            end
        end
        drawnow limitrate;
    end

    function draw_voxel_and_bc()
        % --- voxel ---
        if SHOW_VOXEL
            cube_face = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
            h = config.voxel_half_size;
            nelx = voxel_data.nelx; nely = voxel_data.nely; nelz = voxel_data.nelz;
            valid_mask = voxel_data.valid_grid_mask;
            grid_data  = voxel_data.grid_data;

            n_total = sum(valid_mask(:));
            VertAll = zeros(8*n_total, 3);
            FaceAll = zeros(6*n_total, 4);
            cnt = 0;
            for i = 1:nelx
                for j = 1:nely
                    for k = 1:nelz
                        if ~valid_mask(i,j,k), continue; end
                        gx = grid_data(i,j,k).x;
                        gy = grid_data(i,j,k).y;
                        gz = grid_data(i,j,k).z;
                        v = [gx-h, gy-h, gz-h; gx+h, gy-h, gz-h; ...
                             gx+h, gy-h, gz+h; gx-h, gy-h, gz+h; ...
                             gx-h, gy+h, gz-h; gx+h, gy+h, gz-h; ...
                             gx+h, gy+h, gz+h; gx-h, gy+h, gz+h];
                        base = cnt*8;
                        VertAll(base+1:base+8, :) = v;
                        FaceAll(cnt*6+1:cnt*6+6, :) = cube_face + base;
                        cnt = cnt + 1;
                    end
                end
            end
            VertAll = VertAll(1:cnt*8, :);
            FaceAll = FaceAll(1:cnt*6, :);
            patch('Parent', ax, 'Faces', FaceAll, 'Vertices', VertAll, ...
                  'FaceColor', config.voxel_color, 'EdgeColor', 'none', ...
                  'FaceAlpha', config.voxel_alpha, 'FaceLighting','none');
            fprintf('Drew %d voxels.\n', cnt);
        end

        % --- F arrow ---
        fx = config.bc_load_center(1);
        fy = config.bc_load_center(2);
        fz = config.bc_load_center(3);
        arrow_len = 11;
        quiver3(ax, fx, fy, fz + arrow_len*1.3, 0, 0, -arrow_len, 0, ...
                'Color','r','LineWidth',3.5,'MaxHeadSize',0.55);
        text(ax, fx+1.6, fy+1.6, fz + arrow_len*1.5, 'F', ...
             'Color','r','FontSize',24,'FontWeight','bold','FontAngle','italic');

        % --- fixed supports ---
        for s = 1:size(config.bc_fixed_centers,1)
            cz = config.bc_fixed_centers(s,3) - 1.5;
            draw_fixed_support(ax, config.bc_fixed_centers(s,1), ...
                config.bc_fixed_centers(s,2), cz, 2.8);
        end
        fc = config.bc_fixed_centers(3, :);
        text(ax, fc(1)+5, fc(2)-9, fc(3) - 4.5, ...
             'fixed support', 'FontSize', 14, 'FontAngle','italic');

        % --- style ---
        axis(ax,'equal'); axis(ax,'tight');
        camproj(ax,'perspective');
        camlight(ax,'headlight'); camlight(ax,'left');
        lighting(ax,'gouraud');
        material(ax,'dull');
        grid(ax,'off'); axis(ax,'off');
    end
end  % <== end of main function display_fiber_tubes_interactive


%% =====================================================================
%%                       Subfunctions (no shared state)
%% =====================================================================

function [slider_h, txt_h] = add_slider(parent, y_top, label, ...
        vmin, vmax, vinit, vstep, fmt, cb, continuous)
    if nargin < 10, continuous = false; end
    label_h = 18; slider_h_px = 18; gap = 3;

    uicontrol(parent, 'Style','text', 'String', label, ...
        'Position', [10 y_top-label_h 200 label_h], ...
        'HorizontalAlignment','left', ...
        'BackgroundColor', [0.95 0.95 0.95]);
    txt_h = uicontrol(parent, 'Style','text', ...
        'String', sprintf(['= ' fmt], vinit), ...
        'Position', [210 y_top-label_h 80 label_h], ...
        'HorizontalAlignment','right', ...
        'FontWeight','bold', ...
        'BackgroundColor', [0.95 0.95 0.95]);
    slider_h = uicontrol(parent, 'Style','slider', ...
        'Min', vmin, 'Max', vmax, 'Value', vinit, ...
        'SliderStep', [vstep/(vmax-vmin), vstep*5/(vmax-vmin)], ...
        'Position', [10 y_top-label_h-gap-slider_h_px 280 slider_h_px]);
    if continuous
        addlistener(slider_h, 'ContinuousValueChange', cb);
    else
        set(slider_h, 'Callback', cb);
    end
end


function path_data = load_path_data(PATH_MAT)
    % Same dual-format loader as v5
    assert(exist(PATH_MAT,'file')==2, 'File not found: %s', PATH_MAT);
    tmp = load(PATH_MAT);
    fns = fieldnames(tmp);
    if isempty(fns), error('%s contains no variables', PATH_MAT); end
    var_name = '';
    for k = 1:length(fns)
        if isstruct(tmp.(fns{k})), var_name = fns{k}; break; end
    end
    if isempty(var_name), var_name = fns{1}; end
    fprintf('Loading %s, variable: %s\n', PATH_MAT, var_name);
    s = tmp.(var_name);

    if isfield(s, 'all_layers_data') && isfield(s, 'num_layers')
        path_data.all_layers_data = s.all_layers_data;
        path_data.num_layers      = s.num_layers;
    elseif isfield(s, 'layer_paths_3d') && isfield(s, 'num_layers')
        nL = s.num_layers; lp3d = s.layer_paths_3d;
        ald = struct('success', cell(nL,1), 'paths_3d', cell(nL,1));
        for li = 1:nL
            if li > numel(lp3d), entry = []; else, entry = lp3d{li}; end
            if iscell(entry)
                paths_cell = entry(:).';
            elseif isnumeric(entry) && ~isempty(entry)
                paths_cell = {entry};
            else
                paths_cell = {};
            end
            keep = false(1, numel(paths_cell));
            for pi = 1:numel(paths_cell)
                p = paths_cell{pi};
                keep(pi) = isnumeric(p) && ~isempty(p) && ...
                    size(p,1) >= 2 && size(p,2) >= 3;
            end
            paths_cell = paths_cell(keep);
            ald(li).paths_3d = paths_cell;
            ald(li).success  = ~isempty(paths_cell);
        end
        path_data.all_layers_data = ald;
        path_data.num_layers      = nL;
    else
        error('Variable "%s" in %s has unknown format. Fields: %s', ...
            var_name, PATH_MAT, strjoin(fieldnames(s), ', '));
    end

    % Richness (used by Auto-center)
    layer_richness = zeros(path_data.num_layers, 1);
    for li = 1:path_data.num_layers
        if ~path_data.all_layers_data(li).success, continue; end
        P = path_data.all_layers_data(li).paths_3d;
        n = 0;
        for pi = 1:length(P)
            if ~isempty(P{pi}), n = n + size(P{pi},1); end
        end
        layer_richness(li) = n;
    end
    path_data.layer_richness = layer_richness;
end


function pts_rs = resample_path_uniform(pts, step)
    if size(pts,1) < 2, pts_rs = pts; return; end
    d = [0; cumsum(vecnorm(diff(pts,1,1),2,2))];
    L = d(end);
    if L < 2*step, pts_rs = pts; return; end
    nn = max(8, ceil(L/step));
    d_new = linspace(0, L, nn).';
    pts_rs = [interp1(d, pts(:,1), d_new, 'pchip'), ...
              interp1(d, pts(:,2), d_new, 'pchip'), ...
              interp1(d, pts(:,3), d_new, 'pchip')];
end


function [Xs, Ys, Zs] = tube_along_path(pts, radius, n_sides)
    N = size(pts,1);
    T = zeros(N,3);
    T(1,:) = pts(2,:) - pts(1,:);
    T(N,:) = pts(N,:) - pts(N-1,:);
    for i = 2:N-1
        T(i,:) = pts(i+1,:) - pts(i-1,:);
    end
    Tn = vecnorm(T,2,2); Tn(Tn<1e-12)=1; T = T ./ Tn;

    ref = [0 0 1];
    if abs(dot(T(1,:), ref)) > 0.95, ref = [0 1 0]; end
    Nv = zeros(N,3); Bv = zeros(N,3);
    Nv(1,:) = ref - dot(ref, T(1,:))*T(1,:);
    Nv(1,:) = Nv(1,:)/max(norm(Nv(1,:)),1e-12);
    Bv(1,:) = cross(T(1,:), Nv(1,:));
    for i = 1:N-1
        v1 = pts(i+1,:) - pts(i,:);
        c1 = dot(v1,v1);
        rL = Nv(i,:) - (2/c1)*dot(v1, Nv(i,:))*v1;
        tL = T(i,:)  - (2/c1)*dot(v1, T(i,:))*v1;
        v2 = T(i+1,:) - tL;
        c2 = dot(v2,v2);
        if c2 < 1e-14
            Nv(i+1,:) = Nv(i,:);
        else
            Nv(i+1,:) = rL - (2/c2)*dot(v2, rL)*v2;
        end
        Nv(i+1,:) = Nv(i+1,:) - dot(Nv(i+1,:), T(i+1,:))*T(i+1,:);
        Nv(i+1,:) = Nv(i+1,:)/max(norm(Nv(i+1,:)),1e-12);
        Bv(i+1,:) = cross(T(i+1,:), Nv(i+1,:));
    end
    theta = linspace(0, 2*pi, n_sides+1);
    ct = cos(theta); st = sin(theta);
    Xs = zeros(N, n_sides+1);
    Ys = zeros(N, n_sides+1);
    Zs = zeros(N, n_sides+1);
    for i = 1:N
        for k = 1:n_sides+1
            off = radius*(ct(k)*Nv(i,:) + st(k)*Bv(i,:));
            Xs(i,k) = pts(i,1) + off(1);
            Ys(i,k) = pts(i,2) + off(2);
            Zs(i,k) = pts(i,3) + off(3);
        end
    end
end


function draw_fixed_support(ax, cx, cy, cz, sz)
    V = [cx - sz, cy, cz - sz*0.7;
         cx + sz, cy, cz - sz*0.7;
         cx,      cy, cz + sz*0.6];
    patch('Parent',ax,'Faces',[1 2 3],'Vertices',V,...
          'FaceColor',[0.85 0.85 0.85],'EdgeColor','k','LineWidth',1.0);
    nL = 5;
    for k = 0:nL-1
        x0 = cx - sz + (2*sz)*k/(nL-1);
        plot3(ax, [x0, x0 - 0.7], [cy, cy], ...
              [cz - sz*0.7, cz - sz*0.7 - 1.0], ...
              'k-', 'LineWidth', 1);
    end
end
