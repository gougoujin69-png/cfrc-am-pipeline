%% 3D Printing Path Animation Player - GUI Version
% Interactive animation player with play/pause, speed control, layer selection

function play_path_animation_gui()

    %% Load Data
    input_file = 'printing_path_connected.mat';
    
    if ~exist(input_file, 'file')
        errordlg(['File not found: ' input_file char(10) 'Please run connect_spiral_paths.m first!'], 'Error');
        return;
    end
    data = load(input_file);
    connected_layers_data = data.all_layers_data;
    num_layers = length(connected_layers_data);
    
    %% Initialize State
    state = struct();
    state.current_layer = 1;
    state.is_playing = false;
    state.play_speed = 30;  % points per frame
    state.frame_delay = 0.01;
    state.point_idx = 1;
    state.path_idx = 1;
    state.timer = [];
    
    %% Create GUI Figure
    fig = figure('Name', 'Path Animation Player', ...
        'Position', [100, 100, 1200, 800], ...
        'Color', 'w', ...
        'CloseRequestFcn', @closeCallback, ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'ToolBar', 'figure');
    
    %% Create Axes for Animation
    ax = axes('Parent', fig, 'Position', [0.05, 0.15, 0.9, 0.75]);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X (mm)');
    ylabel(ax, 'Y (mm)');
    zlabel(ax, 'Z (mm)');
    title(ax, 'Select a layer and press Play');
    view(ax, 2);
    
    %% Create Control Panel
    panel = uipanel('Parent', fig, 'Position', [0.05, 0.02, 0.9, 0.1], ...
        'Title', 'Controls', 'FontSize', 10);
    
    % Layer selection
    uicontrol('Parent', panel, 'Style', 'text', 'String', 'Layer:', ...
        'Units', 'normalized', 'Position', [0.01, 0.3, 0.04, 0.4], ...
        'HorizontalAlignment', 'right');
    
    layer_edit = uicontrol('Parent', panel, 'Style', 'edit', ...
        'String', '1', ...
        'Units', 'normalized', 'Position', [0.055, 0.3, 0.05, 0.5], ...
        'Callback', @layerCallback);
    
    uicontrol('Parent', panel, 'Style', 'text', ...
        'String', sprintf('/ %d', num_layers), ...
        'Units', 'normalized', 'Position', [0.11, 0.3, 0.04, 0.4], ...
        'HorizontalAlignment', 'left');
    
    % Load button
    uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Load', ...
        'Units', 'normalized', 'Position', [0.16, 0.2, 0.06, 0.6], ...
        'Callback', @loadCallback);
    
    % Play/Pause button
    play_btn = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Play', ...
        'Units', 'normalized', 'Position', [0.24, 0.2, 0.08, 0.6], ...
        'Callback', @playCallback, ...
        'BackgroundColor', [0.5, 1, 0.5]);
    
    % Stop button
    uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Stop', ...
        'Units', 'normalized', 'Position', [0.33, 0.2, 0.06, 0.6], ...
        'Callback', @stopCallback);
    
    % Speed control
    uicontrol('Parent', panel, 'Style', 'text', 'String', 'Speed:', ...
        'Units', 'normalized', 'Position', [0.42, 0.3, 0.05, 0.4], ...
        'HorizontalAlignment', 'right');
    
    speed_slider = uicontrol('Parent', panel, 'Style', 'slider', ...
        'Min', 1, 'Max', 200, 'Value', 30, ...
        'Units', 'normalized', 'Position', [0.48, 0.35, 0.15, 0.35], ...
        'Callback', @speedCallback);
    
    speed_label = uicontrol('Parent', panel, 'Style', 'text', ...
        'String', '30 pts/frame', ...
        'Units', 'normalized', 'Position', [0.48, 0.02, 0.15, 0.3]);
    
    % Progress bar
    uicontrol('Parent', panel, 'Style', 'text', 'String', 'Progress:', ...
        'Units', 'normalized', 'Position', [0.65, 0.3, 0.06, 0.4], ...
        'HorizontalAlignment', 'right');
    
    progress_label = uicontrol('Parent', panel, 'Style', 'text', ...
        'String', '0%', ...
        'Units', 'normalized', 'Position', [0.72, 0.3, 0.08, 0.4], ...
        'HorizontalAlignment', 'left', ...
        'FontWeight', 'bold');
    
    % Info label
    info_label = uicontrol('Parent', panel, 'Style', 'text', ...
        'String', 'Ready', ...
        'Units', 'normalized', 'Position', [0.82, 0.2, 0.17, 0.6], ...
        'HorizontalAlignment', 'left');
    
    %% Graphics Objects
    h_background = [];
    h_lines = [];
    h_head = [];
    paths = {};
    total_points = 0;
    
    %% Callback Functions
    function closeCallback(~, ~)
        stopAnimation();
        delete(fig);
    end
    
    function layerCallback(src, ~)
        val = str2double(get(src, 'String'));
        if isnan(val) || val < 1 || val > num_layers
            set(src, 'String', num2str(state.current_layer));
        else
            state.current_layer = round(val);
        end
    end
    
    function loadCallback(~, ~)
        stopAnimation();
        
        layer = state.current_layer;
        
        % Check data
        if ~isfield(connected_layers_data, 'connected_paths') || ...
           isempty(connected_layers_data(layer).connected_paths)
            set(info_label, 'String', 'No data!');
            return;
        end
        
        paths = connected_layers_data(layer).connected_paths;
        num_paths = length(paths);
        
        % Calculate total points
        total_points = 0;
        for i = 1:num_paths
            total_points = total_points + size(paths{i}, 1);
        end
        
        % Clear axes
        cla(ax);
        hold(ax, 'on');
        
        % Get axis limits
        all_pts = [];
        for i = 1:num_paths
            all_pts = [all_pts; paths{i}];
        end
        
        x_range = [min(all_pts(:,1)), max(all_pts(:,1))];
        y_range = [min(all_pts(:,2)), max(all_pts(:,2))];
        margin_x = (x_range(2) - x_range(1)) * 0.1;
        margin_y = (y_range(2) - y_range(1)) * 0.1;
        
        xlim(ax, [x_range(1)-margin_x, x_range(2)+margin_x]);
        ylim(ax, [y_range(1)-margin_y, y_range(2)+margin_y]);
        
        % Draw background paths
        h_background = gobjects(num_paths, 1);
        for i = 1:num_paths
            h_background(i) = plot3(ax, paths{i}(:,1), paths{i}(:,2), paths{i}(:,3), ...
                '-', 'Color', [0.85, 0.85, 0.85], 'LineWidth', 1);
        end
        
        % Create animated lines
        colors = lines(num_paths);
        h_lines = gobjects(num_paths, 1);
        for i = 1:num_paths
            h_lines(i) = animatedline(ax, 'Color', colors(i,:), 'LineWidth', 2);
        end
        
        % Create head marker
        h_head = plot3(ax, NaN, NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        
        title(ax, sprintf('Layer %d - %d paths, %d points (Ready)', layer, num_paths, total_points));
        view(ax, 2);
        grid(ax, 'on');
        axis(ax, 'equal');
        
        % Reset state
        state.path_idx = 1;
        state.point_idx = 1;
        
        set(info_label, 'String', sprintf('%d paths, %d pts', num_paths, total_points));
        set(progress_label, 'String', '0%');
        
        drawnow;
    end
    
    function playCallback(~, ~)
        if isempty(paths)
            loadCallback([], []);
            if isempty(paths)
                return;
            end
        end
        
        if state.is_playing
            % Pause
            state.is_playing = false;
            set(play_btn, 'String', 'Play', 'BackgroundColor', [0.5, 1, 0.5]);
            if ~isempty(state.timer) && isvalid(state.timer)
                stop(state.timer);
            end
        else
            % Play
            state.is_playing = true;
            set(play_btn, 'String', 'Pause', 'BackgroundColor', [1, 1, 0.5]);
            
            % Create timer for animation
            state.timer = timer('ExecutionMode', 'fixedRate', ...
                'Period', max(0.01, state.frame_delay), ...
                'TimerFcn', @animationStep);
            start(state.timer);
        end
    end
    
    function stopCallback(~, ~)
        stopAnimation();
        
        % Reset animation
        if ~isempty(h_lines)
            for i = 1:length(h_lines)
                if isvalid(h_lines(i))
                    clearpoints(h_lines(i));
                end
            end
        end
        
        state.path_idx = 1;
        state.point_idx = 1;
        set(progress_label, 'String', '0%');
        set(play_btn, 'String', 'Play', 'BackgroundColor', [0.5, 1, 0.5]);
        
        if ~isempty(h_head) && isvalid(h_head)
            set(h_head, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end
        
        title(ax, sprintf('Layer %d - Stopped', state.current_layer));
    end
    
    function stopAnimation()
        state.is_playing = false;
        if ~isempty(state.timer) && isvalid(state.timer)
            stop(state.timer);
            delete(state.timer);
        end
        state.timer = [];
    end
    
    function speedCallback(src, ~)
        state.play_speed = round(get(src, 'Value'));
        set(speed_label, 'String', sprintf('%d pts/frame', state.play_speed));
    end
    
    function animationStep(~, ~)
        if ~state.is_playing || isempty(paths)
            return;
        end
        
        try
            num_paths = length(paths);
            
            if state.path_idx > num_paths
                % Animation complete
                stopAnimation();
                set(play_btn, 'String', 'Play', 'BackgroundColor', [0.5, 1, 0.5]);
                title(ax, sprintf('Layer %d - Complete!', state.current_layer));
                set(h_head, 'Visible', 'off');
                return;
            end
            
            current_path = paths{state.path_idx};
            n_points = size(current_path, 1);
            
            % Add points
            end_idx = min(state.point_idx + state.play_speed - 1, n_points);
            
            addpoints(h_lines(state.path_idx), ...
                current_path(state.point_idx:end_idx, 1), ...
                current_path(state.point_idx:end_idx, 2), ...
                current_path(state.point_idx:end_idx, 3));
            
            % Update head
            set(h_head, 'XData', current_path(end_idx, 1), ...
                       'YData', current_path(end_idx, 2), ...
                       'ZData', current_path(end_idx, 3), ...
                       'Visible', 'on');
            
            % Calculate progress
            points_done = 0;
            for i = 1:state.path_idx-1
                points_done = points_done + size(paths{i}, 1);
            end
            points_done = points_done + end_idx;
            progress = 100 * points_done / total_points;
            
            set(progress_label, 'String', sprintf('%.1f%%', progress));
            title(ax, sprintf('Layer %d - Path %d/%d', state.current_layer, state.path_idx, num_paths));
            
            % Update indices
            if end_idx >= n_points
                state.path_idx = state.path_idx + 1;
                state.point_idx = 1;
            else
                state.point_idx = end_idx + 1;
            end
            
            drawnow limitrate;
            
        catch ME
            stopAnimation();
            warning('Animation error: %s', ME.message);
        end
    end
    
    %% Initial message
    fprintf('Path Animation Player GUI started\n');
    fprintf('1. Enter layer number and click "Load"\n');
    fprintf('2. Click "Play" to start animation\n');
    fprintf('3. Use speed slider to adjust playback speed\n\n');

end