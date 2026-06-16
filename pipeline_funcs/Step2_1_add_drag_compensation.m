%% Drag Compensation Script for Connected Paths
% Add drag compensation at sharp turns in printing paths
%
% Data structure:
%   Input: all_layers_data(layer).paths_3d (cell of Nx3 doubles)
%   Output: all_layers_data(layer).modified_paths (cell of Nx3 doubles)
%           all_layers_data(layer).compensation_info (compensation details)
%
% Compensation Logic:
%   1. Turn angle = acosd(dot(dir_in, dir_out))
%      - 0° = straight, 90° = perpendicular, >90° = sharp turn
%   2. Sharp turn (>90°) needs compensation if:
%      - Straight segment (angle < 15°) before turn >= MIN_LENGTH
%      - Straight segment (angle < 15°) after turn >= MIN_LENGTH
%   3. Compensation direction: external angle bisector of angle ABC
%      - A = reference point before B (configurable)
%      - B = turn point
%      - C = reference point after B (configurable)

clear; clc; close all;

%% ==================== Parameters ====================
% Turn angle thresholds
SHARP_THRESHOLD = 90;       % degrees - turn angle > this is sharp
SHARP_THRESHOLD = 930;
STRAIGHT_THRESHOLD = 15;    % degrees - turn angle < this is straight

% Straight segment length requirements
MIN_STRAIGHT_BEFORE = 5.0;  % mm - minimum straight length before turn
MIN_STRAIGHT_AFTER = 5.0;   % mm - minimum straight length after turn

% Reference point selection for compensation direction
% Method: 'points' or 'distance'
REF_POINT_METHOD = 'points';
REF_POINT_VALUE = 1;        % If 'points': number of points to skip
                            % If 'distance': distance in mm

% GPR model parameters (for compensation distance calculation)
T_TEMP = 225;               % Temperature
V_SPEED = 10;               % Printing speed
H_HEIGHT = 0.25;            % Layer height

% File paths
input_file = 'all_layers_path_carbon_resin.mat';
model_file = 'GPRmodel.mat';
output_file = 'Modified_printing_path.mat';

%% ==================== Load Data ====================
fprintf('=============================================\n');
fprintf('  Drag Compensation for Connected Paths\n');
fprintf('=============================================\n\n');

fprintf('Parameters:\n');
fprintf('  Sharp turn threshold: > %.0f deg\n', SHARP_THRESHOLD);
fprintf('  Straight threshold: < %.0f deg\n', STRAIGHT_THRESHOLD);
fprintf('  Min straight before: >= %.1f mm\n', MIN_STRAIGHT_BEFORE);
fprintf('  Min straight after: >= %.1f mm\n', MIN_STRAIGHT_AFTER);
fprintf('  Reference point: %s = %.1f\n\n', REF_POINT_METHOD, REF_POINT_VALUE);
% Load path data
if ~exist(input_file, 'file')
    error('Input file not found: %s\nPlease run assemble_all_layers.m first!', input_file);
end
data = load(input_file);
all_layers_data = data.all_layers_data;
num_layers = length(all_layers_data);
fprintf('Loaded: %d layers\n', num_layers);
% Load GPR model
if ~exist(model_file, 'file')
    warning('GPR model not found: %s\nUsing default compensation formula.', model_file);
    gprMdlv1 = [];
    USE_GPR = false;
else
    load(model_file, 'gprMdlv1');
    USE_GPR = true;
    fprintf('GPR model loaded\n');
end
fprintf('\n');

%% ==================== Process All Layers ====================
fprintf('Processing all layers...\n');
fprintf('========================================\n');

total_sharp_turns = 0;
total_compensations = 0;

for layer = 1:num_layers
    % Check if layer has paths_3d
    if ~isfield(all_layers_data(layer), 'paths_3d') || ...
       isempty(all_layers_data(layer).paths_3d)
        all_layers_data(layer).modified_paths = {};
        all_layers_data(layer).compensation_info = {};
        continue;
    end
    
    paths = all_layers_data(layer).paths_3d;
    num_paths = length(paths);   
    modified_paths = cell(1, num_paths);
    compensation_info = cell(1, num_paths);
    layer_sharp_turns = 0;
    layer_compensations = 0;
    
    for path_idx = 1:num_paths
        original_path = paths{path_idx};
        n_points = size(original_path, 1);
        
        if n_points < 3
            modified_paths{path_idx} = original_path;
            compensation_info{path_idx} = [];
            continue;
        end
        
        % Calculate path geometry
        [turn_angles, seg_lengths, directions] = calculate_path_geometry(original_path);
        
        % Find sharp turns
        sharp_indices = find(turn_angles > SHARP_THRESHOLD);
        layer_sharp_turns = layer_sharp_turns + length(sharp_indices);
        
        % Check each sharp turn for compensation eligibility
        comp_points = [];  % [idx, distance, x, y, z, angle, len_before, len_after]
        
        for k = 1:length(sharp_indices)
            idx = sharp_indices(k);
            turn_angle = turn_angles(idx);
            
            % Check straight segment before
            [has_before, len_before] = check_straight_before(...
                idx, turn_angles, seg_lengths, STRAIGHT_THRESHOLD, MIN_STRAIGHT_BEFORE);
            
            % Check straight segment after
            [has_after, len_after] = check_straight_after(...
                idx, turn_angles, seg_lengths, STRAIGHT_THRESHOLD, MIN_STRAIGHT_AFTER);
            
            % Both conditions must be met
            if has_before && has_after
                % Get reference points A and C
                [idx_A, idx_C] = get_reference_points(...
                    idx, n_points, original_path, seg_lengths, REF_POINT_METHOD, REF_POINT_VALUE);
                
                % Calculate compensation distance
                if USE_GPR
                    gpr_input = [T_TEMP, V_SPEED, H_HEIGHT, 180 - turn_angle/2];
                    comp_distance = predict(gprMdlv1, gpr_input);
                    comp_distance = max(0, comp_distance);
                else
                    % Default formula: 0.5mm at 90°, up to 2mm at 180°
                    comp_distance = 0.5 + (turn_angle - 90) / 90 * 1.5;
                end
                
                % Calculate compensation point position
                [comp_x, comp_y, comp_z] = calculate_compensation_point(...
                    original_path, idx, idx_A, idx_C, comp_distance);
                
                comp_points = [comp_points; idx, comp_distance, comp_x, comp_y, comp_z, ...
                    turn_angle, len_before, len_after];
                layer_compensations = layer_compensations + 1;
            end
        end
        
        % Insert compensation points into path
        if ~isempty(comp_points)
            compensated_path = insert_compensation_points(original_path, comp_points);
        else
            compensated_path = original_path;
        end
        
        modified_paths{path_idx} = compensated_path;
        compensation_info{path_idx} = comp_points;
    end
    
    % Save results
    all_layers_data(layer).modified_paths = modified_paths;
    all_layers_data(layer).compensation_info = compensation_info;
    
    total_sharp_turns = total_sharp_turns + layer_sharp_turns;
    total_compensations = total_compensations + layer_compensations;
    
    % Print progress
    if layer_compensations > 0 || mod(layer, 10) == 0 || layer == num_layers
        fprintf('Layer %3d: %3d sharp turns, %3d compensated\n', ...
            layer, layer_sharp_turns, layer_compensations);
    end
end

%% ==================== Save Results ====================
fprintf('\n========================================\n');
fprintf('Saving results...\n');

% Save compensation parameters
compensation_params = struct();
compensation_params.SHARP_THRESHOLD = SHARP_THRESHOLD;
compensation_params.STRAIGHT_THRESHOLD = STRAIGHT_THRESHOLD;
compensation_params.MIN_STRAIGHT_BEFORE = MIN_STRAIGHT_BEFORE;
compensation_params.MIN_STRAIGHT_AFTER = MIN_STRAIGHT_AFTER;
compensation_params.REF_POINT_METHOD = REF_POINT_METHOD;
compensation_params.REF_POINT_VALUE = REF_POINT_VALUE;
compensation_params.T_TEMP = T_TEMP;
compensation_params.V_SPEED = V_SPEED;
compensation_params.H_HEIGHT = H_HEIGHT;
compensation_params.USE_GPR = USE_GPR;
compensation_params.processed_date = datestr(now);

save(output_file, 'all_layers_data', 'compensation_params','-v7.3');
fprintf('Saved to: %s\n', output_file);

%% ==================== Summary ====================
fprintf('\n==================== Summary ====================\n');
fprintf('Total layers: %d\n', num_layers);
fprintf('Total sharp turns found: %d\n', total_sharp_turns);
fprintf('Total compensations added: %d\n', total_compensations);
if total_sharp_turns > 0
    fprintf('Compensation rate: %.1f%%\n', 100 * total_compensations / total_sharp_turns);
end
fprintf('================================================\n');

%% ==================== Visualization ====================
fprintf('\nGenerating visualization...\n');

% Find a layer with compensations
vis_layer = 0;
for layer = 1:num_layers
    if isfield(all_layers_data(layer), 'compensation_info') && ...
       ~isempty(all_layers_data(layer).compensation_info)
        for p = 1:length(all_layers_data(layer).compensation_info)
            if ~isempty(all_layers_data(layer).compensation_info{p})
                vis_layer = layer;
                break;
            end
        end
    end
    if vis_layer > 0, break; end
end

if vis_layer > 0
    figure('Name', 'Compensation Result', 'Position', [50, 50, 1500, 700]);
    
    orig_paths = all_layers_data(vis_layer).paths_3d;
    comp_paths = all_layers_data(vis_layer).modified_paths;
    comp_info = all_layers_data(vis_layer).compensation_info;
    
    % Left: Original paths with sharp turn markers
    subplot(1, 2, 1);
    hold on;
    colors = lines(length(orig_paths));
    
    for p = 1:length(orig_paths)
        path = orig_paths{p};
        if size(path, 1) < 3, continue; end
        
        plot(path(:,1), path(:,2), '-', 'Color', colors(p,:), 'LineWidth', 1.5);
        
        % Mark sharp turns
        [angles, ~, ~] = calculate_path_geometry(path);
        for i = 2:size(path,1)-1
            if angles(i) > SHARP_THRESHOLD
                plot(path(i,1), path(i,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
        end
    end
    hold off;
    xlabel('X (mm)'); ylabel('Y (mm)');
    title(sprintf('Layer %d - Original\nRed circles = sharp turns (>%.0f°)', vis_layer, SHARP_THRESHOLD));
    axis equal; grid on;
    
    % Right: Compensated paths with compensation points
    subplot(1, 2, 2);
    hold on;
    
    total_comp_vis = 0;
    for p = 1:length(comp_paths)
        path = comp_paths{p};
        plot(path(:,1), path(:,2), '-', 'Color', colors(p,:), 'LineWidth', 1.5);
        
        % Mark compensation points
        if ~isempty(comp_info{p})
            for j = 1:size(comp_info{p}, 1)
                cx = comp_info{p}(j, 3);
                cy = comp_info{p}(j, 4);
                angle = comp_info{p}(j, 6);
                dist = comp_info{p}(j, 2);
                
                plot(cx, cy, 'd', 'Color', 'm', 'MarkerSize', 12, ...
                    'MarkerFaceColor', 'm', 'LineWidth', 2);
                text(cx, cy, sprintf('  %.0f°\n  +%.2fmm', angle, dist), ...
                    'FontSize', 8, 'Color', 'm');
                
                total_comp_vis = total_comp_vis + 1;
            end
        end
    end
    hold off;
    xlabel('X (mm)'); ylabel('Y (mm)');
    title(sprintf('Layer %d - Compensated\nMagenta ◆ = %d compensation points', vis_layer, total_comp_vis));
    axis equal; grid on;
    
    sgtitle(sprintf('Drag Compensation Result - Layer %d', vis_layer), 'FontSize', 14);
else
    fprintf('No compensation points found for visualization.\n');
end

%% ==================== Statistics Plot ====================
figure('Name', 'Compensation Statistics', 'Position', [100, 100, 800, 400]);

comp_per_layer = zeros(num_layers, 1);
sharp_per_layer = zeros(num_layers, 1);

for layer = 1:num_layers
    if isfield(all_layers_data(layer), 'compensation_info')
        for p = 1:length(all_layers_data(layer).compensation_info)
            if ~isempty(all_layers_data(layer).compensation_info{p})
                comp_per_layer(layer) = comp_per_layer(layer) + ...
                    size(all_layers_data(layer).compensation_info{p}, 1);
            end
        end
    end
end

bar(comp_per_layer);
xlabel('Layer');
ylabel('Compensation Points');
title('Compensation Points per Layer');
grid on;

fprintf('\nDone!\n');
fprintf('Results saved in:\n');
fprintf('  all_layers_data(layer).modified_paths\n');
fprintf('  all_layers_data(layer).compensation_info\n');

%% ==================== Helper Functions ====================

function [turn_angles, seg_lengths, directions] = calculate_path_geometry(path)
    n = size(path, 1);
    turn_angles = zeros(n, 1);
    seg_lengths = zeros(n-1, 1);
    directions = zeros(n-1, 3);
    
    % Calculate segments
    for i = 1:n-1
        vec = path(i+1, :) - path(i, :);
        seg_lengths(i) = norm(vec);
        if seg_lengths(i) > 1e-10
            directions(i, :) = vec / seg_lengths(i);
        end
    end
    
    % Calculate turn angles
    for i = 2:n-1
        if seg_lengths(i-1) > 1e-10 && seg_lengths(i) > 1e-10
            cos_angle = dot(directions(i-1,:), directions(i,:));
            cos_angle = max(-1, min(1, cos_angle));
            turn_angles(i) = acosd(cos_angle);
        end
    end
end

function [has_straight, total_length] = check_straight_before(idx, turn_angles, seg_lengths, thresh, min_len)
    total_length = 0;
    has_straight = false;
    
    for i = idx-1:-1:1
        if i >= 2 && turn_angles(i) >= thresh
            if total_length >= min_len
                has_straight = true;
                return;
            else
                total_length = 0;
            end
        end
        
        if i <= length(seg_lengths)
            total_length = total_length + seg_lengths(i);
        end
        
        if total_length >= min_len
            has_straight = true;
            return;
        end
    end
    
    if total_length >= min_len
        has_straight = true;
    end
end

function [has_straight, total_length] = check_straight_after(idx, turn_angles, seg_lengths, thresh, min_len)
    n = length(turn_angles);
    total_length = 0;
    has_straight = false;
    
    for i = idx+1:n-1
        if turn_angles(i) >= thresh
            if total_length >= min_len
                has_straight = true;
                return;
            else
                total_length = 0;
            end
        end
        
        if i-1 >= idx && i-1 <= length(seg_lengths)
            total_length = total_length + seg_lengths(i-1);
        end
        
        if total_length >= min_len
            has_straight = true;
            return;
        end
    end
    
    % Add remaining segment
    if idx <= length(seg_lengths)
        total_length = total_length + seg_lengths(idx);
    end
    
    if total_length >= min_len
        has_straight = true;
    end
end

function [idx_A, idx_C] = get_reference_points(idx_B, n_points, path, seg_lengths, method, value)
    if strcmp(method, 'points')
        idx_A = max(1, idx_B - value);
        idx_C = min(n_points, idx_B + value);
    else
        % Distance-based selection
        dist = 0;
        idx_A = idx_B;
        for i = idx_B-1:-1:1
            if i < length(seg_lengths)
                dist = dist + seg_lengths(i);
            end
            idx_A = i;
            if dist >= value, break; end
        end
        
        dist = 0;
        idx_C = idx_B;
        for i = idx_B:min(length(seg_lengths), n_points-1)
            dist = dist + seg_lengths(i);
            idx_C = i + 1;
            if dist >= value, break; end
        end
    end
    
    idx_A = max(1, idx_A);
    idx_C = min(n_points, idx_C);
    
    if idx_A == idx_B && idx_A > 1, idx_A = idx_A - 1; end
    if idx_C == idx_B && idx_C < n_points, idx_C = idx_C + 1; end
end

function [comp_x, comp_y, comp_z] = calculate_compensation_point(path, idx_B, idx_A, idx_C, distance)
    B = path(idx_B, :);
    A = path(idx_A, :);
    C = path(idx_C, :);
    
    vec_BA = A - B;
    vec_BC = C - B;
    
    len_BA = norm(vec_BA);
    len_BC = norm(vec_BC);
    
    if len_BA < 1e-10 || len_BC < 1e-10
        comp_x = B(1); comp_y = B(2); comp_z = B(3);
        return;
    end
    
    dir_BA = vec_BA / len_BA;
    dir_BC = vec_BC / len_BC;
    
    % External bisector (outward direction)
    external_bisector = -(dir_BA + dir_BC);
    ext_norm = norm(external_bisector);
    
    if ext_norm < 1e-10
        perp = [-dir_BA(2), dir_BA(1), 0];
        if norm(perp) < 1e-10
            perp = [0, -dir_BA(3), dir_BA(2)];
        end
        external_bisector = perp;
        ext_norm = norm(external_bisector);
    end
    
    external_bisector = external_bisector / ext_norm;
    comp_point = B + distance * external_bisector;
    
    comp_x = comp_point(1);
    comp_y = comp_point(2);
    comp_z = comp_point(3);
end

function new_path = insert_compensation_points(original_path, comp_points)
    % Sort descending by index
    comp_points = sortrows(comp_points, 1, 'descend');
    
    new_path = original_path;
    
    for i = 1:size(comp_points, 1)
        idx = comp_points(i, 1);
        comp_xyz = comp_points(i, 3:5);
        new_path = [new_path(1:idx, :); comp_xyz; new_path(idx+1:end, :)];
    end
end