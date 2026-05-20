%% ========================================
%% 细化模型完整切片系统 v6
%% 纯几何偏置: 4倍域覆盖 + Z投影 + 包络修复
%% ========================================
% v6 设计哲学 (vs v5b):
%   v5b 问题:
%     a. validity mask + 线性外推交互导致边界塔楼
%     b. 曲率限制per-point制造假山峰(不够向下→相对突起)
%     c. scatteredInterpolant重网格化在cusp处混乱
%   v6 策略: 彻底简化
%     1. 4倍域覆盖: margin=1.5x范围, 内部零边界影响
%     2. 去掉validity mask: 偏置是纯数学运算
%     3. 纯Z投影: Z_off=Z0+offset*nz, 永远不自交
%     4. 去掉scatteredInterpolant重网格化
%     5. 去掉per-point曲率限制
%     6. 新增包络修复: 高斯平滑参考+方向裁剪消除cusp
%%

fprintf('\n');
fprintf('==============================================================\n');
fprintf('     Slicing System v6 (4x Domain + Envelope Repair)        \n');
fprintf('==============================================================\n\n');

%% ========== Step 0: Toolbox Detection ==========
fprintf('[Step 0] Checking toolboxes...\n');
v = ver;
hasParallelToolbox = any(strcmp({v.Name}, 'Parallel Computing Toolbox'));
hasCurveFitting = any(strcmp({v.Name}, 'Curve Fitting Toolbox'));

if hasParallelToolbox
    fprintf('  Parallel Computing Toolbox: installed\n');
    p = gcp('nocreate');
    if isempty(p)
        try
            parpool('local');
            p = gcp('nocreate');
            fprintf('  Workers: %d\n', p.NumWorkers);
        catch
            fprintf('  Warning: parallel pool failed, using serial\n');
            hasParallelToolbox = false;
        end
    else
        fprintf('  Workers: %d\n', p.NumWorkers);
    end
else
    fprintf('  Parallel Computing Toolbox: not installed (serial mode)\n');
end

if hasCurveFitting
    fprintf('  Curve Fitting Toolbox: installed\n');
else
    fprintf('  Curve Fitting Toolbox: not installed\n');
end

%% ========== Step 1: Load Data ==========
fprintf('\n[1] Loading data...\n');

if ~exist('voxel_refined_latest.mat', 'file')
    error('Cannot find voxel_refined_latest.mat');
end

load('voxel_refined_latest.mat');
load('Pre_surface.mat');

grid_data = refined_data.grid_data;
valid_grid_mask = refined_data.valid_grid_mask;
nelx = refined_data.grid_size.nelx;
nely = refined_data.grid_size.nely;
nelz = refined_data.grid_size.nelz;

fprintf('  Grid: %d x %d x %d, valid: %d\n', nelx, nely, nelz, sum(valid_grid_mask(:)));

%% ========== Step 2: Parameters ==========
fprintf('\n[2] Parameters...\n');

SCALE_FACTOR = refined_data.parameters.SCALE_FACTOR;

OFFSET_STEP_ORIG = 1.0;
INITIAL_THRESHOLD_ORIG = 0.70;
THRESHOLD_INCREMENT_ORIG = 0.5;
SURFACE_RESOLUTION_ORIG = 0.04;

OFFSET_STEP = OFFSET_STEP_ORIG * SCALE_FACTOR;
INITIAL_THRESHOLD = INITIAL_THRESHOLD_ORIG * SCALE_FACTOR;
THRESHOLD_INCREMENT = THRESHOLD_INCREMENT_ORIG * SCALE_FACTOR;
SURFACE_RESOLUTION = SURFACE_RESOLUTION_ORIG / SCALE_FACTOR;
DENSITY_THRESHOLD = 0.5;
NEW_ACTIVATION_THRESHOLD = 5;
MAX_OFFSET = 120 / SCALE_FACTOR;
MAX_THRESHOLD_RETRIES = 3;
MAX_CONSECUTIVE_EMPTY = 2;

% v5 parameters
NZ_FLOOR = 0.20;            % soft floor (sigmoid, not hard clamp), can be slightly lower
DERIV_SMOOTH_SIGMA = 3.0;   % BASE Gaussian sigma (will be adaptively increased)
MEDFILT_SIZE = 5;            % BASE median filter window (will be adaptively increased)

fprintf('  OFFSET_STEP: %.4f, SURFACE_RESOLUTION: %.5f\n', OFFSET_STEP, SURFACE_RESOLUTION);
fprintf('  NZ_FLOOR: %.2f, DERIV_SMOOTH_SIGMA: %.1f, MEDFILT: %d\n', ...
    NZ_FLOOR, DERIV_SMOOTH_SIGMA, MEDFILT_SIZE);

%% ========== Step 3: Build Base Surface + Validity Mask ==========
fprintf('\n[3] Building base surface...\n');

sp = refined_data.surface_params;
a = sp.a; b = sp.b; c = sp.c; d = sp.d;
e = sp.e; f = sp.f; g = sp.g; h = sp.h;
X0 = sp.X0; Y0 = sp.Y0; Para_me = sp.Para_me;

Xs = Pre_surface{1}; Ys = Pre_surface{2}; Zs = Pre_surface{3};

% --- Scattered interpolation to fine regular grid ---
F_z_raw = scatteredInterpolant(Xs(:), Ys(:), Zs(:), 'natural', 'none');

xs_min = min(Xs(:)); xs_max = max(Xs(:));
ys_min = min(Ys(:)); ys_max = max(Ys(:));

fine_res = SURFACE_RESOLUTION / 4;
xx_fine = xs_min:fine_res:xs_max;
yy_fine = ys_min:fine_res:ys_max;
[Xg_fine, Yg_fine] = meshgrid(xx_fine, yy_fine);

Z_fine = F_z_raw(Xg_fine, Yg_fine);
fprintf('  Fine grid: %d x %d (res %.4f)\n', length(xx_fine), length(yy_fine), fine_res);

% === KEY: Save the original valid data mask BEFORE any NaN filling ===
valid_data_mask_fine = ~isnan(Z_fine);
fprintf('  Valid data coverage: %.1f%%\n', 100*sum(valid_data_mask_fine(:))/numel(Z_fine));

% --- Median filter ---
Z_smooth = Z_fine;
if sum(valid_data_mask_fine(:)) > 100
    Z_temp = Z_fine;
    Z_temp(~valid_data_mask_fine) = 0;
    Z_smooth = medfilt2(Z_temp, [9 9], 'symmetric');
    Z_smooth(~valid_data_mask_fine) = NaN;
    zero_but_was_nan = (Z_smooth == 0) & ~valid_data_mask_fine;
    Z_smooth(zero_but_was_nan) = NaN;
    Z_smooth(valid_data_mask_fine & isnan(Z_smooth)) = Z_fine(valid_data_mask_fine & isnan(Z_smooth));
end

% --- csaps smooth ---
if hasCurveFitting
    SURFACE_SMOOTH_P = 0.9999;
    [ny_f, nx_f] = size(Z_smooth);
    for row = 1:ny_f
        zl = Z_smooth(row,:);
        vl = ~isnan(zl);
        if sum(vl) < 6, continue; end
        pp = csaps(xx_fine(vl), zl(vl), SURFACE_SMOOTH_P);
        Z_smooth(row, vl) = fnval(pp, xx_fine(vl));
    end
    for col = 1:nx_f
        zl = Z_smooth(:,col)';
        vl = ~isnan(zl);
        if sum(vl) < 6, continue; end
        pp = csaps(yy_fine(vl), zl(vl), SURFACE_SMOOTH_P);
        Z_smooth(vl', col) = fnval(pp, yy_fine(vl)')';
    end
    fprintf('  csaps smooth done (p=%.4f)\n', SURFACE_SMOOTH_P);
end

% --- Fill NaN for griddedInterpolant ---
Z_smooth(~valid_data_mask_fine) = NaN;

nan_interior = valid_data_mask_fine & isnan(Z_smooth);
if any(nan_interior(:))
    gm = ~isnan(Z_smooth);
    if sum(gm(:)) > 50
        F_fill = scatteredInterpolant(Xg_fine(gm), Yg_fine(gm), Z_smooth(gm), 'natural', 'nearest');
        Z_smooth(nan_interior) = F_fill(Xg_fine(nan_interior), Yg_fine(nan_interior));
    end
end

still_nan = isnan(Z_smooth);
if any(still_nan(:))
    gm2 = ~isnan(Z_smooth);
    if sum(gm2(:)) > 50
        F_fill2 = scatteredInterpolant(Xg_fine(gm2), Yg_fine(gm2), Z_smooth(gm2), 'nearest', 'nearest');
        Z_smooth(still_nan) = F_fill2(Xg_fine(still_nan), Yg_fine(still_nan));
    end
end

% --- Build interpolant ---
% === v5b: Smooth boundary extrapolation ===
% v6: With 4x domain coverage (margin = 1.5x range ≈ 30 units), 'nearest'
% extrapolation is safe — the flat zone is far from the data region, so
% derivatives inside are unaffected. 'linear' caused wild divergence outside.

F_z_smooth = griddedInterpolant({yy_fine, xx_fine}, Z_smooth, 'cubic', 'nearest');
fprintf('  Boundary extrapolation: nearest (4x domain makes it safe)\n');

compute_surface_z = @(x, y) F_z_smooth(y, x);

% (Validity mask removed from offset function in v6 - surface covers 4x domain)

fprintf('  Base surface built (griddedInterpolant, cubic)\n');
fprintf('  Z range: [%.3f, %.3f]\n', min(Z_smooth(:)), max(Z_smooth(:)));

%% ========== Step 4: Bounds + Classification ==========
fprintf('\n[4] Bounds and classification...\n');

total_valid_grids = sum(valid_grid_mask(:));
min_valid_x = inf; max_valid_x = -inf;
min_valid_y = inf; max_valid_y = -inf;
min_valid_z = inf; max_valid_z = -inf;

above_surface_mask = zeros(nelx, nely, nelz);
below_surface_mask = zeros(nelx, nely, nelz);

for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if valid_grid_mask(i,j,k)
                gc_x = grid_data(i,j,k).x;
                gc_y = grid_data(i,j,k).y;
                gc_z = grid_data(i,j,k).z;
                min_valid_x = min(min_valid_x, gc_x);
                max_valid_x = max(max_valid_x, gc_x);
                min_valid_y = min(min_valid_y, gc_y);
                max_valid_y = max(max_valid_y, gc_y);
                min_valid_z = min(min_valid_z, gc_z);
                max_valid_z = max(max_valid_z, gc_z);
                
                surf_z = compute_surface_z(gc_x, gc_y);
                if gc_z > surf_z
                    above_surface_mask(i,j,k) = 1;
                else
                    below_surface_mask(i,j,k) = 1;
                end
            end
        end
    end
end

num_above = sum(above_surface_mask(:));
num_below = sum(below_surface_mask(:));
fprintf('  X:[%.2f,%.2f] Y:[%.2f,%.2f] Z:[%.2f,%.2f]\n', ...
    min_valid_x, max_valid_x, min_valid_y, max_valid_y, min_valid_z, max_valid_z);
fprintf('  Above: %d, Below: %d\n', num_above, num_below);

%% ========== Step 4b: Z Height Field ==========
fprintf('\n[4b] Building Z height field...\n');

all_valid_xyz = [];
for i = 1:nelx
    for j = 1:nely
        for k = 1:nelz
            if valid_grid_mask(i,j,k)
                gc = grid_data(i,j,k);
                all_valid_xyz = [all_valid_xyz; gc.x, gc.y, gc.z];
            end
        end
    end
end

zmap_res = SURFACE_RESOLUTION * 2;
zmap_xx = (min_valid_x - 1):zmap_res:(max_valid_x + 1);
zmap_yy = (min_valid_y - 1):zmap_res:(max_valid_y + 1);
[Xzmap, Yzmap] = meshgrid(zmap_xx, zmap_yy);

Z_min_map = NaN(size(Xzmap));
Z_max_map = NaN(size(Xzmap));
search_r = zmap_res * 1.5;

for ri = 1:size(Xzmap, 1)
    for ci = 1:size(Xzmap, 2)
        px = Xzmap(ri, ci);
        py = Yzmap(ri, ci);
        near = abs(all_valid_xyz(:,1) - px) < search_r & ...
               abs(all_valid_xyz(:,2) - py) < search_r;
        if any(near)
            Z_min_map(ri, ci) = min(all_valid_xyz(near, 3));
            Z_max_map(ri, ci) = max(all_valid_xyz(near, 3));
        end
    end
end

valid_zmap = ~isnan(Z_min_map);
if sum(valid_zmap(:)) > 10
    if any(~valid_zmap(:))
        gm_z = valid_zmap;
        F_fill_zmin = scatteredInterpolant(Xzmap(gm_z), Yzmap(gm_z), Z_min_map(gm_z), 'nearest', 'nearest');
        F_fill_zmax = scatteredInterpolant(Xzmap(gm_z), Yzmap(gm_z), Z_max_map(gm_z), 'nearest', 'nearest');
        Z_min_map(~valid_zmap) = F_fill_zmin(Xzmap(~valid_zmap), Yzmap(~valid_zmap));
        Z_max_map(~valid_zmap) = F_fill_zmax(Xzmap(~valid_zmap), Yzmap(~valid_zmap));
    end
    F_zmin = griddedInterpolant({zmap_yy, zmap_xx}, Z_min_map, 'linear', 'nearest');
    F_zmax = griddedInterpolant({zmap_yy, zmap_xx}, Z_max_map, 'linear', 'nearest');
    fprintf('  Z height field built: %dx%d\n', length(zmap_xx), length(zmap_yy));
else
    F_zmin = []; F_zmax = [];
    fprintf('  [WARN] Z height field insufficient data\n');
end

%% ========== Step 5: Base Layer ==========
fprintf('\n[5] Base layer (offset=0)...\n');

surface_layers = {};
num_layers = 0;
global_activated = zeros(nelx, nely, nelz);

num_layers = num_layers + 1;
current_threshold = INITIAL_THRESHOLD;

[X_offset, Y_offset, Z_offset] = generate_offset_surface_v5(...
    0, min_valid_x, max_valid_x, min_valid_y, max_valid_y, ...
    SURFACE_RESOLUTION, compute_surface_z, ...
    NZ_FLOOR, DERIV_SMOOTH_SIGMA, MEDFILT_SIZE);

activated_grids = detect_activation_on_surface(...
    X_offset, Y_offset, Z_offset, ...
    grid_data, valid_grid_mask, current_threshold);

surface_layers{num_layers} = struct(...
    'offset', 0, 'direction', 'original', ...
    'attempts', struct('threshold', current_threshold, ...
                      'num_activated', length(activated_grids), ...
                      'num_newly_activated', length(activated_grids)), ...
    'final_grids', activated_grids, ...
    'final_threshold', current_threshold, ...
    'total_activated', length(activated_grids), ...
    'total_newly_activated', length(activated_grids), ...
    'X_surf', X_offset, 'Y_surf', Y_offset, 'Z_surf', Z_offset);

for g = 1:length(activated_grids)
    gi = activated_grids(g).grid_index;
    global_activated(gi(1), gi(2), gi(3)) = 1;
end

fprintf('  Base layer: activated %d (coverage %.1f%%)\n', ...
    length(activated_grids), 100*sum(global_activated(:))/total_valid_grids);

%% ========== Step 6: Offset Up ==========
fprintf('\n[6] Offset up...\n');
fprintf('  Target: %d upper grids\n\n', num_above);

current_offset = OFFSET_STEP;
current_threshold = INITIAL_THRESHOLD;
threshold_increase_count = 0;
consecutive_empty_offsets = 0;
cached_X = []; cached_Y = []; cached_Z = [];

while current_offset <= MAX_OFFSET
    fprintf('Offset %+.3f (thresh %.3f)... ', current_offset, current_threshold);
    
    if threshold_increase_count == 0
        [cached_X, cached_Y, cached_Z] = generate_offset_surface_v5(...
            current_offset, min_valid_x, max_valid_x, min_valid_y, max_valid_y, ...
            SURFACE_RESOLUTION, compute_surface_z, ...
            NZ_FLOOR, DERIV_SMOOTH_SIGMA, MEDFILT_SIZE);
    end
    
    activated_grids = detect_activation_on_surface(...
        cached_X, cached_Y, cached_Z, ...
        grid_data, valid_grid_mask, current_threshold);
    
    newly_activated = [];
    num_newly_above = 0;
    num_newly_below = 0;
    for g = 1:length(activated_grids)
        gi = activated_grids(g).grid_index;
        if global_activated(gi(1), gi(2), gi(3)) == 0
            newly_activated = [newly_activated, activated_grids(g)];
            global_activated(gi(1), gi(2), gi(3)) = 1;
            if above_surface_mask(gi(1), gi(2), gi(3))
                num_newly_above = num_newly_above + 1;
            else
                num_newly_below = num_newly_below + 1;
            end
        end
    end
    
    num_newly = length(newly_activated);
    above_activated = global_activated & above_surface_mask;
    remaining_above = num_above - sum(above_activated(:));
    
    fprintf('act %d, new %d (up+%d dn+%d) cum %.1f%%, remain %d\n', ...
        length(activated_grids), num_newly, num_newly_above, num_newly_below, ...
        100*sum(global_activated(:))/total_valid_grids, remaining_above);
    
    if threshold_increase_count == 0 && num_newly > 0
        num_layers = num_layers + 1;
        surface_layers{num_layers} = struct(...
            'offset', current_offset, 'direction', 'up', ...
            'attempts', struct('threshold', current_threshold, ...
                              'num_activated', length(activated_grids), ...
                              'num_newly_activated', num_newly), ...
            'final_grids', activated_grids, ...
            'final_threshold', current_threshold, ...
            'total_activated', length(activated_grids), ...
            'total_newly_activated', num_newly, ...
            'X_surf', cached_X, 'Y_surf', cached_Y, 'Z_surf', cached_Z);
    elseif threshold_increase_count > 0 && num_newly > 0
        new_attempt = struct('threshold', current_threshold, ...
                            'num_activated', length(activated_grids), ...
                            'num_newly_activated', num_newly);
        surface_layers{num_layers}.attempts(end+1) = new_attempt;
        surface_layers{num_layers}.final_grids = activated_grids;
        surface_layers{num_layers}.final_threshold = current_threshold;
        surface_layers{num_layers}.total_activated = length(activated_grids);
        surface_layers{num_layers}.total_newly_activated = ...
            surface_layers{num_layers}.total_newly_activated + num_newly;
    end
    
    if num_newly_above < NEW_ACTIVATION_THRESHOLD && remaining_above > 0
        if threshold_increase_count < MAX_THRESHOLD_RETRIES
            current_threshold = current_threshold + THRESHOLD_INCREMENT;
            threshold_increase_count = threshold_increase_count + 1;
        else
            fprintf('  [SKIP] offset %+.3f: %d retries exhausted\n', ...
                current_offset, MAX_THRESHOLD_RETRIES);
            consecutive_empty_offsets = consecutive_empty_offsets + 1;
            current_offset = current_offset + OFFSET_STEP;
            current_threshold = INITIAL_THRESHOLD;
            threshold_increase_count = 0;
        end
    else
        consecutive_empty_offsets = 0;
        current_offset = current_offset + OFFSET_STEP;
        current_threshold = INITIAL_THRESHOLD;
        threshold_increase_count = 0;
    end
    
    if remaining_above == 0
        fprintf('  All upper grids activated!\n'); break;
    end
    if consecutive_empty_offsets >= MAX_CONSECUTIVE_EMPTY
        fprintf('  [STOP] %d consecutive empty offsets\n', MAX_CONSECUTIVE_EMPTY);
        break;
    end
end

%% ========== Step 7: Offset Down ==========
fprintf('\n[7] Offset down...\n');
fprintf('  Target: %d lower grids\n\n', num_below);

current_offset = -OFFSET_STEP;
current_threshold = INITIAL_THRESHOLD;
threshold_increase_count = 0;
consecutive_empty_offsets = 0;
cached_X = []; cached_Y = []; cached_Z = [];

while abs(current_offset) <= MAX_OFFSET
    fprintf('Offset %+.3f (thresh %.3f)... ', current_offset, current_threshold);
    
    if threshold_increase_count == 0
        [cached_X, cached_Y, cached_Z] = generate_offset_surface_v5(...
            current_offset, min_valid_x, max_valid_x, min_valid_y, max_valid_y, ...
            SURFACE_RESOLUTION, compute_surface_z, ...
            NZ_FLOOR, DERIV_SMOOTH_SIGMA, MEDFILT_SIZE);
    end
    
    activated_grids = detect_activation_on_surface(...
        cached_X, cached_Y, cached_Z, ...
        grid_data, valid_grid_mask, current_threshold);
    
    newly_activated = [];
    num_newly_above = 0;
    num_newly_below = 0;
    for g = 1:length(activated_grids)
        gi = activated_grids(g).grid_index;
        if global_activated(gi(1), gi(2), gi(3)) == 0
            newly_activated = [newly_activated, activated_grids(g)];
            global_activated(gi(1), gi(2), gi(3)) = 1;
            if above_surface_mask(gi(1), gi(2), gi(3))
                num_newly_above = num_newly_above + 1;
            else
                num_newly_below = num_newly_below + 1;
            end
        end
    end
    
    num_newly = length(newly_activated);
    below_activated = global_activated & below_surface_mask;
    remaining_below = num_below - sum(below_activated(:));
    
    fprintf('act %d, new %d (up+%d dn+%d) cum %.1f%%, remain %d\n', ...
        length(activated_grids), num_newly, num_newly_above, num_newly_below, ...
        100*sum(global_activated(:))/total_valid_grids, remaining_below);
    
    if threshold_increase_count == 0 && num_newly > 0
        num_layers = num_layers + 1;
        surface_layers{num_layers} = struct(...
            'offset', current_offset, 'direction', 'down', ...
            'attempts', struct('threshold', current_threshold, ...
                              'num_activated', length(activated_grids), ...
                              'num_newly_activated', num_newly), ...
            'final_grids', activated_grids, ...
            'final_threshold', current_threshold, ...
            'total_activated', length(activated_grids), ...
            'total_newly_activated', num_newly, ...
            'X_surf', cached_X, 'Y_surf', cached_Y, 'Z_surf', cached_Z);
    elseif threshold_increase_count > 0 && num_newly > 0
        new_attempt = struct('threshold', current_threshold, ...
                            'num_activated', length(activated_grids), ...
                            'num_newly_activated', num_newly);
        surface_layers{num_layers}.attempts(end+1) = new_attempt;
        surface_layers{num_layers}.final_grids = activated_grids;
        surface_layers{num_layers}.final_threshold = current_threshold;
        surface_layers{num_layers}.total_activated = length(activated_grids);
        surface_layers{num_layers}.total_newly_activated = ...
            surface_layers{num_layers}.total_newly_activated + num_newly;
    end
    
    if num_newly_below < NEW_ACTIVATION_THRESHOLD && remaining_below > 0
        if threshold_increase_count < MAX_THRESHOLD_RETRIES
            current_threshold = current_threshold + THRESHOLD_INCREMENT;
            threshold_increase_count = threshold_increase_count + 1;
        else
            fprintf('  [SKIP] offset %+.3f: %d retries exhausted\n', ...
                current_offset, MAX_THRESHOLD_RETRIES);
            consecutive_empty_offsets = consecutive_empty_offsets + 1;
            current_offset = current_offset - OFFSET_STEP;
            current_threshold = INITIAL_THRESHOLD;
            threshold_increase_count = 0;
        end
    else
        consecutive_empty_offsets = 0;
        current_offset = current_offset - OFFSET_STEP;
        current_threshold = INITIAL_THRESHOLD;
        threshold_increase_count = 0;
    end
    
    if remaining_below == 0
        fprintf('  All lower grids activated!\n'); break;
    end
    if consecutive_empty_offsets >= MAX_CONSECUTIVE_EMPTY
        fprintf('  [STOP] %d consecutive empty offsets\n', MAX_CONSECUTIVE_EMPTY);
        break;
    end
end

%% ========== Step 8: Sort + Stats ==========
fprintf('\n[8] Sorting...\n');

offsets_all = zeros(num_layers, 1);
for i = 1:num_layers, offsets_all(i) = surface_layers{i}.offset; end
[~, sort_idx] = sort(offsets_all, 'ascend');

sorted_layers = {};
for new_id = 1:num_layers
    sorted_layers{new_id} = surface_layers{sort_idx(new_id)};
    sorted_layers{new_id}.layer_id = new_id;
end
surface_layers = sorted_layers;

final_coverage = sum(global_activated(:));
coverage_rate = 100 * final_coverage / total_valid_grids;

fprintf('  Layers: %d\n', num_layers);
fprintf('  Coverage: %.2f%% (%d/%d)\n', coverage_rate, final_coverage, total_valid_grids);

%% ========== Step 9: Save ==========
fprintf('\n[9] Saving...\n');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('slice_results_refined_%s.mat', timestamp);

slice_results = struct();
slice_results.surface_layers = surface_layers;
slice_results.grid_data = grid_data;
slice_results.valid_grid_mask = valid_grid_mask;
slice_results.global_activated = global_activated;
slice_results.above_surface_mask = above_surface_mask;
slice_results.below_surface_mask = below_surface_mask;
slice_results.grid_size = struct('nelx', nelx, 'nely', nely, 'nelz', nelz);
slice_results.statistics = struct(...
    'num_layers', num_layers, ...
    'total_valid_grids', total_valid_grids, ...
    'final_coverage', final_coverage, ...
    'coverage_rate', coverage_rate, ...
    'num_above', num_above, ...
    'num_below', num_below);
slice_results.parameters = struct(...
    'OFFSET_STEP', OFFSET_STEP, ...
    'INITIAL_THRESHOLD', INITIAL_THRESHOLD, ...
    'THRESHOLD_INCREMENT', THRESHOLD_INCREMENT, ...
    'SURFACE_RESOLUTION', SURFACE_RESOLUTION, ...
    'DENSITY_THRESHOLD', DENSITY_THRESHOLD, ...
    'NEW_ACTIVATION_THRESHOLD', NEW_ACTIVATION_THRESHOLD, ...
    'MAX_OFFSET', MAX_OFFSET, ...
    'SCALE_FACTOR', SCALE_FACTOR, ...
    'NZ_FLOOR', NZ_FLOOR, ...
    'DERIV_SMOOTH_SIGMA', DERIV_SMOOTH_SIGMA);
slice_results.surface_params = struct(...
    'a', a, 'b', b, 'c', c, 'd', d, ...
    'e', e, 'f', f, 'g', g, 'h', h, ...
    'X0', X0, 'Y0', Y0, 'Para_me', Para_me);
slice_results.metadata = struct('timestamp', timestamp, 'date', datestr(now), ...
    'version', 'v6_4x_domain_envelope_repair');

slice_results.z_height_field = struct(...
    'xx', zmap_xx, 'yy', zmap_yy, ...
    'Z_min_map', Z_min_map, 'Z_max_map', Z_max_map);

save(filename, 'slice_results', '-v7.3');
fprintf('  Saved: %s\n', filename);

latest_name = 'slice_results_refined_latest.mat';
if exist(latest_name, 'file')
    delete(latest_name);
    pause(0.5);
end
try
    [status, msg] = copyfile(filename, latest_name);
    if status
        fprintf('  Copied to: %s\n', latest_name);
    else
        fprintf('  [WARN] copyfile failed: %s, saving directly...\n', msg);
        save(latest_name, 'slice_results', '-v7.3');
    end
catch ME
    fprintf('  [WARN] copyfile error: %s, saving directly...\n', ME.message);
    save(latest_name, 'slice_results', '-v7.3');
end

fi = dir(filename);
fprintf('  Size: %.2f MB\n', fi.bytes/1024/1024);

fprintf('\n==============================================================\n');
fprintf('  Slicing complete! %d layers, %.1f%% coverage\n', ...
    num_layers, coverage_rate);
fprintf('  [v6] 4x domain + Z-projection + envelope repair + soft sigmoid\n');
fprintf('==============================================================\n\n');


%% ================================================================
%% Sub-functions
%% ================================================================

%% Clean offset surface generation v6
function [X_off, Y_off, Z_off] = generate_offset_surface_v5(...
    offset, min_x, max_x, min_y, max_y, resolution, ...
    compute_surface_z, nz_floor, deriv_sigma, medfilt_sz)
% Pure geometric offset for z=f(x,y) surfaces.
%
% Design philosophy (v6):
%   - Surface is computed over 4x the data domain: NO boundary artifacts
%   - NO validity mask: the surface is a pure mathematical object
%   - Z-projection only: Z_off = Z0 + offset * nz (always well-defined)
%   - Soft sigmoid nz clamping: prevents slope amplification blowup
%   - Envelope filtering: removes self-intersection cusps WITHOUT creating
%     false peaks (the v5b curvature-limiting approach created peaks)
%   - Laplacian smoothing: final cleanup

    % ================================================================
    % 4x domain coverage (eliminates ALL boundary artifacts)
    % ================================================================
    range_x = max_x - min_x;
    range_y = max_y - min_y;
    margin_x = range_x * 1.5;  % 1.5x on each side → total ~4x
    margin_y = range_y * 1.5;
    
    xx = (min_x - margin_x):resolution:(max_x + margin_x);
    yy = (min_y - margin_y):resolution:(max_y + margin_y);
    [X_off, Y_off] = meshgrid(xx, yy);
    [ny, nx] = size(X_off);
    
    % Base surface (covers entire extended domain via 'linear' extrap)
    Z0 = compute_surface_z(X_off, Y_off);
    
    if offset == 0
        Z_off = Z0;
        return;
    end
    
    % ================================================================
    % Adaptive derivative smoothing
    % ================================================================
    adaptive_k = 0.3;
    effective_sigma = deriv_sigma + adaptive_k * abs(offset) / resolution;
    max_sigma = min(nx, ny) / 6;
    effective_sigma = min(effective_sigma, max_sigma);
    
    delta = resolution * 5 + resolution * min(abs(offset) * 0.5, 20);
    
    % ================================================================
    % First derivatives (central difference, large step)
    % ================================================================
    Zxp = compute_surface_z(X_off + delta, Y_off);
    Zxm = compute_surface_z(X_off - delta, Y_off);
    Zyp = compute_surface_z(X_off, Y_off + delta);
    Zym = compute_surface_z(X_off, Y_off - delta);
    
    fx = (Zxp - Zxm) / (2 * delta);
    fy = (Zyp - Zym) / (2 * delta);
    
    nan_mask = isnan(fx) | isnan(fy) | isnan(Z0);
    fx(nan_mask) = 0;
    fy(nan_mask) = 0;
    
    % ================================================================
    % Gaussian smooth derivative fields
    % ================================================================
    if effective_sigma > 0
        r = ceil(effective_sigma * 2.5);
        [gx, gy] = meshgrid(-r:r, -r:r);
        kern = exp(-(gx.^2 + gy.^2) / (2 * effective_sigma^2));
        kern = kern / sum(kern(:));
        
        w = double(~nan_mask);
        fx = conv2(fx .* w, kern, 'same') ./ max(conv2(w, kern, 'same'), 1e-8);
        fy = conv2(fy .* w, kern, 'same') ./ max(conv2(w, kern, 'same'), 1e-8);
    end
    
    % ================================================================
    % Soft sigmoid nz clamping (C-inf smooth)
    % ================================================================
    nz = 1.0 ./ sqrt(fx.^2 + fy.^2 + 1.0);
    
    sharpness = 15.0;
    x_diff = sharpness * (nz - nz_floor);
    softplus_val = zeros(size(x_diff));
    big = x_diff > 20;  small = x_diff < -20;  mid = ~big & ~small;
    softplus_val(big) = x_diff(big);
    softplus_val(mid) = log(1 + exp(x_diff(mid)));
    nz_clamped = nz_floor + softplus_val / sharpness;
    
    % ================================================================
    % Z-projection offset (always well-defined, no re-gridding needed)
    % ================================================================
    Z_off = Z0 + offset * nz_clamped;
    
    % ================================================================
    % Self-intersection envelope repair (conservative)
    % ================================================================
    % Only remove genuine self-intersection cusps: narrow spikes that
    % point AGAINST the offset direction. Normal ridges/valleys are kept.
    %
    % Key insight: a cusp is much NARROWER than a normal ridge.
    % - Normal ridge: width ~ curvature radius, gradual Z variation
    % - Self-intersection cusp: width ~ few grid cells, sharp Z spike
    %
    % Strategy: moderate Gaussian smooth (captures ridges but kills cusps),
    % then only clip EXTREME wrong-direction deviations.
    
    % Sigma in grid cells: small enough to preserve ridges, large enough
    % to smooth over narrow cusps (typically 3-10 cells wide)
    envelope_sigma = max(5, min(abs(offset) * 0.5, 25));
    r_env = ceil(envelope_sigma * 2);
    [ex, ey] = meshgrid(-r_env:r_env, -r_env:r_env);
    env_kern = exp(-(ex.^2 + ey.^2) / (2 * envelope_sigma^2));
    env_kern = env_kern / sum(env_kern(:));
    Z_smooth_ref = conv2(Z_off, env_kern, 'same');
    
    % Generous tolerance: only clip spikes that exceed offset * 30%
    % Normal curvature variation is well within this range
    tol = abs(offset) * 0.3 + 1.0;
    
    if offset < 0
        spike_mask = Z_off > (Z_smooth_ref + tol);
        Z_off(spike_mask) = Z_smooth_ref(spike_mask) + tol;
    else
        spike_mask = Z_off < (Z_smooth_ref - tol);
        Z_off(spike_mask) = Z_smooth_ref(spike_mask) - tol;
    end
    
    % ================================================================
    % Median filter (adaptive size)
    % ================================================================
    effective_medfilt = medfilt_sz + 2 * floor(abs(offset) / 5);
    if mod(effective_medfilt, 2) == 0, effective_medfilt = effective_medfilt + 1; end
    effective_medfilt = min(effective_medfilt, 15);
    
    if effective_medfilt > 1
        Z_off = medfilt2(Z_off, [effective_medfilt effective_medfilt], 'symmetric');
    end
    
    % ================================================================
    % Laplacian diffusion (passes scale with |offset|)
    % ================================================================
    n_passes = min(2 + round(abs(offset) / 3), 20);
    lambda_s = 0.25;
    Z_anchor = Z_off;
    max_drift = abs(offset) * 0.05 + resolution * 3;
    
    for s = 1:n_passes
        Z_pad = Z_off;
        Z_pad(1,:) = Z_pad(2,:);    Z_pad(end,:) = Z_pad(end-1,:);
        Z_pad(:,1) = Z_pad(:,2);    Z_pad(:,end) = Z_pad(:,end-1);
        
        lap = Z_pad(1:end-2, 2:end-1) + Z_pad(3:end, 2:end-1) + ...
              Z_pad(2:end-1, 1:end-2) + Z_pad(2:end-1, 3:end) - ...
              4 * Z_pad(2:end-1, 2:end-1);
        
        Z_off(2:end-1, 2:end-1) = Z_off(2:end-1, 2:end-1) + lambda_s * lap;
        
        % Clamp drift
        drift = Z_off - Z_anchor;
        Z_off(drift > max_drift) = Z_anchor(drift > max_drift) + max_drift;
        Z_off(drift < -max_drift) = Z_anchor(drift < -max_drift) - max_drift;
    end
end

%% Surface activation detection
function activated_grids = detect_activation_on_surface(...
    X_s, Y_s, Z_s, grid_data, valid_grid_mask, grid_threshold)

    [nelx, nely, nelz] = size(valid_grid_mask);
    
    Xf = X_s(:); Yf = Y_s(:); Zf = Z_s(:);
    valid_pts = isfinite(Xf) & isfinite(Yf) & isfinite(Zf);
    Xv = Xf(valid_pts); Yv = Yf(valid_pts); Zv = Zf(valid_pts);
    n_pts = length(Xv);
    
    if n_pts < 10
        activated_grids = [];
        return;
    end
    
    bin_size = max(grid_threshold * 0.8, 0.3);
    x_min_s = min(Xv); y_min_s = min(Yv);
    
    bin_ix = max(1, floor((Xv - x_min_s) / bin_size) + 1);
    bin_iy = max(1, floor((Yv - y_min_s) / bin_size) + 1);
    nbx = max(bin_ix); nby = max(bin_iy);
    
    bins = cell(nbx, nby);
    for p = 1:n_pts
        bx = bin_ix(p); by = bin_iy(p);
        bins{bx, by}(end+1) = p;
    end
    
    search_r = ceil(grid_threshold / bin_size) + 1;
    activated_grids = [];
    
    for i = 1:nelx
        for j = 1:nely
            for k = 1:nelz
                if ~valid_grid_mask(i,j,k), continue; end
                
                gc_x = grid_data(i,j,k).x;
                gc_y = grid_data(i,j,k).y;
                gc_z = grid_data(i,j,k).z;
                
                bx0 = floor((gc_x - x_min_s) / bin_size) + 1;
                by0 = floor((gc_y - y_min_s) / bin_size) + 1;
                
                min_d = inf;
                is_act = false;
                
                for dbx = -search_r:search_r
                    for dby = -search_r:search_r
                        cx = bx0 + dbx; cy = by0 + dby;
                        if cx < 1 || cx > nbx || cy < 1 || cy > nby, continue; end
                        
                        for pidx = bins{cx, cy}
                            dd = sqrt((gc_x-Xv(pidx))^2 + (gc_y-Yv(pidx))^2 + (gc_z-Zv(pidx))^2);
                            if dd < min_d, min_d = dd; end
                            if dd <= grid_threshold, is_act = true; end
                        end
                    end
                end
                
                if is_act
                    gi = grid_data(i,j,k);
                    gi.grid_index = [i, j, k];
                    gi.distance = min_d;
                    activated_grids = [activated_grids, gi];
                end
            end
        end
    end
end