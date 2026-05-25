%% ========================================
%% Step 1: Voxel Refinement
%% Adapted for topo_stress_result.mat format
%% ========================================
%
% Input:  topo_stress_result.mat
%         (contains: nelx, nely, nelz, nele, xPhys, t_xoy, t_xoz, Iter)
%         xPhys: [nely, nelx, nelz]   -- density field
%         t_xoy: [nely, nelx, nelz]   -- azimuth angle (DEGREES)
%         t_xoz: [nely, nelx, nelz]   -- elevation angle (DEGREES)
%
% Output: voxel_refined_latest.mat
%
% Changes vs original:
%   - Loads topo_stress_result.mat instead of test.mat + test1.mat
%   - No SA/SB/.../SH surface parameters (surface generated later)
%   - Auto-detects angle units (degrees vs radians)
%%

fprintf('\n');
fprintf('========================================================\n');
fprintf('  Voxel Refinement - for topo_stress_result.mat\n');
fprintf('========================================================\n\n');

%% ========== Step 1: Check toolboxes ==========
fprintf('[Step 1] Checking toolboxes...\n');

v = ver;
hasParallelToolbox = any(strcmp({v.Name}, 'Parallel Computing Toolbox'));
hasImageToolbox = any(strcmp({v.Name}, 'Image Processing Toolbox'));

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

if hasImageToolbox
    fprintf('  Image Processing Toolbox: installed\n');
end

%% ========== Step 2: Load data ==========
fprintf('\n[Step 2] Loading data...\n');

% Try topo_stress_result.mat first, then fall back to test.mat
data_file = '';
if exist('topo_stress_result.mat', 'file')
    data_file = 'topo_stress_result.mat';
elseif exist('test.mat', 'file')
    data_file = 'test.mat';
end

if ~isempty(data_file) && (~exist('xPhys','var') || ~exist('t_xoy','var'))
    fprintf('  Loading %s...\n', data_file);
    load(data_file, 'nelx','nely','nelz','nele','xPhys','t_xoy','t_xoz');
else
    fprintf('  Using workspace variables\n');
end

% Verify required variables
required = {'nelx', 'nely', 'nelz', 'xPhys', 't_xoy', 't_xoz'};
missing = {};
for i = 1:length(required)
    if ~exist(required{i}, 'var')
        missing{end+1} = required{i};
    end
end
if ~isempty(missing)
    error('Missing variables: %s', strjoin(missing, ', '));
end

fprintf('  Grid: nelx=%d, nely=%d, nelz=%d\n', nelx, nely, nelz);
fprintf('  xPhys: [%s], range [%.4f, %.4f]\n', mat2str(size(xPhys)), min(xPhys(:)), max(xPhys(:)));
fprintf('  t_xoy: [%s], range [%.4f, %.4f]\n', mat2str(size(t_xoy)), min(t_xoy(:)), max(t_xoy(:)));
fprintf('  t_xoz: [%s], range [%.4f, %.4f]\n', mat2str(size(t_xoz)), min(t_xoz(:)), max(t_xoz(:)));

%% ========== Step 3: Parameters ==========
fprintf('\n[Step 3] Configuration...\n');

REFINE_FACTOR = 3;
INTERP_METHOD = 'linear';
USE_SMOOTHING = true;
SMOOTH_SIGMA = 0.5;
DENSITY_THRESHOLD = 0.5;

fprintf('  Refine factor: %d\n', REFINE_FACTOR);
fprintf('  Density threshold: %.2f\n', DENSITY_THRESHOLD);

%% ========== Step 4: Prepare grid data ==========
fprintf('\n[Step 4] Preparing grid data...\n');

nele = nelx * nely * nelz;

% xPhys is [nely, nelx, nelz] from step4_save_to_mat
% Convert to [nelx, nely, nelz] for internal use
fprintf('  Reordering xPhys [nely,nelx,nelz] -> [nelx,nely,nelz]...\n');
density_orig = permute(xPhys, [2, 1, 3]);  % [nely,nelx,nelz] -> [nelx,nely,nelz]

% --- Auto-detect angle units ---
fprintf('  Detecting angle units... ');
if max(abs(t_xoy(:))) > 6.3 || max(abs(t_xoz(:))) > 6.3
    fprintf('DEGREES\n');
    t_xoy_deg = t_xoy(:);
    t_xoz_deg = t_xoz(:);
else
    fprintf('RADIANS (converting to degrees)\n');
    t_xoy_deg = t_xoy(:) .* 180 ./ pi;
    t_xoz_deg = t_xoz(:) .* 180 ./ pi;
end

% Compute direction vectors (using cosd/sind which expect degrees)
xPhys_tem = xPhys(:);
xPhys_tem(xPhys_tem < DENSITY_THRESHOLD) = 0.0000001;

uu_array = cosd(t_xoz_deg) .* cosd(t_xoy_deg) .* xPhys_tem;
vv_array = cosd(t_xoz_deg) .* sind(t_xoy_deg) .* xPhys_tem;
ww_array = -sind(t_xoz_deg) .* xPhys_tem;

fprintf('  Direction vectors computed (uu/vv/ww)\n');

% Reshape to [nelx, nely, nelz]
% Source arrays are linear in [nely, nelx, nelz] order (MATLAB column-major)
% so we reshape to [nely, nelx, nelz] first, then permute
uu_nyx = reshape(uu_array, [nely, nelx, nelz]);
vv_nyx = reshape(vv_array, [nely, nelx, nelz]);
ww_nyx = reshape(ww_array, [nely, nelx, nelz]);
t_xoy_nyx = reshape(t_xoy_deg, [nely, nelx, nelz]);
t_xoz_nyx = reshape(t_xoz_deg, [nely, nelx, nelz]);

% Permute to [nelx, nely, nelz]
uu_orig    = permute(uu_nyx, [2, 1, 3]);
vv_orig    = permute(vv_nyx, [2, 1, 3]);
ww_orig    = permute(ww_nyx, [2, 1, 3]);
t_xoy_orig = permute(t_xoy_nyx, [2, 1, 3]);
t_xoz_orig = permute(t_xoz_nyx, [2, 1, 3]);

% Valid mask
valid_grid_mask = (density_orig > DENSITY_THRESHOLD);
num_valid_original = sum(valid_grid_mask(:));

fprintf('  Valid voxels: %d / %d (%.1f%%)\n', ...
    num_valid_original, numel(density_orig), 100*num_valid_original/numel(density_orig));

%% ========== Step 5: Voxel refinement (interpolation) ==========
fprintf('\n[Step 5] Voxel refinement...\n');

nelx_fine = nelx * REFINE_FACTOR;
nely_fine = nely * REFINE_FACTOR;
nelz_fine = nelz * REFINE_FACTOR;

fprintf('  Target: %d x %d x %d\n', nelx_fine, nely_fine, nelz_fine);

% Original grid coordinates
[Y_orig, X_orig, Z_orig] = meshgrid(1:nely, 1:nelx, 1:nelz);
X_orig = X_orig - 0.5;
Y_orig = Y_orig - 0.5;
Z_orig = Z_orig - 0.5;

% Fine grid coordinates
sub_step = 1.0 / REFINE_FACTOR;
% 修正后：起点为 sub_step/2，对齐原始采样的 [0.5, nelx-0.5] 区间
x_fine = (sub_step/2) : sub_step : (nelx - sub_step/2);
y_fine = (sub_step/2) : sub_step : (nely - sub_step/2);
z_fine = (sub_step/2) : sub_step : (nelz - sub_step/2);
% 后面 x_fine = x_fine(1:nelx_fine); 这三行就可以删掉了（长度已对）
x_fine = x_fine(1:nelx_fine);
y_fine = y_fine(1:nely_fine);
z_fine = z_fine(1:nelz_fine);

[Y_fine, X_fine, Z_fine] = meshgrid(y_fine, x_fine, z_fine);

% Interpolate all fields
fprintf('  Interpolating fields...\n');
tic;
density_fine = interp3(Y_orig, X_orig, Z_orig, density_orig, Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
uu_fine      = interp3(Y_orig, X_orig, Z_orig, uu_orig,      Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
vv_fine      = interp3(Y_orig, X_orig, Z_orig, vv_orig,      Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
ww_fine      = interp3(Y_orig, X_orig, Z_orig, ww_orig,      Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
t_xoy_fine   = interp3(Y_orig, X_orig, Z_orig, t_xoy_orig,   Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
t_xoz_fine   = interp3(Y_orig, X_orig, Z_orig, t_xoz_orig,   Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
fprintf('  Done (%.2fs)\n', toc);

%% ========== Step 6: Smoothing ==========
if USE_SMOOTHING && hasImageToolbox
    fprintf('\n[Step 6] Gaussian smoothing (sigma=%.2f)...\n', SMOOTH_SIGMA);
    tic;
    density_fine = imgaussfilt3(density_fine, SMOOTH_SIGMA);
    uu_fine = imgaussfilt3(uu_fine, SMOOTH_SIGMA);
    vv_fine = imgaussfilt3(vv_fine, SMOOTH_SIGMA);
    ww_fine = imgaussfilt3(ww_fine, SMOOTH_SIGMA);
    fprintf('  Done (%.2fs)\n', toc);
else
    fprintf('\n[Step 6] Skipping smoothing\n');
end

%% ========== Step 7: Build grid_data structure ==========
fprintf('\n[Step 7] Building grid_data structure...\n');

grid_data_fine = struct();
valid_grid_mask_fine = zeros(nelx_fine, nely_fine, nelz_fine);
num_valid_fine = 0;

tic;
for i = 1:nelx_fine
    for j = 1:nely_fine
        for k = 1:nelz_fine
            grid_data_fine(i,j,k).x = x_fine(i);
            grid_data_fine(i,j,k).y = y_fine(j);
            grid_data_fine(i,j,k).z = z_fine(k);
            grid_data_fine(i,j,k).xPhys = density_fine(i,j,k);
            grid_data_fine(i,j,k).t_xoy = t_xoy_fine(i,j,k);
            grid_data_fine(i,j,k).t_xoz = t_xoz_fine(i,j,k);
            grid_data_fine(i,j,k).uu = uu_fine(i,j,k);
            grid_data_fine(i,j,k).vv = vv_fine(i,j,k);
            grid_data_fine(i,j,k).ww = ww_fine(i,j,k);
            grid_data_fine(i,j,k).is_valid = (density_fine(i,j,k) > DENSITY_THRESHOLD);

            if grid_data_fine(i,j,k).is_valid
                valid_grid_mask_fine(i,j,k) = 1;
                num_valid_fine = num_valid_fine + 1;
            end
        end
    end
end
fprintf('  Done (%.2fs)\n', toc);
fprintf('  Valid voxels: %d (%.1f%%)\n', num_valid_fine, ...
    100*num_valid_fine/(nelx_fine*nely_fine*nelz_fine));

%% ========== Step 8: Save ==========
fprintf('\n[Step 8] Saving...\n');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('voxel_refined_%s.mat', timestamp);

refined_data = struct();
refined_data.grid_data = grid_data_fine;
refined_data.valid_grid_mask = valid_grid_mask_fine;
refined_data.grid_size = struct('nelx', nelx_fine, 'nely', nely_fine, 'nelz', nelz_fine);
refined_data.parameters = struct(...
    'REFINE_FACTOR', REFINE_FACTOR, ...
    'INTERP_METHOD', INTERP_METHOD, ...
    'USE_SMOOTHING', USE_SMOOTHING, ...
    'SMOOTH_SIGMA', SMOOTH_SIGMA, ...
    'DENSITY_THRESHOLD', DENSITY_THRESHOLD, ...
    'SCALE_FACTOR', 1.0 / REFINE_FACTOR, ...
    'angle_unit', 'degrees');
refined_data.original_size = struct('nelx', nelx, 'nely', nely, 'nelz', nelz);
refined_data.statistics = struct(...
    'num_valid_original', num_valid_original, ...
    'num_valid_fine', num_valid_fine);
% Dummy surface params for backward compatibility with slice script
% (slice_refined_model_complete.m loads sp.a/b/c... even though it uses Pre_surface.mat)
% generate_reference_surface.m will overwrite these with proper values
refined_data.surface_params = struct(...
    'a', 0, 'b', 0, 'c', 0, 'd', 0, ...
    'e', 0, 'f', 0, 'g', 0, 'h', 0, ...
    'X0', 0, 'Y0', 0, 'Para_me', 0, ...
    'type', 'placeholder');
refined_data.metadata = struct('timestamp', timestamp, 'date', datestr(now), ...
    'source_file', data_file);

save(filename, 'refined_data', '-v7.3');
copyfile(filename, 'voxel_refined_latest.mat');

file_info = dir(filename);
fprintf('  Saved: %s (%.2f MB)\n', filename, file_info.bytes/1024/1024);
fprintf('  Copy:  voxel_refined_latest.mat\n');

%% ========== Step 9: Quick verification ==========
fprintf('\n[Step 9] Verification...\n');

% Check a few voxels: recompute direction from stored angles and compare
n_check = min(20, num_valid_fine);
check_count = 0;
max_err = 0;

for i = 1:nelx_fine
    for j = 1:nely_fine
        for k = 1:nelz_fine
            if valid_grid_mask_fine(i,j,k) && check_count < n_check
                g = grid_data_fine(i,j,k);
                % Recompute from angles
                az = g.t_xoy;  % degrees
                el = g.t_xoz;  % degrees
                uu_check = cosd(el)*cosd(az) * g.xPhys;
                vv_check = cosd(el)*sind(az) * g.xPhys;
                ww_check = -sind(el) * g.xPhys;
                
                err = max([abs(g.uu-uu_check), abs(g.vv-vv_check), abs(g.ww-ww_check)]);
                max_err = max(max_err, err);
                check_count = check_count + 1;
            end
        end
    end
end

fprintf('  Direction vector consistency check (%d samples): max_err = %.8f\n', check_count, max_err);
if max_err < 0.01
    fprintf('  PASS - directions are consistent with angles\n');
else
    fprintf('  FAIL - directions do NOT match angles!\n');
end

%% ========== Done ==========
fprintf('\n');
fprintf('========================================================\n');
fprintf('  Voxel Refinement Complete!\n');
fprintf('========================================================\n\n');
fprintf('Next steps:\n');
fprintf('  >> run(''generate_reference_surface.m'')  %% Step 2.5: generate surface\n');
fprintf('  >> run(''slice_refined_model_complete.m'') %% Step 3: slice\n');
fprintf('\n');