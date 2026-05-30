%% ========================================
%% Step 1: Voxel Refinement  (v3 - 修复加载 + 角度自洽)
%% Adapted for topo_stress_result.mat format
%% ========================================
%
% v3 修复:
%   [FIX-1] 致命: 不再用 workspace 残留变量, 无条件从 DATA_FILE 加载.
%   [FIX-2] 平滑后从方向向量反算角度并重建 uu/vv/ww, 保证自洽 (修验证 FAIL).
%
% ============ 尺寸与分辨率的两个独立旋钮 ============
% 物理量纲关系:
%   物理尺寸        = nelx × ELEM_SIZE                (mm)
%   应力场采样点数   = nelx × REFINE_FACTOR             (个)
%   细网格物理步长   = ELEM_SIZE / REFINE_FACTOR        (mm)
%   切片层高        = SCALE_FACTOR (写入 refined_data, 被 slice_refined_model_v6 读取)
%   线宽            = all_layers_path_generation_v6 的 params.offset_distance (物理 mm, 已天然解耦)
%
% 控制参数 (见 Step 3):
%   ELEM_SIZE      : 体素物理边长(mm). 控制物理尺寸. 放大就调大.
%   REFINE_FACTOR  : 应力场采样加密倍数. 不改尺寸, 只增加采样密度.
%   LAYER_HEIGHT_MODE + TARGET_LAYER_HEIGHT : 控制切片层高
%       'bind'    : SCALE_FACTOR = ELEM_SIZE/REFINE_FACTOR (随放大变化, 旧行为)
%       'decouple': SCALE_FACTOR = TARGET_LAYER_HEIGHT     (物理固定, 推荐)
%%

fprintf('\n');
fprintf('========================================================\n');
fprintf(' Voxel Refinement v3 - 修复加载 + 角度自洽\n');
fprintf('========================================================\n\n');

%% ========== 文件路径配置 ==========
% [FIX-1] 明确指定输入文件. 默认读当前工作目录的 topo_stress_result.mat.
%         如需固定某路径, 直接写完整路径, 例如:
%         DATA_FILE = 'E:\308\傅里叶\拓扑结构\print_test\topo_stress_result.mat';
DATA_FILE = 'topo_stress_result.mat';

%% ========== Step 1: Check toolboxes ==========
fprintf('[Step 1] Checking toolboxes...\n');
v = ver;
hasParallelToolbox = any(strcmp({v.Name}, 'Parallel Computing Toolbox'));
hasImageToolbox = any(strcmp({v.Name}, 'Image Processing Toolbox'));
if hasParallelToolbox
    fprintf(' Parallel Computing Toolbox: installed\n');
    p = gcp('nocreate');
    if isempty(p)
        try
            parpool('local'); p = gcp('nocreate');
            fprintf(' Workers: %d\n', p.NumWorkers);
        catch
            fprintf(' Warning: parallel pool failed, using serial\n');
            hasParallelToolbox = false;
        end
    else
        fprintf(' Workers: %d\n', p.NumWorkers);
    end
else
    fprintf(' Parallel Computing Toolbox: not installed (serial mode)\n');
end
if hasImageToolbox
    fprintf(' Image Processing Toolbox: installed\n');
end

%% ========== Step 2: Load data ==========
% [FIX-1] 无条件从文件加载. 用 S=load(...) 从结构体取字段,
%         完全不依赖 workspace 残留变量, 也不污染 workspace.
fprintf('\n[Step 2] Loading data...\n');
if ~exist(DATA_FILE, 'file')
    error(['找不到文件: %s\n' ...
           '当前工作目录: %s\n' ...
           '请 cd 到包含该文件的目录, 或把脚本顶部 DATA_FILE 改成完整路径.'], ...
           DATA_FILE, pwd);
end
fprintf(' Loading: %s\n', DATA_FILE);
fprintf(' Working dir: %s\n', pwd);
S = load(DATA_FILE, 'nelx', 'nely', 'nelz', 'xPhys', 't_xoy', 't_xoz');
req = {'nelx','nely','nelz','xPhys','t_xoy','t_xoz'};
for ii = 1:numel(req)
    if ~isfield(S, req{ii})
        error('%s 缺少必需字段: %s', DATA_FILE, req{ii});
    end
end
nelx  = S.nelx;  nely  = S.nely;  nelz  = S.nelz;
xPhys = S.xPhys; t_xoy = S.t_xoy; t_xoz = S.t_xoz;
clear S;
fprintf(' Grid: nelx=%d, nely=%d, nelz=%d\n', nelx, nely, nelz);
fprintf(' xPhys: [%s], range [%.4f, %.4f]\n', mat2str(size(xPhys)), min(xPhys(:)), max(xPhys(:)));
fprintf(' t_xoy: [%s], range [%.4f, %.4f]\n', mat2str(size(t_xoy)), min(t_xoy(:)), max(t_xoy(:)));
fprintf(' t_xoz: [%s], range [%.4f, %.4f]\n', mat2str(size(t_xoz)), min(t_xoz(:)), max(t_xoz(:)));

%% ========== Step 3: Parameters ==========
fprintf('\n[Step 3] Configuration...\n');

% ============ 尺寸与分辨率控制 ============
ELEM_SIZE           = 3.0;        % 体素物理边长(mm). 放大就调大. 原始=1.0
REFINE_FACTOR       = 3;          % 应力场采样加密倍数 (不影响物理尺寸)
LAYER_HEIGHT_MODE   = 'decouple'; % 'bind' 或 'decouple'(推荐)
TARGET_LAYER_HEIGHT = 0.2;        % mm, 仅 decouple 模式用
% ==========================================

% --- 入参 sanity check (防止参数写错时崩在下游远处难定位) ---
assert(isnumeric(ELEM_SIZE) && isscalar(ELEM_SIZE) && ELEM_SIZE > 0, ...
    'ELEM_SIZE 必须是正实数, 当前: %s', mat2str(ELEM_SIZE));
assert(isnumeric(REFINE_FACTOR) && isscalar(REFINE_FACTOR) && REFINE_FACTOR >= 1 ...
    && REFINE_FACTOR == round(REFINE_FACTOR), ...
    'REFINE_FACTOR 必须是 >=1 的正整数, 当前: %s', mat2str(REFINE_FACTOR));
assert(any(strcmpi(LAYER_HEIGHT_MODE, {'bind','decouple'})), ...
    'LAYER_HEIGHT_MODE 必须是 ''bind'' 或 ''decouple'', 当前: %s', LAYER_HEIGHT_MODE);
if strcmpi(LAYER_HEIGHT_MODE, 'decouple')
    assert(isnumeric(TARGET_LAYER_HEIGHT) && TARGET_LAYER_HEIGHT > 0, ...
        'decouple 模式下 TARGET_LAYER_HEIGHT 必须是正数, 当前: %s', mat2str(TARGET_LAYER_HEIGHT));
end

INTERP_METHOD = 'linear';
USE_SMOOTHING = true;
SMOOTH_SIGMA  = 0.5;
DENSITY_THRESHOLD = 0.5;

if strcmpi(LAYER_HEIGHT_MODE, 'decouple')
    SCALE_FACTOR_OUT = TARGET_LAYER_HEIGHT;
else
    SCALE_FACTOR_OUT = ELEM_SIZE / REFINE_FACTOR;
end

phys_size_x = nelx * ELEM_SIZE;
phys_size_y = nely * ELEM_SIZE;
phys_size_z = nelz * ELEM_SIZE;
fine_step   = ELEM_SIZE / REFINE_FACTOR;
fprintf(' ---- 尺寸/分辨率 ----\n');
fprintf('   ELEM_SIZE          = %.4f mm\n', ELEM_SIZE);
fprintf('   REFINE_FACTOR      = %d\n', REFINE_FACTOR);
fprintf('   物理尺寸 (X×Y×Z)   = %.1f × %.1f × %.1f mm\n', phys_size_x, phys_size_y, phys_size_z);
fprintf('   应力场细网格步长    = %.4f mm\n', fine_step);
fprintf('   层高模式            = %s\n', LAYER_HEIGHT_MODE);
fprintf('   → 切片层高(SCALE_FACTOR) = %.4f mm\n', SCALE_FACTOR_OUT);
fprintf('   (估计层数 ≈ 物理Z高度/层高 = %.0f)\n', phys_size_z / SCALE_FACTOR_OUT);
fprintf(' --------------------\n');
fprintf(' Density threshold: %.2f\n', DENSITY_THRESHOLD);

%% ========== Step 4: Prepare grid data ==========
fprintf('\n[Step 4] Preparing grid data...\n');
density_orig = permute(xPhys, [2, 1, 3]);

fprintf(' Detecting angle units... ');
if max(abs(t_xoy(:))) > 6.3 || max(abs(t_xoz(:))) > 6.3
    fprintf('DEGREES\n');
    t_xoy_deg = t_xoy(:);
    t_xoz_deg = t_xoz(:);
else
    fprintf('RADIANS (converting to degrees)\n');
    t_xoy_deg = t_xoy(:) .* 180 ./ pi;
    t_xoz_deg = t_xoz(:) .* 180 ./ pi;
end

xPhys_tem = xPhys(:);
xPhys_tem(xPhys_tem < DENSITY_THRESHOLD) = 0.0000001;
uu_array = cosd(t_xoz_deg) .* cosd(t_xoy_deg) .* xPhys_tem;
vv_array = cosd(t_xoz_deg) .* sind(t_xoy_deg) .* xPhys_tem;
ww_array = -sind(t_xoz_deg) .* xPhys_tem;
fprintf(' Direction vectors computed (uu/vv/ww)\n');

uu_nyx = reshape(uu_array, [nely, nelx, nelz]);
vv_nyx = reshape(vv_array, [nely, nelx, nelz]);
ww_nyx = reshape(ww_array, [nely, nelx, nelz]);

uu_orig = permute(uu_nyx, [2, 1, 3]);
vv_orig = permute(vv_nyx, [2, 1, 3]);
ww_orig = permute(ww_nyx, [2, 1, 3]);

valid_grid_mask = (density_orig > DENSITY_THRESHOLD);
num_valid_original = sum(valid_grid_mask(:));
fprintf(' Valid voxels: %d / %d (%.1f%%)\n', ...
    num_valid_original, numel(density_orig), 100*num_valid_original/numel(density_orig));

%% ========== Step 5: Voxel refinement (interpolation) ==========
% 插值在【体素单位】下做 (x_fine 范围 [0, nelx]); 物理放大推迟到 Step 7.
fprintf('\n[Step 5] Voxel refinement...\n');
nelx_fine = nelx * REFINE_FACTOR;
nely_fine = nely * REFINE_FACTOR;
nelz_fine = nelz * REFINE_FACTOR;
fprintf(' Target fine grid: %d x %d x %d\n', nelx_fine, nely_fine, nelz_fine);

[Y_orig, X_orig, Z_orig] = meshgrid(1:nely, 1:nelx, 1:nelz);
X_orig = X_orig - 0.5; Y_orig = Y_orig - 0.5; Z_orig = Z_orig - 0.5;

sub_step = 1.0 / REFINE_FACTOR;
x_fine = (sub_step/2) : sub_step : (nelx - sub_step/2);
y_fine = (sub_step/2) : sub_step : (nely - sub_step/2);
z_fine = (sub_step/2) : sub_step : (nelz - sub_step/2);
x_fine = x_fine(1:nelx_fine);
y_fine = y_fine(1:nely_fine);
z_fine = z_fine(1:nelz_fine);

[Y_fine, X_fine, Z_fine] = meshgrid(y_fine, x_fine, z_fine);

fprintf(' Interpolating fields...\n');
tic;
density_fine = interp3(Y_orig, X_orig, Z_orig, density_orig, Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
uu_fine = interp3(Y_orig, X_orig, Z_orig, uu_orig, Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
vv_fine = interp3(Y_orig, X_orig, Z_orig, vv_orig, Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
ww_fine = interp3(Y_orig, X_orig, Z_orig, ww_orig, Y_fine, X_fine, Z_fine, INTERP_METHOD, 0);
fprintf(' Done (%.2fs)\n', toc);

%% ========== Step 6: Smoothing + 角度自洽重建 ==========
% [FIX-2] 平滑方向向量后, 从向量反算角度, 再用角度重建 uu/vv/ww.
%         保证 uu/vv/ww 与 t_xoy/t_xoz 严格自洽 (修验证 FAIL).
%         角度不能直接平滑(±180°跳变), 必须先平滑向量再 atan2 反算.
if USE_SMOOTHING && hasImageToolbox
    fprintf('\n[Step 6] Gaussian smoothing (sigma=%.2f) + 角度重建...\n', SMOOTH_SIGMA);
    tic;
    density_fine = imgaussfilt3(density_fine, SMOOTH_SIGMA);
    uu_fine = imgaussfilt3(uu_fine, SMOOTH_SIGMA);
    vv_fine = imgaussfilt3(vv_fine, SMOOTH_SIGMA);
    ww_fine = imgaussfilt3(ww_fine, SMOOTH_SIGMA);
    fprintf(' Smoothing done (%.2fs)\n', toc);
else
    fprintf('\n[Step 6] No smoothing, 仅做角度自洽重建...\n');
end

% --- 从(平滑后的)方向向量反算角度 ---
% uu = cos(el)cos(az)*xPhys, vv = cos(el)sin(az)*xPhys, ww = -sin(el)*xPhys
% => az = atan2(vv, uu);  el = atan2(-ww, sqrt(uu^2+vv^2))   (xPhys 正因子不影响)
t_xoy_fine = atan2d(vv_fine, uu_fine);
t_xoz_fine = atan2d(-ww_fine, sqrt(uu_fine.^2 + vv_fine.^2));

% 低密度体素方向不可靠, 角度归零
low_density = density_fine < DENSITY_THRESHOLD;
t_xoy_fine(low_density) = 0;
t_xoz_fine(low_density) = 0;

% --- 用重建角度 + 密度, 重新生成自洽的 uu/vv/ww ---
xp = density_fine;
xp(xp < DENSITY_THRESHOLD) = 1e-7;
uu_fine = cosd(t_xoz_fine) .* cosd(t_xoy_fine) .* xp;
vv_fine = cosd(t_xoz_fine) .* sind(t_xoy_fine) .* xp;
ww_fine = -sind(t_xoz_fine) .* xp;
fprintf(' 角度自洽重建完成 (uu/vv/ww <-> t_xoy/t_xoz)\n');

%% ========== Step 7: Build grid_data structure ==========
% 保存时把网格坐标 × ELEM_SIZE → 物理坐标 (实现放大)
fprintf('\n[Step 7] Building grid_data structure...\n');
fprintf(' Applying physical scale: coord × ELEM_SIZE (%.4f)\n', ELEM_SIZE);
grid_data_fine = struct();
valid_grid_mask_fine = zeros(nelx_fine, nely_fine, nelz_fine);
num_valid_fine = 0;
x_fine_phys = x_fine * ELEM_SIZE;
y_fine_phys = y_fine * ELEM_SIZE;
z_fine_phys = z_fine * ELEM_SIZE;
tic;
for i = 1:nelx_fine
    for j = 1:nely_fine
        for k = 1:nelz_fine
            grid_data_fine(i,j,k).x = x_fine_phys(i);
            grid_data_fine(i,j,k).y = y_fine_phys(j);
            grid_data_fine(i,j,k).z = z_fine_phys(k);
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
fprintf(' Done (%.2fs)\n', toc);
fprintf(' Valid voxels: %d (%.1f%%)\n', num_valid_fine, ...
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
    'SCALE_FACTOR', SCALE_FACTOR_OUT, ...
    'ELEM_SIZE', ELEM_SIZE, ...
    'LAYER_HEIGHT_MODE', LAYER_HEIGHT_MODE, ...
    'TARGET_LAYER_HEIGHT', TARGET_LAYER_HEIGHT,...
    'angle_unit', 'degrees');
refined_data.original_size = struct('nelx', nelx, 'nely', nely, 'nelz', nelz);
refined_data.statistics = struct(...
    'num_valid_original', num_valid_original, ...
    'num_valid_fine', num_valid_fine);
refined_data.surface_params = struct(...
    'a', 0, 'b', 0, 'c', 0, 'd', 0, ...
    'e', 0, 'f', 0, 'g', 0, 'h', 0, ...
    'X0', 0, 'Y0', 0, 'Para_me', 0, ...
    'type', 'placeholder');
refined_data.metadata = struct('timestamp', timestamp, 'date', datestr(now), ...
    'source_file', DATA_FILE);

save(filename, 'refined_data', '-v7.3');
copyfile(filename, 'voxel_refined_latest.mat');
file_info = dir(filename);
fprintf(' Saved: %s (%.2f MB)\n', filename, file_info.bytes/1024/1024);
fprintf(' Copy: voxel_refined_latest.mat\n');

%% ========== Step 9: Quick verification ==========
fprintf('\n[Step 9] Verification...\n');
n_check = min(20, num_valid_fine);
check_count = 0;
max_err = 0;
for i = 1:nelx_fine
    for j = 1:nely_fine
        for k = 1:nelz_fine
            if valid_grid_mask_fine(i,j,k) && check_count < n_check
                g = grid_data_fine(i,j,k);
                az = g.t_xoy; el = g.t_xoz;
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
fprintf(' Direction consistency (%d samples): max_err = %.8f\n', check_count, max_err);
if max_err < 0.01
    fprintf(' PASS - directions consistent with angles\n');
else
    fprintf(' FAIL - directions do NOT match angles!\n');
end

%% ========== Done ==========
fprintf('\n');
fprintf('========================================================\n');
fprintf(' Voxel Refinement v3 Complete!\n');
fprintf('   物理尺寸 = %.1f × %.1f × %.1f mm,  层高 = %.3f mm\n', ...
    phys_size_x, phys_size_y, phys_size_z, SCALE_FACTOR_OUT);
fprintf('   源文件 = %s (grid %dx%dx%d)\n', DATA_FILE, nelx, nely, nelz);
fprintf('========================================================\n\n');
fprintf('Next steps:\n');
fprintf(' >> run(''generate_reference_surface.m'')\n');
fprintf(' >> slice_refined_model_v6()\n');
fprintf('\n');