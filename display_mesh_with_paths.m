%% display_mesh_with_paths.m
%% ======================================================================
%% 同时展示拓扑优化体素网格 + 全层纤维路径
%%
%% 坐标对齐策略：
%%   通过路径数据和体素网格的边界框自动对齐，
%%   不依赖 grid_data 内部字段结构。
%%
%% Usage:
%%   1. 修改 data_dir 和 path_mat 路径
%%   2. 直接运行
%% ======================================================================

clear; clc; close all;

fprintf('====================================================\n');
fprintf('  3D Mesh + Fiber Paths Visualization\n');
fprintf('====================================================\n\n');

%% ========== 配置 ==========
data_dir = 'C:\temp\abaqus_topo';
path_mat = 'all_layers_path_results_v3.mat';

%% ========== 加载网格 ==========
fprintf('[1] Loading mesh density field...\n');
params_file = fullfile(data_dir, 'mesh_params.txt');
fid = fopen(params_file, 'r');
mesh_params = struct();
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line) || line(1) == '#', continue; end
    parts = strsplit(line);
    if length(parts) >= 2
        mesh_params.(parts{1}) = str2double(parts{2});
    end
end
fclose(fid);

nelx = mesh_params.nelx;
nely = mesh_params.nely;
nelz = mesh_params.nelz;

density_file = fullfile(data_dir, 'xPhys_full.txt');
fid = fopen(density_file, 'r');
fgetl(fid);
xPhys_vec = fscanf(fid, '%f');
fclose(fid);
xPhys = reshape(xPhys_vec, [nely, nelx, nelz]);

fprintf('  Grid: nelx=%d, nely=%d, nelz=%d\n', nelx, nely, nelz);
fprintf('  Density range: [%.4f, %.4f]\n', min(xPhys(:)), max(xPhys(:)));

%% ========== 加载路径 ==========
fprintf('[2] Loading path data...\n');
if ~exist(path_mat, 'file')
    error('Path file not found: %s', path_mat);
end
tmp = load(path_mat);
results = tmp.results;
all_layers_data = results.all_layers_data;
num_layers = results.num_layers;

% 收集全部三维路径点，计算边界框
all_pts = [];
total_paths = 0;
for li = 1:num_layers
    if ~all_layers_data(li).success, continue; end
    for pi = 1:length(all_layers_data(li).paths_3d)
        pts = all_layers_data(li).paths_3d{pi};
        if ~isempty(pts) && size(pts, 1) >= 2
            all_pts = [all_pts; pts];
            total_paths = total_paths + 1;
        end
    end
end
fprintf('  Layers: %d, Total 3D paths: %d, Total points: %d\n', ...
    num_layers, total_paths, size(all_pts, 1));

%% ========== 坐标对齐 ==========
fprintf('[3] Computing coordinate alignment...\n');

% 体素显示空间边界框
% display coords: Xd = (i-1), Yd = (k-1), Zd = (j-1)
% 即 Xd in [0, nelx-1], Yd in [0, nelz-1], Zd in [0, nely-1]
vox_x_range = [0, nelx - 1];
vox_y_range = [0, nelz - 1];  % height = Z_phys
vox_z_range = [0, nely - 1];  % depth  = Y_phys

% 路径空间边界框
path_x_min = min(all_pts(:,1)); path_x_max = max(all_pts(:,1));
path_y_min = min(all_pts(:,2)); path_y_max = max(all_pts(:,2));
path_z_min = min(all_pts(:,3)); path_z_max = max(all_pts(:,3));

fprintf('  Path bbox: X=[%.2f, %.2f], Y=[%.2f, %.2f], Z=[%.2f, %.2f]\n', ...
    path_x_min, path_x_max, path_y_min, path_y_max, path_z_min, path_z_max);
fprintf('  Voxel display bbox: Xd=[%d, %d], Yd=[%d, %d], Zd=[%d, %d]\n', ...
    vox_x_range(1), vox_x_range(2), vox_y_range(1), vox_y_range(2), ...
    vox_z_range(1), vox_z_range(2));

% 线性映射：path coords -> voxel display coords
% path X -> display X
% path Y -> display Z (depth)
% path Z -> display Y (height)
sx = diff(vox_x_range) / max(path_x_max - path_x_min, 1e-6);
sy = diff(vox_z_range) / max(path_y_max - path_y_min, 1e-6);  % path_Y -> disp_Z
sz = diff(vox_y_range) / max(path_z_max - path_z_min, 1e-6);  % path_Z -> disp_Y

ox = vox_x_range(1) - path_x_min * sx;
oy = vox_z_range(1) - path_y_min * sy;  % path_Y -> disp_Z
oz = vox_y_range(1) - path_z_min * sz;  % path_Z -> disp_Y

fprintf('  Scale: sx=%.4f, sy=%.4f, sz=%.4f\n', sx, sy, sz);

%% ========== 绘图 ==========
fprintf('[4] Drawing...\n');

figure('Name', '3D Mesh + Fiber Paths', 'NumberTitle', 'off', ...
    'Position', [50, 50, 1400, 1000], 'Color', 'w');

% --- 绘制体素网格（与 display_3D_stress 完全一致） ---
fprintf('  Drawing voxel mesh...\n');
threshold = 0.5;
voxel_alpha = 0.08;
face_color = [0.6, 0.6, 0.6];
edge_color = [0.75, 0.75, 0.75];

hx = 1; hy = 1; hz = 1;
cube_face = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];

num_solid = 0;
for k = 1:nelz
    for i = 1:nelx
        for j = 1:nely
            if xPhys(j, i, k) > threshold
                x = (i-1)*hx; y = (j-1)*hy; z = (k-1)*hz;
                vert = [x,y,z; x+hx,y,z; x+hx,y,z+hz; x,y,z+hz;
                        x,y+hy,z; x+hx,y+hy,z; x+hx,y+hy,z+hz; x,y+hy,z+hz];
                dv = vert;
                dv(:,2) = vert(:,3);  % Z_phys -> Y_disp
                dv(:,3) = vert(:,2);  % Y_phys -> Z_disp

                patch('Faces', cube_face, 'Vertices', dv, ...
                      'FaceColor', face_color, ...
                      'EdgeColor', edge_color, 'LineWidth', 0.2, ...
                      'FaceAlpha', voxel_alpha);
                hold on;
                num_solid = num_solid + 1;
            end
        end
    end
end
fprintf('    Solid elements: %d\n', num_solid);

% --- 绘制路径（变换到体素显示坐标系） ---
fprintf('  Drawing fiber paths...\n');
layer_colors = jet(num_layers);
path_count = 0;

for li = 1:num_layers
    ld = all_layers_data(li);
    if ~ld.success, continue; end
    
    c = layer_colors(li, :);
    
    for pi = 1:length(ld.paths_3d)
        pts = ld.paths_3d{pi};
        if isempty(pts) || size(pts, 1) < 2, continue; end
        
        % 变换: path [x, y, z] -> display [Xd, Yd, Zd]
        Xd = pts(:,1) * sx + ox;          % path X -> display X
        Yd = pts(:,3) * sz + oz;          % path Z -> display Y (height)
        Zd = pts(:,2) * sy + oy;          % path Y -> display Z (depth)
        
        plot3(Xd, Yd, Zd, '-', 'Color', c, 'LineWidth', 1.2);
        path_count = path_count + 1;
    end
end
fprintf('    Drawn paths: %d\n', path_count);

% --- 坐标轴和光照 ---
axis equal; axis tight;
box on; grid on;
view(3);
rotate3d on;

light('Position', [-nelx/2, nely*2, nelz*2], 'Style', 'local', 'Color', [1,1,0.9]);
light('Position', [nelx/2, nely*0.5, nelz*0.2], 'Style', 'local', 'Color', [1,1,0.9]);
light('Position', [nelx*1.5, nely, nelz], 'Style', 'infinite', 'Color', [0.9,0.9,1]);
lighting gouraud;
material dull;

xlabel('X (nelx)');
ylabel('Z (nelz) - Height');
zlabel('Y (nely) - Depth');
title(sprintf('Mesh (%d voxels) + Paths (%d paths, %d layers)', ...
    num_solid, path_count, num_layers), 'FontSize', 13);

colormap(jet);
c_bar = colorbar;
c_bar.Label.String = 'Layer';
caxis([1, num_layers]);

hold off;

saveas(gcf, 'mesh_with_paths.png');
saveas(gcf, 'mesh_with_paths.fig');
fprintf('\n  Saved: mesh_with_paths.png / .fig\n');

fprintf('\n====================================================\n');
fprintf('  Done!\n');
fprintf('====================================================\n');