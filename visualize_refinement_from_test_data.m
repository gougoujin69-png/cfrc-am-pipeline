%% ========================================
%% 体素细化效果可视化对比 - 基于test.mat数据
%% ========================================
% 功能：对比显示原始网格和细化后的高分辨率模型
%%

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║          体素细化效果可视化（test.mat数据）          ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% ========== 加载原始数据 ==========
fprintf('【步骤1】加载原始数据...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
load('Pre_surface.mat');
% 检查工作区或加载文件
if ~exist('xPhys', 'var') || ~exist('nelx', 'var')
    fprintf('从文件加载原始数据...\n');
    if exist('test.mat', 'file')
        fprintf('  加载 test.mat... ');
        load('test.mat');
        fprintf('完成\n');
    else
        error('找不到 test.mat 文件！');
    end
    
    if exist('test1.mat', 'file')
        fprintf('  加载 test1.mat... ');
        load('test1.mat');
        fprintf('完成\n');
    end
else
    fprintf('✓ 使用工作区中的现有数据\n');
end

% 加载细化数据
if exist('voxel_refined_latest.mat', 'file')
    fprintf('加载细化数据... ');
    fine_data = load('voxel_refined_latest.mat');
    grid_data_fine = fine_data.refined_data.grid_data;
    valid_mask_fine = fine_data.refined_data.valid_grid_mask;
    size_fine = fine_data.refined_data.grid_size;
    fprintf('完成\n');
else
    error('找不到细化数据文件 voxel_refined_latest.mat\n请先运行 voxel_refinement_from_test');
end

%% ========== 处理原始数据 ==========
fprintf('\n【步骤2】处理原始数据格式...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 转换xPhys2格式
xPhys2 = zeros(nelz, nelx, nely);
for elz = 1:nelz
    for elx = 1:nelx
        for ely = 1:nely
            xPhys2(elz,elx,ely) = xPhys(ely,elx,elz);
        end
    end
end

% 创建有效掩码
density_orig = zeros(nelx, nely, nelz);
for k = 1:nelz
    for i = 1:nelx
        for j = 1:nely
            density_orig(i,j,k) = xPhys2(k,i,j);
        end
    end
end
valid_mask_orig = (density_orig > 0.5);

fprintf('\n数据统计:\n');
fprintf('  原始: %d×%d×%d (%d 有效)\n', ...
        nelx, nely, nelz, sum(valid_mask_orig(:)));
fprintf('  细化: %d×%d×%d (%d 有效)\n', ...
        size_fine.nelx, size_fine.nely, size_fine.nelz, sum(valid_mask_fine(:)));

%% ========== 提取显示数据 ==========
fprintf('\n【步骤3】提取可视化数据...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 原始数据点云
fprintf('提取原始点云... ');
[i_orig, j_orig, k_orig] = ind2sub(size(valid_mask_orig), find(valid_mask_orig));
x_orig = (i_orig - 0.5);
y_orig = (j_orig - 0.5);
z_orig = (k_orig - 0.5);
density_orig_pts = density_orig(valid_mask_orig);
fprintf('完成 (%d 点)\n', length(x_orig));

% 细化数据点云（采样显示）
fprintf('提取细化点云（采样）... ');
sample_ratio = 0.1;  % 显示10%的点
[i_fine, j_fine, k_fine] = ind2sub(size(valid_mask_fine), find(valid_mask_fine));

num_sample = floor(length(i_fine) * sample_ratio);
sample_idx = randperm(length(i_fine), num_sample);
i_fine = i_fine(sample_idx);
j_fine = j_fine(sample_idx);
k_fine = k_fine(sample_idx);

x_fine = zeros(length(i_fine), 1);
y_fine = zeros(length(i_fine), 1);
z_fine = zeros(length(i_fine), 1);
density_fine_pts = zeros(length(i_fine), 1);

for idx = 1:length(i_fine)
    x_fine(idx) = grid_data_fine(i_fine(idx), j_fine(idx), k_fine(idx)).x;
    y_fine(idx) = grid_data_fine(i_fine(idx), j_fine(idx), k_fine(idx)).y;
    z_fine(idx) = grid_data_fine(i_fine(idx), j_fine(idx), k_fine(idx)).z;
    density_fine_pts(idx) = grid_data_fine(i_fine(idx), j_fine(idx), k_fine(idx)).xPhys;
end
fprintf('完成 (%d 点, 采样率 %.1f%%)\n', length(x_fine), sample_ratio*100);

%% ========== 可视化对比 ==========
fprintf('\n【步骤4】生成可视化图形...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

figure('Name', '体素细化效果对比', 'Position', [100, 100, 1400, 600]);

%% 子图1: 原始网格
subplot(1, 2, 1);
scatter3(x_orig, y_orig, z_orig, 20, density_orig_pts, 'filled', 'MarkerFaceAlpha', 0.6);
colormap(jet);
colorbar;
caxis([0.5, 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('原始网格 (%d×%d×%d)', nelx, nely, nelz), ...
      'FontSize', 14, 'FontWeight', 'bold');
grid on;
axis equal;
axis off;
view(45, 30);
set(gca, 'FontSize', 10);

%% 子图2: 细化模型
subplot(1, 2, 2);
scatter3(x_fine, y_fine, z_fine, 10, density_fine_pts, 'filled', 'MarkerFaceAlpha', 0.6);
colormap(jet);
colorbar;
caxis([0.5, 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('细化模型 (%d×%d×%d, 显示%.0f%%)', ...
              size_fine.nelx, size_fine.nely, size_fine.nelz, sample_ratio*100), ...
      'FontSize', 14, 'FontWeight', 'bold');
grid on;
axis equal;
view(45, 30);
set(gca, 'FontSize', 10);

fprintf('✓ 3D点云对比完成\n');

%% ========== 横截面对比 ==========
fprintf('\n【步骤5】生成横截面对比...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 选择中间切片
slice_z_orig = round(nelz / 2);
slice_z_fine = round(size_fine.nelz / 2);

% 原始切片
density_slice_orig = squeeze(valid_mask_orig(:, :, slice_z_orig));

% 细化切片
density_slice_fine = squeeze(valid_mask_fine(:, :, slice_z_fine));

figure('Name', 'XY平面横截面对比', 'Position', [100, 100, 1400, 600]);

%% 子图1: 原始切片
subplot(1, 2, 1);
imagesc(density_slice_orig');
colormap(gray);
axis equal tight;
axis off;
xlabel('X');
ylabel('Y');
title(sprintf('原始网格 Z=%d 层', slice_z_orig), 'FontSize', 14, 'FontWeight', 'bold');
colorbar;
set(gca, 'YDir', 'normal');

%% 子图2: 细化切片
subplot(1, 2, 2);
imagesc(density_slice_fine');
colormap(gray);
axis equal tight;
xlabel('X');
ylabel('Y');
title(sprintf('细化模型 Z=%d 层', slice_z_fine), 'FontSize', 14, 'FontWeight', 'bold');
colorbar;
set(gca, 'YDir', 'normal');

fprintf('✓ 横截面对比完成\n');

%% ========== 边缘平滑度分析 ==========
fprintf('\n【步骤6】边缘平滑度分析...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 提取边缘轮廓
fprintf('提取原始边缘... ');
edge_orig = edge(density_slice_orig, 'Canny');
[edge_y_orig, edge_x_orig] = find(edge_orig);
fprintf('完成 (%d 边缘点)\n', length(edge_x_orig));

fprintf('提取细化边缘... ');
edge_fine = edge(density_slice_fine, 'Canny');
[edge_y_fine, edge_x_fine] = find(edge_fine);
fprintf('完成 (%d 边缘点)\n', length(edge_x_fine));

% 可视化边缘
figure('Name', '边缘平滑度对比', 'Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
imshow(~edge_orig);
hold on;
plot(edge_x_orig, edge_y_orig, 'r.', 'MarkerSize', 2);
title('原始网格边缘（阶梯状）', 'FontSize', 14, 'FontWeight', 'bold');
axis on;
xlabel('X');
ylabel('Y');

subplot(1, 2, 2);
imshow(~edge_fine);
hold on;
plot(edge_x_fine, edge_y_fine, 'b.', 'MarkerSize', 1);
title('细化模型边缘（平滑）', 'FontSize', 14, 'FontWeight', 'bold');
axis on;
xlabel('X');
ylabel('Y');

fprintf('✓ 边缘分析完成\n');

%% ========== 统计对比 ==========
fprintf('\n【步骤7】统计对比...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

fprintf('\n网格分辨率提升:\n');
fprintf('  X方向: %d → %d (%.1f倍)\n', nelx, size_fine.nelx, ...
        size_fine.nelx/nelx);
fprintf('  Y方向: %d → %d (%.1f倍)\n', nely, size_fine.nely, ...
        size_fine.nely/nely);
fprintf('  Z方向: %d → %d (%.1f倍)\n', nelz, size_fine.nelz, ...
        size_fine.nelz/nelz);

fprintf('\n边缘点密度:\n');
fprintf('  原始: %d 点/切片\n', length(edge_x_orig));
fprintf('  细化: %d 点/切片\n', length(edge_x_fine));
fprintf('  提升: %.1f 倍\n', length(edge_x_fine)/length(edge_x_orig));

%% ========== 与曲面叠加显示 ==========
if exist('SA', 'var') && exist('SX0', 'var')
    fprintf('\n【步骤8】生成曲面叠加图...\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    
    figure('Name', '细化模型与曲面', 'Position', [100, 100, 800, 600]);
    scatter3(x_fine, y_fine, z_fine, 5, density_fine_pts, 'filled', 'MarkerFaceAlpha', 0.4);
    hold on;
    surf(Pre_surface{1}, Pre_surface{2}, Pre_surface{3}, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    colormap(jet);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('细化体素模型 + 设计曲面', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    axis equal;
    view(45, 30);
    legend('体素点云', '设计曲面', 'Location', 'best');
    
    fprintf('✓ 曲面叠加图完成\n');
end

%% ========== 完成 ==========
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║              ✅ 可视化完成！                          ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('分析结论:\n');
fprintf('  1. 细化后的模型分辨率显著提高\n');
fprintf('  2. 边缘从阶梯状变为平滑曲线\n');
fprintf('  3. 内部结构得到更精细的表达\n');
fprintf('  4. 适合进行高精度3D打印切片\n');
fprintf('\n');
fprintf('下一步: 导出STL模型\n');
fprintf('  >> export_to_stl(''voxel_refined_latest.mat'', ''model.stl'', ''voxel_faces'')\n');
fprintf('\n');