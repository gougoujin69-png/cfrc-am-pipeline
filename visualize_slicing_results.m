%% ========================================
%% 切片结果可视化
%% ========================================
% 功能：可视化细化模型的切片结果
%%

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║          切片结果可视化                               ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% ========== 加载数据 ==========
fprintf('【步骤1】加载切片结果...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

if ~exist('slice_results_refined_latest.mat', 'file')
    error('找不到切片结果文件！请先运行 slice_refined_model_complete.m');
end

load('slice_results_refined_latest.mat');

fprintf('✓ 成功加载切片结果\n');
fprintf('  总层数: %d\n', slice_results.statistics.num_layers);
fprintf('  覆盖率: %.2f%%\n', slice_results.statistics.coverage_rate);

%% ========== 显示层信息表格 ==========
fprintf('\n【步骤2】层信息汇总...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

surface_layers = slice_results.surface_layers;
num_layers = length(surface_layers);

fprintf('\n层号  偏移     方向  尝试  最终阈值  总激活  新激活\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
for i = 1:num_layers
    layer = surface_layers{i};
    dir_str = layer.direction;
    if strcmp(dir_str, 'original')
        dir_str = '原始';
    elseif strcmp(dir_str, 'up')
        dir_str = '向上';
    else
        dir_str = '向下';
    end
    
    fprintf('%3d  %+6.3f  %4s  %3d   %6.3f    %5d   %5d\n', ...
            i, layer.offset, dir_str, ...
            length(layer.attempts), layer.final_threshold, ...
            layer.total_activated, layer.total_newly_activated);
end
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

%% ========== 可视化1：3D切片层 ==========
fprintf('\n【步骤3】生成3D切片层可视化...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

figure('Name', '切片层3D可视化', 'Position', [100, 100, 1200, 800]);

% 显示前10层（或全部，如果少于10层）
num_display = min(17, num_layers);
colors = jet(num_display);

hold on;
for i = 1:1
    layer = surface_layers{i};
    X_surf = layer.X_surf;
    Y_surf = layer.Y_surf;
    Z_surf = layer.Z_surf;
    
    surf(X_surf, Y_surf, Z_surf, 'FaceColor', colors(i,:), ...
         'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('切片层可视化（前%d层）', num_display), 'FontSize', 14, 'FontWeight', 'bold');
colormap(jet);
colorbar('Ticks', linspace(0,1,num_display), ...
         'TickLabels', arrayfun(@(x) sprintf('层%d', x), 1:num_display, 'UniformOutput', false));
grid on;
axis equal;
view(45, 30);
hold off;

fprintf('✓ 3D切片层可视化完成\n');

%% ========== 可视化2：激活分布 ==========
fprintf('\n【步骤4】生成激活分布图...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

figure('Name', '激活网格分布', 'Position', [100, 100, 1400, 600]);

% 子图1：激活的网格
subplot(1, 2, 1);
global_activated = slice_results.global_activated;
[i_act, j_act, k_act] = ind2sub(size(global_activated), find(global_activated));

% 采样显示（避免过多点）
sample_ratio = 0.2;
num_sample = floor(length(i_act) * sample_ratio);
if num_sample > 0
    sample_idx = randperm(length(i_act), num_sample);
    i_act = i_act(sample_idx);
    j_act = j_act(sample_idx);
    k_act = k_act(sample_idx);
    
    grid_data = slice_results.grid_data;
    x_act = zeros(length(i_act), 1);
    y_act = zeros(length(i_act), 1);
    z_act = zeros(length(i_act), 1);
    
    for idx = 1:length(i_act)
        x_act(idx) = grid_data(i_act(idx), j_act(idx), k_act(idx)).x;
        y_act(idx) = grid_data(i_act(idx), j_act(idx), k_act(idx)).y;
        z_act(idx) = grid_data(i_act(idx), j_act(idx), k_act(idx)).z;
    end
    
    scatter3(x_act, y_act, z_act, 5, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('激活的网格', 'FontSize', 14, 'FontWeight', 'bold');
grid on; axis equal; view(45, 30);

% 子图2：未激活的网格
subplot(1, 2, 2);
valid_mask = slice_results.valid_grid_mask;
unactivated_mask = valid_mask & ~global_activated;
[i_un, j_un, k_un] = ind2sub(size(unactivated_mask), find(unactivated_mask));

if ~isempty(i_un)
    x_un = zeros(length(i_un), 1);
    y_un = zeros(length(i_un), 1);
    z_un = zeros(length(i_un), 1);
    
    for idx = 1:length(i_un)
        x_un(idx) = grid_data(i_un(idx), j_un(idx), k_un(idx)).x;
        y_un(idx) = grid_data(i_un(idx), j_un(idx), k_un(idx)).y;
        z_un(idx) = grid_data(i_un(idx), j_un(idx), k_un(idx)).z;
    end
    
    scatter3(x_un, y_un, z_un, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.5);
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('未激活的网格 (%d个)', length(i_un)), 'FontSize', 14, 'FontWeight', 'bold');
grid on; axis equal; view(45, 30);

fprintf('✓ 激活分布图完成\n');

%% ========== 可视化3：偏移-激活曲线 ==========
fprintf('\n【步骤5】生成偏移-激活曲线...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

figure('Name', '偏移-激活分析', 'Position', [100, 100, 1000, 600]);

offsets_list = zeros(num_layers, 1);
new_activated_list = zeros(num_layers, 1);
cumulative_coverage = zeros(num_layers, 1);

for i = 1:num_layers
    offsets_list(i) = surface_layers{i}.offset;
    new_activated_list(i) = surface_layers{i}.total_newly_activated;
    if i == 1
        cumulative_coverage(i) = new_activated_list(i);
    else
        cumulative_coverage(i) = cumulative_coverage(i-1) + new_activated_list(i);
    end
end

% 转换为百分比
total_valid = slice_results.statistics.total_valid_grids;
cumulative_coverage_pct = 100 * cumulative_coverage / total_valid;

% 子图1：新激活数量
subplot(2, 1, 1);
bar(offsets_list, new_activated_list, 'FaceColor', [0.3 0.6 0.9]);
xlabel('偏移量');
ylabel('新激活网格数');
title('每层新激活网格数量', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% 子图2：累计覆盖率
subplot(2, 1, 2);
plot(offsets_list, cumulative_coverage_pct, '-o', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('偏移量');
ylabel('累计覆盖率 (%)');
title('累计覆盖率曲线', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
ylim([0, 105]);
hold on;
plot([offsets_list(1), offsets_list(end)], [95, 95], 'r--', 'LineWidth', 1.5);
plot([offsets_list(1), offsets_list(end)], [99, 99], 'g--', 'LineWidth', 1.5);
legend('覆盖率', '95%目标', '99%目标', 'Location', 'southeast');
hold off;

fprintf('✓ 偏移-激活曲线完成\n');

%% ========== 可视化4：横截面切片 ==========
fprintf('\n【步骤6】生成横截面切片...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

nelz = slice_results.grid_size.nelz;
slice_z = round(nelz / 2);

figure('Name', '横截面切片', 'Position', [100, 100, 1400, 600]);

% 子图1：有效网格
subplot(1, 3, 1);
valid_slice = squeeze(valid_mask(:, :, slice_z));
imagesc(valid_slice');
colormap(gray);
axis equal tight;
title('有效网格', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');
set(gca, 'YDir', 'normal');

% 子图2：激活网格
subplot(1, 3, 2);
activated_slice = squeeze(global_activated(:, :, slice_z));
imagesc(activated_slice');
colormap(gray);
axis equal tight;
title('激活网格', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');
set(gca, 'YDir', 'normal');

% 子图3：未激活网格
subplot(1, 3, 3);
unactivated_slice = squeeze(valid_mask(:, :, slice_z) & ~global_activated(:, :, slice_z));
imagesc(unactivated_slice');
colormap(gray);
axis equal tight;
title('未激活网格', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');
set(gca, 'YDir', 'normal');

sgtitle(sprintf('Z=%d层 横截面分析', slice_z), 'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ 横截面切片完成\n');

%% ========== 统计总结 ==========
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║              统计总结                                 ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

stats = slice_results.statistics;
fprintf('切片统计:\n');
fprintf('  总层数: %d\n', stats.num_layers);
fprintf('  偏移范围: %.3f 到 %.3f\n', offsets_list(1), offsets_list(end));
fprintf('  总有效网格: %d\n', stats.total_valid_grids);
fprintf('  已激活网格: %d\n', stats.final_coverage);
fprintf('  覆盖率: %.2f%%\n', stats.coverage_rate);
fprintf('  曲面上方: %d\n', stats.num_above);
fprintf('  曲面下方: %d\n', stats.num_below);

if stats.coverage_rate >= 99
    fprintf('\n✅ 优秀！接近完美覆盖\n');
elseif stats.coverage_rate >= 95
    fprintf('\n✅ 很好！覆盖率超过95%%\n');
elseif stats.coverage_rate >= 90
    fprintf('\n⚠️  良好，但可以改进参数提高覆盖率\n');
else
    fprintf('\n⚠️  覆盖率偏低，建议调整参数\n');
end

fprintf('\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('✅ 可视化完成！请查看所有图形窗口\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('\n');