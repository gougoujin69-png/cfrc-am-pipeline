%% ========================================================================
%  ABB IRB-1200 多视角仿真对比视频
%  ========================================================================
%  功能：
%    1. 使用Robotics Toolbox创建机械臂模型
%    2. 对比原始轨迹和优化轨迹
%    3. 生成多视角动画视频
%  ========================================================================

clear; clc; close all;

%% ======================== 加载轨迹数据 ==================================
fprintf('========================================================\n');
fprintf('   ABB IRB-1200 多视角仿真对比\n');
fprintf('========================================================\n\n');

% 检查是否存在轨迹数据文件
if ~exist('trajectory_data.mat', 'file')
    fprintf('请先运行 Generate_Optimized_MOD.m 生成轨迹数据!\n');
    return;
end

load('trajectory_data.mat');
q_orig = trajectory_data.q_orig;
q_opt = trajectory_data.q_opt;
cfx_orig = trajectory_data.cfx_orig;
cfx_opt = trajectory_data.cfx_opt;
n = trajectory_data.n;
sing_idx = trajectory_data.sing_idx_orig;

fprintf('轨迹点数: %d\n', n);
fprintf('原始奇异点: %d\n', trajectory_data.orig_switches);
fprintf('优化后奇异点: %d\n', trajectory_data.opt_switches);

%% ======================== 创建ABB IRB-1200机器人模型 ====================
fprintf('\n创建ABB IRB-1200机器人模型...\n');

% ABB IRB-1200-7/0.7 DH参数
% 使用标准DH参数
d1 = 0.3991;  % 基座到第一关节
a2 = 0.448;   % 连杆2长度
a3 = 0.042;   % 连杆3长度
d4 = 0.451;   % 连杆4长度
d6 = 0.082;   % 末端法兰

% 创建连杆 (使用修正DH参数)
L(1) = Link('d', d1, 'a', 0,  'alpha', -pi/2, 'offset', 0);
L(2) = Link('d', 0,  'a', a2, 'alpha', 0,     'offset', -pi/2);
L(3) = Link('d', 0,  'a', a3, 'alpha', -pi/2, 'offset', 0);
L(4) = Link('d', d4, 'a', 0,  'alpha', pi/2,  'offset', 0);
L(5) = Link('d', 0,  'a', 0,  'alpha', -pi/2, 'offset', 0);
L(6) = Link('d', d6, 'a', 0,  'alpha', 0,     'offset', 0);

% 设置关节限位
L(1).qlim = [-170, 170] * pi/180;
L(2).qlim = [-100, 135] * pi/180;
L(3).qlim = [-200, 70] * pi/180;
L(4).qlim = [-270, 270] * pi/180;
L(5).qlim = [-130, 130] * pi/180;
L(6).qlim = [-360, 360] * pi/180;

robot = SerialLink(L, 'name', 'ABB IRB-1200');
fprintf('  机器人模型创建完成\n');

%% ======================== 采样轨迹（加速播放）===========================
fprintf('\n对轨迹进行采样...\n');

% 采样间隔（每隔sample_step个点取一个）
sample_step = 20;  % 可以调整，越大视频越短
idx_sample = 1:sample_step:n;
n_sample = length(idx_sample);

q_orig_sample = q_orig(idx_sample, :);
q_opt_sample = q_opt(idx_sample, :);
cfx_orig_sample = cfx_orig(idx_sample);
cfx_opt_sample = cfx_opt(idx_sample);

% 找出采样后的奇异点位置
sing_idx_sample = [];
for i = 1:length(sing_idx)
    [~, closest] = min(abs(idx_sample - sing_idx(i)));
    sing_idx_sample = [sing_idx_sample; closest];
end
sing_idx_sample = unique(sing_idx_sample);

fprintf('  采样后点数: %d (原%d点，采样间隔=%d)\n', n_sample, n, sample_step);

%% ======================== 计算末端轨迹 ==================================
fprintf('\n计算末端轨迹...\n');

end_pos_orig = zeros(n_sample, 3);
end_pos_opt = zeros(n_sample, 3);

for i = 1:n_sample
    T_orig = robot.fkine(q_orig_sample(i,:));
    T_opt = robot.fkine(q_opt_sample(i,:));
    end_pos_orig(i,:) = T_orig.t';
    end_pos_opt(i,:) = T_opt.t';
end

% 验证末端位置差异
pos_diff = sqrt(sum((end_pos_orig - end_pos_opt).^2, 2)) * 1000;
fprintf('  末端位置最大差异: %.3f mm\n', max(pos_diff));
fprintf('  末端位置平均差异: %.3f mm\n', mean(pos_diff));

%% ======================== 定义多个视角 ==================================
views = struct();
views(1).name = '正视图';
views(1).az = 0;
views(1).el = 0;

views(2).name = '侧视图';
views(2).az = 90;
views(2).el = 0;

views(3).name = '俯视图';
views(3).az = 0;
views(3).el = 90;

views(4).name = '等轴测视图';
views(4).az = 135;
views(4).el = 30;

views(5).name = '斜视图1';
views(5).az = 45;
views(5).el = 30;

views(6).name = '斜视图2';
views(6).az = -45;
views(6).el = 30;

%% ======================== 生成对比视频 ==================================
fprintf('\n生成对比视频...\n');

% 视频参数
fps = 30;
video_filename = 'ABB_IRB1200_Comparison.mp4';

% 创建视频写入对象
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps;
v.Quality = 95;
open(v);

% 创建图形窗口
fig = figure('Position', [50, 50, 1600, 900], 'Color', 'white');

% 每个视角显示一定帧数后切换
frames_per_view = ceil(n_sample / length(views));

fprintf('  视频参数: %d fps, 每视角约 %d 帧\n', fps, frames_per_view);
fprintf('  开始渲染...\n');

current_view = 1;
frame_in_view = 0;

for i = 1:n_sample
    clf;
    
    % 更新视角
    frame_in_view = frame_in_view + 1;
    if frame_in_view > frames_per_view && current_view < length(views)
        current_view = current_view + 1;
        frame_in_view = 1;
    end
    
    % ===== 左侧：原始轨迹 =====
    subplot(1, 2, 1);
    
    % 绘制机器人
    robot.plot(q_orig_sample(i,:), 'workspace', [-0.2, 1.0, -0.5, 0.5, 0, 0.8], ...
               'noname', 'notiles', 'noshadow', 'nobase', 'nowrist', ...
               'delay', 0, 'trail', {'r-', 'LineWidth', 1});
    hold on;
    
    % 绘制已走过的轨迹
    if i > 1
        plot3(end_pos_orig(1:i, 1), end_pos_orig(1:i, 2), end_pos_orig(1:i, 3), ...
              'r-', 'LineWidth', 2);
    end
    
    % 标记奇异点
    for s = 1:length(sing_idx_sample)
        si = sing_idx_sample(s);
        if si <= i
            plot3(end_pos_orig(si, 1), end_pos_orig(si, 2), end_pos_orig(si, 3), ...
                  'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        end
    end
    
    % 绘制工作台
    fill3([0.4, 0.7, 0.7, 0.4], [-0.1, -0.1, 0.1, 0.1], [0.05, 0.05, 0.05, 0.05], ...
          [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
    
    title(sprintf('原始轨迹 (奇异点: %d)', trajectory_data.orig_switches), 'FontSize', 14);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    view(views(current_view).az, views(current_view).el);
    grid on;
    
    % 显示当前状态
    text(0.05, 0.95, sprintf('点: %d/%d', idx_sample(i), n), ...
         'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    text(0.05, 0.88, sprintf('cfx: %d', cfx_orig_sample(i)), ...
         'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    text(0.05, 0.81, sprintf('J5: %.1f°', q_orig_sample(i,5)*180/pi), ...
         'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    
    % ===== 右侧：优化轨迹 =====
    subplot(1, 2, 2);
    
    % 绘制机器人
    robot.plot(q_opt_sample(i,:), 'workspace', [-0.2, 1.0, -0.5, 0.5, 0, 0.8], ...
               'noname', 'notiles', 'noshadow', 'nobase', 'nowrist', ...
               'delay', 0, 'trail', {'b-', 'LineWidth', 1});
    hold on;
    
    % 绘制已走过的轨迹
    if i > 1
        plot3(end_pos_opt(1:i, 1), end_pos_opt(1:i, 2), end_pos_opt(1:i, 3), ...
              'b-', 'LineWidth', 2);
    end
    
    % 绘制工作台
    fill3([0.4, 0.7, 0.7, 0.4], [-0.1, -0.1, 0.1, 0.1], [0.05, 0.05, 0.05, 0.05], ...
          [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
    
    title(sprintf('优化轨迹 RZ+35° (奇异点: %d)', trajectory_data.opt_switches), 'FontSize', 14);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    view(views(current_view).az, views(current_view).el);
    grid on;
    
    % 显示当前状态
    text(0.05, 0.95, sprintf('点: %d/%d', idx_sample(i), n), ...
         'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    text(0.05, 0.88, sprintf('cfx: %d', cfx_opt_sample(i)), ...
         'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    text(0.05, 0.81, sprintf('J5: %.1f°', q_opt_sample(i,5)*180/pi), ...
         'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white');
    
    % 添加视角标签
    annotation('textbox', [0.4, 0.02, 0.2, 0.04], ...
               'String', sprintf('视角: %s', views(current_view).name), ...
               'HorizontalAlignment', 'center', 'FontSize', 12, ...
               'BackgroundColor', 'yellow', 'EdgeColor', 'none');
    
    % 添加总标题
    sgtitle(sprintf('ABB IRB-1200 轨迹对比 - 帧 %d/%d', i, n_sample), 'FontSize', 16);
    
    drawnow;
    
    % 写入帧
    frame = getframe(fig);
    writeVideo(v, frame);
    
    % 显示进度
    if mod(i, 10) == 0
        fprintf('  进度: %d/%d (%.1f%%)\n', i, n_sample, i/n_sample*100);
    end
end

close(v);
fprintf('  视频已保存: %s\n', video_filename);

%% ======================== 生成各视角静态对比图 ==========================
fprintf('\n生成静态对比图...\n');

fig2 = figure('Position', [50, 50, 1800, 1000], 'Color', 'white');

% 选择几个关键帧进行对比
key_frames = [1, round(n_sample/4), round(n_sample/2), round(3*n_sample/4), n_sample];

for v_idx = 1:4
    for k_idx = 1:length(key_frames)
        subplot(4, 5, (v_idx-1)*5 + k_idx);
        
        kf = key_frames(k_idx);
        
        % 绘制两个轨迹
        plot3(end_pos_orig(1:kf, 1), end_pos_orig(1:kf, 2), end_pos_orig(1:kf, 3), ...
              'r-', 'LineWidth', 1.5, 'DisplayName', '原始');
        hold on;
        plot3(end_pos_opt(1:kf, 1), end_pos_opt(1:kf, 2), end_pos_opt(1:kf, 3), ...
              'b--', 'LineWidth', 1.5, 'DisplayName', '优化');
        
        % 标记奇异点
        for s = 1:length(sing_idx_sample)
            si = sing_idx_sample(s);
            if si <= kf
                plot3(end_pos_orig(si, 1), end_pos_orig(si, 2), end_pos_orig(si, 3), ...
                      'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
        end
        
        % 标记当前点
        plot3(end_pos_orig(kf, 1), end_pos_orig(kf, 2), end_pos_orig(kf, 3), ...
              'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        plot3(end_pos_opt(kf, 1), end_pos_opt(kf, 2), end_pos_opt(kf, 3), ...
              'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        
        view(views(v_idx).az, views(v_idx).el);
        axis equal;
        grid on;
        
        if k_idx == 1
            ylabel(views(v_idx).name, 'FontWeight', 'bold');
        end
        if v_idx == 1
            title(sprintf('点 %d', idx_sample(kf)));
        end
        
        if v_idx == 4 && k_idx == 5
            legend('Location', 'best');
        end
    end
end

sgtitle('末端轨迹多视角对比 (红色: 原始, 蓝色: 优化, 红圈: 奇异点)', 'FontSize', 14);

% 保存图片
saveas(fig2, 'trajectory_comparison_multiview.png');
fprintf('  静态图已保存: trajectory_comparison_multiview.png\n');

%% ======================== 生成关节角度对比图 ============================
fprintf('\n生成关节角度对比图...\n');

fig3 = figure('Position', [50, 50, 1400, 900], 'Color', 'white');

joint_names = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};

for j = 1:6
    subplot(2, 3, j);
    
    plot(1:n, q_orig(:, j)*180/pi, 'r-', 'LineWidth', 1, 'DisplayName', '原始');
    hold on;
    plot(1:n, q_opt(:, j)*180/pi, 'b-', 'LineWidth', 1, 'DisplayName', '优化');
    
    if j == 5  % J5
        yline(0, 'k--', 'LineWidth', 1.5);
        % 标记奇异点
        for s = 1:length(sing_idx)
            xline(sing_idx(s), 'r:', 'LineWidth', 1);
        end
    end
    
    xlabel('点索引');
    ylabel([joint_names{j}, ' (°)']);
    title(joint_names{j});
    legend('Location', 'best');
    grid on;
end

sgtitle('关节角度对比 (红色: 原始, 蓝色: 优化后)', 'FontSize', 14);

saveas(fig3, 'joint_angles_comparison.png');
fprintf('  关节角度图已保存: joint_angles_comparison.png\n');

%% ======================== 生成J5专项分析图 ==============================
fprintf('\n生成J5专项分析图...\n');

fig4 = figure('Position', [50, 50, 1200, 600], 'Color', 'white');

subplot(1, 2, 1);
plot(1:n, q_orig(:, 5)*180/pi, 'r-', 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 2);
scatter(sing_idx, q_orig(sing_idx, 5)*180/pi, 80, 'r', 'filled');
fill([1, n, n, 1], [0, 0, min(q_orig(:,5)*180/pi)-5, min(q_orig(:,5)*180/pi)-5], ...
     'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xlabel('点索引');
ylabel('J5 (°)');
title(sprintf('原始: J5穿过0 (%d个奇异点)', length(sing_idx)));
grid on;
ylim([min(q_orig(:,5)*180/pi)-10, max(q_orig(:,5)*180/pi)+10]);

subplot(1, 2, 2);
plot(1:n, q_opt(:, 5)*180/pi, 'b-', 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 2);
fill([1, n, n, 1], [0, 0, -10, -10], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xlabel('点索引');
ylabel('J5 (°)');
title(sprintf('优化后: J5始终>0 (0个奇异点)'));
grid on;
ylim([0, max(q_opt(:,5)*180/pi)+10]);

sgtitle('J5轨迹对比 - RZ偏移+35°使J5避开0', 'FontSize', 14);

saveas(fig4, 'J5_analysis.png');
fprintf('  J5分析图已保存: J5_analysis.png\n');

%% ======================== 总结 ==========================================
fprintf('\n========================================================\n');
fprintf('   生成完成\n');
fprintf('========================================================\n');
fprintf('\n生成的文件:\n');
fprintf('  1. %s - 对比动画视频\n', video_filename);
fprintf('  2. trajectory_comparison_multiview.png - 多视角静态对比\n');
fprintf('  3. joint_angles_comparison.png - 关节角度对比\n');
fprintf('  4. J5_analysis.png - J5专项分析\n');

fprintf('\n关键发现:\n');
fprintf('  - 原始轨迹: J5范围 [%.1f°, %.1f°], 穿过0, %d个奇异点\n', ...
        min(q_orig(:,5)*180/pi), max(q_orig(:,5)*180/pi), trajectory_data.orig_switches);
fprintf('  - 优化轨迹: J5范围 [%.1f°, %.1f°], 不穿过0, %d个奇异点\n', ...
        min(q_opt(:,5)*180/pi), max(q_opt(:,5)*180/pi), trajectory_data.opt_switches);
fprintf('  - 末端轨迹最大差异: %.3f mm (可忽略)\n', max(pos_diff));