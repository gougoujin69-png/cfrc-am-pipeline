%% ========================================================================
%  调试脚本：详细分析奇异点检测差异
%  ========================================================================
%  目的：找出我的脚本与YZ_AMV1.m计算奇异点数量不同的原因
%  ========================================================================

clear; clc; close all;

%% ======================== 参数设置（完全复制YZ_AMV1.m）==================
firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;  % 0.05653 m

% 碳纤维工具参数
tool_hc = -50.7321e-3;
tool_yc = -50.0753e-3;
tool_lc = 70.963e-3;
tool_ac = 30/180*pi;

% 树脂工具参数
tool_hr = -50.3541e-3;
tool_yr = 50.9039e-3;
tool_lr = 71.4291e-3;
tool_ar = -30/180*pi;

% 补偿值
x_comp = -0.5;
y_comp = -1.5;
z_comp = 0.2;

base_x_offset = 0.55;
ql1_default = [0, 0, pi/6.0, 0, 0, 0];

%% ======================== 读取Excel文件 =================================
excel_file = 'path_23_1.xlsx';
fprintf('========================================================\n');
fprintf('   调试：奇异点检测详细分析\n');
fprintf('========================================================\n\n');

YM = xlsread(excel_file);
n = size(YM, 1);
fprintf('路径文件: %s\n', excel_file);
fprintf('数据维度: %d 行 x %d 列\n', size(YM, 1), size(YM, 2));

%% ======================== 模拟YZ_AMV1.m的完整逻辑 =======================
fprintf('\n模拟YZ_AMV1.m的完整逻辑...\n');

q_traj = zeros(n, 6);
rcf_traj = zeros(n, 4);
cfx_traj = zeros(n, 1);

% 记录所有rcf变化（不仅仅是cfx）
rcf_changes = [];  % [点索引, 哪个rcf变了, 旧值, 新值]

ql = ql1_default;
prev_rcf = [];

for i = 1:n
    % ===== 根据工具类型选择参数（完全按照YZ_AMV1.m）=====
    if size(YM, 2) >= 8
        tool_type = YM(i, 8);
    else
        tool_type = 111;  % 默认碳纤维
    end
    
    if tool_type == 111
        tool_l = tool_lc;
        tool_h = tool_hc;
        tool_y = tool_yc;
        tool_a = tool_ac;
        x_off = x_comp;
        y_off = y_comp;
        z_off = z_comp;
    else
        tool_l = tool_lr;
        tool_h = tool_hr;
        tool_y = tool_yr;
        tool_a = tool_ar;
        x_off = 0;
        y_off = 0;
        z_off = 0;
    end
    
    % ===== 计算目标位姿 =====
    target_x = base_x_offset + (YM(i,1) + x_off) / 1000.0;
    target_y = (YM(i,2) + y_off) / 1000.0;
    target_z = firstl + (YM(i,3) + z_off) / 1000.0;
    
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0;
        ry = 0;
    end
    
    % 构建变换矩阵
    T_target = transl_m(target_x, target_y, target_z) * ...
               troty_m(pi) * trotx_m(rx) * troty_m(ry);
    
    % ===== 逆运动学 =====
    Q = ikabb_m(T_target, tool_l, tool_h, tool_y, tool_a);
    
    try
        qi = ikbest_m(ql, Q);
    catch ME
        fprintf('警告: 点%d IK失败: %s\n', i, ME.message);
        qi = ql;
    end
    
    q_traj(i, :) = qi;
    
    % ===== 计算rcf =====
    rcf = rscfx_m(qi);
    rcf_traj(i, :) = rcf;
    cfx_traj(i) = rcf(4);
    
    % ===== 检测rcf变化 =====
    if i > 1
        for k = 1:4
            if rcf(k) ~= prev_rcf(k)
                rcf_changes = [rcf_changes; i, k, prev_rcf(k), rcf(k)];
            end
        end
    end
    
    ql = qi;
    prev_rcf = rcf;
end

%% ======================== 分析结果 ======================================
fprintf('\n========================================================\n');
fprintf('   分析结果\n');
fprintf('========================================================\n');

% 统计各种rcf变化
cfx_change_count = sum(rcf_changes(:,2) == 4);
cf1_change_count = sum(rcf_changes(:,2) == 1);
cf4_change_count = sum(rcf_changes(:,2) == 2);
cf6_change_count = sum(rcf_changes(:,2) == 3);

fprintf('\nrcf变化统计:\n');
fprintf('  cf1 (J1区间) 变化次数: %d\n', cf1_change_count);
fprintf('  cf4 (J4区间) 变化次数: %d\n', cf4_change_count);
fprintf('  cf6 (J6区间) 变化次数: %d\n', cf6_change_count);
fprintf('  cfx (腕部配置) 变化次数: %d\n', cfx_change_count);

% 只统计cfx变化的点（这是我之前脚本的逻辑）
cfx_only_changes = rcf_changes(rcf_changes(:,2) == 4, :);
fprintf('\n仅cfx变化的点: %d 个\n', size(cfx_only_changes, 1));
if ~isempty(cfx_only_changes)
    fprintf('  位置: %s\n', mat2str(cfx_only_changes(:,1)'));
end

% 统计任意rcf变化的点（可能是YZ_AMV1.m的逻辑？）
unique_change_points = unique(rcf_changes(:,1));
fprintf('\n任意rcf变化的点: %d 个\n', length(unique_change_points));
if ~isempty(unique_change_points)
    fprintf('  位置: %s\n', mat2str(unique_change_points'));
end

%% ======================== 详细列出所有rcf变化 ===========================
fprintf('\n========================================================\n');
fprintf('   所有rcf变化详情\n');
fprintf('========================================================\n');

fprintf('\n点索引\trcf分量\t旧值\t新值\t说明\n');
fprintf('------\t-------\t----\t----\t----\n');

rcf_names = {'cf1(J1)', 'cf4(J4)', 'cf6(J6)', 'cfx(腕)'};

for i = 1:size(rcf_changes, 1)
    idx = rcf_changes(i, 1);
    rcf_idx = rcf_changes(i, 2);
    old_val = rcf_changes(i, 3);
    new_val = rcf_changes(i, 4);
    
    fprintf('%d\t%s\t%d\t%d\n', idx, rcf_names{rcf_idx}, old_val, new_val);
end

%% ======================== 检查YZ_AMV1.m的判断逻辑 =======================
fprintf('\n========================================================\n');
fprintf('   模拟YZ_AMV1.m的MoveAbsJ判断逻辑\n');
fprintf('========================================================\n');

% YZ_AMV1.m中的判断逻辑（第117行和第125行）：
% if rcf(4) ~= rcfl(4)  % 只检查cfx
%     flag = 1;  % 使用MoveAbsJ
% end

moveabsj_points = [];
rcfl = rscfx_m(q_traj(1,:));  % 第一个点的rcf作为参考

for i = 2:n
    rcf = rcf_traj(i, :);
    rcfp = rcf_traj(i-1, :);  % 前一个点的rcf
    
    % 检查cfx是否变化
    if rcf(4) ~= rcfp(4)
        moveabsj_points = [moveabsj_points; i];
    end
end

fprintf('\n按照"只检查cfx(4)"的逻辑:\n');
fprintf('  MoveAbsJ点数: %d\n', length(moveabsj_points));
fprintf('  位置: %s\n', mat2str(moveabsj_points'));

%% ======================== 检查是否遗漏了某些变化 ========================
fprintf('\n========================================================\n');
fprintf('   检查可能遗漏的情况\n');
fprintf('========================================================\n');

% 检查cfx在边界值附近的点
j5_angles = q_traj(:, 5) * 180 / pi;
near_zero_points = find(abs(j5_angles) < 5);  % J5接近0的点

fprintf('\nJ5接近0°(±5°)的点: %d 个\n', length(near_zero_points));
if length(near_zero_points) <= 30
    fprintf('  位置: %s\n', mat2str(near_zero_points'));
end

% 检查cfx轨迹中的跳变
cfx_diff = diff(cfx_traj);
cfx_jumps = find(cfx_diff ~= 0) + 1;

fprintf('\ncfx跳变的点（diff!=0）: %d 个\n', length(cfx_jumps));
fprintf('  位置: %s\n', mat2str(cfx_jumps'));

%% ======================== J5轨迹分析 ====================================
fprintf('\n========================================================\n');
fprintf('   J5轨迹分析\n');
fprintf('========================================================\n');

fprintf('\nJ5范围: [%.2f°, %.2f°]\n', min(j5_angles), max(j5_angles));
fprintf('J5穿过0°的次数: ');

% 统计J5从正到负或从负到正的次数
j5_sign_changes = 0;
j5_cross_points = [];
for i = 2:n
    if j5_angles(i-1) * j5_angles(i) < 0  % 符号变化
        j5_sign_changes = j5_sign_changes + 1;
        j5_cross_points = [j5_cross_points; i];
    end
end
fprintf('%d 次\n', j5_sign_changes);
if ~isempty(j5_cross_points)
    fprintf('  穿过点: %s\n', mat2str(j5_cross_points'));
end

%% ======================== 对比分析 ======================================
fprintf('\n========================================================\n');
fprintf('   对比总结\n');
fprintf('========================================================\n');

fprintf('\n我的脚本检测到的奇异点（cfx变化）: %d 个\n', length(cfx_jumps));
fprintf('J5穿过0°的次数: %d 次\n', j5_sign_changes);
fprintf('用户.mod文件中的MoveAbsJ: 9 个\n');
fprintf('  (假设第1个是初始MoveJ，则路径奇异点应为 8 个)\n');

fprintf('\n差异分析:\n');
if length(cfx_jumps) ~= 8
    fprintf('  ⚠ 存在差异！我检测到 %d 个，预期 8 个\n', length(cfx_jumps));
    fprintf('  可能的原因:\n');
    fprintf('    1. YZ_AMV1.m可能检查了其他rcf分量\n');
    fprintf('    2. 初始ql1的设置可能不同\n');
    fprintf('    3. ikbest选择的解可能不同\n');
end

%% ======================== 可视化 ========================================
figure('Name', '调试分析', 'Position', [50, 50, 1400, 800]);

subplot(2, 3, 1);
plot(1:n, j5_angles, 'b-', 'LineWidth', 1);
hold on;
yline(0, 'r--', 'LineWidth', 2);
scatter(cfx_jumps, j5_angles(cfx_jumps), 80, 'r', 'filled');
scatter(j5_cross_points, j5_angles(j5_cross_points), 60, 'g', 'filled');
xlabel('点索引');
ylabel('J5 (°)');
title('J5轨迹 (红点=cfx变化, 绿点=J5过0)');
grid on;

subplot(2, 3, 2);
plot(1:n, cfx_traj, 'k-', 'LineWidth', 1.5);
hold on;
scatter(cfx_jumps, cfx_traj(cfx_jumps), 80, 'r', 'filled');
xlabel('点索引');
ylabel('cfx');
title(sprintf('cfx轨迹 (变化=%d次)', length(cfx_jumps)));
grid on;

subplot(2, 3, 3);
plot(1:n, rcf_traj(:,1), 'b-', 'LineWidth', 1, 'DisplayName', 'cf1');
hold on;
plot(1:n, rcf_traj(:,2), 'g-', 'LineWidth', 1, 'DisplayName', 'cf4');
plot(1:n, rcf_traj(:,3), 'm-', 'LineWidth', 1, 'DisplayName', 'cf6');
xlabel('点索引');
ylabel('rcf值');
title('cf1, cf4, cf6轨迹');
legend('Location', 'best');
grid on;

subplot(2, 3, 4);
histogram(j5_angles, 50);
xlabel('J5 (°)');
ylabel('频次');
title('J5分布');
xline(0, 'r--', 'LineWidth', 2);
grid on;

subplot(2, 3, 5);
bar([length(cfx_jumps), j5_sign_changes, 8]);
set(gca, 'XTickLabel', {'cfx变化', 'J5过0', '预期(mod-1)'});
ylabel('次数');
title('奇异点数量对比');
grid on;

subplot(2, 3, 6);
% 显示rcf各分量变化次数
bar([cf1_change_count, cf4_change_count, cf6_change_count, cfx_change_count]);
set(gca, 'XTickLabel', {'cf1', 'cf4', 'cf6', 'cfx'});
ylabel('变化次数');
title('各rcf分量变化次数');
grid on;

sgtitle('奇异点检测调试分析');

%% ========================================================================
%  辅助函数
%% ========================================================================

function T = transl_m(x, y, z)
    T = eye(4); T(1:3, 4) = [x; y; z];
end

function T = trotx_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4); T(1:3, 1:3) = [1, 0, 0; 0, c, -s; 0, s, c];
end

function T = troty_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4); T(1:3, 1:3) = [c, 0, s; 0, 1, 0; -s, 0, c];
end

function Q = ikabb_m(To, tool_l, tool_h, tool_y, tool_a)
    T = To * trotx_m(-tool_a) * transl_m(-tool_h, -tool_y, -tool_l-0.082);
    T(3,4) = T(3,4) - 0.3991;
    a2 = 0.448; a3 = 0.042; d4 = 0.451;
    k = ((T(1,4)^2+T(2,4)^2+T(3,4)^2) - a2^2-a3^2-d4^2)/(2*a2);
    nx = T(1,1); ny = T(2,1); nz = T(3,1);
    ox = T(1,2); oy = T(2,2); oz = T(3,2);
    ax = T(1,3); ay = T(2,3); az = T(3,3);
    Q = zeros(8, 6);
    signs = [1,1,1; 1,1,-1; 1,-1,1; 1,-1,-1; -1,1,1; -1,1,-1; -1,-1,1; -1,-1,-1];
    for idx = 1:8
        Q(idx,:) = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, signs(idx,1), signs(idx,2), signs(idx,3));
    end
end

function ik = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, sign2, sign3, sign5)
    theta3 = atan2(a3, d4) - atan2(k, sign3*sqrt(a3^2+d4^2-k^2));
    u1 = a3*cos(theta3) - d4*sin(theta3) + a2;
    u2 = a3*sin(theta3) + d4*cos(theta3);
    theta2 = atan2(-T(3,4), sign2*sqrt(u1^2+u2^2-T(3,4)^2)) - atan2(u2, u1);
    v1 = cos(theta2)*u1 - sin(theta2)*u2;
    if v1 > 0, theta1 = atan2(T(2,4), T(1,4));
    elseif v1 < 0, theta1 = atan2(-T(2,4), -T(1,4));
    else, theta1 = 0; end
    c1 = cos(theta1); s1 = sin(theta1);
    c23 = cos(theta2+theta3); s23 = sin(theta2+theta3);
    c5 = -ax*c1*s23 - ay*s1*s23 - az*c23;
    theta5 = atan2(sign5*sqrt(1-c5^2), c5);
    s5 = sin(theta5);
    if s5 > 0
        theta4 = atan2(-ax*s1+ay*c1, -ax*c1*c23-ay*s1*c23+az*s23);
        theta6 = atan2(-ox*c1*s23-oy*s1*s23-oz*c23, nx*c1*s23+ny*s1*s23+nz*c23);
    elseif s5 < 0
        theta4 = atan2(ax*s1-ay*c1, ax*c1*c23+ay*s1*c23-az*s23);
        theta6 = atan2(ox*c1*s23+oy*s1*s23+oz*c23, -nx*c1*s23-ny*s1*s23-nz*c23);
    else, theta4 = 0; theta6 = 0; end
    theta2 = theta2 + pi/2;
    ik = [theta1, theta2, theta3, theta4, theta5, theta6];
end

function qi = ikbest_m(qn, Q)
    s = zeros(1, 8);
    for i = 1:8
        for j = 1:6
            s(i) = s(i) + abs(Q(i,j) - qn(j));
        end
    end
    for i = 1:9
        [~, b] = min(s);
        output = 1;
        if Q(b,1) > (17*pi/18) || Q(b,1) < (-17*pi/18), output = 0; end
        if Q(b,2) > (13*pi/18) || Q(b,2) < (-10*pi/18), output = 0; end
        if Q(b,3) > (7*pi/18) || Q(b,3) < (-20*pi/18), output = 0; end
        if Q(b,4) > (27*pi/18) || Q(b,4) < (-27*pi/18), output = 0; end
        if Q(b,5) > (17*pi/18) || Q(b,5) < (-17*pi/18), output = 0; end
        if Q(b,6) > (13*pi/18) || Q(b,6) < (-13*pi/18), output = 0; end
        if output == 1, qi = Q(b,:); return;
        elseif i <= 8, s(b) = 10000;
        else, error('The solution does not exist!'); end
    end
end

function cfx = rscfx_m(q)
    cfx = zeros(1, 4); d = 1e-14;
    if q(1) > d-pi && q(1) <= d-pi/2, cfx(1) = -2;
    elseif q(1) > d-pi/2 && q(1) <= -d, cfx(1) = -1;
    elseif q(1) > -d && q(1) <= d+pi/2, cfx(1) = 0;
    elseif q(1) > d+pi/2 && q(1) <= d+pi, cfx(1) = 1; end
    if q(4) > d-3*pi/2 && q(4) <= d-pi, cfx(2) = -3;
    elseif q(4) > d-pi && q(4) <= d-pi/2, cfx(2) = -2;
    elseif q(4) > d-pi/2 && q(4) <= -d, cfx(2) = -1;
    elseif q(4) > -d && q(4) <= d+pi/2, cfx(2) = 0;
    elseif q(4) > d+pi/2 && q(4) <= d+pi, cfx(2) = 1;
    elseif q(4) > d+pi && q(4) <= d+3*pi/2, cfx(2) = 2; end
    if q(6) > d-3*pi/2 && q(6) <= d-pi, cfx(3) = -3;
    elseif q(6) > d-pi && q(6) <= d-pi/2, cfx(3) = -2;
    elseif q(6) > d-pi/2 && q(6) <= -d, cfx(3) = -1;
    elseif q(6) > -d && q(6) <= d+pi/2, cfx(3) = 0;
    elseif q(6) > d+pi/2 && q(6) <= d+pi, cfx(3) = 1;
    elseif q(6) > d+pi && q(6) <= d+3*pi/2, cfx(3) = 2; end
    cfx_a = 448*sin(q(2)) + 42*sin(q(2)+q(3)) + 451*sin(pi/2-(q(2)+q(3)));
    cfx_b = q(3); cfx_c = q(5); cfx_r = 0;
    if cfx_a >= 0 && cfx_b > -1.459095 && cfx_c >= 0, cfx_r = 0;
    elseif cfx_a >= 0 && cfx_b > -1.459095 && cfx_c < 0, cfx_r = 1;
    elseif cfx_a >= 0 && cfx_b < -1.50971 && cfx_c >= 0, cfx_r = 2;
    elseif cfx_a >= 0 && cfx_b < -1.50971 && cfx_c < 0, cfx_r = 3;
    elseif cfx_a < 0 && cfx_b > -1.459095 && cfx_c >= 0, cfx_r = 4;
    elseif cfx_a < 0 && cfx_b > -1.459095 && cfx_c < 0, cfx_r = 5;
    elseif cfx_a < 0 && cfx_b < -1.50971 && cfx_c >= 0, cfx_r = 6;
    elseif cfx_a < 0 && cfx_b < -1.50971 && cfx_c < 0, cfx_r = 7; end
    cfx(4) = cfx_r;
end