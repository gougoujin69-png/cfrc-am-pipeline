%% ========================================================================
%  ABB IRB-1200 利用Z轴旋转自由度避免奇异点
%  ========================================================================
%  核心思想：
%    1. 路径位置(X,Y,Z)不变
%    2. 工具垂直于路径的姿态(RX,RY)不变
%    3. 但绕工具Z轴的旋转(RZ)是自由的
%    4. 通过添加RZ偏移，改变腕部配置，使J5不穿过0
%  ========================================================================

clear; clc; close all;

%% ======================== 参数设置 ======================================
firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;
tool_hc = -50.7321e-3; tool_yc = -50.0753e-3; tool_lc = 70.963e-3; tool_ac = 30/180*pi;
x_comp = -0.5; y_comp = -1.5; z_comp = 0.2;
base_x_offset = 0.55;
ql1_default = [0, 0, pi/6.0, 0, 0, 0];

%% ======================== 读取路径 ======================================
excel_file = 'path_23_1.xlsx';
fprintf('========================================================\n');
fprintf('   利用Z轴旋转自由度避免奇异点\n');
fprintf('========================================================\n\n');

YM = xlsread(excel_file);
n = size(YM, 1);
fprintf('路径点数: %d\n', n);

%% ======================== 分析原始配置 ==================================
fprintf('\n第一步：分析原始配置...\n');

tool_params = [tool_lc, tool_hc, tool_yc, tool_ac];

% 计算原始轨迹
[orig_q, orig_cfx, orig_switches, orig_switch_idx] = computeTrajectory(YM, tool_params, ql1_default, n, firstl, base_x_offset, x_comp, y_comp, z_comp, zeros(n,1));

fprintf('原始配置:\n');
fprintf('  J5范围: [%.2f°, %.2f°]\n', min(orig_q(:,5))*180/pi, max(orig_q(:,5))*180/pi);
fprintf('  J5穿过0: 是\n');
fprintf('  奇异点数: %d\n', orig_switches);

%% ======================== 搜索最优全局RZ偏移 ============================
fprintf('\n第二步：搜索最优全局RZ偏移...\n');

% 测试不同的全局偏移角度
delta_range = -90:1:90;
results = zeros(length(delta_range), 4);  % [delta, switches, j5_min, j5_max]

best_delta = 0;
best_switches = orig_switches;
best_q = orig_q;

for idx = 1:length(delta_range)
    delta_deg = delta_range(idx);
    delta_rad = delta_deg * pi / 180;
    
    % 所有点使用相同的RZ偏移
    rz_offset = delta_rad * ones(n, 1);
    
    [q_traj, cfx_traj, switches, ~] = computeTrajectory(YM, tool_params, ql1_default, n, firstl, base_x_offset, x_comp, y_comp, z_comp, rz_offset);
    
    j5_range = q_traj(:,5) * 180 / pi;
    results(idx, :) = [delta_deg, switches, min(j5_range), max(j5_range)];
    
    if switches < best_switches
        best_switches = switches;
        best_delta = delta_deg;
        best_q = q_traj;
        fprintf('  发现更优: δ = %+.0f°, 奇异点 = %d, J5=[%.1f°, %.1f°]\n', ...
                delta_deg, switches, min(j5_range), max(j5_range));
    end
end

fprintf('\n全局RZ偏移搜索结果:\n');
fprintf('  最优偏移: δ = %+.0f°\n', best_delta);
fprintf('  奇异点: %d → %d\n', orig_switches, best_switches);

%% ======================== 显示不同偏移的效果 ============================
fprintf('\n不同RZ偏移的效果:\n');
fprintf('δ(°)\t奇异点\tJ5最小\tJ5最大\tJ5穿0\n');
fprintf('----\t------\t------\t------\t-----\n');
for idx = 1:length(delta_range)
    crosses_zero = (results(idx,3) < 0 && results(idx,4) > 0);
    cross_str = '是';
    if ~crosses_zero
        cross_str = '否 ✓';
    end
    if results(idx, 2) <= orig_switches
        fprintf('%+4.0f\t%d\t%.1f\t%.1f\t%s\n', results(idx,1), results(idx,2), results(idx,3), results(idx,4), cross_str);
    end
end

%% ======================== 自适应RZ偏移策略 ==============================
fprintf('\n========================================================\n');
fprintf('   第三步：自适应RZ偏移策略\n');
fprintf('========================================================\n');

% 策略：根据J5的值动态调整RZ偏移
% 目标：让J5始终保持正值（或始终负值）

fprintf('分析J5轨迹特征...\n');

j5_orig = orig_q(:,5) * 180 / pi;

% 找到J5接近0的区域
j5_near_zero = abs(j5_orig) < 45;  % 接近0的区域
fprintf('  J5在±45°范围内的点: %d (%.1f%%)\n', sum(j5_near_zero), sum(j5_near_zero)/n*100);

% 计算需要多少偏移才能让J5保持正
needed_offset_for_positive = max(0, -min(j5_orig)) + 5;  % +5°余量
fprintf('  让J5保持正值需要的偏移: +%.1f°\n', needed_offset_for_positive);

% 计算需要多少偏移才能让J5保持负
needed_offset_for_negative = min(0, -max(j5_orig)) - 5;  % -5°余量
fprintf('  让J5保持负值需要的偏移: %.1f°\n', needed_offset_for_negative);

% 测试保持J5正值的偏移
fprintf('\n测试保持J5正值的偏移 (+%.0f°)...\n', needed_offset_for_positive);
rz_offset_pos = needed_offset_for_positive * pi / 180 * ones(n, 1);
[q_pos, cfx_pos, switches_pos, ~] = computeTrajectory(YM, tool_params, ql1_default, n, firstl, base_x_offset, x_comp, y_comp, z_comp, rz_offset_pos);
j5_pos = q_pos(:,5) * 180 / pi;
fprintf('  结果: J5范围 [%.1f°, %.1f°], 奇异点: %d\n', min(j5_pos), max(j5_pos), switches_pos);

% 测试保持J5负值的偏移
fprintf('\n测试保持J5负值的偏移 (%.0f°)...\n', needed_offset_for_negative);
rz_offset_neg = needed_offset_for_negative * pi / 180 * ones(n, 1);
[q_neg, cfx_neg, switches_neg, ~] = computeTrajectory(YM, tool_params, ql1_default, n, firstl, base_x_offset, x_comp, y_comp, z_comp, rz_offset_neg);
j5_neg = q_neg(:,5) * 180 / pi;
fprintf('  结果: J5范围 [%.1f°, %.1f°], 奇异点: %d\n', min(j5_neg), max(j5_neg), switches_neg);

%% ======================== 逐点优化RZ偏移 ================================
fprintf('\n========================================================\n');
fprintf('   第四步：逐点优化RZ偏移\n');
fprintf('========================================================\n');

% 目标：为每个点找到最优的RZ偏移，使cfx保持不变
% 策略：维持与前一点相同的cfx

fprintf('为每个点计算最优RZ偏移...\n');

optimal_rz = zeros(n, 1);
optimal_q_adaptive = zeros(n, 6);
optimal_cfx_adaptive = zeros(n, 1);

ql = ql1_default;
target_cfx = 0;  % 初始目标cfx

for i = 1:n
    % 构建目标变换矩阵
    target_x = base_x_offset + (YM(i,1) + x_comp) / 1000.0;
    target_y = (YM(i,2) + y_comp) / 1000.0;
    target_z = firstl + (YM(i,3) + z_comp) / 1000.0;
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0; ry = 0;
    end
    
    T_base = transl_m(target_x, target_y, target_z) * troty_m(pi) * trotx_m(rx) * troty_m(ry);
    
    % 搜索最优RZ偏移
    best_rz = 0;
    best_q_point = [];
    best_cfx_point = target_cfx;
    found_target = false;
    
    % 尝试不同的RZ偏移
    for rz_deg = -180:10:180
        rz_rad = rz_deg * pi / 180;
        T_target = T_base * trotz_m(rz_rad);
        
        Q = ikabb_m(T_target, tool_params(1), tool_params(2), tool_params(3), tool_params(4));
        
        % 检查是否有目标cfx的有效解
        for j = 1:8
            if isValidConfig(Q(j,:))
                rcf = rscfx_m(Q(j,:));
                if rcf(4) == target_cfx
                    % 找到了！选择最接近的解
                    dist = sum(abs(Q(j,:) - ql));
                    if isempty(best_q_point) || dist < sum(abs(best_q_point - ql))
                        best_q_point = Q(j,:);
                        best_rz = rz_deg;
                        best_cfx_point = target_cfx;
                        found_target = true;
                    end
                end
            end
        end
    end
    
    if ~found_target
        % 如果找不到目标cfx，则必须切换
        % 尝试找另一个cfx
        new_target_cfx = 1 - target_cfx;
        for rz_deg = -180:10:180
            rz_rad = rz_deg * pi / 180;
            T_target = T_base * trotz_m(rz_rad);
            
            Q = ikabb_m(T_target, tool_params(1), tool_params(2), tool_params(3), tool_params(4));
            
            for j = 1:8
                if isValidConfig(Q(j,:))
                    rcf = rscfx_m(Q(j,:));
                    if rcf(4) == new_target_cfx
                        dist = sum(abs(Q(j,:) - ql));
                        if isempty(best_q_point) || dist < sum(abs(best_q_point - ql))
                            best_q_point = Q(j,:);
                            best_rz = rz_deg;
                            best_cfx_point = new_target_cfx;
                        end
                    end
                end
            end
        end
        target_cfx = new_target_cfx;
    end
    
    if isempty(best_q_point)
        % 如果还是找不到，使用原始方法
        T_target = T_base;
        Q = ikabb_m(T_target, tool_params(1), tool_params(2), tool_params(3), tool_params(4));
        best_q_point = ikbest_m(ql, Q);
        rcf = rscfx_m(best_q_point);
        best_cfx_point = rcf(4);
        best_rz = 0;
    end
    
    optimal_rz(i) = best_rz;
    optimal_q_adaptive(i,:) = best_q_point;
    optimal_cfx_adaptive(i) = best_cfx_point;
    ql = best_q_point;
    
    if mod(i, 500) == 0
        fprintf('  进度: %d/%d\n', i, n);
    end
end

% 计算自适应方案的切换次数
adaptive_switches = sum(diff(optimal_cfx_adaptive) ~= 0);
adaptive_switch_idx = find(diff(optimal_cfx_adaptive) ~= 0) + 1;

fprintf('\n自适应RZ偏移结果:\n');
fprintf('  奇异点数: %d\n', adaptive_switches);
fprintf('  RZ偏移范围: [%.0f°, %.0f°]\n', min(optimal_rz), max(optimal_rz));

%% ======================== 结果对比 ======================================
fprintf('\n========================================================\n');
fprintf('   结果对比\n');
fprintf('========================================================\n');

fprintf('\n方案\t\t\t奇异点\tJ5范围\n');
fprintf('----\t\t\t------\t------\n');
fprintf('原始(无偏移)\t\t%d\t[%.1f°, %.1f°]\n', orig_switches, min(j5_orig), max(j5_orig));
fprintf('全局偏移(δ=%+.0f°)\t%d\t[%.1f°, %.1f°]\n', best_delta, best_switches, min(best_q(:,5))*180/pi, max(best_q(:,5))*180/pi);
fprintf('保持J5正\t\t%d\t[%.1f°, %.1f°]\n', switches_pos, min(j5_pos), max(j5_pos));
fprintf('保持J5负\t\t%d\t[%.1f°, %.1f°]\n', switches_neg, min(j5_neg), max(j5_neg));
fprintf('自适应偏移\t\t%d\t[%.1f°, %.1f°]\n', adaptive_switches, min(optimal_q_adaptive(:,5))*180/pi, max(optimal_q_adaptive(:,5))*180/pi);

% 选择最优方案
all_results = [orig_switches, best_switches, switches_pos, switches_neg, adaptive_switches];
[min_switches, best_method] = min(all_results);
method_names = {'原始', '全局偏移', '保持J5正', '保持J5负', '自适应'};

fprintf('\n最优方案: %s, 奇异点: %d\n', method_names{best_method}, min_switches);

%% ======================== 输出实现代码 ==================================
if min_switches < orig_switches
    fprintf('\n========================================================\n');
    fprintf('   实现方案\n');
    fprintf('========================================================\n');
    
    if best_method == 2  % 全局偏移
        fprintf('\n【方案：全局RZ偏移】\n');
        fprintf('在YZ_AMV1.m中修改末端姿态计算:\n\n');
        fprintf('原代码 (第80行):\n');
        fprintf('  E1 = transl(...) * troty(pi) * trotx(rx) * troty(ry);\n\n');
        fprintf('修改为:\n');
        fprintf('  E1 = transl(...) * troty(pi) * trotx(rx) * troty(ry) * trotz(%.6f);\n', best_delta*pi/180);
        fprintf('  %%添加绕Z轴旋转%.0f°\n', best_delta);
        
    elseif best_method == 3  % 保持J5正
        fprintf('\n【方案：RZ偏移保持J5正值】\n');
        fprintf('偏移角度: +%.0f°\n', needed_offset_for_positive);
        fprintf('\n修改代码:\n');
        fprintf('  E1 = transl(...) * troty(pi) * trotx(rx) * troty(ry) * trotz(%.6f);\n', needed_offset_for_positive*pi/180);
        
    elseif best_method == 4  % 保持J5负
        fprintf('\n【方案：RZ偏移保持J5负值】\n');
        fprintf('偏移角度: %.0f°\n', needed_offset_for_negative);
        fprintf('\n修改代码:\n');
        fprintf('  E1 = transl(...) * troty(pi) * trotx(rx) * troty(ry) * trotz(%.6f);\n', needed_offset_for_negative*pi/180);
        
    elseif best_method == 5  % 自适应
        fprintf('\n【方案：自适应RZ偏移】\n');
        fprintf('每个点使用不同的RZ偏移\n');
        fprintf('RZ偏移保存在变量 optimal_rz 中\n');
        fprintf('可以将其添加到Excel文件的新列中\n');
    end
else
    fprintf('\n⚠ RZ偏移无法减少奇异点\n');
    fprintf('可能的原因: J5的变化范围太大，无法通过RZ偏移完全避开0\n');
end

%% ======================== 可视化 ========================================
figure('Name', 'RZ偏移优化效果', 'Position', [50, 50, 1400, 800]);

% J5对比
subplot(2, 3, 1);
hold on;
plot(1:n, j5_orig, 'r-', 'LineWidth', 1, 'DisplayName', '原始');
if best_switches < orig_switches
    plot(1:n, best_q(:,5)*180/pi, 'b-', 'LineWidth', 1, 'DisplayName', sprintf('全局偏移(%+.0f°)', best_delta));
end
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('点索引');
ylabel('J5 (°)');
title('J5角度对比');
legend('Location', 'best');
grid on;

% 偏移角度vs奇异点
subplot(2, 3, 2);
plot(results(:,1), results(:,2), 'bo-', 'LineWidth', 1.5);
hold on;
scatter(best_delta, best_switches, 100, 'r', 'filled');
xlabel('RZ偏移 (°)');
ylabel('奇异点数');
title('全局RZ偏移 vs 奇异点数');
grid on;

% J5范围vs偏移
subplot(2, 3, 3);
hold on;
plot(results(:,1), results(:,3), 'b-', 'LineWidth', 1, 'DisplayName', 'J5 min');
plot(results(:,1), results(:,4), 'r-', 'LineWidth', 1, 'DisplayName', 'J5 max');
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('RZ偏移 (°)');
ylabel('J5 (°)');
title('RZ偏移 vs J5范围');
legend('Location', 'best');
grid on;

% 自适应RZ偏移
subplot(2, 3, 4);
plot(1:n, optimal_rz, 'g-', 'LineWidth', 1);
xlabel('点索引');
ylabel('RZ偏移 (°)');
title(sprintf('自适应RZ偏移 (奇异点=%d)', adaptive_switches));
grid on;

% cfx对比
subplot(2, 3, 5);
hold on;
plot(1:n, orig_cfx, 'r-', 'LineWidth', 1, 'DisplayName', '原始');
plot(1:n, optimal_cfx_adaptive + 0.1, 'g-', 'LineWidth', 1, 'DisplayName', '自适应');
xlabel('点索引');
ylabel('cfx');
title('cfx轨迹对比');
legend('Location', 'best');
ylim([-0.5, 1.5]);
grid on;

% 结果柱状图
subplot(2, 3, 6);
bar([orig_switches, best_switches, switches_pos, switches_neg, adaptive_switches]);
set(gca, 'XTickLabel', {'原始', '全局偏移', 'J5正', 'J5负', '自适应'});
ylabel('奇异点数');
title('各方案奇异点对比');
grid on;

sgtitle('RZ偏移优化效果');

%% ========================================================================
%  辅助函数
%% ========================================================================

function [q_traj, cfx_traj, switch_count, switch_idx] = computeTrajectory(YM, tool_params, ql1, n, firstl, base_x, x_c, y_c, z_c, rz_offset)
    q_traj = zeros(n, 6);
    cfx_traj = zeros(n, 1);
    switch_count = 0;
    switch_idx = [];
    ql = ql1;
    prev_cfx = [];
    
    for i = 1:n
        target_x = base_x + (YM(i,1) + x_c) / 1000.0;
        target_y = (YM(i,2) + y_c) / 1000.0;
        target_z = firstl + (YM(i,3) + z_c) / 1000.0;
        if size(YM, 2) >= 5
            rx = YM(i, 4) * pi / 180;
            ry = YM(i, 5) * pi / 180;
        else
            rx = 0; ry = 0;
        end
        
        % 添加RZ偏移
        T = transl_m(target_x, target_y, target_z) * troty_m(pi) * trotx_m(rx) * troty_m(ry) * trotz_m(rz_offset(i));
        Q = ikabb_m(T, tool_params(1), tool_params(2), tool_params(3), tool_params(4));
        
        try
            qi = ikbest_m(ql, Q);
        catch
            qi = ql;
        end
        
        q_traj(i,:) = qi;
        rcf = rscfx_m(qi);
        cfx_traj(i) = rcf(4);
        
        if ~isempty(prev_cfx) && rcf(4) ~= prev_cfx
            switch_count = switch_count + 1;
            switch_idx = [switch_idx; i];
        end
        
        ql = qi;
        prev_cfx = rcf(4);
    end
end

function valid = isValidConfig(q)
    q_min = [-17*pi/18, -10*pi/18, -20*pi/18, -27*pi/18, -17*pi/18, -13*pi/18];
    q_max = [17*pi/18, 13*pi/18, 7*pi/18, 27*pi/18, 17*pi/18, 13*pi/18];
    valid = true;
    for i = 1:6
        if q(i) < q_min(i) || q(i) > q_max(i) || isnan(q(i)) || isinf(q(i))
            valid = false;
            return;
        end
    end
end

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

function T = trotz_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4); T(1:3, 1:3) = [c, -s, 0; s, c, 0; 0, 0, 1];
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