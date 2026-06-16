%% ========================================================================
%  ABB IRB-1200 奇异点分析系统 V2 - 区分配置切换与真正奇异点
%  ========================================================================
%  
%  【两个不同概念】
%  
%  1. 配置切换 (Configuration Change) - YZ_AMV1.m中sing=1检测的
%     - 条件: J5从正变负，或从负变正
%     - 检测: sign(J5_i) ≠ sign(J5_{i-1})
%     - 后果: MoveL插值方向不确定
%     - 处理: 使用MoveAbsJ
%  
%  2. 真正奇异点 (True Singularity) - 基于雅可比矩阵
%     - 条件: sin(θ5) = 0, 即 θ5 = 0° 或 180°
%     - 检测: det(J) = 0 或 |sin(θ5)| < ε
%     - 后果: 关节速度 → ∞
%     - 处理: 必须避开
%  
%  ========================================================================

clear; clc; close all;

%% ======================== 用户配置 ======================================
EXCEL_FILE = 'path_23_1.xlsx';

% 真正奇异点阈值（严格）
TRUE_SING_THRESH = 1;           % 度，|θ5| < 1° 认为是真正奇异点

% 配置切换检测（这是YZ_AMV1.m做的事情）
% 配置切换发生在J5符号改变时

% 近奇异警告阈值
NEAR_SING_THRESH = 5;           % 度，|θ5| < 5° 认为接近奇异

%% ======================== IRB-1200 参数 =================================
d1 = 0.3991; a2 = 0.448; a3 = 0.042; d4 = 0.451; d6 = 0.082;

firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;
tool_hc = -50.7321e-3; tool_yc = -50.0753e-3; tool_lc = 70.963e-3; tool_ac = 30/180*pi;
tool_hr = -50.3541e-3; tool_yr = 50.9039e-3; tool_lr = 71.4291e-3; tool_ar = -30/180*pi;
x_comp = -0.5; y_comp = -1.5; z_comp = 0.2;
base_x_offset = 0.55;
ql1_default = [0, 0, pi/6.0, 0, 0, 0];

%% ======================== 读取数据 ======================================
fprintf('================================================================\n');
fprintf('   奇异点分析系统 V2 - 区分配置切换与真正奇异点\n');
fprintf('================================================================\n\n');

try
    YM = xlsread(EXCEL_FILE);
catch
    error('无法读取文件: %s', EXCEL_FILE);
end
n = size(YM, 1);
fprintf('文件: %s, 点数: %d\n\n', EXCEL_FILE, n);

%% ======================== 分析原始轨迹 ==================================
fprintf('【步骤1】分析原始轨迹 (RZ = 0°)\n');
fprintf('------------------------------------------------------------\n');

[q_orig, info_orig] = analyzeAll(YM, n, ql1_default, 0, ...
    firstl, base_x_offset, x_comp, y_comp, z_comp, ...
    tool_lc, tool_hc, tool_yc, tool_ac, tool_lr, tool_hr, tool_yr, tool_ar, ...
    d1, a2, a3, d4, d6, TRUE_SING_THRESH, NEAR_SING_THRESH);

printAnalysis(info_orig, '原始', 0);

%% ======================== 搜索最优RZ偏移 ================================
fprintf('\n【步骤2】搜索最优RZ偏移\n');
fprintf('------------------------------------------------------------\n');

delta_range = -90:5:90;
results = zeros(length(delta_range), 7);

for idx = 1:length(delta_range)
    delta_deg = delta_range(idx);
    [q_test, info_test] = analyzeAll(YM, n, ql1_default, delta_deg*pi/180, ...
        firstl, base_x_offset, x_comp, y_comp, z_comp, ...
        tool_lc, tool_hc, tool_yc, tool_ac, tool_lr, tool_hr, tool_yr, tool_ar, ...
        d1, a2, a3, d4, d6, TRUE_SING_THRESH, NEAR_SING_THRESH);
    
    results(idx, :) = [delta_deg, ...
                       info_test.config_changes, ...      % 配置切换数
                       info_test.true_singular, ...       % 真正奇异点数
                       info_test.near_singular, ...       % 近奇异点数
                       info_test.j5_min, ...
                       info_test.j5_max, ...
                       info_test.min_abs_j5];             % 最小|J5|
end

%% ======================== 选择最优方案 ==================================
% 优先级: 1.真正奇异点最少 2.配置切换最少 3.min|J5|最大
[~, sorted_idx] = sortrows(results, [3, 2, -7]);
best_idx = sorted_idx(1);
best_delta = results(best_idx, 1);

fprintf('\n搜索结果:\n');
fprintf('RZ(°)  配置切换  真奇异  近奇异   J5min    J5max   min|J5|\n');
fprintf('-----  --------  ------  ------  ------   ------  -------\n');

for idx = 1:length(delta_range)
    mark = '';
    if results(idx, 1) == best_delta, mark = ' ← 推荐'; end
    if results(idx, 1) == 0, mark = [mark ' (原始)']; end
    
    % 只显示配置切换较少的方案
    if results(idx, 2) <= 15
        fprintf('%+4d      %2d        %2d      %3d   %+6.1f   %+6.1f    %5.1f%s\n', ...
            results(idx, 1), results(idx, 2), results(idx, 3), ...
            results(idx, 4), results(idx, 5), results(idx, 6), ...
            results(idx, 7), mark);
    end
end

%% ======================== 分析最优方案 ==================================
fprintf('\n【步骤3】分析推荐方案 (RZ = %+d°)\n', best_delta);
fprintf('------------------------------------------------------------\n');

[q_opt, info_opt] = analyzeAll(YM, n, ql1_default, best_delta*pi/180, ...
    firstl, base_x_offset, x_comp, y_comp, z_comp, ...
    tool_lc, tool_hc, tool_yc, tool_ac, tool_lr, tool_hr, tool_yr, tool_ar, ...
    d1, a2, a3, d4, d6, TRUE_SING_THRESH, NEAR_SING_THRESH);

printAnalysis(info_opt, '推荐', best_delta);

%% ======================== 对比总结 ======================================
fprintf('\n【步骤4】对比总结\n');
fprintf('============================================================\n');
fprintf('                        原始(RZ=0°)    推荐(RZ=%+d°)   改善\n', best_delta);
fprintf('------------------------------------------------------------\n');
fprintf('配置切换(需MoveAbsJ)      %2d              %2d          %+d\n', ...
    info_orig.config_changes, info_opt.config_changes, ...
    info_opt.config_changes - info_orig.config_changes);
fprintf('真正奇异点(J5≈0°/180°)    %2d              %2d          %+d\n', ...
    info_orig.true_singular, info_opt.true_singular, ...
    info_opt.true_singular - info_orig.true_singular);
fprintf('近奇异点(|J5|<%d°)       %3d             %3d         %+d\n', ...
    NEAR_SING_THRESH, info_orig.near_singular, info_opt.near_singular, ...
    info_opt.near_singular - info_orig.near_singular);
fprintf('J5范围(°)              [%+.1f,%+.1f]     [%+.1f,%+.1f]\n', ...
    info_orig.j5_min, info_orig.j5_max, info_opt.j5_min, info_opt.j5_max);
fprintf('min|J5|(°)               %5.1f           %5.1f\n', ...
    info_orig.min_abs_j5, info_opt.min_abs_j5);
fprintf('============================================================\n');

%% ======================== 生成图表 ======================================
fprintf('\n【步骤5】生成分析图\n');

figure('Position', [50, 50, 1600, 900], 'Color', 'white');

% 图1: J5轨迹对比（核心图）
subplot(2, 3, 1);
hold on;
% 奇异区域
fill([1 n n 1], [-TRUE_SING_THRESH -TRUE_SING_THRESH TRUE_SING_THRESH TRUE_SING_THRESH], ...
    [1 0.8 0.8], 'EdgeColor', 'none', 'DisplayName', sprintf('真奇异区(±%d°)', TRUE_SING_THRESH));
fill([1 n n 1], [-NEAR_SING_THRESH -NEAR_SING_THRESH -TRUE_SING_THRESH -TRUE_SING_THRESH], ...
    [1 0.9 0.9], 'EdgeColor', 'none', 'DisplayName', sprintf('近奇异区(±%d°)', NEAR_SING_THRESH));
fill([1 n n 1], [TRUE_SING_THRESH TRUE_SING_THRESH NEAR_SING_THRESH NEAR_SING_THRESH], ...
    [1 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
% 轨迹
plot(1:n, q_orig(:,5)*180/pi, 'r-', 'LineWidth', 1.5, 'DisplayName', '原始');
plot(1:n, q_opt(:,5)*180/pi, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('RZ%+d°', best_delta));
yline(0, 'k--', 'LineWidth', 1);
xlabel('点索引'); ylabel('J5 ');
title('J5轨迹与奇异区域');
legend('Location', 'best'); grid on;
ylim([min([q_orig(:,5);q_opt(:,5)])*180/pi-10, max([q_orig(:,5);q_opt(:,5)])*180/pi+10]);

% 图2: |sin(θ5)| - 腕部奇异因子
subplot(2, 3, 2);
plot(1:n, abs(sin(q_orig(:,5))), 'r-', 'LineWidth', 1.2); hold on;
plot(1:n, abs(sin(q_opt(:,5))), 'b-', 'LineWidth', 1.2);
yline(sin(TRUE_SING_THRESH*pi/180), 'r--', 'LineWidth', 1.5);
yline(sin(NEAR_SING_THRESH*pi/180), 'm:', 'LineWidth', 1);
xlabel('点索引'); ylabel('|sin(θ5)|');
title('腕部奇异因子 (越小=越接近奇异)');
legend('原始', sprintf('RZ%+d°', best_delta), '真奇异阈值', '近奇异阈值', 'Location', 'best');
grid on;

% 图3: 配置切换点标记
subplot(2, 3, 3);
plot(1:n, q_orig(:,5)*180/pi, 'r-', 'LineWidth', 1); hold on;
% 标记配置切换点
if ~isempty(info_orig.config_change_points)
    scatter(info_orig.config_change_points, q_orig(info_orig.config_change_points,5)*180/pi, ...
        50, 'ro', 'filled', 'DisplayName', '原始配置切换');
end
plot(1:n, q_opt(:,5)*180/pi, 'b-', 'LineWidth', 1);
if ~isempty(info_opt.config_change_points)
    scatter(info_opt.config_change_points, q_opt(info_opt.config_change_points,5)*180/pi, ...
        50, 'bs', 'filled', 'DisplayName', '优化后配置切换');
end
yline(0, 'k--', 'LineWidth', 2);
xlabel('点索引'); ylabel('J5 ');
title('配置切换点 (J5穿过0)');
legend('Location', 'best'); grid on;

% 图4: J3轨迹
subplot(2, 3, 4);
plot(1:n, q_orig(:,3)*180/pi, 'r-', 'LineWidth', 1.2); hold on;
plot(1:n, q_opt(:,3)*180/pi, 'b-', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 1);
yline(180, 'k--', 'LineWidth', 1);
yline(-180, 'k--', 'LineWidth', 1);
xlabel('点索引'); ylabel('J3 (°)');
title('J3轨迹 (0°或±180°=肘部奇异)');
legend('原始', sprintf('RZ%+d°', best_delta), 'Location', 'best');
grid on;

% 图5: 优化后关节角度
subplot(2, 3, 5);
colors = lines(6);
for j = 1:6
    plot(1:n, q_opt(:,j)*180/pi, '-', 'Color', colors(j,:), 'LineWidth', 1);
    hold on;
end
xlabel('点索引'); ylabel('角度 (°)');
title(sprintf('优化后关节角度 (RZ=%+d°)', best_delta));
legend({'J1','J2','J3','J4','J5','J6'}, 'Location', 'best');
grid on;

% 图6: RZ偏移效果对比
subplot(2, 3, 6);
yyaxis left;
bar(results(:, 1), results(:, 2), 'FaceColor', [0.7 0.7 0.9], 'EdgeColor', 'none');
ylabel('配置切换数 (需MoveAbsJ)');
yyaxis right;
plot(results(:, 1), results(:, 7), 'g-o', 'LineWidth', 2, 'MarkerSize', 4);
ylabel('min|J5| (°) - 越大越安全');
xlabel('RZ偏移 (°)');
title('RZ偏移效果');
xline(best_delta, 'r--', 'LineWidth', 2);
xline(0, 'k:', 'LineWidth', 1);
legend('配置切换', 'min|J5|', '推荐', '原始', 'Location', 'best');
grid on;

sgtitle(sprintf('奇异点分析: %s (配置切换 vs 真正奇异)', EXCEL_FILE), 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'Singularity_Analysis_V2.png');
fprintf('  保存: Singularity_Analysis_V2.png\n');

%% ======================== 输出修改建议 ==================================
fprintf('\n================================================================\n');
fprintf('   理解与建议\n');
fprintf('================================================================\n\n');

fprintf('【概念澄清】\n\n');
fprintf('  您的YZ_AMV1.m中 sing=1 检测的是"配置切换"，不是"真正奇异点"！\n');
fprintf('  这两者是不同的概念：\n\n');
fprintf('  ┌────────────────┬────────────────────┬────────────────────┐\n');
fprintf('  │                │   配置切换          │   真正奇异点        │\n');
fprintf('  ├────────────────┼────────────────────┼────────────────────┤\n');
fprintf('  │ 条件           │ J5符号改变          │ J5=0°或180°        │\n');
fprintf('  │                │ (穿过0°)            │ (等于0°或180°)     │\n');
fprintf('  ├────────────────┼────────────────────┼────────────────────┤\n');
fprintf('  │ 检测方法       │ sign(J5)变化        │ sin(J5)≈0          │\n');
fprintf('  ├────────────────┼────────────────────┼────────────────────┤\n');
fprintf('  │ 危险程度       │ 低(插值方向不确定)  │ 高(关节速度→∞)     │\n');
fprintf('  ├────────────────┼────────────────────┼────────────────────┤\n');
fprintf('  │ 处理方法       │ 使用MoveAbsJ        │ 必须避开           │\n');
fprintf('  ├────────────────┼────────────────────┼────────────────────┤\n');
fprintf('  │ YZ_AMV1.m      │ sing=1 ✓检测到     │ 未检测 ✗           │\n');
fprintf('  └────────────────┴────────────────────┴────────────────────┘\n\n');

fprintf('【您的轨迹分析】\n\n');
fprintf('  原始轨迹(RZ=0°):\n');
fprintf('    - J5范围: [%+.1f°, %+.1f°]，穿过0° %d次\n', ...
    info_orig.j5_min, info_orig.j5_max, info_orig.config_changes);
fprintf('    - 最小|J5|: %.1f° %s\n', info_orig.min_abs_j5, ...
    ternary(info_orig.min_abs_j5 < TRUE_SING_THRESH, '← 真正接近奇异！', ...
    ternary(info_orig.min_abs_j5 < NEAR_SING_THRESH, '← 接近奇异区', '← 安全')));
fprintf('\n');
fprintf('  推荐方案(RZ=%+d°):\n', best_delta);
fprintf('    - J5范围: [%+.1f°, %+.1f°]，穿过0° %d次\n', ...
    info_opt.j5_min, info_opt.j5_max, info_opt.config_changes);
fprintf('    - 最小|J5|: %.1f° %s\n', info_opt.min_abs_j5, ...
    ternary(info_opt.min_abs_j5 < TRUE_SING_THRESH, '← 真正接近奇异！', ...
    ternary(info_opt.min_abs_j5 < NEAR_SING_THRESH, '← 接近奇异区', '← 安全')));

fprintf('\n【建议】\n\n');
if info_opt.config_changes < info_orig.config_changes
    fprintf('  ✓ RZ=%+d° 可以减少配置切换 (%d→%d)\n', best_delta, ...
        info_orig.config_changes, info_opt.config_changes);
    fprintf('    修改方法: 在工具变换后添加 trotz(%.6f)\n', best_delta*pi/180);
else
    fprintf('  ✗ 没有找到能显著减少配置切换的RZ偏移\n');
    fprintf('    这条路径可能本身就需要多次配置切换\n');
end

fprintf('\n================================================================\n');

%% ========================================================================
%  函数定义
%% ========================================================================

function [q_traj, info] = analyzeAll(YM, n, ql1, rz_offset, ...
    firstl, base_x, x_c, y_c, z_c, ...
    tool_lc, tool_hc, tool_yc, tool_ac, tool_lr, tool_hr, tool_yr, tool_ar, ...
    d1, a2, a3, d4, d6, true_thresh, near_thresh)
    
    q_traj = zeros(n, 6);
    ql = ql1;
    
    for i = 1:n
        if size(YM, 2) >= 8 && YM(i, 8) == 222
            tool_l = tool_lr; tool_h = tool_hr; tool_y = tool_yr; tool_a = tool_ar;
            x_off = 0; y_off = 0; z_off = 0;
        else
            tool_l = tool_lc; tool_h = tool_hc; tool_y = tool_yc; tool_a = tool_ac;
            x_off = x_c; y_off = y_c; z_off = z_c;
        end
        
        px = base_x + (YM(i,1) + x_off) / 1000.0;
        py = (YM(i,2) + y_off) / 1000.0;
        pz = firstl + (YM(i,3) + z_off) / 1000.0;
        
        if size(YM, 2) >= 5
            rx = YM(i, 4) * pi / 180;
            ry = YM(i, 5) * pi / 180;
        else
            rx = 0; ry = 0;
        end
        
        T = transl_m(px, py, pz) * troty_m(pi) * trotx_m(rx) * troty_m(ry) * trotz_m(rz_offset);
        Q = ikabb_m(T, tool_l, tool_h, tool_y, tool_a);
        
        try
            qi = ikbest_m(ql, Q);
        catch
            qi = ql;
        end
        
        q_traj(i, :) = qi;
        ql = qi;
    end
    
    % 分析结果
    j5_deg = q_traj(:, 5) * 180 / pi;
    
    info = struct();
    info.j5_min = min(j5_deg);
    info.j5_max = max(j5_deg);
    info.min_abs_j5 = min(abs(j5_deg));
    
    % 配置切换检测（J5符号变化）
    j5_sign = sign(j5_deg);
    j5_sign(j5_sign == 0) = 1;  % 0归为正
    sign_changes = find(diff(j5_sign) ~= 0) + 1;
    info.config_changes = length(sign_changes);
    info.config_change_points = sign_changes;
    
    % 真正奇异点检测（|J5| < 阈值）
    true_sing = find(abs(j5_deg) < true_thresh);
    info.true_singular = length(true_sing);
    info.true_singular_points = true_sing;
    
    % 近奇异点检测
    near_sing = find(abs(j5_deg) >= true_thresh & abs(j5_deg) < near_thresh);
    info.near_singular = length(near_sing);
end

function printAnalysis(info, name, rz_deg)
    fprintf('\n--- %s配置 (RZ=%+d°) ---\n', name, rz_deg);
    fprintf('J5范围: [%.1f°, %.1f°]\n', info.j5_min, info.j5_max);
    fprintf('min|J5|: %.2f°\n', info.min_abs_j5);
    fprintf('配置切换(穿过0°): %d 次\n', info.config_changes);
    fprintf('真正奇异点(|J5|<1°): %d 个\n', info.true_singular);
    fprintf('近奇异点(1°≤|J5|<5°): %d 个\n', info.near_singular);
end

function result = ternary(cond, true_val, false_val)
    if cond
        result = true_val;
    else
        result = false_val;
    end
end

%% 变换函数
function T = transl_m(x, y, z)
    T = eye(4); T(1:3, 4) = [x; y; z];
end

function T = trotx_m(a)
    c = cos(a); s = sin(a);
    T = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function T = troty_m(a)
    c = cos(a); s = sin(a);
    T = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end

function T = trotz_m(a)
    c = cos(a); s = sin(a);
    T = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end

%% 逆运动学函数
function Q = ikabb_m(To, tool_l, tool_h, tool_y, tool_a)
    T = To * trotx_m(-tool_a) * transl_m(-tool_h, -tool_y, -tool_l-0.082);
    T(3,4) = T(3,4) - 0.3991;
    
    a2 = 0.448; a3 = 0.042; d4 = 0.451;
    k = ((T(1,4)^2 + T(2,4)^2 + T(3,4)^2) - a2^2 - a3^2 - d4^2) / (2*a2);
    
    nx = T(1,1); ny = T(2,1); nz = T(3,1);
    ox = T(1,2); oy = T(2,2); oz = T(3,2);
    ax = T(1,3); ay = T(2,3); az = T(3,3);
    
    Q = zeros(8, 6);
    signs = [1,1,1; 1,1,-1; 1,-1,1; 1,-1,-1; -1,1,1; -1,1,-1; -1,-1,1; -1,-1,-1];
    
    for idx = 1:8
        Q(idx,:) = ik_solve_m(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, ...
            signs(idx,1), signs(idx,2), signs(idx,3));
    end
end

function ik = ik_solve_m(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, s2, s3, s5)
    t3 = atan2(a3, d4) - atan2(k, s3*sqrt(max(0, a3^2 + d4^2 - k^2)));
    
    u1 = a3*cos(t3) - d4*sin(t3) + a2;
    u2 = a3*sin(t3) + d4*cos(t3);
    t2 = atan2(-T(3,4), s2*sqrt(max(0, u1^2 + u2^2 - T(3,4)^2))) - atan2(u2, u1);
    
    v1 = cos(t2)*u1 - sin(t2)*u2;
    if v1 > 0
        t1 = atan2(T(2,4), T(1,4));
    elseif v1 < 0
        t1 = atan2(-T(2,4), -T(1,4));
    else
        t1 = 0;
    end
    
    c1 = cos(t1); s1 = sin(t1);
    c23 = cos(t2+t3); s23 = sin(t2+t3);
    
    c5 = max(-1, min(1, -ax*c1*s23 - ay*s1*s23 - az*c23));
    t5 = atan2(s5*sqrt(max(0, 1-c5^2)), c5);
    
    ss5 = sin(t5);
    if abs(ss5) > 1e-10
        if ss5 > 0
            t4 = atan2(-ax*s1+ay*c1, -ax*c1*c23-ay*s1*c23+az*s23);
            t6 = atan2(-ox*c1*s23-oy*s1*s23-oz*c23, nx*c1*s23+ny*s1*s23+nz*c23);
        else
            t4 = atan2(ax*s1-ay*c1, ax*c1*c23+ay*s1*c23-az*s23);
            t6 = atan2(ox*c1*s23+oy*s1*s23+oz*c23, -nx*c1*s23-ny*s1*s23-nz*c23);
        end
    else
        t4 = 0;
        t6 = 0;
    end
    
    t2 = t2 + pi/2;
    ik = [t1, t2, t3, t4, t5, t6];
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
        ok = 1;
        
        if Q(b,1) > 17*pi/18 || Q(b,1) < -17*pi/18, ok = 0; end
        if Q(b,2) > 13*pi/18 || Q(b,2) < -10*pi/18, ok = 0; end
        if Q(b,3) > 7*pi/18 || Q(b,3) < -20*pi/18, ok = 0; end
        if Q(b,4) > 27*pi/18 || Q(b,4) < -27*pi/18, ok = 0; end
        if Q(b,5) > 17*pi/18 || Q(b,5) < -17*pi/18, ok = 0; end
        if Q(b,6) > 13*pi/18 || Q(b,6) < -13*pi/18, ok = 0; end
        if any(isnan(Q(b,:))) || any(isinf(Q(b,:))), ok = 0; end
        
        if ok == 1
            qi = Q(b,:);
            return;
        elseif i <= 8
            s(b) = 10000;
        else
            error('No valid IK solution!');
        end
    end
end