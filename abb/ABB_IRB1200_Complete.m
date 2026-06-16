%% ========================================================================
%  ABB IRB-1200 奇异点检测 - 修正版 (只检测rcf(4)变化)
%  ========================================================================
%  关键修正: 只有 rcf(4) (cfx) 变化时才是奇异点!
%  这与YZ_AMV1.m第117行和第125行的逻辑完全一致:
%    if rob1{i, j}.rcf(4) ~= rob1{i, j-1}.rcf(4)
%        rob1{i, j}.sing = 1;
%  ========================================================================

clear; clc; close all;

%% ======================== 参数设置 (与YZ_AMV1.m完全一致) ================
firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;  % m

% 碳纤维工具参数 (111)
tool_hc = -50.7321e-3;
tool_yc = -50.0753e-3;
tool_lc = 70.963e-3;
tool_ac = 30/180*pi;

% 树脂工具参数 (222)
tool_hr = -50.3541e-3;
tool_yr = 50.9039e-3;
tool_lr = 71.4291e-3;
tool_ar = -30/180*pi;

% 纤维头补偿值 (mm)
x_comp = -0.5;
y_comp = -1.5;
z_comp = 0.2;

% 初始关节角度
ql1_init = [0, 0, pi/6.0, 0, 0, 0];

% 基座X偏移
base_x_offset = 0.55;  % m

%% ======================== 读取路径 ======================================
excel_file = 'path_23_2.xlsx';

fprintf('========================================================\n');
fprintf('   ABB IRB-1200 奇异点检测 (修正版)\n');
fprintf('   只检测 rcf(4)/cfx 变化\n');
fprintf('========================================================\n\n');

if exist(excel_file, 'file')
    YM = xlsread(excel_file);
    fprintf('读取路径: %s\n', excel_file);
    fprintf('  点数: %d\n', size(YM, 1));
else
    error('未找到文件: %s', excel_file);
end

%% ======================== 分析路径 ======================================
fprintf('\n分析路径中...\n');

n = size(YM, 1);
ql = ql1_init;

% 存储结果
results.q = zeros(n, 6);
results.rcf = zeros(n, 4);
results.is_singular = false(n, 1);
results.end_pos = zeros(n, 3);

prev_cfx = [];  % 只跟踪cfx (rcf的第4个分量)
singular_count = 0;
singular_indices = [];

% 第一个点总是奇异点 (与YZ_AMV1.m第113行一致: rob1{1,1}.sing = 1)
first_point_is_singular = true;

for i = 1:n
    % 工具类型
    if size(YM, 2) >= 8
        tool_type = YM(i, 8);
    else
        tool_type = 111;
    end
    
    % 选择工具参数
    if tool_type == 111
        tool_l = tool_lc; tool_h = tool_hc;
        tool_y = tool_yc; tool_a = tool_ac;
        x_off = x_comp; y_off = y_comp; z_off = z_comp;
    else
        tool_l = tool_lr; tool_h = tool_hr;
        tool_y = tool_yr; tool_a = tool_ar;
        x_off = 0; y_off = 0; z_off = 0;
    end
    
    % 计算末端变换矩阵
    target_x = base_x_offset + (YM(i,1) + x_off) / 1000.0;
    target_y = (YM(i,2) + y_off) / 1000.0;
    target_z = firstl + (YM(i,3) + z_off) / 1000.0;
    
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0; ry = 0;
    end
    
    T_target = transl_m(target_x, target_y, target_z) * ...
               troty_m(pi) * trotx_m(rx) * troty_m(ry);
    
    % 逆运动学
    Q = ikabb_m(T_target, tool_l, tool_h, tool_y, tool_a);
    
    try
        qi = ikbest_m(ql, Q);
    catch
        qi = ql;
        fprintf('  ⚠ 点 %d: 无有效解\n', i);
    end
    
    % 计算轴配置
    rcf = rscfx_m(qi);
    
    % 存储结果
    results.q(i, :) = qi;
    results.rcf(i, :) = rcf;
    results.end_pos(i, :) = [target_x, target_y, target_z] * 1000;
    
    % ========== 关键修正: 只检测 rcf(4) 变化 ==========
    is_sing = false;
    
    if i == 1
        % 第一个点不标记（在YZ_AMV1中，第一个点是初始位置，单独处理）
        % 但路径中第一个点本身不会被标记为奇异
    else
        % 只检测 rcf(4) 即 cfx 的变化
        if rcf(4) ~= prev_cfx
            is_sing = true;
        end
    end
    
    results.is_singular(i) = is_sing;
    
    if is_sing
        singular_count = singular_count + 1;
        singular_indices = [singular_indices; i];
    end
    
    % 更新
    ql = qi;
    prev_cfx = rcf(4);
    
    if mod(i, 500) == 0
        fprintf('  进度: %d/%d, 奇异点: %d\n', i, n, singular_count);
    end
end

results.n_singularities = singular_count;
results.singular_indices = singular_indices;

%% ======================== 输出结果 ======================================
fprintf('\n========================================================\n');
fprintf('   分析结果\n');
fprintf('========================================================\n');
fprintf('总点数: %d\n', n);
fprintf('奇异点数量 (rcf(4)变化): %d\n', singular_count);

% 额外统计：所有rcf变化
all_rcf_changes = 0;
for i = 2:n
    if any(results.rcf(i,:) ~= results.rcf(i-1,:))
        all_rcf_changes = all_rcf_changes + 1;
    end
end
fprintf('参考: 所有rcf变化次数: %d\n', all_rcf_changes);

if singular_count > 0
    fprintf('\n奇异点详情 (rcf(4)/cfx 变化):\n');
    fprintf('索引\t\tcfx值\t\t前一个cfx\n');
    fprintf('----\t\t-----\t\t--------\n');
    for i = 1:min(20, singular_count)
        idx = singular_indices(i);
        if idx > 1
            fprintf('%d\t\t%d\t\t%d\n', idx, results.rcf(idx,4), results.rcf(idx-1,4));
        end
    end
    
    if singular_count > 20
        fprintf('... 还有 %d 个奇异点\n', singular_count - 20);
    end
end

%% ======================== 与.mod文件对比 ================================
fprintf('\n========================================================\n');
fprintf('   验证\n');
fprintf('========================================================\n');
fprintf('本脚本检测到的奇异点 (rcf(4)变化): %d\n', singular_count);
fprintf('请与.mod文件中的MoveAbsJ数量对比\n');
fprintf('\n注意: .mod文件中第一个点总是用MoveAbsJ (初始位置)\n');
fprintf('所以 MoveAbsJ总数 = 1(初始) + %d(路径中奇异点) = %d\n', singular_count, singular_count + 1);

%% ======================== 可视化 ========================================
figure('Name', '奇异点分析 (只检测cfx变化)', 'Position', [50, 50, 1400, 600]);

% 关节角度
subplot(2, 3, 1);
q_deg = results.q * 180 / pi;
plot(1:n, q_deg);
hold on;
for idx = singular_indices'
    xline(idx, 'r:', 'Alpha', 0.5);
end
xlabel('点索引');
ylabel('关节角度 (°)');
title('关节轨迹 (红线=奇异点)');
legend('J1','J2','J3','J4','J5','J6', 'Location', 'eastoutside');
grid on;

% cfx值变化
subplot(2, 3, 2);
plot(1:n, results.rcf(:,4), 'b-', 'LineWidth', 1.5);
hold on;
if ~isempty(singular_indices)
    scatter(singular_indices, results.rcf(singular_indices, 4), 50, 'r', 'filled');
end
xlabel('点索引');
ylabel('cfx (rcf(4))');
title(sprintf('cfx变化 = 奇异点 (共%d个)', singular_count));
grid on;

% cfx计算的三个分量
subplot(2, 3, 3);
hold on;
cfx_a = 448*sin(results.q(:,2)) + 42*sin(results.q(:,2)+results.q(:,3)) + ...
        451*sin(pi/2-(results.q(:,2)+results.q(:,3)));
plot(1:n, cfx_a, 'r-', 'DisplayName', 'cfx\_a');
plot(1:n, results.q(:,3), 'g-', 'DisplayName', 'q3 (cfx\_b)');
plot(1:n, results.q(:,5), 'b-', 'DisplayName', 'q5 (cfx\_c)');
yline(0, 'k--');
yline(-1.459095, 'm--', 'DisplayName', '-1.459');
yline(-1.50971, 'c--', 'DisplayName', '-1.510');
xlabel('点索引');
ylabel('值');
title('cfx计算分量');
legend('Location', 'best');
grid on;

% 奇异点位置分布
subplot(2, 3, 4);
if ~isempty(singular_indices)
    stem(singular_indices, ones(size(singular_indices)), 'r', 'Marker', 'none');
end
xlabel('点索引');
ylabel('奇异点');
title(sprintf('奇异点分布 (共%d个)', singular_count));
xlim([1, n]);
grid on;

% 3D路径
subplot(2, 3, [5, 6]);
plot3(results.end_pos(:,1), results.end_pos(:,2), results.end_pos(:,3), ...
      'b-', 'LineWidth', 0.5);
hold on;
if ~isempty(singular_indices)
    scatter3(results.end_pos(singular_indices,1), ...
             results.end_pos(singular_indices,2), ...
             results.end_pos(singular_indices,3), ...
             50, 'r', 'filled');
end
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title(sprintf('3D路径 (红点=奇异点, 共%d个)', singular_count));
axis equal;
grid on;
view(135, 30);

sgtitle(sprintf('ABB IRB-1200 奇异点分析: 检测到 %d 个奇异点 (rcf(4)/cfx变化)', singular_count));

%% ========================================================================
%  辅助函数
%% ========================================================================

function T = transl_m(x, y, z)
    T = eye(4);
    T(1:3, 4) = [x; y; z];
end

function T = trotx_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4);
    T(1:3, 1:3) = [1, 0, 0; 0, c, -s; 0, s, c];
end

function T = troty_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4);
    T(1:3, 1:3) = [c, 0, s; 0, 1, 0; -s, 0, c];
end

%% ikabb (完全复制)
function Q = ikabb_m(To, tool_l, tool_h, tool_y, tool_a)
    T = To * trotx_m(-tool_a) * transl_m(-tool_h, -tool_y, -tool_l-0.082);
    T(3,4) = T(3,4) - 0.3991;
    
    a2 = 0.448;
    a3 = 0.042;
    d4 = 0.451;
    
    k = ((T(1,4)^2+T(2,4)^2+T(3,4)^2) - a2^2-a3^2-d4^2)/(2*a2);
    
    nx = T(1,1); ny = T(2,1); nz = T(3,1);
    ox = T(1,2); oy = T(2,2); oz = T(3,2);
    ax = T(1,3); ay = T(2,3); az = T(3,3);
    
    ik1 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, 1, 1, 1);
    ik2 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, 1, 1, -1);
    ik3 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, 1, -1, 1);
    ik4 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, 1, -1, -1);
    ik5 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, -1, 1, 1);
    ik6 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, -1, 1, -1);
    ik7 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, -1, -1, 1);
    ik8 = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, -1, -1, -1);
    
    Q = [ik1; ik2; ik3; ik4; ik5; ik6; ik7; ik8];
end

function ik = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, sign2, sign3, sign5)
    theta3 = atan2(a3, d4) - atan2(k, sign3*sqrt(a3^2+d4^2-k^2));
    
    u1 = a3*cos(theta3) - d4*sin(theta3) + a2;
    u2 = a3*sin(theta3) + d4*cos(theta3);
    theta2 = atan2(-T(3,4), sign2*sqrt(u1^2+u2^2-T(3,4)^2)) - atan2(u2, u1);
    
    v1 = cos(theta2)*u1 - sin(theta2)*u2;
    if v1 > 0
        theta1 = atan2(T(2,4), T(1,4));
    elseif v1 < 0
        theta1 = atan2(-T(2,4), -T(1,4));
    else
        theta1 = 0;
    end
    
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
    else
        theta4 = 0;
        theta6 = 0;
    end
    
    theta2 = theta2 + pi/2;
    ik = [theta1, theta2, theta3, theta4, theta5, theta6];
end

%% ikbest (完全复制)
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
        
        if Q(b,1) > (17*pi/18) || Q(b,1) < (-17*pi/18)
            output = 0;
        end
        if Q(b,2) > (13*pi/18) || Q(b,2) < (-10*pi/18)
            output = 0;
        end
        if Q(b,3) > (7*pi/18) || Q(b,3) < (-20*pi/18)
            output = 0;
        end
        if Q(b,4) > (27*pi/18) || Q(b,4) < (-27*pi/18)
            output = 0;
        end
        if Q(b,5) > (17*pi/18) || Q(b,5) < (-17*pi/18)
            output = 0;
        end
        if Q(b,6) > (13*pi/18) || Q(b,6) < (-13*pi/18)
            output = 0;
        end
        
        if output == 1
            qi = Q(b,:);
            return;
        elseif i <= 8
            s(b) = 10000;
        else
            error('The solution does not exist!');
        end
    end
end

%% rscfx (完全复制)
function cfx = rscfx_m(q)
    cfx = zeros(1, 4);
    d = 1e-14;
    
    % cf1
    if q(1) > d-pi && q(1) <= d-pi/2
        cfx(1) = -2;
    elseif q(1) > d-pi/2 && q(1) <= -d
        cfx(1) = -1;
    elseif q(1) > -d && q(1) <= d+pi/2
        cfx(1) = 0;
    elseif q(1) > d+pi/2 && q(1) <= d+pi
        cfx(1) = 1;
    end
    
    % cf4
    if q(4) > d-3*pi/2 && q(4) <= d-pi
        cfx(2) = -3;
    elseif q(4) > d-pi && q(4) <= d-pi/2
        cfx(2) = -2;
    elseif q(4) > d-pi/2 && q(4) <= -d
        cfx(2) = -1;
    elseif q(4) > -d && q(4) <= d+pi/2
        cfx(2) = 0;
    elseif q(4) > d+pi/2 && q(4) <= d+pi
        cfx(2) = 1;
    elseif q(4) > d+pi && q(4) <= d+3*pi/2
        cfx(2) = 2;
    end
    
    % cf6
    if q(6) > d-3*pi/2 && q(6) <= d-pi
        cfx(3) = -3;
    elseif q(6) > d-pi && q(6) <= d-pi/2
        cfx(3) = -2;
    elseif q(6) > d-pi/2 && q(6) <= -d
        cfx(3) = -1;
    elseif q(6) > -d && q(6) <= d+pi/2
        cfx(3) = 0;
    elseif q(6) > d+pi/2 && q(6) <= d+pi
        cfx(3) = 1;
    elseif q(6) > d+pi && q(6) <= d+3*pi/2
        cfx(3) = 2;
    end
    
    % cfx (第4个分量)
    cfx_a = 448*sin(q(2)) + 42*sin(q(2)+q(3)) + 451*sin(pi/2-(q(2)+q(3)));
    cfx_b = q(3);
    cfx_c = q(5);
    cfx_r = 0;
    
    if cfx_a >= 0 && cfx_b > -1.459095 && cfx_c >= 0
        cfx_r = 0;
    elseif cfx_a >= 0 && cfx_b > -1.459095 && cfx_c < 0
        cfx_r = 1;
    elseif cfx_a >= 0 && cfx_b < -1.50971 && cfx_c >= 0
        cfx_r = 2;
    elseif cfx_a >= 0 && cfx_b < -1.50971 && cfx_c < 0
        cfx_r = 3;
    elseif cfx_a < 0 && cfx_b > -1.459095 && cfx_c >= 0
        cfx_r = 4;
    elseif cfx_a < 0 && cfx_b > -1.459095 && cfx_c < 0
        cfx_r = 5;
    elseif cfx_a < 0 && cfx_b < -1.50971 && cfx_c >= 0
        cfx_r = 6;
    elseif cfx_a < 0 && cfx_b < -1.50971 && cfx_c < 0
        cfx_r = 7;
    end
    
    cfx(4) = cfx_r;
end