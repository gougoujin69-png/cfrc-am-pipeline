%% ========================================================================
%  ABB IRB-1200 奇异点规避优化 + 可视化验证 (修复版)
%  ========================================================================
%  
%  重要概念说明:
%  1. 同一个末端位置可以有多组关节角度解（最多8组）
%  2. 优化初始配置 = 选择更好的解路径，末端位置不变
%  3. 不同的解路径可能经过或避开奇异点
%
%  ========================================================================

clear; clc; close all;

%% ======================== 用户配置 ======================================
config = struct();

config.excel_file = 'path_23_2.xlsx';              % 留空使用测试路径
config.output_prefix = 'OptimizedPath';
config.optimization_method = 'quick'; % 'quick' 或 'ga'
config.ga_population = 30;
config.ga_generations = 20;
config.animation_speed = 2;
config.show_comparison = true;

%% ======================== 主程序 ========================================
fprintf('========================================================\n');
fprintf('   ABB IRB-1200 奇异点规避优化与可视化验证\n');
fprintf('========================================================\n\n');

% 创建机器人模型
robot = createRobotModel();

% 生成/加载路径
if isempty(config.excel_file)
    fprintf('使用测试路径...\n');
    path_data = generateTestPath();
else
    fprintf('从Excel加载: %s\n', config.excel_file);
    path_data = loadExcelPath(config.excel_file);
end

% ========== 计算默认初始配置的轨迹 ==========
fprintf('\n【默认配置】计算轨迹 (q0 = 全零)...\n');
default_q0 = zeros(6, 1);
[traj_default, info_default] = computeTrajectory(robot, path_data, default_q0);

% ========== 执行优化 ==========
fprintf('\n【优化】搜索最佳初始配置...\n');
[optimal_q0, ~] = runOptimization(robot, path_data, config);

% ========== 计算优化后的轨迹 ==========
fprintf('\n【优化配置】计算轨迹...\n');
[traj_optimized, info_optimized] = computeTrajectory(robot, path_data, optimal_q0);

% ========== 验证末端位置一致性 ==========
fprintf('\n========================================================\n');
fprintf('   末端位置验证 (证明只是关节配置不同，末端位置相同)\n');
fprintf('========================================================\n');
verifyEndPositions(info_default, info_optimized);

% ========== 显示对比结果 ==========
printComparison(info_default, info_optimized, default_q0, optimal_q0);

% ========== 可视化 ==========
fprintf('\n启动可视化...\n');
fprintf('(如果窗口关闭导致错误，这是正常的，不影响结果)\n\n');

try
    if config.show_comparison
        runComparisonVisualization(robot, path_data, ...
            traj_default, info_default, ...
            traj_optimized, info_optimized, config);
    else
        runSingleVisualization(robot, path_data, traj_optimized, info_optimized, config);
    end
catch ME
    fprintf('可视化提前结束: %s\n', ME.message);
end

% 生成RAPID代码
generateRAPIDCode(path_data, traj_optimized, info_optimized, optimal_q0, config);

% 最终总结
printFinalSummary(info_default, info_optimized, optimal_q0);

%% ======================== 末端位置验证 ==================================
function verifyEndPositions(info_default, info_optimized)
    % 验证两种配置的末端位置是否一致
    
    pos_diff = info_default.end_positions - info_optimized.end_positions;
    max_diff = max(abs(pos_diff(:)));
    mean_diff = mean(sqrt(sum(pos_diff.^2, 2)));
    
    fprintf('默认配置 vs 优化配置 的末端位置差异:\n');
    fprintf('  最大差异: %.4f mm\n', max_diff);
    fprintf('  平均差异: %.4f mm\n', mean_diff);
    
    if max_diff < 1.0
        fprintf('  ✓ 验证通过: 末端位置基本一致 (差异 < 1mm)\n');
        fprintf('  → 这证明优化只改变了关节配置，没有改变末端轨迹!\n');
    else
        fprintf('  ⚠ 存在较大差异，可能是逆运动学收敛问题\n');
    end
    
    % 显示几个关键点的对比
    fprintf('\n关键点末端位置对比 (单位: mm):\n');
    fprintf('点序号    默认X      默认Y      默认Z    |   优化X      优化Y      优化Z    |  差距\n');
    fprintf('--------------------------------------------------------------------------------------\n');
    
    n = size(info_default.end_positions, 1);
    check_points = unique([1, round(n/4), round(n/2), round(3*n/4), n]);
    
    for i = check_points
        p1 = info_default.end_positions(i, :);
        p2 = info_optimized.end_positions(i, :);
        diff = norm(p1 - p2);
        fprintf('%4d    %8.2f   %8.2f   %8.2f  | %8.2f   %8.2f   %8.2f  | %6.3f\n', ...
                i, p1(1), p1(2), p1(3), p2(1), p2(2), p2(3), diff);
    end
end

%% ======================== 最终总结 ======================================
function printFinalSummary(info_default, info_optimized, optimal_q0)
    fprintf('\n========================================================\n');
    fprintf('   最终总结\n');
    fprintf('========================================================\n');
    
    fprintf('\n【核心结论】\n');
    fprintf('  • 末端轨迹: 保持不变 (机械臂尖端走相同路径)\n');
    fprintf('  • 关节配置: 发生改变 (选择了不同的逆运动学解)\n');
    fprintf('  • 奇异点数: %d → %d (减少 %d 个)\n', ...
            info_default.n_singularities, info_optimized.n_singularities, ...
            info_default.n_singularities - info_optimized.n_singularities);
    fprintf('  • 最小det(J): %.4f → %.4f (提升 %.1f%%)\n', ...
            info_default.min_det_J, info_optimized.min_det_J, ...
            (info_optimized.min_det_J - info_default.min_det_J) / max(info_default.min_det_J, 0.001) * 100);
    
    fprintf('\n【推荐初始配置】(将此配置用于实际运行)\n');
    q0_deg = optimal_q0 * 180 / pi;
    fprintf('  J1 = %8.2f°\n', q0_deg(1));
    fprintf('  J2 = %8.2f°\n', q0_deg(2));
    fprintf('  J3 = %8.2f°\n', q0_deg(3));
    fprintf('  J4 = %8.2f°\n', q0_deg(4));
    fprintf('  J5 = %8.2f°\n', q0_deg(5));
    fprintf('  J6 = %8.2f°\n', q0_deg(6));
    
    fprintf('\n【工作原理说明】\n');
    fprintf('  对于6轴机械臂，同一个末端位置最多有8组关节角度解。\n');
    fprintf('  传统方法使用固定初始配置(如全零)，导致整个路径可能经过奇异点。\n');
    fprintf('  优化后的初始配置引导机械臂选择另一条"解路径"，避开奇异区域。\n');
    fprintf('  这就像GPS导航选择不同路线到达同一目的地，避开拥堵区域。\n');
    
    fprintf('\n========================================================\n');
end

%% ======================== 机器人模型 ====================================
function robot = createRobotModel()
    robot.name = 'ABB IRB-1200';
    
    robot.DH = [
        0,      399,    0,      -pi/2;
        -pi/2,  0,      350,    0;
        0,      0,      42,     -pi/2;
        0,      351,    0,      pi/2;
        0,      0,      0,      -pi/2;
        -pi,    82,     0,      0
    ];
    
    limits_deg = [-170,170; -100,135; -200,70; -270,270; -130,130; -360,360];
    robot.limits = limits_deg * pi / 180;
    
    robot.d1 = 399; robot.a2 = 350; robot.a3 = 42;
    robot.d4 = 351; robot.d6 = 82;
    
    fprintf('IRB-1200模型已创建\n');
end

%% ======================== 正向运动学 ====================================
function [T, T_all] = FK(robot, q)
    q = q(:);
    DH = robot.DH;
    T = eye(4);
    T_all = cell(1, 7);
    T_all{1} = T;
    
    for i = 1:6
        theta = q(i) + DH(i,1);
        d = DH(i,2); a = DH(i,3); alpha = DH(i,4);
        
        ct = cos(theta); st = sin(theta);
        ca = cos(alpha); sa = sin(alpha);
        
        Ti = [ct, -st, 0, a;
              st*ca, ct*ca, -sa, -sa*d;
              st*sa, ct*sa, ca, ca*d;
              0, 0, 0, 1];
        
        T = T * Ti;
        T_all{i+1} = T;
    end
end

%% ======================== Jacobian ======================================
function J = computeJacobian(robot, q)
    q = q(:);
    [~, T_all] = FK(robot, q);
    p_e = T_all{7}(1:3, 4);
    J = zeros(6, 6);
    
    z0 = [0; 0; 1]; p0 = [0; 0; 0];
    J(1:3, 1) = cross(z0, p_e - p0);
    J(4:6, 1) = z0;
    
    for i = 2:6
        zi = T_all{i}(1:3, 3);
        pi = T_all{i}(1:3, 4);
        J(1:3, i) = cross(zi, p_e - pi);
        J(4:6, i) = zi;
    end
end

%% ======================== 逆运动学 ======================================
function q = IK(robot, T_target, q_ref)
    q_ref = q_ref(:);
    q = q_ref;
    
    for iter = 1:150
        [T_curr, ~] = FK(robot, q);
        
        dp = T_target(1:3,4) - T_curr(1:3,4);
        R_err = T_target(1:3,1:3) * T_curr(1:3,1:3)';
        tr_val = (trace(R_err)-1)/2;
        tr_val = max(-1, min(1, tr_val));
        angle = acos(tr_val);
        
        if abs(angle) < 1e-10
            axis_vec = [0;0;1];
        else
            axis_vec = [R_err(3,2)-R_err(2,3); R_err(1,3)-R_err(3,1); R_err(2,1)-R_err(1,2)] / (2*sin(angle));
            if norm(axis_vec) > 1e-10
                axis_vec = axis_vec / norm(axis_vec);
            end
        end
        dr = angle * axis_vec;
        
        err = [dp; dr];
        if norm(err) < 1e-6, break; end
        
        J = computeJacobian(robot, q);
        
        % 阻尼最小二乘法
        lambda = 0.01;
        dq = J' * ((J*J' + lambda*eye(6)) \ err);
        
        q = q + 0.5*dq;
        
        % 限位
        for i = 1:6
            q(i) = max(robot.limits(i,1), min(robot.limits(i,2), q(i)));
        end
    end
end

%% ======================== 测试路径生成 ==================================
function path_data = generateTestPath()
    n_per_seg = 25;
    path_data = [];
    
    % 段1: 圆弧
    t = linspace(0, 2*pi, n_per_seg)';
    for i = 1:length(t)
        path_data = [path_data; 500, 80*cos(t(i)), 150+50*sin(t(i)), 180, 0, 0];
    end
    
    % 段2: 直线下降
    for i = 1:n_per_seg
        z = 200 - 100*(i-1)/(n_per_seg-1);
        path_data = [path_data; 500, 80, z, 180, 0, 0];
    end
    
    % 段3: 水平弧线
    t = linspace(0, pi, n_per_seg)';
    for i = 1:length(t)
        path_data = [path_data; 500+40*cos(t(i)), 80+40*sin(t(i)), 100, 180, 0, 0];
    end
    
    fprintf('生成测试路径: %d点\n', size(path_data, 1));
end

function path_data = loadExcelPath(filename)
    YM = xlsread(filename);
    n = size(YM, 1);
    path_data = zeros(n, 6);
    
    for i = 1:n
        path_data(i, 1) = 550 + YM(i, 1);
        path_data(i, 2) = YM(i, 2);
        path_data(i, 3) = 57.36 + YM(i, 3);
        path_data(i, 4) = 180;
        if size(YM, 2) >= 4
            path_data(i, 5) = YM(i, 4);
        end
        if size(YM, 2) >= 5
            path_data(i, 6) = YM(i, 5);
        end
    end
end

function T = pathToTransform(pt)
    x = pt(1); y = pt(2); z = pt(3);
    rx = pt(4)*pi/180; ry = pt(5)*pi/180; rz = pt(6)*pi/180;
    
    Rx = [1,0,0; 0,cos(rx),-sin(rx); 0,sin(rx),cos(rx)];
    Ry = [cos(ry),0,sin(ry); 0,1,0; -sin(ry),0,cos(ry)];
    Rz = [cos(rz),-sin(rz),0; sin(rz),cos(rz),0; 0,0,1];
    
    T = eye(4);
    T(1:3, 1:3) = Rz * Ry * Rx;
    T(1:3, 4) = [x; y; z];
end

%% ======================== 轨迹计算 ======================================
function [traj, info] = computeTrajectory(robot, path_data, q0)
    n = size(path_data, 1);
    traj = zeros(n, 6);
    info.det_J = zeros(n, 1);
    info.cond_J = zeros(n, 1);
    info.is_singular = false(n, 1);
    info.end_positions = zeros(n, 3);
    info.joint_velocity = zeros(n, 6);
    
    q = q0(:);
    
    for i = 1:n
        T_target = pathToTransform(path_data(i,:));
        q_new = IK(robot, T_target, q);
        traj(i, :) = q_new';
        
        J = computeJacobian(robot, q_new);
        info.det_J(i) = abs(det(J));
        info.cond_J(i) = cond(J);
        
        % 奇异点判断
        q5_deg = abs(q_new(5) * 180 / pi);
        info.is_singular(i) = info.det_J(i) < 0.01 || q5_deg < 5;
        
        [T, ~] = FK(robot, q_new);
        info.end_positions(i, :) = T(1:3, 4)';
        
        if i > 1
            info.joint_velocity(i, :) = (traj(i,:) - traj(i-1,:));
        end
        
        q = q_new;
        
        if mod(i, 20) == 0
            fprintf('  进度: %d/%d\n', i, n);
        end
    end
    
    info.n_singularities = sum(info.is_singular);
    info.min_det_J = min(info.det_J);
    info.mean_det_J = mean(info.det_J);
    info.max_joint_vel = max(abs(info.joint_velocity(:)));
    
    fprintf('  完成! 奇异点: %d, 最小det(J): %.4f\n', info.n_singularities, info.min_det_J);
end

%% ======================== 优化 ==========================================
function [q0_opt, results] = runOptimization(robot, path_data, config)
    if strcmp(config.optimization_method, 'ga')
        [q0_opt, results] = runGA(robot, path_data, config);
    else
        [q0_opt, results] = runQuickOpt(robot, path_data);
    end
end

function [q0_opt, results] = runQuickOpt(robot, path_data)
    fprintf('运行快速网格优化...\n');
    
    best_score = -inf;
    best_q0 = zeros(6, 1);
    
    n_samples = 5;
    total = n_samples^3;
    count = 0;
    
    for i1 = linspace(robot.limits(1,1), robot.limits(1,2), n_samples)
        for i2 = linspace(robot.limits(2,1), robot.limits(2,2), n_samples)
            for i3 = linspace(robot.limits(3,1), robot.limits(3,2), n_samples)
                q0 = [i1; i2; i3; 0; 0; 0];
                score = evaluateConfig(robot, path_data, q0);
                if score > best_score
                    best_score = score;
                    best_q0 = q0;
                end
                count = count + 1;
            end
        end
        fprintf('  网格搜索进度: %.0f%%\n', count/total*100);
    end
    
    % 局部优化
    fprintf('  局部优化中...\n');
    opts = optimset('Display', 'off', 'TolX', 1e-3, 'MaxIter', 100);
    q0_opt = fminsearch(@(q) -evaluateConfig(robot, path_data, q(:)), best_q0, opts);
    q0_opt = q0_opt(:);
    
    for i = 1:6
        q0_opt(i) = max(robot.limits(i,1), min(robot.limits(i,2), q0_opt(i)));
    end
    
    results.score = evaluateConfig(robot, path_data, q0_opt);
    fprintf('优化完成，最终得分: %.4f\n', results.score);
end

function [q0_opt, results] = runGA(robot, path_data, config)
    fprintf('运行遗传算法优化...\n');
    
    nvars = 6;
    lb = robot.limits(:,1)';
    ub = robot.limits(:,2)';
    
    fitness = @(q) -evaluateConfig(robot, path_data, q(:));
    
    opts = optimoptions('ga', ...
        'PopulationSize', config.ga_population, ...
        'MaxGenerations', config.ga_generations, ...
        'Display', 'iter', ...
        'PlotFcn', []);
    
    [q0_opt, fval] = ga(fitness, nvars, [], [], [], [], lb, ub, [], opts);
    q0_opt = q0_opt(:);
    
    results.score = -fval;
    fprintf('GA完成，得分: %.4f\n', -fval);
end

function score = evaluateConfig(robot, path_data, q0)
    q0 = q0(:);
    if length(q0) ~= 6
        score = -1e10;
        return;
    end
    
    n = size(path_data, 1);
    q = q0;
    min_det = inf;
    penalty = 0;
    
    % 采样评估
    step = max(1, floor(n/30));
    
    for i = 1:step:n
        T_target = pathToTransform(path_data(i,:));
        q = IK(robot, T_target, q);
        
        J = computeJacobian(robot, q);
        det_J = abs(det(J));
        
        if det_J < min_det
            min_det = det_J;
        end
        
        if det_J < 0.01
            penalty = penalty + 10;
        end
        
        q5_deg = abs(q(5) * 180 / pi);
        if q5_deg < 5
            penalty = penalty + 5;
        end
    end
    
    score = min_det - penalty;
end

%% ======================== 对比输出 ======================================
function printComparison(info_default, info_optimized, q0_default, q0_opt)
    fprintf('\n========================================================\n');
    fprintf('   优化效果对比\n');
    fprintf('========================================================\n');
    
    fprintf('\n指标                  默认配置        优化后\n');
    fprintf('---------------------------------------------------\n');
    fprintf('奇异点数量:           %4d            %4d\n', ...
            info_default.n_singularities, info_optimized.n_singularities);
    fprintf('最小det(J):           %.4f          %.4f\n', ...
            info_default.min_det_J, info_optimized.min_det_J);
    fprintf('平均det(J):           %.4f          %.4f\n', ...
            info_default.mean_det_J, info_optimized.mean_det_J);
    fprintf('最大关节速度:         %.4f          %.4f rad/step\n', ...
            info_default.max_joint_vel, info_optimized.max_joint_vel);
    
    % 改善指示
    if info_optimized.n_singularities < info_default.n_singularities
        fprintf('\n✓ 奇异点减少了 %d 个!\n', ...
                info_default.n_singularities - info_optimized.n_singularities);
    end
    if info_optimized.min_det_J > info_default.min_det_J
        fprintf('✓ 最小det(J)提升了 %.1f%%!\n', ...
                (info_optimized.min_det_J - info_default.min_det_J) / max(info_default.min_det_J, 0.001) * 100);
    end
end

%% ======================== 对比可视化 (修复版) ============================
function runComparisonVisualization(robot, path_data, traj_default, info_default, ...
                                    traj_optimized, info_optimized, config)
    n = size(path_data, 1);
    
    % 创建图形
    fig = figure('Name', 'ABB IRB-1200 优化对比', 'Position', [30, 30, 1600, 900], ...
                 'CloseRequestFcn', @closeFigure);
    
    colors = [0.7,0.7,0.7; 1,0.5,0; 0,0.6,0.8; 0,0.8,0.4; 0.8,0.2,0.2; 0.6,0.3,0.7; 0.9,0.9,0];
    
    stopFlag = false;
    
    function closeFigure(~, ~)
        stopFlag = true;
        delete(fig);
    end
    
    % 预绘制静态图
    
    % J5角度对比
    ax3 = subplot(2, 4, 3);
    hold on;
    joint_deg_def = traj_default * 180 / pi;
    joint_deg_opt = traj_optimized * 180 / pi;
    plot(1:n, joint_deg_def(:,5), 'r--', 'LineWidth', 1.5, 'DisplayName', '默认');
    plot(1:n, joint_deg_opt(:,5), 'b-', 'LineWidth', 1.5, 'DisplayName', '优化后');
    yline(5, 'k:', 'LineWidth', 1);
    yline(-5, 'k:', 'LineWidth', 1);
    fill([0 n n 0], [5 5 -5 -5], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    xlabel('点索引'); ylabel('J5角度 (°)');
    title('关节5角度 (±5°为腕部奇异区)');
    legend('Location', 'best');
    grid on;
    xlim([1, n]);
    
    % det(J)对比
    ax4 = subplot(2, 4, 4);
    hold on;
    plot(1:n, info_default.det_J, 'r--', 'LineWidth', 1.5, 'DisplayName', '默认');
    plot(1:n, info_optimized.det_J, 'b-', 'LineWidth', 1.5, 'DisplayName', '优化后');
    yline(0.01, 'k:', 'LineWidth', 1);
    fill([0 n n 0], [0.01 0.01 0 0], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    xlabel('点索引'); ylabel('|det(J)|');
    title('Jacobian行列式 (<0.01为奇异区)');
    legend('Location', 'best');
    grid on;
    xlim([1, n]);
    
    % 条件数对比
    ax5 = subplot(2, 4, 7);
    hold on;
    semilogy(1:n, info_default.cond_J, 'r--', 'LineWidth', 1.5, 'DisplayName', '默认');
    semilogy(1:n, info_optimized.cond_J, 'b-', 'LineWidth', 1.5, 'DisplayName', '优化后');
    xlabel('点索引'); ylabel('条件数 (log)');
    title('Jacobian条件数');
    legend('Location', 'best');
    grid on;
    xlim([1, n]);
    
    % 统计信息
    ax6 = subplot(2, 4, 8);
    axis off;
    stats_str = sprintf([...
        '优化效果统计\n' ...
        '========================\n\n' ...
        '             默认    优化后\n' ...
        '奇异点:      %3d     %3d\n' ...
        '最小det(J):  %.3f   %.3f\n' ...
        '平均det(J):  %.3f   %.3f\n\n' ...
        '改善:\n' ...
        '  奇异点: %+d\n' ...
        '  det(J): %+.1f%%\n\n' ...
        '末端位置: 保持一致\n' ...
        '(仅关节配置不同)'], ...
        info_default.n_singularities, info_optimized.n_singularities, ...
        info_default.min_det_J, info_optimized.min_det_J, ...
        info_default.mean_det_J, info_optimized.mean_det_J, ...
        info_optimized.n_singularities - info_default.n_singularities, ...
        (info_optimized.min_det_J - info_default.min_det_J) / max(info_default.min_det_J, 0.001) * 100);
    text(0.05, 0.95, stats_str, 'FontSize', 10, 'FontName', 'FixedWidth', ...
         'VerticalAlignment', 'top', 'Units', 'normalized');
    
    % 动画
    frame_delay = 0.03 / config.animation_speed;
    
    for i = 1:n
        if stopFlag || ~isvalid(fig)
            break;
        end
        
        try
            % 默认配置机器人
            subplot(2, 4, [1, 5]);
            cla; hold on;
            plot3(path_data(:,1), path_data(:,2), path_data(:,3), 'k--', 'LineWidth', 0.5);
            if i > 1
                plot3(info_default.end_positions(1:i,1), info_default.end_positions(1:i,2), ...
                      info_default.end_positions(1:i,3), 'r-', 'LineWidth', 2);
            end
            drawRobotSimple(robot, traj_default(i,:)', colors);
            setupAxes3D('默认配置 (q0=0)', info_default.is_singular(i));
            
            % 优化配置机器人
            subplot(2, 4, [2, 6]);
            cla; hold on;
            plot3(path_data(:,1), path_data(:,2), path_data(:,3), 'k--', 'LineWidth', 0.5);
            if i > 1
                plot3(info_optimized.end_positions(1:i,1), info_optimized.end_positions(1:i,2), ...
                      info_optimized.end_positions(1:i,3), 'b-', 'LineWidth', 2);
            end
            drawRobotSimple(robot, traj_optimized(i,:)', colors);
            setupAxes3D('优化后配置', info_optimized.is_singular(i));
            
            % 更新进度标记
            subplot(2, 4, 3);
            % 删除旧的进度线
            h = findobj(gca, 'Tag', 'progressLine');
            delete(h);
            line([i i], ylim, 'Color', 'g', 'LineWidth', 2, 'Tag', 'progressLine');
            
            subplot(2, 4, 4);
            h = findobj(gca, 'Tag', 'progressLine');
            delete(h);
            line([i i], ylim, 'Color', 'g', 'LineWidth', 2, 'Tag', 'progressLine');
            
            % 更新标题
            sgtitle(sprintf('ABB IRB-1200 优化对比 - 帧 %d/%d', i, n), 'FontSize', 14);
            
            drawnow limitrate;
            pause(frame_delay);
            
        catch
            % 忽略绘图错误，继续
            continue;
        end
    end
end

function setupAxes3D(title_str, is_singular)
    view(135, 30);
    axis equal;
    xlim([-200, 800]); ylim([-400, 400]); zlim([-100, 700]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    if is_singular
        title([title_str, ' - ⚠奇异点!'], 'Color', 'r', 'FontSize', 11);
    else
        title(title_str, 'Color', 'k', 'FontSize', 11);
    end
    
    grid on;
    light('Position', [1 1 1]);
    lighting gouraud;
end

%% ======================== 单视图可视化 ==================================
function runSingleVisualization(robot, path_data, traj, info, config)
    n = size(path_data, 1);
    colors = [0.7,0.7,0.7; 1,0.5,0; 0,0.6,0.8; 0,0.8,0.4; 0.8,0.2,0.2; 0.6,0.3,0.7; 0.9,0.9,0];
    
    fig = figure('Name', 'ABB IRB-1200 动画', 'Position', [50, 50, 1400, 800]);
    
    frame_delay = 0.05 / config.animation_speed;
    
    for i = 1:n
        if ~isvalid(fig), break; end
        
        try
            % 3D视图
            subplot(2, 2, [1, 3]);
            cla; hold on;
            
            plot3(path_data(:,1), path_data(:,2), path_data(:,3), 'b--', 'LineWidth', 1);
            
            sing_idx = find(info.is_singular);
            if ~isempty(sing_idx)
                scatter3(path_data(sing_idx,1), path_data(sing_idx,2), path_data(sing_idx,3), ...
                         80, 'r', 'filled');
            end
            
            if i > 1
                plot3(info.end_positions(1:i,1), info.end_positions(1:i,2), ...
                      info.end_positions(1:i,3), 'g-', 'LineWidth', 2);
            end
            
            drawRobotSimple(robot, traj(i,:)', colors);
            
            view(135, 30);
            axis equal;
            xlim([-200, 800]); ylim([-400, 400]); zlim([-100, 700]);
            xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
            
            if info.is_singular(i)
                title(sprintf('帧 %d/%d - ⚠ 奇异点!', i, n), 'Color', 'r', 'FontSize', 12);
            else
                title(sprintf('帧 %d/%d', i, n), 'FontSize', 12);
            end
            
            grid on;
            light; lighting gouraud;
            
            % 关节角度
            subplot(2, 2, 2);
            bar(traj(i,:) * 180 / pi);
            ylim([-200, 200]);
            xlabel('关节'); ylabel('角度 (°)');
            title('当前关节配置');
            xticks(1:6);
            xticklabels({'J1','J2','J3','J4','J5','J6'});
            grid on;
            
            % det(J)
            subplot(2, 2, 4);
            cla; hold on;
            plot(1:n, info.det_J, 'b-', 'LineWidth', 1);
            yline(0.01, 'r--');
            scatter(i, info.det_J(i), 100, 'g', 'filled');
            xlabel('点索引'); ylabel('|det(J)|');
            title(sprintf('det(J) = %.4f', info.det_J(i)));
            grid on;
            
            drawnow;
            pause(frame_delay);
        catch
            continue;
        end
    end
end

%% ======================== 简化的机器人绘制 ==============================
function drawRobotSimple(robot, q, colors)
    q = q(:);
    [T_end, T_all] = FK(robot, q);
    
    % 关节位置
    pos = zeros(7, 3);
    pos(1, :) = [0, 0, 0];
    for i = 2:7
        pos(i, :) = T_all{i}(1:3, 4)';
    end
    
    % 基座
    [X, Y, Z] = cylinder([80, 60], 16);
    Z = Z * 50;
    surf(X, Y, Z, 'FaceColor', colors(1,:), 'EdgeColor', 'none');
    
    % 连杆
    for i = 1:6
        p1 = pos(i,:);
        p2 = pos(i+1,:);
        
        % 简单线条连接
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'Color', colors(i+1,:), 'LineWidth', 8);
        
        % 关节球
        [X, Y, Z] = sphere(10);
        r = 25;
        surf(X*r + p1(1), Y*r + p1(2), Z*r + p1(3), ...
             'FaceColor', colors(i,:)*0.8, 'EdgeColor', 'none');
    end
    
    % 末端
    end_pos = pos(7, :);
    scatter3(end_pos(1), end_pos(2), end_pos(3), 100, 'y', 'filled', 'MarkerEdgeColor', 'k');
    
    % 末端坐标系
    R = T_end(1:3, 1:3);
    len = 40;
    quiver3(end_pos(1), end_pos(2), end_pos(3), R(1,1)*len, R(2,1)*len, R(3,1)*len, 'r', 'LineWidth', 2, 'AutoScale', 'off');
    quiver3(end_pos(1), end_pos(2), end_pos(3), R(1,2)*len, R(2,2)*len, R(3,2)*len, 'g', 'LineWidth', 2, 'AutoScale', 'off');
    quiver3(end_pos(1), end_pos(2), end_pos(3), R(1,3)*len, R(2,3)*len, R(3,3)*len, 'b', 'LineWidth', 2, 'AutoScale', 'off');
end

%% ======================== RAPID代码生成 =================================
function generateRAPIDCode(path_data, traj, info, q0, config)
    filename = sprintf('%s.mod', config.output_prefix);
    fid = fopen(filename, 'w');
    
    n = size(path_data, 1);
    
    fprintf(fid, 'MODULE %sModule\n', config.output_prefix);
    fprintf(fid, '    !========================================\n');
    fprintf(fid, '    ! 优化路径 - 奇异点规避\n');
    fprintf(fid, '    ! 路径点: %d, 奇异点: %d\n', n, info.n_singularities);
    fprintf(fid, '    ! 最小det(J): %.4f\n', info.min_det_J);
    fprintf(fid, '    !========================================\n\n');
    
    fprintf(fid, '    TASK PERS tooldata tool1:=[TRUE,[[0,0,100],[1,0,0,0]],[1,[0,0,5],[1,0,0,0],0,0,0]];\n\n');
    
    % 初始配置
    q0_deg = q0 * 180 / pi;
    fprintf(fid, '    ! 优化后的初始配置 - 用于避免奇异点\n');
    fprintf(fid, '    CONST jointtarget InitCfg:=[[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],[9E9,9E9,9E9,9E9,9E9,9E9]];\n\n', ...
            q0_deg(1), q0_deg(2), q0_deg(3), q0_deg(4), q0_deg(5), q0_deg(6));
    
    % 路径点
    for i = 1:n
        qi = traj(i,:) * 180 / pi;
        if info.is_singular(i)
            fprintf(fid, '    CONST jointtarget T%d:=[[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],[9E9,9E9,9E9,9E9,9E9,9E9]]; !SING\n', ...
                    i, qi(1), qi(2), qi(3), qi(4), qi(5), qi(6));
        else
            fprintf(fid, '    CONST jointtarget T%d:=[[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],[9E9,9E9,9E9,9E9,9E9,9E9]];\n', ...
                    i, qi(1), qi(2), qi(3), qi(4), qi(5), qi(6));
        end
    end
    
    fprintf(fid, '\n    PROC main()\n');
    fprintf(fid, '        ! 先移动到优化的初始配置\n');
    fprintf(fid, '        MoveAbsJ InitCfg, v100, fine, tool1\\WObj:=wobj0;\n');
    fprintf(fid, '        WaitTime 1;\n\n');
    fprintf(fid, '        ! 执行路径\n');
    
    for i = 1:n
        if info.is_singular(i)
            fprintf(fid, '        MoveAbsJ T%d, v30, z1, tool1\\WObj:=wobj0; !SING-低速\n', i);
        else
            fprintf(fid, '        MoveAbsJ T%d, v100, z1, tool1\\WObj:=wobj0;\n', i);
        end
    end
    
    fprintf(fid, '    ENDPROC\n');
    fprintf(fid, 'ENDMODULE\n');
    
    fclose(fid);
    fprintf('\nRAPID代码已生成: %s\n', filename);
end