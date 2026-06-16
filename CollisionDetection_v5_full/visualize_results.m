function visualize_results(path_data, results, config)
%% VISUALIZE_RESULTS 可视化碰撞检测结果
%
% 生成3个图形窗口:
%   图1: 碰撞风险3D云图
%   图2: 关键帧工具位置详图 (显示锥面与工具Z轴平行)
%   图3: 分析图表

    xyz_mm = path_data.xyz_mm;
    num_points = path_data.num_points;
    
    %% 图1: 碰撞风险云图
    figure('Name', '碰撞风险分布', 'Position', [50, 50, 1500, 500]);
    
    subplot(1, 3, 1);
    plot_collision_cloud(xyz_mm, results.overall_status, '综合碰撞风险');
    
    subplot(1, 3, 2);
    plot_collision_cloud(xyz_mm, results.toolr_status, 'ToolR尖端碰撞风险');
    
    subplot(1, 3, 3);
    plot_collision_cloud(xyz_mm, results.cone_status, 'ToolC锥面碰撞风险');
    
    %% 图2: 关键帧工具位置详图
    figure('Name', '关键帧工具位置分析', 'Position', [100, 100, 1600, 550]);
    
    key_idx = round(linspace(max(30, num_points*0.1), num_points*0.9, 3));
    
    for f = 1:3
        subplot(1, 3, f);
        idx = key_idx(f);
        
        % 已打印路径
        if idx > 1
            plot3(xyz_mm(1:idx-1, 1), xyz_mm(1:idx-1, 2), xyz_mm(1:idx-1, 3), ...
                'b-', 'LineWidth', 1.5);
            hold on;
        end
        
        % ToolC尖端 (=路径点)
        scatter3(results.toolc_tip(idx,1)*1000, results.toolc_tip(idx,2)*1000, ...
            results.toolc_tip(idx,3)*1000, 250, 'g', 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % 法兰盘
        scatter3(results.flange(idx,1)*1000, results.flange(idx,2)*1000, ...
            results.flange(idx,3)*1000, 180, 'c', 's', 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % ToolR尖端
        scatter3(results.toolr_tip(idx,1)*1000, results.toolr_tip(idx,2)*1000, ...
            results.toolr_tip(idx,3)*1000, 250, 'm', 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % 连接线: 法兰盘到ToolC
        plot3([results.flange(idx,1), results.toolc_tip(idx,1)]*1000, ...
              [results.flange(idx,2), results.toolc_tip(idx,2)]*1000, ...
              [results.flange(idx,3), results.toolc_tip(idx,3)]*1000, ...
              'g-', 'LineWidth', 2.5);
        
        % 连接线: 法兰盘到ToolR
        plot3([results.flange(idx,1), results.toolr_tip(idx,1)]*1000, ...
              [results.flange(idx,2), results.toolr_tip(idx,2)]*1000, ...
              [results.flange(idx,3), results.toolr_tip(idx,3)]*1000, ...
              'm-', 'LineWidth', 2.5);
        
        % 绘制工具Z轴方向 (红色箭头，指向喷嘴出口)
        tool_z = results.tool_z_axis(idx, :);
        arrow_len = 10;  % mm
        quiver3(results.toolc_tip(idx,1)*1000, results.toolc_tip(idx,2)*1000, ...
                results.toolc_tip(idx,3)*1000, ...
                tool_z(1)*arrow_len, tool_z(2)*arrow_len, tool_z(3)*arrow_len, ...
                0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % 绘制检测锥面 (从cone_apex开始，沿cone_axis方向展开)
        % 锥面向远离喷嘴的方向展开
        draw_cone_mesh(results.cone_apex(idx,:)*1000, results.cone_axis(idx,:), ...
            config.cone.half_angle, config.cone.depth*1000, [1, 0.5, 0], 0.35);
        
        hold off;
        
        xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
        status_str = {'安全', '风险', '碰撞'};
        title(sprintf('帧%d | 状态:%s | Rx=%.1f° Ry=%.1f°', idx, ...
            status_str{results.overall_status(idx)+1}, ...
            results.Rx_deg(idx), results.Ry_deg(idx)));
        legend({'已打印', 'ToolC尖端', '法兰盘', 'ToolR尖端'}, ...
            'Location', 'best', 'FontSize', 9);
        view(45, 25);
        grid on;
        axis equal;
    end
    
    %% 图3: 分析图表
    figure('Name', '碰撞分析图表', 'Position', [150, 150, 1400, 450]);
    
    % 距离分析
    subplot(1, 4, 1);
    semilogy(results.toolr_dist*1000, 'b-', 'LineWidth', 1); hold on;
    semilogy(results.cone_dist*1000, 'r-', 'LineWidth', 1);
    yline(config.collision.safe_dist*1000, '--g', '安全0.45mm', 'LineWidth', 1.5);
    yline(config.collision.warn_dist*1000, '--r', '碰撞0.1mm', 'LineWidth', 1.5);
    xlabel('点索引'); ylabel('距离 (mm, log)'); title('碰撞距离');
    legend('ToolR', '锥面', 'Location', 'best'); grid on;
    
    % 姿态角Rx
    subplot(1, 4, 2);
    plot(rad2deg(results.Rx_rad), 'b-', 'LineWidth', 1);
    xlabel('点索引'); ylabel('Rx (°)'); title('X轴旋转角度');
    grid on;
    
    % 姿态角Ry
    subplot(1, 4, 3);
    plot(rad2deg(results.Ry_rad), 'r-', 'LineWidth', 1);
    xlabel('点索引'); ylabel('Ry (°)'); title('Y轴旋转角度');
    grid on;
    
    % 统计柱状图
    subplot(1, 4, 4);
    counts = [sum(results.overall_status==0), ...
              sum(results.overall_status==1), ...
              sum(results.overall_status==2)];
    b = bar(counts);
    b.FaceColor = 'flat';
    b.CData = [0, 0.8, 0; 1, 0.8, 0; 1, 0, 0];
    xticklabels({'安全', '风险', '碰撞'});
    ylabel('点数'); title('碰撞状态统计');
    for i = 1:3
        text(i, counts(i)+max(counts)*0.03, num2str(counts(i)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    grid on;
end

function plot_collision_cloud(points, status, title_str)
%% 绘制碰撞云图
    colors = zeros(size(points, 1), 3);
    colors(status == 0, :) = repmat([0, 0.8, 0], sum(status == 0), 1);
    colors(status == 1, :) = repmat([1, 0.8, 0], sum(status == 1), 1);
    colors(status == 2, :) = repmat([1, 0, 0], sum(status == 2), 1);
    
    scatter3(points(:,1), points(:,2), points(:,3), 15, colors, 'filled');
    hold on;
    plot3(points(:,1), points(:,2), points(:,3), 'Color', [0.6, 0.6, 0.6], 'LineWidth', 0.3);
    
    % 图例
    h1 = scatter3(nan, nan, nan, 60, [0, 0.8, 0], 'filled');
    h2 = scatter3(nan, nan, nan, 60, [1, 0.8, 0], 'filled');
    h3 = scatter3(nan, nan, nan, 60, [1, 0, 0], 'filled');
    legend([h1, h2, h3], {'安全(>0.45mm)', '风险(0.1-0.45mm)', '碰撞(<0.1mm)'}, ...
        'Location', 'best', 'FontSize', 8);
    hold off;
    
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(title_str);
    view(45, 30); grid on; axis equal;
end

function draw_cone_mesh(apex_mm, axis, half_angle, depth_mm, color, alpha)
%% 绘制锥面网格
% 锥面从顶点(apex)沿轴向(axis)展开
%
% 输入:
%   apex_mm   - 顶点位置 (mm)
%   axis      - 轴向 (单位向量，指向锥面展开方向)
%   half_angle - 半角 (rad)
%   depth_mm  - 深度 (mm)
%   color     - 颜色 [r,g,b]
%   alpha     - 透明度

    apex = apex_mm(:)';
    axis = axis(:)' / norm(axis(:));
    
    % 生成锥面网格 (在局部坐标系下，Z轴为锥轴)
    [TH, H] = meshgrid(linspace(0, 2*pi, 20), linspace(0, depth_mm, 10));
    R = H * tan(half_angle);
    X_local = R .* cos(TH);
    Y_local = R .* sin(TH);
    Z_local = H;
    
    % 计算旋转矩阵，将局部Z轴对齐到axis方向
    z0 = [0, 0, 1];
    v = cross(z0, axis);
    s = norm(v);
    c = dot(z0, axis);
    
    if s < 1e-8
        if c > 0
            Rot = eye(3);
        else
            Rot = diag([1, -1, -1]);
        end
    else
        vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
        Rot = eye(3) + vx + vx*vx * (1-c) / s^2;
    end
    
    % 变换到世界坐标系
    X_world = zeros(size(X_local));
    Y_world = zeros(size(Y_local));
    Z_world = zeros(size(Z_local));
    
    for i = 1:numel(X_local)
        p_local = [X_local(i); Y_local(i); Z_local(i)];
        p_world = Rot * p_local + apex(:);
        X_world(i) = p_world(1);
        Y_world(i) = p_world(2);
        Z_world(i) = p_world(3);
    end
    
    % 绘制锥面
    surf(X_world, Y_world, Z_world, 'FaceColor', color, 'FaceAlpha', alpha, ...
        'EdgeColor', 'none');
end
