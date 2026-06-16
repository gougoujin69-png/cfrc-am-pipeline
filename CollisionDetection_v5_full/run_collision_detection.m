function results = run_collision_detection(path_data, config)
%% RUN_COLLISION_DETECTION 执行碰撞检测主循环
%
% 检测逻辑:
%   1. ToolR尖端: 检测是否与已打印路径碰撞
%   2. ToolC锥面: 轴向与工具Z轴平行，检测喷嘴周围区域是否与已打印路径碰撞
%
% 输入:
%   path_data - 路径数据结构体
%   config    - 配置参数结构体
%
% 输出:
%   results - 检测结果结构体

    num_points = path_data.num_points;
    
    % 初始化结果
    results.toolc_tip = zeros(num_points, 3);     % ToolC尖端 = 路径点
    results.flange = zeros(num_points, 3);        % 法兰盘位置
    results.toolr_tip = zeros(num_points, 3);     % ToolR尖端位置
    results.tool_z_axis = zeros(num_points, 3);   % 工具Z轴方向
    results.cone_apex = zeros(num_points, 3);     % 锥面顶点位置
    results.cone_axis = zeros(num_points, 3);     % 锥面轴向（远离喷嘴方向）
    
    results.toolr_status = zeros(num_points, 1);  % ToolR碰撞状态
    results.cone_status = zeros(num_points, 1);   % 锥面碰撞状态
    results.toolr_dist = inf(num_points, 1);      % ToolR最小距离
    results.cone_dist = inf(num_points, 1);       % 锥面最小距离
    results.overall_status = zeros(num_points, 1);% 综合状态
    
    results.Rx_rad = path_data.Rx_rad;            % X轴旋转角度(弧度)
    results.Rx_deg = path_data.Rx_deg;            % X轴旋转角度(度)
    results.Ry_rad = path_data.Ry_rad;            % Y轴旋转角度(弧度)
    results.Ry_deg = path_data.Ry_deg;            % Y轴旋转角度(度)
    
    % 已打印路径
    printed_path = [];
    
    % 进度显示
    fprintf('检测进度: ');
    progress_step = max(1, floor(num_points / 40));
    
    for i = 1:num_points
        if mod(i, progress_step) == 0
            fprintf('.');
        end
        
        % 当前路径点 = ToolC尖端
        toolc_tip = path_data.xyz_m(i, :);
        Rx = path_data.Rx_rad(i);
        Ry = path_data.Ry_rad(i);
        
        results.toolc_tip(i, :) = toolc_tip;
        
        % 计算工具位置和姿态
        [flange, toolr_tip, tool_z_axis, cone_apex, cone_axis] = compute_tool_positions(...
            toolc_tip, Rx, Ry, config.toolc, config.toolr, config.cone);
        
        results.flange(i, :) = flange;
        results.toolr_tip(i, :) = toolr_tip;
        results.tool_z_axis(i, :) = tool_z_axis;
        results.cone_apex(i, :) = cone_apex;
        results.cone_axis(i, :) = cone_axis;
        
        % 碰撞检测 (针对已打印路径)
        if size(printed_path, 1) >= 2
            % ToolR尖端检测
            [s1, d1] = detect_point_collision(toolr_tip, printed_path, config.collision);
            results.toolr_status(i) = s1;
            results.toolr_dist(i) = d1;
            
            % 锥面检测 (轴向为远离喷嘴的方向)
            [s2, d2] = detect_cone_collision(cone_apex, cone_axis, config.cone, ...
                printed_path, config.collision);
            results.cone_status(i) = s2;
            results.cone_dist(i) = d2;
            
            results.overall_status(i) = max(s1, s2);
        end
        
        % 更新已打印路径
        printed_path = [printed_path; toolc_tip];
    end
    
    fprintf(' 完成!\n');
end
