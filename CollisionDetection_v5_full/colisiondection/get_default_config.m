function config = get_default_config()
%% GET_DEFAULT_CONFIG 获取默认配置参数
%
% 工具参数与Duanliang.m完全一致:
%   tool_h = 89.4e-3;   % X方向偏移
%   tool_l = 74.53e-3;  % Z方向长度
%   tool_y = 16.59e-3;  % Y方向偏移
%   tool_a = pi/2;      % 安装角度

    % ToolC参数 (主打印喷头)
    config.toolc.h = 89.4e-3;       % m
    config.toolc.l = 74.53e-3;      % m
    config.toolc.y = 16.59e-3;      % m (正值)
    config.toolc.a = pi/2;          % rad
    
    % ToolR参数 (辅助喷头，与ToolC对称)
    config.toolr.h = 89.4e-3;       % m
    config.toolr.l = 74.53e-3;      % m
    config.toolr.y = -16.59e-3;     % m (负值，对称)
    config.toolr.a = pi/2;          % rad
    
    % 检测锥面参数 (ToolC喷嘴周围的锥形检测区域)
    % 锥面轴向与工具Z轴平行，代表喷嘴的锥形外壳
    config.cone.half_angle = deg2rad(40);  % 半角40度 (全角80度)
    config.cone.depth = 5e-3;               % 锥面深度 5mm (FIX: 原为 50e-3=50mm, 与注释/README 不符, 巨锥会把几乎所有点判碰撞)
    config.cone.apex_offset = 0.5e-3;      % 锥面顶点距喷嘴尖端的安全距离 0.5mm
    
    % 碰撞阈值
    config.collision.safe_dist = 0.45e-3;     % 安全距离 0.45mm
    config.collision.warn_dist = 0.1e-3;   % 碰撞距离 0.1mm
    config.collision.path_radius = 0.1e-3; % 路径半径 0.1mm
    
    % Z轴优化参数
    config.optimize.enabled = true;
    config.optimize.coarse_step = deg2rad(5);   % 粗搜索步长
    config.optimize.fine_step = deg2rad(1);     % 细搜索步长
    config.optimize.range = [-pi, pi];          % 搜索范围
end
