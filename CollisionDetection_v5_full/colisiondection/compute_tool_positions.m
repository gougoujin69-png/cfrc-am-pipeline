function [flange, toolr_tip, tool_z_axis, cone_apex, cone_axis] = compute_tool_positions(toolc_tip, Rx, Ry, toolc, toolr, cone_params)
%% COMPUTE_TOOL_POSITIONS 计算法兰盘、ToolR位置和工具姿态
%
% 坐标变换逻辑 (与Duanliang.m一致):
%   E1 = transl(x,y,z) * troty(pi)                                    % 工具末端位姿
%   RE1 = E1 * troty(-tool_a) * transl(-tool_h,-tool_y,-tool_l)       % 法兰盘位姿
%
% 从上式推导:
%   法兰盘位置 = 尖端位置 + R_e * troty(-tool_a) * [-tool_h; -tool_y; -tool_l]
%
% 其中R_e是工具末端的姿态矩阵
%
% 输入:
%   toolc_tip   - [x,y,z] ToolC尖端位置 (m)，即路径点
%   Rx          - 绕X轴旋转角度 (rad)
%   Ry          - 绕Y轴旋转角度 (rad)
%   toolc       - ToolC参数结构体
%   toolr       - ToolR参数结构体
%   cone_params - 锥面参数 (可选)
%
% 输出:
%   flange      - [x,y,z] 法兰盘位置 (m)，在尖端上方
%   toolr_tip   - [x,y,z] ToolR尖端位置 (m)
%   tool_z_axis - [x,y,z] 工具Z轴方向 (单位向量，指向喷嘴出口)
%   cone_apex   - [x,y,z] 锥面顶点位置 (m)
%   cone_axis   - [x,y,z] 锥面轴向 (单位向量，远离喷嘴出口方向)

    toolc_tip = toolc_tip(:)';
    
    % ========== 构建工具末端姿态矩阵 R_e ==========
    % 基础姿态: troty(pi) - 工具垂直向下
    c_pi = cos(pi); s_pi = sin(pi);
    R_base = [c_pi, 0, s_pi; 0, 1, 0; -s_pi, 0, c_pi];  % troty(pi)
    
    % 姿态调整: Rx和Ry
    c_x = cos(Rx); s_x = sin(Rx);
    R_x = [1, 0, 0; 0, c_x, -s_x; 0, s_x, c_x];  % rotx(Rx)
    
    c_y = cos(Ry); s_y = sin(Ry);
    R_y_mat = [c_y, 0, s_y; 0, 1, 0; -s_y, 0, c_y];  % roty(Ry)
    
    % 工具末端姿态 (世界坐标系下)
    % FIX: 合成顺序必须与角度生成端一致 (Step3_2_Robotic_carbon_path /
    % calculate_angles_from_normal 用 troty(pi)*trotx(Rx)*troty(Ry))。
    % 原写法 R_x*R_y_mat*R_base 会把工具轴的 Y 分量符号反掉 -> 法兰/ToolR/
    % 锥体方向全部算错。
    R_e = R_base * R_x * R_y_mat;
    
    % ========== 计算工具Z轴方向 ==========
    tool_z_local = [0; 0; 1];
    tool_z_axis = (R_e * tool_z_local)';
    tool_z_axis = tool_z_axis / norm(tool_z_axis);
    
    % ========== 计算法兰盘位置 ==========
    % 根据Duanliang.m: RE1 = E1 * troty(-tool_a) * transl(-tool_h,-tool_y,-tool_l)
    % 法兰盘位置 = 尖端位置 + R_e * troty(-tool_a) * [-tool_h; -tool_y; -tool_l]
    
    % troty(-tool_a)，其中tool_a = pi/2
    c_a = cos(-toolc.a); s_a = sin(-toolc.a);
    R_tool_neg = [c_a, 0, s_a; 0, 1, 0; -s_a, 0, c_a];  % troty(-tool_a)
    
    % ToolC: 从尖端到法兰盘的偏移向量
    toolc_offset_local = [-toolc.h; -toolc.y; -toolc.l];
    toolc_offset_world = R_e * R_tool_neg * toolc_offset_local;
    
    % 法兰盘位置 = 尖端位置 + 偏移 (法兰盘在尖端上方)
    flange = toolc_tip + toolc_offset_world';
    
    % ========== 计算ToolR尖端位置 ==========
    % 从法兰盘正向计算ToolR尖端:
    % ToolR尖端 = 法兰盘位置 + R_e * troty(-tool_a) * [tool_h; tool_y_r; tool_l]
    % 注意: toolr.y 是负值 (-16.59mm)
    
    toolr_offset_local = [toolr.h; toolr.y; toolr.l];
    toolr_offset_world = R_e * R_tool_neg * toolr_offset_local;
    
    % ToolR尖端 = 法兰盘 + 偏移
    toolr_tip = flange + toolr_offset_world';
    
    % ========== 计算锥面顶点和轴向 ==========
    % 锥面模型 (喷嘴周围的检测区域):
    %
    %            /       \
    %           /         \     ← 锥面向远离喷嘴的方向展开
    %          /           \
    %         -------▲-------  ← 锥面顶点 (cone_apex)，在尖端上方
    %                |
    %           喷嘴尖端 (toolc_tip)
    %                ↓
    %          tool_z_axis (喷嘴出口方向，指向下方)
    
    if nargin >= 6 && isfield(cone_params, 'apex_offset')
        apex_offset = cone_params.apex_offset;
    else
        apex_offset = 0.5e-3;  % 默认0.5mm
    end
    
    % 锥面顶点在喷嘴尖端"上方"（沿工具Z轴负方向偏移）
    % tool_z_axis指向喷嘴出口（向下），所以减去它就是向上
    cone_apex = toolc_tip - apex_offset * tool_z_axis;
    
    % 锥面轴向 = 工具Z轴的负方向（向远离喷嘴出口的方向展开，即向上）
    cone_axis = -tool_z_axis;
end
