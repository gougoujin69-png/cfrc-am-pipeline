%% 双喷头CFRC 3D打印碰撞检测系统 v3.3 - 主程序
% 
% 基于ABB IRB-1200机器人 & Duanliang.m脚本逻辑
% 
% Excel输入格式 (8列，无表头):
%   列1: X坐标 (mm) - ToolC尖端
%   列2: Y坐标 (mm) - ToolC尖端
%   列3: Z坐标 (mm) - ToolC尖端
%   列4: Rx - 绕X轴旋转角度 (度)
%   列5: Ry - 绕Y轴旋转角度 (度)
%   列6-8: 信号参数

clear; clc; close all;

fprintf('================================================================\n');
fprintf('   双喷头CFRC 3D打印碰撞检测系统 v3.3\n');
fprintf('================================================================\n\n');

%% ======================== 参数配置 ========================
% 工具参数 - 与Duanliang.m完全一致
config = get_default_config();

fprintf('工具参数:\n');
fprintf('  ToolC: h=%.2fmm, l=%.2fmm, y=+%.2fmm\n', ...
    config.toolc.h*1000, config.toolc.l*1000, config.toolc.y*1000);
fprintf('  ToolR: y=%.2fmm (与ToolC对称)\n', config.toolr.y*1000);
fprintf('  检测锥: 深度%.1fmm, 全角%.0f°\n', ...
    config.cone.depth*1000, rad2deg(config.cone.half_angle)*2);

%% ======================== 读取Excel数据 ========================
fprintf('\n--- 读取Excel路径数据 ---\n');

% 弹出文件选择对话框，选择Excel文件
[filename, filepath] = uigetfile({'*.xlsx;*.xls', 'Excel文件 (*.xlsx, *.xls)'}, '选择路径Excel文件');
if isequal(filename, 0)
    error('未选择文件！程序终止。');
end
excel_file = fullfile(filepath, filename);
[path_data, excel_name] = read_excel_path(excel_file);

fprintf('文件: %s\n', excel_name);
fprintf('路径点数: %d\n', path_data.num_points);
fprintf('坐标范围 (mm): X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]\n', ...
    min(path_data.xyz_mm(:,1)), max(path_data.xyz_mm(:,1)), ...
    min(path_data.xyz_mm(:,2)), max(path_data.xyz_mm(:,2)), ...
    min(path_data.xyz_mm(:,3)), max(path_data.xyz_mm(:,3)));
fprintf('姿态角范围: Rx[%.2f°, %.2f°], Ry[%.2f°, %.2f°]\n', ...
    min(path_data.Rx_deg), max(path_data.Rx_deg), ...
    min(path_data.Ry_deg), max(path_data.Ry_deg));

%% ======================== 碰撞检测 ========================
fprintf('\n--- 执行碰撞检测 ---\n');

results = run_collision_detection(path_data, config);

%% ======================== 可视化 ========================
fprintf('\n--- 生成可视化 ---\n');

visualize_results(path_data, results, config);

%% ======================== 输出报告 ========================
print_report(results, path_data.num_points);

%% ======================== 保存结果 ========================
output_dir = pwd;  % 保存到当前工作目录

% 保存MAT文件
save(fullfile(output_dir, 'collision_results_v3.mat'), ...
    'path_data', 'results', 'config');

% 保存图形
saveas(figure(1), fullfile(output_dir, 'collision_cloud_v3.png'));
saveas(figure(2), fullfile(output_dir, 'key_frames_v3.png'));
saveas(figure(3), fullfile(output_dir, 'analysis_v3.png'));

fprintf('\n结果已保存到: %s\n', output_dir);
fprintf('完成!\n');
