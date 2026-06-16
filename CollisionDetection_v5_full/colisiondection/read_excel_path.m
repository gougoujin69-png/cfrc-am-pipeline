function [path_data, excel_name] = read_excel_path(excel_input)
%% READ_EXCEL_PATH 读取Excel路径数据
%
% 输入:
%   excel_input - Excel文件完整路径，或包含Excel文件的目录路径
%
% Excel格式 (8列，无表头):
%   列1: X坐标 (mm)
%   列2: Y坐标 (mm)
%   列3: Z坐标 (mm)
%   列4: 挤出参数
%   列5: Z轴旋转角度 (度)
%   列6-8: 信号参数

    % 判断输入是文件还是目录
    if exist(excel_input, 'file') == 2
        % 输入是文件路径
        excel_path = excel_input;
        [~, name, ext] = fileparts(excel_path);
        excel_name = [name, ext];
    elseif exist(excel_input, 'dir') == 7
        % 输入是目录，查找Excel文件
        excel_files = dir(fullfile(excel_input, '*.xlsx'));
        if isempty(excel_files)
            excel_files = dir(fullfile(excel_input, '*.xls'));
        end
        
        if isempty(excel_files)
            error('未找到Excel文件！请上传.xlsx或.xls格式的路径文件');
        end
        
        [~, newest_idx] = max([excel_files.datenum]);
        excel_path = fullfile(excel_input, excel_files(newest_idx).name);
        excel_name = excel_files(newest_idx).name;
    else
        error('输入路径无效: %s', excel_input);
    end
    
    % 读取数据
    try
        raw_data = readmatrix(excel_path);
    catch
        try
            raw_data = xlsread(excel_path);
        catch ME
            error('无法读取Excel文件: %s', ME.message);
        end
    end
    
    % 移除NaN行
    raw_data = raw_data(~any(isnan(raw_data(:,1:3)), 2), :);
    
    if isempty(raw_data)
        error('Excel文件中没有有效数据');
    end
    
    num_points = size(raw_data, 1);
    num_cols = size(raw_data, 2);
    
    % 解析坐标 (列1-3, mm)
    path_data.xyz_mm = raw_data(:, 1:3);
    path_data.xyz_m = raw_data(:, 1:3) / 1000;
    
    % 解析姿态角
    % 列4: Rx - 绕X轴旋转角度 (度)
    % 列5: Ry - 绕Y轴旋转角度 (度)
    if num_cols >= 5
        path_data.Rx_deg = raw_data(:, 4);           % 列4是度
        path_data.Rx_rad = deg2rad(raw_data(:, 4));  % 转换为弧度
        path_data.Ry_deg = raw_data(:, 5);           % 列5是度
        path_data.Ry_rad = deg2rad(raw_data(:, 5));  % 转换为弧度
    else
        path_data.Rx_deg = zeros(num_points, 1);
        path_data.Rx_rad = zeros(num_points, 1);
        path_data.Ry_deg = zeros(num_points, 1);
        path_data.Ry_rad = zeros(num_points, 1);
    end
    
    % 解析信号参数 (列6-8)
    if num_cols >= 8
        path_data.signals = raw_data(:, 6:8);
    else
        path_data.signals = ones(num_points, 3);
    end
    
    path_data.num_points = num_points;
    path_data.raw_data = raw_data;
end
