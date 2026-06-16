%根据更改路径信息生成excel表格
%输入基础路径：compensated_layers_data
%首先确定处理的层数
load('compensated_paths.mat');
layer_num=length(compensated_layers_data);
pathCell=compensated_layers_data(layer_num).compensated_paths;
pointCloud_data=compensated_layers_data(layer_num).pointCloud_data;
rowExcel = 1;   % Excel 当前写入行
for i = 1:numel(pathCell)
    filename = 'path.xlsx';
    data = pathCell{i};        % mi × 3
    mi = size(data,1);
    % 预分配当前 cell 的输出
    outCell = zeros(mi, 8);    % 3 + 5 列
    for j = 1:mi
        % ---------- 前三列 ----------
        xyz = data(j, :);      % [x y z]
        x = xyz(1);
        y = xyz(2);
        z = xyz(3);
        % ---------- 后五列（你自己写） ----------
        % ↓↓↓ 这里是你真正关心的部分 ↓↓↓
        normals = getPathNormals(xyz, pointCloud_data);
        nx=normals(1);
        ny=normals(2);
        nz=normals(3);
        v2 = rad2deg(asin(-nx));
        v1 = rad2deg(atan2(-ny, -nz));
        v3 = 1;
        v4 = 14;
        v5 = 111;
        % ---------- 拼成一行 ----------
        outCell(j,:) = [x, y, z, v1, v2, v3, v4, v5];
    end
    % 一次性写入 Excel
    range = sprintf('A%d:H%d', rowExcel, rowExcel + mi - 1);
    writematrix(outCell, filename, 'Range', range);
    %rowExcel = rowExcel + mi;
end

