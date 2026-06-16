% 加载数据
load('Manufacturing_printing_path.mat');
%此处添加根据需求调整路径
X_add=0;
Y_add=0;
Z_add=0;
%确定起始点和终点
%%根据第一层点云计算
%Point_test = all_layers_data(1).pointCloud_data;
%Xt = Point_test.X;
%Yt = Point_test.Y;
%Zt = Point_test.Z;
%xc = mean(Xt(:));
%yc = mean(Yt(:));
%zc = mean(Zt(:));
%Point_center = [xc, yc+150, zc+250];
% 遍历每一层
for layer_idx = 1:length(all_layers_data)
%for layer_idx = 1:2
    
    % 获取当前层的数据
    pathCell = all_layers_data(layer_idx).resin_connected_paths;
    pointCloud_data = all_layers_data(layer_idx).pointCloud_data;
    
    % 遍历当前层中的每一个 pathCell
    for cell_idx = 1:numel(pathCell)
        
        % 1. 构造当前 Cell 的文件名: path_层数_pathCell编号.xlsx
        filename = sprintf('resin_path_%d_%d.xlsx', layer_idx, cell_idx);
        
        % 2. 获取路径点数据 (mi x 3)
        current_path = pathCell{cell_idx}; 
        mi = size(current_path, 1);
        
        % 3. 批量计算该 Cell 内所有点的法向量 (提高效率)
        % 假设 getPathNormals 已修改为支持矩阵输入，返回 mi x 3
        all_normals = getPathNormals(current_path, pointCloud_data);
        
        % 4. 预分配输出矩阵 [x, y, z, v1, v2, v3, v4, v5]
        outCell = zeros(mi, 8);
        
        for j = 1:mi
            % 提取坐标
            x = current_path(j, 1)+X_add;
            y = current_path(j, 2)+Y_add;
            z = current_path(j, 3)+Z_add;
            
            % 提取对应的法向量分量
            nx = all_normals(j, 1);
            ny = all_normals(j, 2);
            nz = all_normals(j, 3);
            
            % 计算姿态角 (基于之前推导的公式)
            % YM(i,5) = v2, YM(i,4) = v1
            v2 = rad2deg(asin(-nx));
            v1 = rad2deg(atan2(-ny, -nz));
            
            % 其他固定参数
            v3 = 0;
            v4 = 23;
            v5 = 222;
            
            % 填充行
            outCell(j, :) = [x, y, z, v1, v2, v3, v4, v5];
        end
        
        % 5. 一次性写入当前 Cell 对应的独立 Excel 文件
        % 使用 writematrix，不指定 Range 则默认从 A1 开始写入
        writematrix(outCell, filename);
        
        fprintf('已生成文件: resin%s\n', filename);
    end
end