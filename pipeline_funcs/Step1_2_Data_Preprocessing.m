%% ========== 第一项：任意层的路径与旋转处理 ==========
all_layers_data2 = all_layers_data;
num_layers = length(all_layers_data);
fprintf('Loaded: %d layers\n\n', num_layers);
% --- 1. 定义旋转矩阵 ---
theta_z = deg2rad(90);  % Z轴旋转角度
theta_x = deg2rad(165);  % X轴旋转角度
TransX=30;
TransY=70;
TransZ=130;
% 绕Z轴旋转矩阵
Rz = [cos(theta_z), -sin(theta_z), 0;
      sin(theta_z),  cos(theta_z), 0;
      0,             0,            1];

% 绕X轴旋转矩阵
Rx = [1, 0,            0;
      0, cos(theta_x), -sin(theta_x);
      0, sin(theta_x),  cos(theta_x)];

% 总旋转矩阵 (先Z后X，矩阵乘法顺序为 Rx * Rz)
R_total = Rx * Rz;

for layer = 1:num_layers
    paths_3d = all_layers_data(layer).paths_3d;
    paths_3dr = all_layers_data(layer).paths_3dr;
    pointCloud_data = all_layers_data(layer).pointCloud_data;
    
    % 初始化存储容器
    paths_3d2 = cell(size(paths_3d));
    paths_3dr2 = cell(size(paths_3dr));
    
    % --- 2. 处理 paths_3d ---
    for i = 1:length(paths_3d)
        temp_path = paths_3d{i};
        % 应用旋转 (点矩阵 * R_total')
        paths_3d2{i} = temp_path * R_total';
    end
    
    % --- 3. 处理 paths_3dr ---
    for i = 1:length(paths_3dr)
        temp_path = paths_3dr{i};
        % 原始Z轴翻转处理
        % 应用旋转
        paths_3dr2{i} = temp_path * R_total';
    end
    
    % --- 4. 处理 pointCloud_data ---
    % 先进行Z轴翻转
    X = pointCloud_data.X;
    Y = pointCloud_data.Y;
    Z = pointCloud_data.Z;
    
    % 将点云展平为 Nx3 矩阵以便进行矩阵运算
    pts = [X(:), Y(:), Z(:)];
    % 应用旋转
    pts_rotated = pts * R_total';
    
    % 重新构造回原始的 X Y Z 形状
    pointCloud_data2.X = reshape(pts_rotated(:,1), size(X));
    pointCloud_data2.Y = reshape(pts_rotated(:,2), size(Y));
    pointCloud_data2.Z = reshape(pts_rotated(:,3), size(Z));
    
    % --- 5. 写回结构体 ---
    all_layers_data2(layer).paths_3d = paths_3d2;
    all_layers_data2(layer).paths_3dr = paths_3dr2;
    all_layers_data2(layer).pointCloud_data = pointCloud_data2;
end
all_layers_data = all_layers_data2;
%% ========== 第二项：任意层的路径与点集平移 ==========
for layer = 1:num_layers
    paths_3d = all_layers_data(layer).paths_3d;
    paths_3d2=paths_3d;
    paths_3dr = all_layers_data(layer).paths_3dr;
    paths_3dr2=paths_3dr;
    pointCloud_data=all_layers_data(layer).pointCloud_data;
    pointCloud_data2=pointCloud_data;
    for i=1:length(paths_3d)
        paths_3d2{i}=paths_3d{i};
        paths_3d2{i}(:,1)=paths_3d2{i}(:,1)+TransX;
        paths_3d2{i}(:,2)=paths_3d2{i}(:,2)+TransY;
        paths_3d2{i}(:,3)=paths_3d2{i}(:,3)+TransZ;
    end
    for i=1:length(paths_3dr)
        paths_3dr2{i}=paths_3dr{i};
        paths_3dr2{i}(:,1)=paths_3dr2{i}(:,1)+TransX;
        paths_3dr2{i}(:,2)=paths_3dr2{i}(:,2)+TransY;
        paths_3dr2{i}(:,3)=paths_3dr2{i}(:,3)+TransZ;
    end
    pointCloud_data2.X=pointCloud_data.X+TransX;
    pointCloud_data2.Y=pointCloud_data.Y+TransY;
    pointCloud_data2.Z=pointCloud_data.Z+TransZ;    
    all_layers_data2(layer).paths_3d=paths_3d2; 
    all_layers_data2(layer).paths_3dr=paths_3dr2;
    all_layers_data2(layer).pointCloud_data=pointCloud_data2;   
end
all_layers_data = all_layers_data2;
%% ========== 第三项：自行选择是否需要倒叙struct结构 ==========
for i=1:num_layers
    all_layers_data2(i)=all_layers_data(num_layers+1-i);
end
all_layers_data=all_layers_data2;
fprintf('Transformation complete.\n');
save('all_layers_path_carbon_resin2.mat', 'all_layers_data','-v7.3');

