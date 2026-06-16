%%%此处生成STL模型
%% 1. 数据准备 (假设 X, Y, Z 是矩阵)
X = all_layers_data(1).pointCloud_data.X;
Y = all_layers_data(1).pointCloud_data.Y;
Z = all_layers_data(1).pointCloud_data.Z;
step = 5; % 根据精度需求调整
X = X(1:step:end, 1:step:end); 
Y = Y(1:step:end, 1:step:end); 
Z = Z(1:step:end, 1:step:end);
% 如果 Z 包含 NaN，建议先处理
Z(isnan(Z)) = 0; 

%% 2. 生成顶面和底面的 Patch 数据
% 使用 surf2patch 将网格转换为三角面片
[f_top, v_top] = surf2patch(X, Y, Z, 'triangles');
[f_bot, v_bot] = surf2patch(X, Y, zeros(size(Z)), 'triangles');

% 翻转底部的面法线（STL 要求法线向外）
f_bot = fliplr(f_bot);

%% 3. 生成侧壁
% 提取边界索引
[rows, cols] = size(Z);
% 顺时针/逆时针提取边界点下标
b_idx = [ (1:rows)', ones(rows,1);             % 左边缘
          rows*ones(cols-1,1), (2:cols)';      % 下边缘
          (rows-1:-1:1)', cols*ones(rows-1,1); % 右边缘
          ones(cols-2,1), (cols-1:-1:2)' ];    % 上边缘

% 转换为线性索引
lin_idx = sub2ind([rows, cols], b_idx(:,1), b_idx(:,2));

% 构建侧壁面片
v_num_half = numel(X);
f_side = [];
for i = 1:length(lin_idx)
    p1 = lin_idx(i);
    p2 = lin_idx(mod(i, length(lin_idx)) + 1);
    
    % 侧面由两个三角形组成 (顶1, 顶2, 底1) 和 (底1, 顶2, 底2)
    t1 = [p1, p2, p1 + v_num_half];
    t2 = [p1 + v_num_half, p2, p2 + v_num_half];
    f_side = [f_side; t1; t2];
end

%% 4. 合并所有几何体
vertices = [v_top; v_bot];
% 调整底部索引偏移
faces = [f_top; f_bot + v_num_half; f_side];

%% 5. 导出 STL
TR = triangulation(faces, vertices);

% 可视化检查
figure;
trisurf(TR, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
title('生成的封闭模型');

% 写入文件
stlwrite(TR, 'Solid_Model.stl');
fprintf('STL 文件已生成：Solid_Model.stl\n');