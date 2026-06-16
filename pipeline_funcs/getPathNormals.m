function normals = getPathNormals(pathPoints, pointCloud_data)
    % pathPoints: Nx3 矩阵，包含路径的 [x, y, z]
    % pointCloud_data: 包含 X, Y, Z 的结构体或表格
    % normals: 返回 Nx3 的法向量矩阵
    
    numPoints = size(pathPoints, 1);
    normals = zeros(numPoints, 3);
    k = 30; % 邻域点数
    
    % 准备点云数据矩阵
    cloudPos = [pointCloud_data.X(:), pointCloud_data.Y(:), pointCloud_data.Z(:)];
    
    for i = 1:numPoints
        % 1. 获取当前路径点坐标
        qx = pathPoints(i,1);
        qy = pathPoints(i,2);
        qz = pathPoints(i,3);
        
        % 2. 查找最近邻点 (使用三维距离更准确)
        dists = sum((cloudPos - [qx, qy, qz]).^2, 2);
        [~, idx] = mink(dists, k);
        neighbors = cloudPos(idx, :);
        
        % 3. PCA (主成分分析) 计算法向量
        % 计算中心化点
        center = mean(neighbors, 1);
        centered = neighbors - center;
        
        % 计算协方差矩阵并进行特征分解
        covariance = (centered' * centered) ./ (k-1);
        [V, D] = eig(covariance);
        
        % 提取最小特征值对应的向量 (即法向量)
        [~, minIdx] = min(diag(D));
        normal = V(:, minIdx)';
        
        % 4. 方向一致性处理：确保法向量朝上 (nz > 0)
        % 对接运动学代码：法向量朝上，troty(pi)会处理下压动作
        if normal(3) > 0
            normal = -normal;
        end
        
        % 归一化
        normals(i, :) = normal / norm(normal);
    end
end