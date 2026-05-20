% 法向量计算函数
function normal = getNormalAtPoint(x, y, pointCloud_data)
    % 在点云中查找最近的k个点
    k = 9; % 使用9个最近点计算法向量
    
    % 计算所有点与目标点的距离
    dist = sqrt((pointCloud_data.X - x).^2 + (pointCloud_data.Y - y).^2);
    
    % 找到最近的k个点
    [~, idx] = mink(dist(:), k);
    
    % 提取这些点的坐标
    nearestX = pointCloud_data.X(idx);
    nearestY = pointCloud_data.Y(idx);
    nearestZ = pointCloud_data.Z(idx);
    
    % 计算这些点的中心
    center = [mean(nearestX), mean(nearestY), mean(nearestZ)];
    
    % 将点中心化
    centeredX = nearestX - center(1);
    centeredY = nearestY - center(2);
    centeredZ = nearestZ - center(3);
    
    % 构建协方差矩阵
    covariance = [centeredX, centeredY, centeredZ]' * [centeredX, centeredY, centeredZ];
    
    % 计算特征值和特征向量
    [eigenvectors, ~] = eig(covariance);
    
    % 最小特征值对应的特征向量就是法向量
    normal = eigenvectors(:, 1)';
    
    % 确保法向量朝上（z分量为正）
    if normal(3) < 0
        normal = -normal;
    end
    
    % 归一化法向量
    normal = normal / norm(normal);
end