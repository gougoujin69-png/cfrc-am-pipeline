function t_filtered = filter_orientation(t, xold, r, m)
% t:     Nx × Ny 的角度矩阵 (单位: rad)
% xold:  Nx × Ny 的密度矩阵 [0,1]
% r:     滤波半径（元素为单位距离）
% m:     滤波迭代次数
%
% t_filtered: 滤波后的角度矩阵

    t_filtered = t;
    [nx, ny] = size(t);
    
    % 预计算邻域索引偏移（加速）
    [dx, dy] = meshgrid(-r:r, -r:r);
    mask = (dx.^2 + dy.^2) <= r^2;
    dx = dx(mask);
    dy = dy(mask);
    numN = numel(dx);

    for iter = 1:m
        t_new = zeros(nx, ny);

        % 将角度转化为向量（避免角度 π 跳变问题）
        C = cos(t_filtered);
        S = sin(t_filtered);

        for i = 1:nx
            for j = 1:ny

                % 初始化加权和
                sumC = 0;
                sumS = 0;
                sumW = 0;

                for k = 1:numN
                    ii = i + dx(k);
                    jj = j + dy(k);

                    % 边界检查
                    if ii < 1 || ii > nx || jj < 1 || jj > ny
                        continue;
                    end

                    % 密度权重
                    w = xold(ii, jj);

                    sumC = sumC + C(ii, jj) * w;
                    sumS = sumS + S(ii, jj) * w;
                    sumW = sumW + w;
                end

                if sumW > 0
                    % 反转为角度
                    t_new(i, j) = atan2(sumS, sumC);
                else
                    t_new(i, j) = t_filtered(i, j);
                end
            end
        end

        t_filtered = t_new;
    end
end
