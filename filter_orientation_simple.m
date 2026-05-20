function t_filtered = filter_orientation_simple(t, xold, r, m)
% t:     Nx × Ny 的角度矩阵 (rad)
% xold:  Nx × Ny 的密度矩阵 [0,1]
% r:     滤波半径（整数像素）
% m:     迭代次数
%
% 在每次迭代后，将角度映射到 [-pi/2, pi/2]

    [nx, ny] = size(t);
    t_new = t;
        % 将角度规约到 [-pi/2, pi/2]
    t_new(t_new > pi/2)  = t_new(t_new > pi/2)  - pi;
    t_new(t_new < -pi/2) = t_new(t_new < -pi/2) + pi;
    t_filtered = t_new;
    % 预计算邻域偏移（不含自身偏移 (0,0)）
    [DX, DY] = meshgrid(-r:r, -r:r);
    dist2 = DX.^2 + DY.^2;
    mask = (dist2 <= r^2) & ~(DX==0 & DY==0);
    dx = DX(mask);
    dy = DY(mask);
    dists = sqrt(dx.^2 + dy.^2);

    for iter = 1:m
        t_new = zeros(nx, ny);

        % 当前分量矩阵
        C = cos(t_filtered);
        S = sin(t_filtered);

        for i = 1:nx
            for j = 1:ny
                sumC = C(i,j);
                sumS = S(i,j);
                sumW = 1;

                for k = 1:numel(dx)
                    ii = i + dx(k);
                    jj = j + dy(k);

                    if ii < 1 || ii > nx || jj < 1 || jj > ny
                        continue;
                    end

                    w = xold(ii, jj);
                    if w == 0
                        continue;
                    end

                    denom = dists(k) + 1;
                    sumC = sumC + w * C(ii, jj) / denom;
                    sumS = sumS + w * S(ii, jj) / denom;
                    sumW = sumW + w / denom;
                end

                avgC = sumC / sumW;
                avgS = sumS / sumW;
                t_new(i, j) = atan2(avgS, avgC);
            end
        end
    end
    t_filtered = t_new;
end
