function [angle_4, angle_5] = calculate_angles_from_normal(nx, ny, nz)
% CALCULATE_ANGLES_FROM_NORMAL
% 输入：法向量 nx, ny, nz
% 输出：YM(i,4) 和 YM(i,5) 的角度值 (Degrees)
% 对应运动学链： ... * troty(pi) * trotx(angle_4) * troty(angle_5)

    % 1. 归一化法向量 (防止输入向量长度不为1)
    n = [nx, ny, nz];
    n_norm = norm(n);
    
    if n_norm == 0
        error('法向量不能为零向量');
    end
    
    nx = n(1) / n_norm;
    ny = n(2) / n_norm;
    nz = n(3) / n_norm;

    % 2. 计算 Angle 5 (beta)
    % 公式推导：nx = -sin(beta)
    % 增加数值保护，防止浮点误差导致 asin 输入超出 [-1, 1]
    val_beta = -nx;
    val_beta = max(min(val_beta, 1.0), -1.0); 
    beta_rad = asin(val_beta);

    % 3. 计算 Angle 4 (alpha)
    % 公式推导：ny = -sin(alpha)cos(beta), nz = -cos(alpha)cos(beta)
    % 使用 atan2(-ny, -nz)
    alpha_rad = atan2(-ny, -nz);

    % 4. 转换为角度 (Degrees)
    angle_4 = rad2deg(alpha_rad);
    angle_5 = rad2deg(beta_rad);
end