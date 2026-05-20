function [X_fix, Y_fix, Z_fix] = trim_offset_surface(X_offset, Y_offset, Z_offset, ...
    X_prev, Y_prev, Z_prev, direction)
%% TRIM_OFFSET_SURFACE - 修复单层偏置曲面的自交和层间交错
%
% 输入:
%   X_offset, Y_offset, Z_offset - 当前层偏置曲面 (参数空间矩阵)
%   X_prev, Y_prev, Z_prev       - 上一层偏置曲面 (参数空间矩阵)
%                                   首层传入空矩阵 []
%   direction                     - 'up' 或 'down'
%
% 输出:
%   X_fix, Y_fix, Z_fix - 修复后的曲面 (交错区域设为 NaN)
%
% 用法示例（嵌入 slice_refined_model_complete.m）:
%   [activated_grids, X_offset, Y_offset, Z_offset] = detect_activation_refined(...);
%   if num_layers >= 2
%       prev = surface_layers{num_layers-1};
%       [X_offset, Y_offset, Z_offset] = trim_offset_surface(...
%           X_offset, Y_offset, Z_offset, ...
%           prev.X_surf, prev.Y_surf, prev.Z_surf, 'up');
%   end

    X_fix = X_offset;
    Y_fix = Y_offset;
    Z_fix = Z_offset;
    
    if isempty(X_offset) || all(isnan(X_offset(:)))
        return;
    end
    
    %% ---- 第1步：自交检测（Jacobian行列式法）----
    [dXdi, dXdj] = gradient(X_offset);
    [dYdi, dYdj] = gradient(Y_offset);
    
    detJ = dXdi .* dYdj - dXdj .* dYdi;
    
    % det(J) <= 0 表示曲面翻转
    fold_mask = (detJ <= 0);
    
    % 膨胀1像素确保边界干净
    if any(fold_mask(:))
        se = ones(3);
        fold_mask = conv2(double(fold_mask), se, 'same') > 0;
        
        X_fix(fold_mask) = NaN;
        Y_fix(fold_mask) = NaN;
        Z_fix(fold_mask) = NaN;
    end
    
    %% ---- 第2步：层间交错修复 ----
    if isempty(X_prev) || all(isnan(X_prev(:)))
        return;  % 没有上一层，跳过
    end
    
    % 构建公共网格
    valid_cur  = ~isnan(X_fix) & ~isnan(Y_fix) & ~isnan(Z_fix);
    valid_prev = ~isnan(X_prev) & ~isnan(Y_prev) & ~isnan(Z_prev);
    
    if sum(valid_cur(:)) < 10 || sum(valid_prev(:)) < 10
        return;
    end
    
    % 确定公共范围
    all_x = [X_fix(valid_cur); X_prev(valid_prev)];
    all_y = [Y_fix(valid_cur); Y_prev(valid_prev)];
    
    N_grid = 200;  % 公共网格分辨率
    xc = linspace(min(all_x), max(all_x), N_grid);
    yc = linspace(min(all_y), max(all_y), N_grid);
    [Xc, Yc] = meshgrid(xc, yc);
    
    % 投影到公共网格
    F_cur = scatteredInterpolant(X_fix(valid_cur), Y_fix(valid_cur), ...
        Z_fix(valid_cur), 'linear', 'none');
    F_prev = scatteredInterpolant(X_prev(valid_prev), Y_prev(valid_prev), ...
        Z_prev(valid_prev), 'linear', 'none');
    
    Zc_cur  = F_cur(Xc, Yc);
    Zc_prev = F_prev(Xc, Yc);
    
    both_valid = ~isnan(Zc_cur) & ~isnan(Zc_prev);
    
    % 检测交错
    if strcmpi(direction, 'up')
        % 向上偏移：当前层 Z 应 >= 上一层 Z
        violation = both_valid & (Zc_cur < Zc_prev);
    else
        % 向下偏移：当前层 Z 应 <= 上一层 Z
        violation = both_valid & (Zc_cur > Zc_prev);
    end
    
    if ~any(violation(:))
        return;
    end
    
    % 膨胀裁剪区域
    se = strel('disk', 2);
    violation = imdilate(violation, se) & both_valid;
    
    % 构建裁剪掩模的插值器，映射回参数空间
    trim_indicator = ones(size(Xc));
    trim_indicator(violation) = 0;
    trim_indicator(isnan(Zc_cur)) = NaN;
    
    F_trim = scatteredInterpolant(Xc(:), Yc(:), trim_indicator(:), 'nearest', 'nearest');
    
    keep_flag = F_trim(X_fix(valid_cur), Y_fix(valid_cur));
    trim_pts = keep_flag < 0.5;
    
    if any(trim_pts)
        full_trim = false(size(X_fix));
        valid_indices = find(valid_cur);
        full_trim(valid_indices(trim_pts)) = true;
        
        X_fix(full_trim) = NaN;
        Y_fix(full_trim) = NaN;
        Z_fix(full_trim) = NaN;
    end
end
