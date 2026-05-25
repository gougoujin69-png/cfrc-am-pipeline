function [xold, t, nelx, nely, nelz] = extract_layer_2d_projection(activated_grids, grid_data)
[nelx, nely, nelz] = size(grid_data);
xold = zeros(nely, nelx);
t    = zeros(nely, nelx);
num_grids = length(activated_grids);
for i = 1:num_grids
    g = activated_grids(i);
    % --- FIX: 使用 1-based 整数索引 grid_index，而非物理坐标 .x/.y ---
    % grid_index 由 detect_activation_on_surface 写入（slice_refined_model_v6.m L858）
    % 在 [1, nelx] x [1, nely] 范围内，对任意 REFINE_FACTOR 都安全
    ix = g.grid_index(1);   % i (x direction), in [1, nelx]
    iy = g.grid_index(2);   % j (y direction), in [1, nely]
    xold(iy, ix) = 1;
    t(iy, ix)    = g.t_xoy;
end
end