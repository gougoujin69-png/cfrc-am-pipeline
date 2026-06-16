function pipeline_step1_stl(file_in, file_out, step)
%PIPELINE_STEP1_STL 根据第 1 层点云生成封闭 STL 实体
%   把曲面点云缝合成顶面 + 底面 (Z=0) + 侧壁的封闭三角网格,
%   导出 STL 文件用于碰撞检查或基底导入.
%
%   输入:
%     file_in  - mat 文件路径 (含 all_layers_data, 取第 1 层点云)
%     file_out - 输出 STL 路径
%     step     - 点云下采样步长 (大: 模型粗糙快; 小: 精细慢)

    S = load(file_in);
    pc = S.all_layers_data(1).pointCloud_data;
    X = pc.X(1:step:end, 1:step:end);
    Y = pc.Y(1:step:end, 1:step:end);
    Z = pc.Z(1:step:end, 1:step:end);
    Z(isnan(Z)) = 0;

    % --- 顶面与底面三角面片 ---
    [f_top, v_top] = surf2patch(X, Y, Z, 'triangles');
    [f_bot, v_bot] = surf2patch(X, Y, zeros(size(Z)), 'triangles');
    f_bot = fliplr(f_bot);  % 翻转底面法线 (朝下)

    % --- 边界索引 (顺时针绕一圈) ---
    [rows, cols] = size(Z);
    b_idx = [(1:rows)',          ones(rows,1);
             rows*ones(cols-1,1), (2:cols)';
             (rows-1:-1:1)',     cols*ones(rows-1,1);
             ones(cols-2,1),     (cols-1:-1:2)'];
    lin_idx = sub2ind([rows, cols], b_idx(:,1), b_idx(:,2));

    % --- 侧壁三角面片 ---
    v_num_half = numel(X);
    f_side = zeros(2 * length(lin_idx), 3);
    for i = 1:length(lin_idx)
        p1 = lin_idx(i);
        p2 = lin_idx(mod(i, length(lin_idx)) + 1);
        f_side(2*i-1, :) = [p1,              p2,              p1 + v_num_half];
        f_side(2*i,   :) = [p1 + v_num_half, p2,              p2 + v_num_half];
    end

    % --- 合并并导出 ---
    vertices = [v_top; v_bot];
    faces    = [f_top; f_bot + v_num_half; f_side];
    TR = triangulation(faces, vertices);
    stlwrite(TR, file_out);
    fprintf('  STL saved -> %s (%d vertices, %d faces)\n', ...
        file_out, size(vertices,1), size(faces,1));
end
