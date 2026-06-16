function pipeline_step3_excel(file_in, in_field, prefix, tag, offset, K)
%PIPELINE_STEP3_EXCEL 生成机械臂可读的 8 列 Excel 文件
%   合并原 Step3_2_Robotic_carbon_path 和 Step3_2_Robotic_Resin_path,
%   通过 in_field/prefix/tag 参数区分碳纤维与树脂.
%
%   每条路径生成一个 Excel, 文件名: <prefix>_<layer>_<cell>.xlsx
%   列定义: [x, y, z, v1, v2, v3, v4, v5]
%     x,y,z - 路径点坐标 (mm), 加 offset 偏移
%     v1    - 绕 X 轴角 alpha (deg), = rad2deg(atan2(-ny, -nz))
%     v2    - 绕 Y 轴角 beta  (deg), = rad2deg(asin(-nx))
%     v3,v4,v5 - 由 tag 参数指定 (动作/工具号/参数集编号)
%
%   末端姿态对应运动学链: ... * troty(pi) * trotx(v1) * troty(v2)
%
%   输入:
%     file_in   - mat 文件路径
%     in_field  - 读入字段名, 'connected_paths' 或 'resin_connected_paths'
%     prefix    - Excel 文件名前缀, 'carbon_path' 或 'resin_path'
%     tag       - [v3, v4, v5] 三个固定参数
%     offset    - [dx, dy, dz] 路径整体平移
%     K         - 法向量估计的 KNN 邻域数

    S = load(file_in);
    all_layers_data = S.all_layers_data;
    n = length(all_layers_data);
    count = 0;

    for L = 1:n
        if ~isfield(all_layers_data(L), in_field), continue; end
        paths = all_layers_data(L).(in_field);
        pc = all_layers_data(L).pointCloud_data;

        for ci = 1:numel(paths)
            current = paths{ci};
            if isempty(current), continue; end
            m = size(current, 1);

            % 批量计算法向量
            normals = local_path_normals(current, pc, K);

            % 拼成 8 列输出矩阵
            out = zeros(m, 8);
            for j = 1:m
                x = current(j, 1) + offset(1);
                y = current(j, 2) + offset(2);
                z = current(j, 3) + offset(3);
                nx = normals(j, 1); ny = normals(j, 2); nz = normals(j, 3);
                v2 = rad2deg(asin(max(-1, min(1, -nx))));   % beta  (clip 防 NaN)
                v1 = rad2deg(atan2(-ny, -nz));              % alpha
                out(j, :) = [x, y, z, v1, v2, tag(1), tag(2), tag(3)];
            end

            fname = sprintf('%s_%d_%d.xlsx', prefix, L, ci);
            writematrix(out, fname);
            count = count + 1;
        end
    end
    fprintf('  Generated %d %s_*.xlsx files\n', count, prefix);
end

%% ==================== LOCAL HELPER ====================

function normals = local_path_normals(pathPts, pc, K)
%LOCAL_PATH_NORMALS PCA 估计路径点上的曲面法向量
%   对每个路径点, 在点云中取 K 个最近邻, PCA 求最小特征值对应特征向量
%   即法向量; 强制 nz < 0 (法向量朝下, 配合 troty(pi) 实现下压姿态).
    np = size(pathPts, 1); normals = zeros(np, 3);
    cp = [pc.X(:), pc.Y(:), pc.Z(:)];
    Kuse = min(K, size(cp, 1));
    for i = 1:np
        q = pathPts(i, :);
        d = sum((cp - q).^2, 2);
        [~, idx] = mink(d, Kuse);
        nb = cp(idx, :);
        c = mean(nb, 1); ct = nb - c;
        cov_m = (ct' * ct) / max(1, Kuse - 1);
        [V, D] = eig(cov_m);
        [~, mi] = min(diag(D));
        nv = V(:, mi)';
        if nv(3) > 0, nv = -nv; end
        nrm = norm(nv);
        if nrm > 1e-12, normals(i, :) = nv / nrm;
        else, normals(i, :) = [0, 0, -1]; end
    end
end
