function pipeline_step1_transform(file_in, file_out, P)
%PIPELINE_STEP1_TRANSFORM 坐标变换 (缩放 + 旋转 + 平移 + 可选倒序)
%   合并原 Step1_1 (缩放) 与 Step1_2 (旋转+平移+倒序), 一次性处理:
%   - paths_2d / paths_2dr: 仅缩放
%   - paths_3d / paths_3dr: 缩放 + 旋转 + 平移
%   - pointCloud_data:      缩放 + 旋转 + 平移
%
%   输入:
%     file_in  - 切片输出 mat 文件路径
%     file_out - Step 1 输出 mat 文件路径
%     P        - 参数结构体, 字段:
%                  SCALE_FACTOR, ROT_Z_DEG, ROT_X_DEG, TRANS, REVERSE_LAYERS

    if ~exist(file_in, 'file')
        error('Slice input not found: %s', file_in);
    end
    S = load(file_in);
    data = S.all_layers_data;
    n = length(data);
    fprintf('  Loaded %d layers from %s\n', n, file_in);

    % --- 构造旋转矩阵 ---
    theta_z = deg2rad(P.ROT_Z_DEG);
    theta_x = deg2rad(P.ROT_X_DEG);
    Rz = [cos(theta_z), -sin(theta_z), 0;
          sin(theta_z),  cos(theta_z), 0;
          0,             0,            1];
    Rx = [1, 0,            0;
          0, cos(theta_x), -sin(theta_x);
          0, sin(theta_x),  cos(theta_x)];
    R = Rx * Rz;
    T = P.TRANS;

    out = data;
    for L = 1:n
        % --- 3D 路径: 缩放 + 旋转 + 平移 ---
        fields_3d = {'paths_3d', 'paths_3dr'};
        for fi = 1:numel(fields_3d)
            f = fields_3d{fi};
            if isfield(data, f)
                src = data(L).(f);
                dst = cell(size(src));
                for k = 1:length(src)
                    p = src{k} * P.SCALE_FACTOR;
                    p = p * R';
                    p(:,1) = p(:,1) + T(1);
                    p(:,2) = p(:,2) + T(2);
                    p(:,3) = p(:,3) + T(3);
                    dst{k} = p;
                end
                out(L).(f) = dst;
            end
        end

        % --- 2D 路径: 仅缩放 ---
        fields_2d = {'paths_2d', 'paths_2dr'};
        for fi = 1:numel(fields_2d)
            f = fields_2d{fi};
            if isfield(data, f)
                src = data(L).(f);
                dst = cell(size(src));
                for k = 1:length(src)
                    dst{k} = src{k} * P.SCALE_FACTOR;
                end
                out(L).(f) = dst;
            end
        end

        % --- 点云: 缩放 + 旋转 + 平移 ---
        pc = data(L).pointCloud_data;
        sz = size(pc.X);
        pts = [pc.X(:), pc.Y(:), pc.Z(:)] * P.SCALE_FACTOR;
        pts = pts * R';
        pts(:,1) = pts(:,1) + T(1);
        pts(:,2) = pts(:,2) + T(2);
        pts(:,3) = pts(:,3) + T(3);
        pc2.X = reshape(pts(:,1), sz);
        pc2.Y = reshape(pts(:,2), sz);
        pc2.Z = reshape(pts(:,3), sz);
        out(L).pointCloud_data = pc2;
    end

    if P.REVERSE_LAYERS
        out = out(end:-1:1);
        fprintf('  Reversed layer order\n');
    end

    all_layers_data = out; %#ok<NASGU>
    save(file_out, 'all_layers_data', '-v7.3');
    fprintf('  Saved -> %s\n', file_out);
end
