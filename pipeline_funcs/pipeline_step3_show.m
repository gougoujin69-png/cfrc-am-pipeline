function pipeline_step3_show(xlsx_file)
%PIPELINE_STEP3_SHOW 读取 Excel, 绘制路径 + 末端 Z 轴方向
%   验证生成的 Excel 文件中, 每个路径点对应的末端执行器姿态是否合理.
%   末端 Z 轴方向由旋转矩阵 R = Ry(pi) * Rx(alpha) * Ry(beta) 的第 3 列给出.
%
%   输入:
%     xlsx_file - Excel 文件路径, 8 列 [x,y,z,v1,v2,v3,v4,v5]
%                 若该文件不存在, 自动找当前目录第一个 carbon_path_*.xlsx

    % 若指定文件不存在, 自动 fallback 到第一个存在的 carbon_path_*.xlsx
    if ~exist(xlsx_file, 'file')
        cand = dir('carbon_path_*.xlsx');
        if isempty(cand)
            cand = dir('resin_path_*.xlsx');
        end
        if isempty(cand)
            warning('No path Excel files found in %s', pwd);
            return;
        end
        xlsx_file = cand(1).name;
        fprintf('  (auto-picked %s for visualization)\n', xlsx_file);
    end

    YM = readmatrix(xlsx_file);
    if isempty(YM) || size(YM, 2) < 5
        warning('File %s has no valid data', xlsx_file);
        return;
    end
    px = YM(:, 1); py = YM(:, 2); pz = YM(:, 3);
    np = size(YM, 1);
    vx = zeros(np, 1); vy = zeros(np, 1); vz = zeros(np, 1);

    for i = 1:np
        a4 = deg2rad(YM(i, 4));  % alpha
        a5 = deg2rad(YM(i, 5));  % beta

        Ry_pi = [cos(pi), 0, sin(pi); 0, 1, 0; -sin(pi), 0, cos(pi)];
        Rx_a4 = [1, 0, 0; 0, cos(a4), -sin(a4); 0, sin(a4), cos(a4)];
        Ry_a5 = [cos(a5), 0, sin(a5); 0, 1, 0; -sin(a5), 0, cos(a5)];
        R = Ry_pi * Rx_a4 * Ry_a5;

        vx(i) = R(1, 3); vy(i) = R(2, 3); vz(i) = R(3, 3);
    end

    figure('Color', 'w', 'Name', xlsx_file);
    plot3(px, py, pz, 'b-', 'LineWidth', 1.5); hold on;
    quiver3(px, py, pz, vx, vy, vz, 2, 'r');
    grid on; axis equal;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(['Path + End-Effector Z axis: ' xlsx_file], 'Interpreter', 'none');
    legend('Path', 'End-effector Z', 'Location', 'best');
    view(45, 30);
end
