function path_out = ensureClockwise(path)
    % ensureClockwise - 确保二维路径为顺时针方向
    % 输入：path (Nx2 double)
    % 输出：path_out (Nx2 double, 保证为CW)

    % ---------- 检查输入 ----------
    if ~ismatrix(path) || size(path,2) ~= 2
        error('输入路径必须是 Nx2 的矩阵');
    end

    % ---------- 提取坐标 ----------
    x = path(:,1);
    y = path(:,2);

    % ---------- 保证路径闭合 ----------
    if x(1) ~= x(end) || y(1) ~= y(end)
        x = [x; x(1)];
        y = [y; y(1)];
    end

    % ---------- 计算有向面积 ----------
    A = 0.5 * sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1));

    % ---------- 判断并反转 ----------
    if A > 0
        % A>0 表示逆时针，需要倒序
        disp('路径是逆时针，已转换为顺时针方向。');
        path_out = flipud(path);
    else
        disp('路径已是顺时针，无需修改。');
        path_out = path;
    end
end
