function [outer_path, inner_path] = polyshape_to_cell(region_result)
%POLYSHAPE_TO_CELL_NESTED 提取 polyshape 的外轮廓和内轮廓（两层cell）
%
% 输入：
%   region_result —— 1x1 polyshape
% 输出：
%   outer_path —— 外轮廓点集 { [x y] }
%   inner_path —— 内轮廓点集 {{ [x y], [x y], ... }, ... }

    if ~isa(region_result, 'polyshape')
        error('输入必须是 polyshape 对象');
    end

    % 提取所有边界
    [x, y] = boundary(region_result);

    % 找出 NaN 分隔
    nan_idx = isnan(x) | isnan(y);
    idx = find(nan_idx);
    seg_start = [1; idx + 1];
    seg_end   = [idx - 1; numel(x)];

    % 删除越界
    seg_start(seg_start > numel(x)) = [];
    seg_end(seg_end < 1) = [];

    % 收集所有轮廓
    loops = {};
    for i = 1:numel(seg_start)
        xi = x(seg_start(i):seg_end(i));
        yi = y(seg_start(i):seg_end(i));
        loops{end+1} = [xi, yi];
    end

    % 第一段是外轮廓
    outer_path = loops(1);

    % 后面是内轮廓
    inner_loops = loops(2:end);

    % 两层 cell：每个内轮廓组
    inner_path = {};
    for i = 1:length(inner_loops)
        % 这里简单按每个 loop 单独成组，你可以根据需要改成嵌套关系
        inner_path{end+1} = {inner_loops{i}};
    end
end
