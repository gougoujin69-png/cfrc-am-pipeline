function [filtered_extended_streamlines, valid_flags] = filterStreamlinesInsideContours(extended_streamlines, outer_contours, inner_contours)
% 仅保留流线在内外轮廓之间的最长连续段
% 输入:
%   extended_streamlines - 流线细胞数组 {N×2}
%   outer_contours - 外轮廓细胞数组 {N×2}
%   inner_contours - 内轮廓细胞数组 {N×2}
% 输出:
%   filtered_extended_streamlines - 过滤后的流线（仅保留最长有效段）
%   valid_flags - 每条流线是否完全位于内外轮廓之间（true 表示原流线完全在有效区域内）

    filtered_extended_streamlines = cell(size(extended_streamlines));
    valid_flags = false(length(extended_streamlines), 1);

    for i = 1:length(extended_streamlines)
        streamline = extended_streamlines{i};
        if isempty(streamline) || size(streamline, 1) < 2
            continue;
        end

        inside_flags = false(size(streamline, 1), 1);

        % 判断每个点是否在外轮廓内且不在内轮廓内
        for j = 1:size(streamline, 1)
            pt = streamline(j, :);
            in_outer = false;
            for k = 1:length(outer_contours)
                if inpolygon(pt(1), pt(2), outer_contours{k}(:,1), outer_contours{k}(:,2))
                    in_outer = true; break;
                end
            end

            in_inner = false;
            for k = 1:length(inner_contours)
                if inpolygon(pt(1), pt(2), inner_contours{k}(:,1), inner_contours{k}(:,2))
                    in_inner = true; break;
                end
            end

            inside_flags(j) = in_outer && ~in_inner;
        end

        % 如果所有点都在有效区域内，则直接保留整条流线
        if all(inside_flags)
            filtered_extended_streamlines{i} = streamline;
            valid_flags(i) = true;
            continue;
        end

        % 否则提取所有“在有效区域内”的连续段
        inside_segments = findContiguousSegments(inside_flags, streamline);

        if isempty(inside_segments)
            filtered_extended_streamlines{i} = [];
            continue;
        end

        % 选择最长的一段（点数最多）
        segment_lengths = cellfun(@(seg) size(seg,1), inside_segments);
        [~, idx_longest] = max(segment_lengths);

        filtered_extended_streamlines{i} = inside_segments{idx_longest};
    end

    fprintf('筛选完成: %d 条流线被保留（最长有效段）\n', sum(~cellfun(@isempty, filtered_extended_streamlines)));
end


function segments = findContiguousSegments(inside_flags, streamline)
% 找到连续的“在有效区域内”的段
    segments = {};
    n = length(inside_flags);
    i = 1;

    while i <= n
        if inside_flags(i)
            start_idx = i;
            while i <= n && inside_flags(i)
                i = i + 1;
            end
            end_idx = i - 1;

            if end_idx - start_idx >= 1
                segments{end+1} = streamline(start_idx:end_idx, :); %#ok<AGROW>
            end
        else
            i = i + 1;
        end
    end
end
