function segments = find_continuous_segments(logical_vec)
    segments = [];
    start_idx = find(diff([0; logical_vec; 0]) == 1);
    end_idx = find(diff([0; logical_vec; 0]) == -1) - 1;
    if ~isempty(start_idx)
        segments = [start_idx, end_idx];
    end
end