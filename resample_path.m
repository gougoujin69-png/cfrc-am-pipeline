function dense_path = resample_path(path, target_spacing, preserve_ends)
    if isempty(path)
        dense_path = [];
        return;
    end
    
    valid_idx = all(isfinite(path), 2);
    path = path(valid_idx, :);
    
    if size(path, 1) < 2
        dense_path = path;
        return;
    end
    
    [~, unique_idx] = unique(path, 'rows', 'stable');
    path = path(unique_idx, :);
    
    if size(path, 1) < 2
        dense_path = path;
        return;
    end
    
    d = sqrt(sum(diff(path).^2, 2));
    s = [0; cumsum(d)];
    
    if s(end) < eps || ~isfinite(s(end))
        dense_path = path;
        return;
    end
    
    n_samples = max(ceil(s(end) / target_spacing), 2);
    
    try
        xq = interp1(s, path(:,1), linspace(0, s(end), n_samples), 'linear');
        yq = interp1(s, path(:,2), linspace(0, s(end), n_samples), 'linear');
        dense_path = [xq(:), yq(:)];
    catch
        dense_path = path;
        return;
    end
    
    if preserve_ends && size(dense_path, 1) >= 2
        dense_path(1,:) = path(1,:);
        dense_path(end,:) = path(end,:);
    end
end