function processed = preprocess_contour(contour)
    if size(contour, 1) > 200
        step = ceil(size(contour, 1) / 200);
        contour = contour(1:step:end, :);
    end
    
    if ~isequal(contour(1,:), contour(end,:))
        contour(end+1,:) = contour(1,:);
    end
    
    window_size = min(5, floor(size(contour,1)/10));
    if window_size > 1 && size(contour,1) > window_size*2
        x_smooth = smooth(contour(:,1), window_size);
        y_smooth = smooth(contour(:,2), window_size);
        processed = [x_smooth, y_smooth];
    else
        processed = contour;
    end
end