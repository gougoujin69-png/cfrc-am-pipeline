function straightness = calculate_straightness(points)
    if size(points, 1) < 2
        straightness = 0;
        return;
    end
    straight_distance = norm(points(end,:) - points(1,:));
    actual_distance = sum(sqrt(sum(diff(points).^2, 2)));
    if actual_distance > 0
        straightness = straight_distance / actual_distance;
    else
        straightness = 0;
    end
end