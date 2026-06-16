%%%%%%Step1_3 check data 此处检查数据
%% ========== 第一项：任意层的路径 ==========
testnum =4;
all_paths_2dr = all_layers_data(testnum).paths_2dr;
all_paths_3dr = all_layers_data(testnum).paths_3dr;
all_paths_2d = all_layers_data(testnum).paths_2d;
all_paths_3d = all_layers_data(testnum).paths_3d;
pointCloud_data=all_layers_data(testnum).pointCloud_data;
% 2D 路径图
figure('Name', sprintf('第 %d 层 2D 路径', testnum));
for i = 1:length(all_paths_2dr)
    if ~isempty(all_paths_2dr{i})
        plot(all_paths_2dr{i}(:,1), all_paths_2dr{i}(:,2), 'b-', 'LineWidth', 1);
        hold on;
    end
end
title(sprintf('第 %d 层 2D 路径 (树脂)', testnum));
axis equal;
hold off;

% 3D 路径图
figure('Name', sprintf('第 %d 层 3D 路径', testnum));
for i = 1:length(all_paths_3dr)
    pts = all_paths_3dr{i};
    if ~isempty(pts)
        plot3(pts(:,1), pts(:,2), pts(:,3), 'Color', 'blue', 'LineWidth', 0.5);
        hold on;
    end
end
for i = 1:length(all_paths_3d)
    pts = all_paths_3d{i};
    if ~isempty(pts)
        plot3(pts(:,1), pts(:,2), pts(:,3), 'Color', 'red', 'LineWidth', 1.5);
    end
end
title(sprintf('第 %d 层 3D 路径 (蓝:树脂 红:碳纤维)', testnum));
axis equal;
hold on;
 surf(pointCloud_data.X, pointCloud_data.Y, pointCloud_data.Z, ...
     'EdgeColor','none', ...
     'FaceAlpha',0.8);
axis equal
%zlim([0,60]);   % 例如
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off;
fprintf('全部完成！\n');

%% ========== 第2项：三维整体可视化 ==========
fprintf('\n【步骤7】生成三维整体可视化...\n');
num_layers=length(all_layers_data);
figure('Name', '全层三维路径 V3', 'Position', [100, 100, 1200, 900]);
hold on;

layer_colors = jet(num_layers);

for layer_idx = 1:num_layers
    layer_data = all_layers_data(layer_idx);
    
    if ~layer_data.success
        continue;
    end
    
    for i = 1:length(layer_data.paths_3d)
        pts = layer_data.paths_3d{i};
        if ~isempty(pts) && size(pts, 1) >= 2
            plot3(pts(:,1), pts(:,2), pts(:,3), ...
                'Color', layer_colors(layer_idx,:), 'LineWidth', 1);
        end
    end
end

hold off;
axis equal;
axis off;
view(-37.5, 30);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('全层三维路径 V3', 'FontSize', 14);

colormap(jet);
c = colorbar;
c.Label.String = '层号';
caxis([1, num_layers]);


%% ========== 第2项：三维树脂路径可视化 ==========
fprintf('\n【步骤7】生成三维树脂路径整体可视化...\n');
num_layers=length(all_layers_data);
figure('Name', '全层三维路径 V3', 'Position', [100, 100, 1200, 900]);
hold on;

layer_colors = jet(num_layers);

for layer_idx = 1:num_layers
    layer_data = all_layers_data(layer_idx);
    
    if ~layer_data.success
        continue;
    end
    
    for i = 1:length(layer_data.paths_3dr)
        pts = layer_data.paths_3dr{i};
        if ~isempty(pts) && size(pts, 1) >= 2
            plot3(pts(:,1), pts(:,2), pts(:,3), ...
                'Color', layer_colors(layer_idx,:), 'LineWidth', 1);
        end
    end
end

hold off;
axis equal;
axis off;
view(-37.5, 30);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('全层三维路径 V3', 'FontSize', 14);

colormap(jet);
c = colorbar;
c.Label.String = '层号';
caxis([1, num_layers]);