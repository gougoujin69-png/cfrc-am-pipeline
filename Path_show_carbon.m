%%%%%path_show
all_layers_data=results.all_layers_data;
num_layers=length(all_layers_data);
%% ========== 第7步：三维整体可视化 ==========
fprintf('\n【步骤7】生成三维整体可视化...\n');

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
title('Printing Path', 'FontSize', 14);

colormap(jet);
c = colorbar;
c.Label.String = 'Layer Number';
caxis([1, num_layers]);
c.Position = [0.92, 0.2, 0.02, 0.6]; % [left bottom width height]，适当缩短高度
saveas(gcf, 'all_layers_3d_v3.png');
fprintf('  三维图已保存: all_layers_3d_v3.png\n');
%%%%%%选择一层显示路径
%选择层显示路径
m=10;
figure('Name', '多层路径生成结果', 'Position', [50, 50, 1800, 1000]);
layer_data = all_layers_data(mm);
hold on;
    for i = 1:length(layer_data.outer_contours)
        plot(layer_data.outer_contours{i}(:,1), layer_data.outer_contours{i}(:,2), ...
            'k-', 'LineWidth', 2.5);
    end

    % 绘制内轮廓
    for i = 1:length(layer_data.inner_contours)
        plot(layer_data.inner_contours{i}(:,1), layer_data.inner_contours{i}(:,2), ...
            'r-', 'LineWidth', 2);
    end
    % 绘制二维路径
    for i = 1:length(layer_data.paths_2d)
        pts = layer_data.paths_2d{i};
        if ~isempty(pts) && size(pts, 1) >= 2
            plot(pts(:,2), pts(:,1), 'b-', 'LineWidth', 1.5);
        end
end
axis equal tight;
grid off;
axis off;
hold off;
%%%%%单层三维路径
%选择层显示路径
m=10;
figure;
layer_data = all_layers_data(mm);
hold on;
    % 绘制二维路径
    for i = 1:length(layer_data.paths_3d)
        pts = layer_data.paths_3d{i};
        plot3(pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 1.5);
        hold on;
end
axis equal;
grid off;
axis off;
hold off;

