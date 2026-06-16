function pipeline_step1_visualize(file_in, layer_id)
%PIPELINE_STEP1_VISUALIZE 可视化检查任意层路径与点云
%   绘制指定层的 3D 树脂路径(蓝)、碳纤维路径(红)、参考点云曲面.
%   用于在做完坐标变换后, 检查坐标系是否对齐机械臂工作站.
%
%   输入:
%     file_in  - mat 文件路径 (Step 1 输出)
%     layer_id - 要可视化的层号

    S = load(file_in);
    data = S.all_layers_data;
    if layer_id > length(data)
        warning('layer_id 越界 (max=%d)', length(data));
        return;
    end
    d = data(layer_id);

    figure('Name', sprintf('Layer %d Check', layer_id));
    hold on;

    % 树脂路径 (蓝)
    if isfield(d, 'paths_3dr')
        for i = 1:length(d.paths_3dr)
            p = d.paths_3dr{i};
            if ~isempty(p)
                plot3(p(:,1), p(:,2), p(:,3), 'b-', 'LineWidth', 0.5);
            end
        end
    end

    % 碳纤维路径 (红)
    if isfield(d, 'paths_3d')
        for i = 1:length(d.paths_3d)
            p = d.paths_3d{i};
            if ~isempty(p)
                plot3(p(:,1), p(:,2), p(:,3), 'r-', 'LineWidth', 1.5);
            end
        end
    end

    % 点云曲面 (半透明)
    pc = d.pointCloud_data;
    surf(pc.X, pc.Y, pc.Z, 'EdgeColor', 'none', 'FaceAlpha', 0.3);

    axis equal; grid on; view(3);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(sprintf('Layer %d (blue=resin, red=carbon)', layer_id));
end
