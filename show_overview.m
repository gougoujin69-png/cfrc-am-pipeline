function show_overview(surface_layers, grid_data, valid_grid_mask, ...
                       min_valid_x, max_valid_x, ...
                       min_valid_y, max_valid_y, ...
                       min_valid_z, max_valid_z, ...
                       nelx, nely, nelz)

    fprintf('\nGenerating publication-quality overview...\n');

    num_layers = length(surface_layers);

    %% ===================== Figure =====================
    fig = figure('Name','Overview (Publication Quality)', ...
                 'Color','w', ...
                 'Position',[50 50 1600 900]);

    %% ===================== 3D Overview =====================
    ax = subplot(2,2,[1 3]);
    hold(ax,'on');

    %% ---- Color setting (NO jet!) ----
    base_color = [0.1 0.4 0.8];   % 科研常用蓝
    layer_alpha = 0.15;

    %% ---- Plot surfaces ----
    for k = 1:num_layers
        S = surface_layers{k};
        surf(ax, S.X_surf, S.Y_surf, S.Z_surf, ...
            'FaceColor', base_color, ...
            'FaceAlpha', layer_alpha, ...
            'EdgeColor', 'none');
    end

    %% ---- Plot activated voxels as skeleton points ----
    sample_rate = max(1, floor(min([nelx nely nelz]) / 20));

    pts = [];
    for i = 1:sample_rate:nelx
        for j = 1:sample_rate:nely
            for k = 1:sample_rate:nelz
                if valid_grid_mask(i,j,k)
                    pts(end+1,:) = [ ...
                        grid_data(i,j,k).x, ...
                        grid_data(i,j,k).y, ...
                        grid_data(i,j,k).z ];
                end
            end
        end
    end

    plot3(ax, pts(:,1), pts(:,2), pts(:,3), '.', ...
          'Color',[0.2 0.2 0.2], ...
          'MarkerSize',6);

    %% ---- Axis & camera ----
    axis(ax, [min_valid_x max_valid_x ...
              min_valid_y max_valid_y ...
              min_valid_z max_valid_z]);
    axis(ax,'equal');
    axis(ax,'tight');

    xlabel(ax,'X','FontSize',12);
    ylabel(ax,'Y','FontSize',12);
    zlabel(ax,'Z','FontSize',12);

    view(ax,3);
    camproj(ax,'perspective');
    camzoom(ax,1.3);

    %% ---- Lighting (CRITICAL) ----
    lighting(ax,'gouraud');
    camlight(ax,'headlight');
    camlight(ax,'right');
    material(ax,'dull');

    grid(ax,'on');
    title(ax, sprintf('Global Overview (%d Layers)',num_layers), ...
          'FontSize',14,'FontWeight','bold');

    %% ===================== Statistics =====================
    subplot(2,2,2);
    layer_ids = 1:num_layers;
    activated = arrayfun(@(i) surface_layers{i}.total_activated, layer_ids);
    newly = arrayfun(@(i) surface_layers{i}.total_newly_activated, layer_ids);

    bar(layer_ids,[newly' activated'],'grouped');
    xlabel('Layer index');
    ylabel('Number of cells');
    title('Activation statistics','FontWeight','bold');
    legend({'New','Total'},'Location','northwest');
    grid on;

    %% ===================== Text Info =====================
    subplot(2,2,4);
    axis off;

    txt = sprintf([ ...
        'Number of layers : %d\n' ...
        'Offset range     : %.2f ~ %.2f\n' ...
        'Total new cells  : %d\n\n' ...
        'Layer   Offset   Attempts   New   Total\n' ...
        '----------------------------------------\n'], ...
        num_layers, ...
        surface_layers{1}.offset, ...
        surface_layers{end}.offset, ...
        sum(newly));

    for i = 1:min(8,num_layers)
        txt = [txt sprintf('%3d   %+6.2f     %2d     %4d   %4d\n', ...
            i, surface_layers{i}.offset, ...
            length(surface_layers{i}.attempts), ...
            surface_layers{i}.total_newly_activated, ...
            surface_layers{i}.total_activated)];
    end

    text(0.05,0.95,txt, ...
        'FontName','Courier', ...
        'FontSize',10, ...
        'VerticalAlignment','top');

    fprintf('✓ Publication-quality overview generated.\n\n');
end
