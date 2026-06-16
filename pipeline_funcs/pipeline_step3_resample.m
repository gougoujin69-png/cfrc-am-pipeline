function pipeline_step3_resample(file_inout, target_spacing)
%PIPELINE_STEP3_RESAMPLE 路径等间距重采样 + 碳纤维 Z 投影到曲面
%   对 connected_paths 和 resin_connected_paths 分别重采样到 target_spacing
%   间距, 其中:
%     - 碳纤维路径 (connected_paths): Z 值用 interp2/scatteredInterpolant
%       投影到点云曲面, 确保紧贴打印基底
%     - 树脂路径 (resin_connected_paths): Z 值仅做线性插值, 保持原层高
%
%   修复原 Step3_1 只处理 connected_paths 的 bug, 现在树脂也会被重采样.
%
%   输入:
%     file_inout      - mat 文件路径 (原地读写)
%     target_spacing  - 重采样间距 (mm)

    S = load(file_inout);
    all_layers_data = S.all_layers_data;
    n = length(all_layers_data);

    fields_to_resample = {'connected_paths', 'resin_connected_paths'};

    for L = 1:n
        pc = all_layers_data(L).pointCloud_data;
        for fi = 1:length(fields_to_resample)
            f = fields_to_resample{fi};
            if ~isfield(all_layers_data(L), f), continue; end
            paths = all_layers_data(L).(f);
            project_z = strcmp(f, 'connected_paths');  % 仅碳纤维投影

            for i = 1:length(paths)
                op = paths{i};
                if isempty(op) || size(op, 1) < 2, continue; end

                % --- 1. 数据清洗: 去除距离过近的重复点 ---
                d = sqrt(sum(diff(op(:, 1:3)).^2, 2));
                keep = [true; d > 1e-6];
                op = op(keep, :);
                if size(op, 1) < 2, paths{i} = op; continue; end

                % --- 2. 等弧长重采样准备 ---
                diffs = diff(op(:, 1:3));
                segs = sqrt(sum(diffs.^2, 2));
                cd = [0; cumsum(segs)];
                td = cd(end);
                [cdu, um] = unique(cd);
                pp = op(um, :);
                nn = max(2, round(td / target_spacing));
                nq = linspace(0, td, nn);

                % --- 3. X, Y 线性插值 ---
                nx = interp1(cdu, pp(:, 1), nq, 'linear')';
                ny = interp1(cdu, pp(:, 2), nq, 'linear')';

                % --- 4. Z 处理: 碳纤维投影 / 树脂保留 ---
                if project_z
                    try
                        nz = interp2(pc.X, pc.Y, pc.Z, nx, ny, 'linear');
                        if any(isnan(nz))
                            Ff = scatteredInterpolant(pc.X(:), pc.Y(:), pc.Z(:), 'nearest', 'nearest');
                            m = isnan(nz);
                            nz(m) = Ff(nx(m), ny(m));
                        end
                    catch
                        Ff = scatteredInterpolant(pc.X(:), pc.Y(:), pc.Z(:), 'linear', 'nearest');
                        nz = Ff(nx, ny);
                    end
                else
                    nz = interp1(cdu, pp(:, 3), nq, 'linear')';
                end

                paths{i} = [nx, ny, nz];
            end
            all_layers_data(L).(f) = paths;
        end
    end
    save(file_inout, 'all_layers_data', '-v7.3');
    fprintf('  Resampled (spacing=%.2fmm), saved -> %s\n', target_spacing, file_inout);
end
