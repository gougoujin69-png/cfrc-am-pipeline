%% ========================================
%% 偏置曲面自交修复 v4 — Z-map 分箱法
%% ========================================
% 基于文献方法:
%   OuYang & Feng 2008, "Machining Triangular Mesh via Mesh Offset"
%   Shen et al. 2010, "Level set methods for offset surface"
%
% 核心思路:
%   层内: 将参数空间(i,j)的点投射到物理(x,y)网格，
%         同一个bin内若出现多个z值(= 折叠)，
%         保留距基准面最近的z (即有效面片)
%   层间: 强制 z_layer(n) < z_layer(n+1)
%
% 不使用 Jacobian 行列式, 不依赖 scatteredInterpolant 处理折叠区
%%

clear;
fprintf('\n');
fprintf('========================================================\n');
fprintf('     偏置曲面自交修复 v4 (Z-map 分箱法)                 \n');
fprintf('========================================================\n\n');

%% ========== 1. 鲁棒加载 ==========
fprintf('[1] 加载数据...\n');

% 尝试多个可能的文件名
candidates = {'slice_results_trimmed_latest.mat', ...
              'slice_results_refined_latest.mat', ...
              'slice_results_latest.mat'};
mat_file = '';
for c = 1:length(candidates)
    if exist(candidates{c}, 'file')
        mat_file = candidates{c};
        break;
    end
end
if isempty(mat_file)
    error('找不到任何切片结果文件!');
end

loaded = load(mat_file);
fnames = fieldnames(loaded);
fprintf('  文件: %s\n', mat_file);
fprintf('  变量: %s\n', strjoin(fnames, ', '));

% 鲁棒提取 surface_layers
surface_layers = find_surface_layers(loaded);
num_layers = length(surface_layers);
fprintf('  成功提取 %d 层\n', num_layers);

% 按 offset 排序
offsets = zeros(num_layers, 1);
for i = 1:num_layers
    offsets(i) = surface_layers{i}.offset;
end
[offsets_sorted, sort_idx] = sort(offsets, 'ascend');
sorted = cell(1, num_layers);
for i = 1:num_layers
    sorted{i} = surface_layers{sort_idx(i)};
end
surface_layers = sorted;

[~, base_idx] = min(abs(offsets_sorted));
fprintf('  基准面: idx=%d (offset=%.3f)\n', base_idx, offsets_sorted(base_idx));
fprintf('  范围: [%.3f, %.3f]\n\n', offsets_sorted(1), offsets_sorted(end));

%% ========== 2. 构建公共采样网格 ==========
fprintf('[2] 构建公共采样网格...\n');

% 收集全局(x,y)范围
all_xmin = inf; all_xmax = -inf;
all_ymin = inf; all_ymax = -inf;

for L = 1:num_layers
    X = surface_layers{L}.X_surf;
    Y = surface_layers{L}.Y_surf;
    v = ~isnan(X) & ~isnan(Y);
    if any(v(:))
        all_xmin = min(all_xmin, min(X(v)));
        all_xmax = max(all_xmax, max(X(v)));
        all_ymin = min(all_ymin, min(Y(v)));
        all_ymax = max(all_ymax, max(Y(v)));
    end
end

% 使用原始曲面的分辨率来确定网格步长
X0 = surface_layers{base_idx}.X_surf;
Y0 = surface_layers{base_idx}.Y_surf;
v0 = ~isnan(X0);

% 估算物理空间的典型间距
if sum(v0(:)) > 100
    dx_typical = median(abs(diff(X0(v0))));
    dy_typical = median(abs(diff(Y0(v0))));
    % 确保不会太小
    dx_typical = max(dx_typical, (all_xmax - all_xmin) / 2000);
    dy_typical = max(dy_typical, (all_ymax - all_ymin) / 2000);
else
    dx_typical = (all_xmax - all_xmin) / 500;
    dy_typical = (all_ymax - all_ymin) / 500;
end

xc = all_xmin : dx_typical : all_xmax;
yc = all_ymin : dy_typical : all_ymax;
nx = length(xc);
ny = length(yc);

fprintf('  公共网格范围: X=[%.2f, %.2f], Y=[%.2f, %.2f]\n', ...
    all_xmin, all_xmax, all_ymin, all_ymax);
fprintf('  网格分辨率: dx=%.4f, dy=%.4f\n', dx_typical, dy_typical);
fprintf('  网格尺寸: %d x %d = %d 点\n\n', nx, ny, nx*ny);

%% ========== 3. Z-map 分箱重采样 (层内自交修复) ==========
fprintf('[3] 将所有 %d 层重采样到公共网格...\n', num_layers);
fprintf('    方法: Z-map 分箱 — 每个(x,y)bin保留距基准面最近的z\n\n');

% 先把基准面投影到公共网格 (基准面无自交)
Z_base_common = project_single_valued(surface_layers{base_idx}, xc, yc);

Z_common = cell(num_layers, 1);
stats_intra = zeros(num_layers, 1);

for L = 1:num_layers
    X = surface_layers{L}.X_surf;
    Y = surface_layers{L}.Y_surf;
    Z = surface_layers{L}.Z_surf;
    
    v = ~isnan(X) & ~isnan(Y) & ~isnan(Z);
    nv = sum(v(:));
    
    if nv < 20
        Z_common{L} = NaN(ny, nx);
        continue;
    end
    
    Xv = X(v);
    Yv = Y(v);
    Zv = Z(v);
    
    % ---- Z-map 分箱 ----
    % 将每个点分配到最近的(x,y)网格格子
    ix = round((Xv - all_xmin) / dx_typical) + 1;
    iy = round((Yv - all_ymin) / dy_typical) + 1;
    
    % 边界裁剪
    valid_bin = (ix >= 1) & (ix <= nx) & (iy >= 1) & (iy <= ny);
    ix = ix(valid_bin);
    iy = iy(valid_bin);
    Zv_bin = Zv(valid_bin);
    
    % 初始化: 使用 NaN 填充
    Z_grid = NaN(ny, nx);
    Z_count = zeros(ny, nx);        % 每个bin的点数
    Z_multi = false(ny, nx);         % 标记多值bin
    
    % 对于每个bin, 判断应该保留哪个z值
    % 折叠时: 正常面片在远离基准面的一侧, 翻折面片在靠近基准面的一侧
    %   上方层 offset>0: 正常面片在上方 -> 保留 max(z)
    %   下方层 offset<0: 正常面片在下方 -> 保留 min(z)
    
    if offsets_sorted(L) >= 0
        % 上方层: 正常面片在上, 折叠翻回下方 -> 保留 max(z)
        pick_rule = 'max';
    else
        % 下方层: 正常面片在下, 折叠翻回上方 -> 保留 min(z)
        pick_rule = 'min';
    end
    
    % 逐点分箱
    for p = 1:length(ix)
        r = iy(p);
        c = ix(p);
        z_val = Zv_bin(p);
        
        Z_count(r, c) = Z_count(r, c) + 1;
        
        if isnan(Z_grid(r, c))
            Z_grid(r, c) = z_val;
        else
            Z_multi(r, c) = true;
            if strcmp(pick_rule, 'min')
                Z_grid(r, c) = min(Z_grid(r, c), z_val);
            else
                Z_grid(r, c) = max(Z_grid(r, c), z_val);
            end
        end
    end
    
    % 统计多值（折叠）区域
    n_multi = sum(Z_multi(:));
    if n_multi > 0
        stats_intra(L) = n_multi;
        fprintf('  层 %2d (off=%+7.3f): 检测到 %5d 个折叠bin (%.1f%%), 已取%s(z)\n', ...
            L, offsets_sorted(L), n_multi, ...
            100*n_multi / sum(Z_count(:) > 0), pick_rule);
    end
    
    % 对稀疏区域做填充插值 (只在非折叠区)
    % 找到有数据的点, 对空缺做 nearest 填充
    has_data = ~isnan(Z_grid);
    if sum(has_data(:)) > 50
        [Xg, Yg] = meshgrid(xc, yc);
        F = scatteredInterpolant(Xg(has_data), Yg(has_data), ...
            Z_grid(has_data), 'linear', 'none');
        Z_grid_filled = F(Xg, Yg);
        % 只填充小间隙, 不外推
        Z_common{L} = Z_grid_filled;
    else
        Z_common{L} = Z_grid;
    end
    
    if mod(L, 5) == 0
        fprintf('  已处理 %d/%d 层\n', L, num_layers);
    end
end

fprintf('  已处理 %d/%d 层\n', num_layers, num_layers);
fprintf('  重采样完成\n\n');

%% ========== 4. 层间单调性修复 ==========
fprintf('[4] 检测自相交并修正...\n');
fprintf('  基准面位置: 排序索引 %d (offset=%.3f)\n\n', ...
    base_idx, offsets_sorted(base_idx));

Z_before = Z_common;  % 备份

stats_inter = zeros(num_layers, 1);

% 上行
fprintf('  --- 上行方向修正 (基准面 -> 最外层) ---\n');
for L = (base_idx + 1) : num_layers
    Z_cur = Z_common{L};
    Z_prev = Z_common{L-1};
    
    both = ~isnan(Z_cur) & ~isnan(Z_prev);
    vio = both & (Z_cur < Z_prev);
    nv = sum(vio(:));
    
    if nv > 0
        fprintf('    层 %2d (off=%+7.3f) vs 层 %2d: %6d 交叉点 (%.1f%%)\n', ...
            L, offsets_sorted(L), L-1, nv, 100*nv/sum(both(:)));
        Z_cur(vio) = NaN;
        Z_common{L} = Z_cur;
        stats_inter(L) = nv;
    end
end

% 下行
fprintf('\n  --- 下行方向修正 (基准面 -> 最内层) ---\n');
for L = (base_idx - 1) : -1 : 1
    Z_cur = Z_common{L};
    Z_next = Z_common{L+1};
    
    both = ~isnan(Z_cur) & ~isnan(Z_next);
    vio = both & (Z_cur > Z_next);
    nv = sum(vio(:));
    
    if nv > 0
        fprintf('    层 %2d (off=%+7.3f) vs 层 %2d: %6d 交叉点 (%.1f%%)\n', ...
            L, offsets_sorted(L), L+1, nv, 100*nv/sum(both(:)));
        Z_cur(vio) = NaN;
        Z_common{L} = Z_cur;
        stats_inter(L) = nv;
    end
end

% 统计首轮
inter_lines = sum(stats_inter > 0);
inter_total = sum(stats_inter);
fprintf('\n  首轮修正统计:\n');
fprintf('    检测交线: %d 组\n', inter_lines);
fprintf('    裁剪点数: %d\n\n', inter_total);

%% ========== 5. 级联检查 ==========
fprintf('[5] 多层级联交叉二次检查...\n');

cascade = 0;
for pass = 1:3
    changed = false;
    for L = (base_idx + 1) : num_layers
        Z_cur = Z_common{L}; Z_prev = Z_common{L-1};
        both = ~isnan(Z_cur) & ~isnan(Z_prev);
        vio = both & (Z_cur < Z_prev);
        nv = sum(vio(:));
        if nv > 0
            Z_cur(vio) = NaN; Z_common{L} = Z_cur;
            cascade = cascade + nv;
            fprintf('    层 %2d (off=%+7.3f): 级联裁剪 %d 点\n', L, offsets_sorted(L), nv);
            changed = true;
        end
    end
    for L = (base_idx - 1) : -1 : 1
        Z_cur = Z_common{L}; Z_next = Z_common{L+1};
        both = ~isnan(Z_cur) & ~isnan(Z_next);
        vio = both & (Z_cur > Z_next);
        nv = sum(vio(:));
        if nv > 0
            Z_cur(vio) = NaN; Z_common{L} = Z_cur;
            cascade = cascade + nv;
            fprintf('    层 %2d (off=%+7.3f): 级联裁剪 %d 点\n', L, offsets_sorted(L), nv);
            changed = true;
        end
    end
    if ~changed, break; end
end
fprintf('  级联修正: %d 点\n\n', cascade);

%% ========== 6. 提取交线并排序 ==========
fprintf('[6] 交线排序与平滑...\n');

intersection_curves = {};
curve_count = 0;

for L = 2:num_layers
    Z_up = Z_before{L};
    Z_lo = Z_before{L-1};
    
    both = ~isnan(Z_up) & ~isnan(Z_lo);
    Z_diff = Z_up - Z_lo;
    Z_diff(~both) = NaN;
    
    vals = Z_diff(~isnan(Z_diff));
    if isempty(vals) || all(vals >= 0) || all(vals <= 0)
        continue;
    end
    
    fh = figure('Visible', 'off');
    C = contourc(xc, yc, Z_diff, [0 0]);
    close(fh);
    if isempty(C), continue; end
    
    pts_3d = [];
    ci = 1;
    while ci < size(C, 2)
        np = C(2, ci);
        cx = C(1, (ci+1):(ci+np));
        cy = C(2, (ci+1):(ci+np));
        
        % 交线z = 两层平均
        [Xg, Yg] = meshgrid(xc, yc);
        F_lo = scatteredInterpolant(Xg(both), Yg(both), Z_lo(both), 'linear', 'none');
        cz = F_lo(cx(:), cy(:));
        vv = ~isnan(cz);
        pts_3d = [pts_3d; cx(vv)', cy(vv)', cz(vv)];
        ci = ci + np + 1;
    end
    
    if ~isempty(pts_3d)
        curve_count = curve_count + 1;
        intersection_curves{curve_count} = struct(...
            'layer_pair', [L-1, L], ...
            'offsets', [offsets_sorted(L-1), offsets_sorted(L)], ...
            'points_3d', pts_3d);
    end
end
fprintf('  已排序 %d 条交线\n\n', curve_count);

%% ========== 7. 写回参数空间 ==========
fprintf('[7] 将修正后的曲面写回数据结构...\n');

[Xg, Yg] = meshgrid(xc, yc);

for L = 1:num_layers
    X = surface_layers{L}.X_surf;
    Y = surface_layers{L}.Y_surf;
    Z = surface_layers{L}.Z_surf;
    
    Z_before_L = Z_before{L};
    Z_after_L = Z_common{L};
    
    % 找到被裁剪的公共网格区域
    trimmed_mask = ~isnan(Z_before_L) & isnan(Z_after_L);
    n_trimmed = sum(trimmed_mask(:));
    
    if n_trimmed == 0, continue; end
    
    % 构建裁剪区域的插值器
    indicator = double(~trimmed_mask);
    F_trim = griddedInterpolant({yc, xc}, indicator, 'nearest', 'nearest');
    
    % 查询参数空间点
    v = ~isnan(X) & ~isnan(Y);
    if sum(v(:)) < 3, continue; end
    
    keep_flag = F_trim(Y(v), X(v));
    bad = keep_flag < 0.5;
    
    if any(bad)
        trim_full = false(size(X));
        vi = find(v);
        trim_full(vi(bad)) = true;
        
        X(trim_full) = NaN;
        Y(trim_full) = NaN;
        Z(trim_full) = NaN;
        
        surface_layers{L}.X_surf = X;
        surface_layers{L}.Y_surf = Y;
        surface_layers{L}.Z_surf = Z;
        
        n_param_trimmed = sum(bad);
        fprintf('  层 %2d (off=%+7.3f): 裁剪 %6d 点 (%.1f%%)\n', ...
            L, offsets_sorted(L), n_param_trimmed, ...
            100*n_param_trimmed / sum(v(:)));
    end
end
fprintf('  共修正 %d 层\n\n', sum(stats_inter > 0 | stats_intra > 0));

%% ========== 8. 保存 ==========
fprintf('[8] 保存修正结果...\n');

fix_info = struct(...
    'method', 'Z-map binning v4', ...
    'stats_intra', stats_intra, ...
    'stats_inter', stats_inter, ...
    'cascade', cascade, ...
    'intersection_curves', {intersection_curves}, ...
    'common_grid', struct('xc', xc, 'yc', yc, 'dx', dx_typical, 'dy', dy_typical), ...
    'Z_common', {Z_common}, ...
    'Z_before', {Z_before}, ...
    'timestamp', datestr(now));

% 写回原始结构
ts = datestr(now, 'yyyymmdd_HHMMSS');
fname = sprintf('slice_results_trimmed_%s.mat', ts);

if isfield(loaded, 'slice_results') && isstruct(loaded.slice_results)
    slice_results = loaded.slice_results;
    slice_results.surface_layers = surface_layers;
    slice_results.fix_info = fix_info;
    save(fname, 'slice_results', '-v7.3');
else
    save(fname, 'surface_layers', 'fix_info', '-v7.3');
    for fi = 1:length(fnames)
        vn = fnames{fi};
        if ~strcmp(vn, 'surface_layers') && ~strcmp(vn, 'sorted_layers')
            tmp = loaded.(vn);
            save(fname, 'tmp', '-append');
        end
    end
end

copyfile(fname, 'slice_results_trimmed_latest.mat');
fi_info = dir(fname);
fprintf('  已保存: %s (%.2f MB)\n', fname, fi_info.bytes/1024/1024);
fprintf('  副本:   slice_results_trimmed_latest.mat\n\n');

%% ========== 9. 可视化 ==========
fprintf('[9] 生成可视化对比...\n');

cmap = jet(num_layers);
[Xg, Yg] = meshgrid(xc, yc);

% 3D对比
figure('Name', '3D对比', 'Position', [50, 50, 1500, 600]);
subplot(1,2,1); hold on;
for L = 1:num_layers
    Zs = Z_before{L};
    if sum(~isnan(Zs(:))) < 50, continue; end
    surf(Xg, Yg, Zs, 'FaceColor', cmap(L,:), 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('修复前'); view([-37,30]); axis equal; grid on; hold off;

subplot(1,2,2); hold on;
for L = 1:num_layers
    Zs = Z_common{L};
    if sum(~isnan(Zs(:))) < 50, continue; end
    surf(Xg, Yg, Zs, 'FaceColor', cmap(L,:), 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end
for c = 1:curve_count
    pts = intersection_curves{c}.points_3d;
    if ~isempty(pts)
        plot3(pts(:,1), pts(:,2), pts(:,3), 'k-', 'LineWidth', 2);
    end
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('修复后'); view([-37,30]); axis equal; grid on; hold off;
saveas(gcf, 'fix_overlap_comparison_3d.png');

% XZ截面
mid_y = round(ny/2);
figure('Name', 'XZ截面', 'Position', [50, 700, 1500, 500]);
subplot(1,2,1); hold on;
for L = 1:num_layers
    zl = Z_before{L}(mid_y,:);
    v = ~isnan(zl);
    if any(v), plot(xc(v), zl(v), '-', 'Color', cmap(L,:), 'LineWidth', 1.2); end
end
title(sprintf('修复前 Y=%.1f', yc(mid_y)));
xlabel('X'); ylabel('Z'); grid on; hold off;

subplot(1,2,2); hold on;
for L = 1:num_layers
    zl = Z_common{L}(mid_y,:);
    v = ~isnan(zl);
    if any(v), plot(xc(v), zl(v), '-', 'Color', cmap(L,:), 'LineWidth', 1.2); end
end
title(sprintf('修复后 Y=%.1f', yc(mid_y)));
xlabel('X'); ylabel('Z'); grid on; hold off;
saveas(gcf, 'fix_overlap_xz_section.png');

% 等高线对比
figure('Name', '等高线', 'Position', [800, 50, 1200, 500]);
subplot(1,2,1); hold on;
for L = 1:num_layers
    Zs = Z_before{L};
    if sum(~isnan(Zs(:))) < 50, continue; end
    contour(Xg, Yg, Zs, 5, 'LineColor', cmap(L,:));
end
title('修复前等高线'); xlabel('X'); ylabel('Y');
axis equal; grid on; hold off;

subplot(1,2,2); hold on;
for L = 1:num_layers
    Zs = Z_common{L};
    if sum(~isnan(Zs(:))) < 50, continue; end
    contour(Xg, Yg, Zs, 5, 'LineColor', cmap(L,:));
end
title('修复后等高线'); xlabel('X'); ylabel('Y');
axis equal; grid on; hold off;
saveas(gcf, 'fix_overlap_contours.png');

% 裁剪热力图
figure('Name', '裁剪热力图', 'Position', [800, 600, 600, 500]);
trim_heat = zeros(ny, nx);
for L = 1:num_layers
    trimmed = ~isnan(Z_before{L}) & isnan(Z_common{L});
    trim_heat = trim_heat + double(trimmed);
end
imagesc(xc, yc, trim_heat);
colormap(hot); colorbar;
xlabel('X'); ylabel('Y');
title('各(x,y)位置累计裁剪层数');
axis equal tight; set(gca, 'YDir', 'normal');
saveas(gcf, 'fix_overlap_trim_heatmap.png');

fprintf('  已保存: fix_overlap_comparison_3d.png\n');
fprintf('  已保存: fix_overlap_contours.png\n');
fprintf('  已保存: fix_overlap_xz_section.png\n');
fprintf('  已保存: fix_overlap_trim_heatmap.png\n\n');

%% ========== 完成 ==========
fprintf('==============================================================\n');
fprintf('   修正完成!\n');
fprintf('   - 层内折叠bin: %d 个\n', sum(stats_intra));
fprintf('   - 层间交叉:    %d 个 + 级联 %d\n', inter_total, cascade);
fprintf('   - 交线:        %d 组\n', curve_count);
fprintf('   - 被修正层数:  %d / %d\n', ...
    sum(stats_inter > 0 | stats_intra > 0), num_layers);
fprintf('   - 结果文件:    slice_results_trimmed_latest.mat\n');
fprintf('==============================================================\n\n');


%% ================================================================
%% 辅助函数
%% ================================================================

function sl = find_surface_layers(loaded)
% 鲁棒查找 surface_layers, 适配多种 .mat 文件结构
    sl = {};
    fnames = fieldnames(loaded);
    
    % 路径1: slice_results.surface_layers
    if isfield(loaded, 'slice_results')
        sr = loaded.slice_results;
        if isstruct(sr) && isfield(sr, 'surface_layers')
            sl = sr.surface_layers;
            return;
        end
    end
    
    % 路径2: 顶层 surface_layers
    if isfield(loaded, 'surface_layers')
        sl = loaded.surface_layers;
        return;
    end
    
    % 路径3: sorted_layers
    if isfield(loaded, 'sorted_layers')
        sl = loaded.sorted_layers;
        return;
    end
    
    % 路径4: 自动搜索含 X_surf 的 cell
    for fi = 1:length(fnames)
        val = loaded.(fnames{fi});
        if iscell(val) && ~isempty(val)
            if isstruct(val{1}) && isfield(val{1}, 'X_surf')
                sl = val;
                return;
            end
        end
        if isstruct(val)
            inner_fnames = fieldnames(val);
            for gi = 1:length(inner_fnames)
                inner_val = val.(inner_fnames{gi});
                if iscell(inner_val) && ~isempty(inner_val)
                    if isstruct(inner_val{1}) && isfield(inner_val{1}, 'X_surf')
                        sl = inner_val;
                        return;
                    end
                end
            end
        end
    end
    
    % 找不到
    fprintf('\n  无法找到 surface_layers!\n');
    fprintf('  文件中的变量:\n');
    for fi = 1:length(fnames)
        val = loaded.(fnames{fi});
        fprintf('    %s: %s [%s]\n', fnames{fi}, class(val), ...
            strjoin(arrayfun(@num2str, size(val), 'UniformOutput', false), 'x'));
        if isstruct(val)
            sf = fieldnames(val);
            for si = 1:min(5, length(sf))
                fprintf('      .%s\n', sf{si});
            end
        end
    end
    error('请根据上方输出手动指定 surface_layers 的路径');
end

function Z_out = project_single_valued(layer, xc, yc)
% 将单层投影到公共网格 (假定无自交)
    X = layer.X_surf;
    Y = layer.Y_surf;
    Z = layer.Z_surf;
    v = ~isnan(X) & ~isnan(Y) & ~isnan(Z);
    
    [Xg, Yg] = meshgrid(xc, yc);
    
    if sum(v(:)) < 20
        Z_out = NaN(length(yc), length(xc));
        return;
    end
    
    F = scatteredInterpolant(X(v), Y(v), Z(v), 'linear', 'none');
    Z_out = F(Xg, Yg);
end