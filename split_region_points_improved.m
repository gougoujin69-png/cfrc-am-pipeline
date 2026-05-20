function [regions_points, regions_poly] = split_region_points_improved(outer_contours, inner_contours, extended_streamlines, varargin)
% [regions_points, regions_poly] = split_region_points_improved(outer_contours, inner_contours, extended_streamlines, ...)
% 更鲁棒的 PSLG 划分，输出每个子区域的顶点（cell of Nx2 arrays）
%
% 可选参数 (name,value):
%   'point_tol' (default 1e-6)  - 点合并/交点容差
%   'dist_tol'  (default 1e-6)  - 点到线的距离容差
%   'area_tol'  (default 1e-12) - 面过滤最小面积
%   'debug'     (default false) - 若为 true 显示调试图（交点/候选面）
%
% 说明：
% - 处理共线重叠段（将重叠区端点加入交点集合）
% - 端点靠近边界/其它点会被合并（基于 point_tol）
% - 面提取使用有向边遍历并去重（基于质心+面积）
% - 返回 regions_points（cell of Nx2）, regions_poly (polyshape array)

    % parse optional args
    p = inputParser;
    addParameter(p,'point_tol',1e-6,@(x)isnumeric(x)&&x>0);
    addParameter(p,'dist_tol',1e-6,@(x)isnumeric(x)&&x>0);
    addParameter(p,'area_tol',1e-12,@(x)isnumeric(x)&&x>=0);
    addParameter(p,'debug',false,@islogical);
    parse(p,varargin{:});
    point_tol = p.Results.point_tol;
    dist_tol  = p.Results.dist_tol;
    area_tol  = p.Results.area_tol;
    debug_on  = p.Results.debug;

    %% ---------- 构造主区域 ----------
    main_poly = polyshape(outer_contours{1}(:,1), outer_contours{1}(:,2));
    for hh = 1:numel(inner_contours)
        hole = polyshape(inner_contours{hh}(:,1), inner_contours{hh}(:,2));
        main_poly = subtract(main_poly, hole);
    end

    %% ---------- 收集所有原始线段 ----------
    S = zeros(0,4); % [x1 y1 x2 y2]
    % 外轮廓
    for k = 1:numel(outer_contours)
        P = outer_contours{k};
        if ~isequal(P(1,:), P(end,:)), P(end+1,:) = P(1,:); end
        for i = 1:size(P,1)-1
            S(end+1,:) = [P(i,:) P(i+1,:)]; %#ok<AGROW>
        end
    end
    % 内轮廓
    for k = 1:numel(inner_contours)
        P = inner_contours{k};
        if isempty(P), continue; end
        if ~isequal(P(1,:), P(end,:)), P(end+1,:) = P(1,:); end
        for i = 1:size(P,1)-1
            S(end+1,:) = [P(i,:) P(i+1,:)];
        end
    end
    % 流线（折线每段）
    for k = 1:numel(extended_streamlines)
        P = extended_streamlines{k};
        for i = 1:size(P,1)-1
            S(end+1,:) = [P(i,:) P(i+1,:)];
        end
    end
    ns = size(S,1);
    if ns==0
        regions_points = {}; regions_poly = polyshape.empty;
        return;
    end

    %% ---------- 计算所有端点 + 交点（含共线重叠端点） ----------
    pts = [S(:,1:2); S(:,3:4)]; % 初始端点集合
    for i = 1:ns-1
        p1 = S(i,1:2); p2 = S(i,3:4);
        for j = i+1:ns
            q1 = S(j,1:2); q2 = S(j,3:4);
            Pxy = segSegIntersectionAll(p1,p2,q1,q2, point_tol);
            if ~isempty(Pxy)
                pts = [pts; Pxy]; %#ok<AGROW>
            end
        end
    end

    % 合并点（基于四舍五入）
    scale = 1/point_tol;
    pr = round(pts*scale)/scale;
    [~, ia, ~] = unique(pr,'rows','stable');
    V = pts(ia,:);  % 顶点列表

    %% ---------- 将每条原始线段按这些顶点切分（得到边列表） ----------
    Elist = zeros(0,2);
    for s = 1:ns
        p1 = S(s,1:2); p2 = S(s,3:4);
        dir = p2 - p1; L2 = dot(dir,dir);
        if L2 < eps, continue; end
        % 计算参数 t 和到线距离
        t = ((V(:,1)-p1(1))*dir(1) + (V(:,2)-p1(2))*dir(2)) / L2;
        crossv = (V(:,1)-p1(1))*dir(2) - (V(:,2)-p1(2))*dir(1);
        dist = abs(crossv) / sqrt(L2);
        mask = (t >= -1e-9) & (t <= 1+1e-9) & (dist <= dist_tol);
        idxs = find(mask);
        if numel(idxs) < 2, continue; end
        % 对 t 排序并去掉在同一点太近的点
        ts = t(idxs); [ts_sorted, ord] = sort(ts);
        idx_sorted = idxs(ord);
        % 去掉非常接近的重复参数
        keep = true(size(idx_sorted));
        for k2 = 2:numel(idx_sorted)
            if abs(ts_sorted(k2)-ts_sorted(k2-1)) < 1e-9
                keep(k2) = false;
            end
        end
        idx_sorted = idx_sorted(keep);
        for m = 1:numel(idx_sorted)-1
            a = idx_sorted(m); b = idx_sorted(m+1);
            if a==b, continue; end
            Elist(end+1,:) = sort([a b]); %#ok<AGROW>
        end
    end
    Elist = unique(Elist,'rows','stable');
    if isempty(Elist)
        regions_points = {}; regions_poly = polyshape.empty;
        return;
    end

    %% ---------- 建邻接并按角度排序 ----------
    nV = size(V,1);
    neighbors = cell(nV,1);
    for e = 1:size(Elist,1)
        u = Elist(e,1); v = Elist(e,2);
        neighbors{u} = [neighbors{u}, v];
        neighbors{v} = [neighbors{v}, u];
    end
    for v = 1:nV
        nbs = neighbors{v};
        if isempty(nbs), continue; end
        angs = atan2(V(nbs,2)-V(v,2), V(nbs,1)-V(v,1));
        % sort in descending order for consistent CCW ordering
        [~, ord] = sort(angs,'descend');
        neighbors{v} = nbs(ord);
    end

    %% ---------- 有向边遍历以提取面 ----------
    visited = sparse(nV,nV);
    faces = {};
    face_keys = containers.Map('KeyType','char','ValueType','double');

    for u = 1:nV
        for idx_nb = 1:numel(neighbors{u})
            v = neighbors{u}(idx_nb);
            if visited(u,v), continue; end
            start_u = u; start_v = v;
            curr_u = start_u; curr_v = start_v;
            poly_idx = zeros(1,0);
            safety = 0;
            while true
                visited(curr_u,curr_v) = true;
                poly_idx(end+1) = curr_u; %#ok<AGROW>
                % 在 curr_v 的邻居中找到 curr_u 的位置
                nlist = neighbors{curr_v};
                pos = find(nlist==curr_u,1);
                if isempty(pos)
                    break;
                end
                % 下一个邻边选取索引 pos-1（逆时针前一条）
                next_idx = pos - 1;
                if next_idx < 1, next_idx = numel(nlist); end
                next_w = nlist(next_idx);
                next_u = curr_v; next_v = next_w;
                curr_u = next_u; curr_v = next_v;
                safety = safety + 1;
                if safety > 20000
                    warning('环路遍历过长，可能存在异常拓扑，跳出该环路。');
                    break;
                end
                if curr_u==start_u && curr_v==start_v
                    break;
                end
            end
            if numel(poly_idx) < 3, continue; end
            px = V(poly_idx,1); py = V(poly_idx,2);
            A = polyarea(px,py);
            if A <= area_tol, continue; end
            cand = polyshape(px,py);
            inter = intersect(cand, main_poly);
            if isempty(inter.Vertices), continue; end
            a_cand = area(cand); a_inter = area(inter);
            % 允许一定的数值偏差：只要大部分在主区域内就接受
            %if a_inter / a_cand < 0.5
            %    continue;
            %end
            [Cx,Cy] = polygon_centroid(px,py);
            key = sprintf('%.6f_%.6f_%.6f', round(Cx/point_tol)*point_tol, round(Cy/point_tol)*point_tol, round(a_cand/point_tol)*point_tol);
            if ~isKey(face_keys,key)
                face_keys(key) = 1;
                if abs(a_inter - area(main_poly)) < area_tol
                continue;
                end
                faces{end+1} = cand; %#ok<AGROW>
            end
        end
    end

    %% ---------- 输出：polyshape & 点集 ----------
    if isempty(faces)
        regions_poly = polyshape.empty;
        regions_points = {};
    else
        regions_poly = [faces{:}];
        regions_points = cell(1,numel(regions_poly));
        for k = 1:numel(regions_poly)
            v = regions_poly(k).Vertices;
            % 保证点闭合（首尾相同）
            if ~isequal(v(1,:), v(end,:))
                v(end+1,:) = v(1,:);
            end
            regions_points{k} = v;
        end
    end

    %% ---------- debug 可视化 ----------
    if debug_on
        figure('Color','w'); hold on; axis equal off;
        title('划分调试视图');
        % 绘制主轮廓
        plot(main_poly,'FaceColor',[0.95 0.95 0.95],'FaceAlpha',0.5,'EdgeColor','k');
        % 绘制所有原始线段（蓝）
        for s = 1:ns
            plot([S(s,1) S(s,3)],[S(s,2) S(s,4)],'-b','LineWidth',1);
        end
        % 绘制所有交点（红）
        plot(V(:,1), V(:,2), 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
        % 绘制提取到的面（半透明）
        for k = 1:numel(faces)
            plot(faces{k}, 'FaceAlpha', 0.6);
            v = faces{k}.Vertices;
            cx = mean(v(:,1)); cy = mean(v(:,2));
            text(cx,cy,sprintf('F%d',k),'HorizontalAlignment','center','FontWeight','bold');
        end
        drawnow;
    end
end

%% ================= 辅助：线段全部相交检测（返回所有相关交点，包括重叠端点） =================
function Pxy = segSegIntersectionAll(p1,p2,q1,q2, tol)
    % 返回空或 Mx2 的交点集合
    Pxy = [];
    r = p2 - p1; s = q2 - q1;
    denom = r(1)*s(2) - r(2)*s(1);
    if abs(denom) < 1e-12
        % 共线或平行
        % 检查是否共线
        if abs((q1(1)-p1(1))*r(2) - (q1(2)-p1(2))*r(1)) > tol*max(norm(r),norm(s))
            return; % 平行不共线
        end
        % 共线：把两段的投影一维参数区间求交
        if norm(r) < eps || norm(s) < eps
            % 一段退化为点，返回两端点中重合的
            if norm(p1-q1) < tol, Pxy = [Pxy; p1]; end
            if norm(p1-q2) < tol, Pxy = [Pxy; p1]; end
            if norm(p2-q1) < tol, Pxy = [Pxy; p2]; end
            if norm(p2-q2) < tol, Pxy = [Pxy; p2]; end
            return;
        end
        % 用 p1 为参照，计算 q 的 t 参数
        t0 = dot((q1-p1), r) / dot(r,r);
        t1 = dot((q2-p1), r) / dot(r,r);
        a = min(t0,t1); b = max(t0,t1);
        % 段 p 的参数范围是 [0,1]
        L = max(a,0); R = min(b,1);
        if R < L - tol
            return;
        end
        % 返回重叠区端点（clamp到[0,1]）
        Lpt = p1 + max(L,0)*r;
        Rpt = p1 + min(R,1)*r;
        Pxy = [Pxy; Lpt; Rpt];
        return;
    else
        t = ((q1(1)-p1(1))*s(2) - (q1(2)-p1(2))*s(1)) / denom;
        u = ((q1(1)-p1(1))*r(2) - (q1(2)-p1(2))*r(1)) / denom;
        if t >= -tol && t <= 1+tol && u >= -tol && u <= 1+tol
            P = p1 + t*r;
            Pxy = [Pxy; P];
            return;
        end
    end
end

%% ================= 辅助：多边形重心 =================
function [Cx,Cy] = polygon_centroid(x,y)
    n = numel(x);
    if n < 3, Cx = mean(x); Cy = mean(y); return; end
    x2 = x([2:end,1]); y2 = y([2:end,1]);
    cross = x.*y2 - x2.*y;
    A = sum(cross)/2;
    if abs(A) < eps, Cx = mean(x); Cy = mean(y); return; end
    Cx = sum((x + x2).*cross)/(6*A);
    Cy = sum((y + y2).*cross)/(6*A);
end
