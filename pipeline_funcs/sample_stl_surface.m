function pts = sample_stl_surface(stl_file, spacing)
%% SAMPLE_STL_SURFACE  把 STL 表面均匀采样成点云(面积加权), 作碰撞静态障碍
% 输入:
%   stl_file - STL 路径 (如 generate_base_and_interface_support 产出的 Base_Reduced.stl)
%   spacing  - 目标点间距(mm), 默认 1.0. 越小越密(更准但更慢).
% 输出:
%   pts      - M×3 点云(mm). 含所有顶点 + 各三角面内部采样点.
% 用法:
%   bpts = sample_stl_surface('Base_Reduced.stl', 1.0);
% =====================================================================
if nargin<2 || isempty(spacing), spacing = 1.0; end
TR = stlread(stl_file);                 % triangulation (R2018b+)
V = TR.Points;  F = TR.ConnectivityList;
acc = {V};                              % 含全部顶点
for t = 1:size(F,1)
    p1=V(F(t,1),:); p2=V(F(t,2),:); p3=V(F(t,3),:);
    a = 0.5*norm(cross(p2-p1, p3-p1));  % 三角面积
    n = round(a/(spacing^2));           % 每 spacing^2 约 1 点
    if n>=1
        r1=rand(n,1); r2=rand(n,1);
        m=(r1+r2)>1; r1(m)=1-r1(m); r2(m)=1-r2(m);
        acc{end+1} = p1 + r1.*(p2-p1) + r2.*(p3-p1); %#ok<AGROW>
    end
end
pts = cat(1, acc{:});
% 体素下采样去冗余 (~spacing/2 网格)
g = max(spacing*0.5, 1e-6);
pts = unique(round(pts/g)*g, 'rows');
fprintf('[sample_stl] %s -> 点云 %d 点 (间距~%.2fmm)\n', stl_file, size(pts,1), spacing);
end
