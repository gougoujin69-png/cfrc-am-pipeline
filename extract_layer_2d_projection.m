function [xold, t, nelx, nely, nelz] = extract_layer_2d_projection(activated_grids, grid_data)
[nelx, nely, nelz] = size(grid_data);
xold = zeros(nely, nelx);
% --- 方向场去噪(源头): 一列里多个 z 的材料投影到同一 (ix,iy) 时, 原先 t=g.t_xoy
%     是"最后写入"(任意取某个 z 的角度) -> 厚层方向场带噪. 改为在双角度空间
%     (cos2t/sin2t) 对该列所有 z 求平均, 再还原角度 -> 每列方向更干净/稳定.
%     (双角度正确处理主应力线场的 180° 歧义; xold 仍为二值, 轮廓不受影响.) ---
c2sum = zeros(nely, nelx);   % 双角度累加(线方向: 正确处理 180° 歧义)
s2sum = zeros(nely, nelx);
vcsum = zeros(nely, nelx);   % 有符号矢量累加(保留 σ1 矢量的真实指向)
vssum = zeros(nely, nelx);
num_grids = length(activated_grids);
for i = 1:num_grids
    g = activated_grids(i);
    % grid_index: 1-based 整数索引 (slice_refined_model_v6.m L858 写入), 对任意
    % REFINE_FACTOR 都安全, 比物理坐标 .x/.y 稳妥
    ix = g.grid_index(1);   % i (x direction), in [1, nelx]
    iy = g.grid_index(2);   % j (y direction), in [1, nely]
    xold(iy, ix)  = 1;
    % --- [关键] 用 σ1 矢量的面内分量 (uu,vv) 定义方向, 而不是角度 t_xoy. ---
    %   uu=cos(t_xoz)cos(t_xoy), vv=cos(t_xoz)sin(t_xoy): 约一半体素 cos(t_xoz)<0,
    %   使 t_xoy 的符号相对真实 σ1 矢量翻 180° 且逐层不一致(前层方向场翻转的根因).
    %   (uu,vv) 是真实矢量投影, 跨层一致、干净. 退化(纯面外)时回退 t_xoy.
    nrm = hypot(g.uu, g.vv);
    if nrm > 1e-9, cphi = g.uu/nrm; sphi = g.vv/nrm;
    else,          cphi = cos(g.t_xoy); sphi = sin(g.t_xoy); end
    c2sum(iy, ix) = c2sum(iy, ix) + (cphi*cphi - sphi*sphi);  % cos(2φ)
    s2sum(iy, ix) = s2sum(iy, ix) + (2*cphi*sphi);            % sin(2φ)
    vcsum(iy, ix) = vcsum(iy, ix) + cphi;                     % 有符号矢量均值
    vssum(iy, ix) = vssum(iy, ix) + sphi;
end
% 双角度平均得到干净的"线方向", 再用有符号矢量均值把真实指向对齐回去
% -> 去噪(180°歧义正确处理) + 保留 σ1 矢量的一致指向(跨层不再翻转).
t = 0.5 * atan2(s2sum, c2sum);
flip = (cos(t).*vcsum + sin(t).*vssum) < 0;
t(flip) = t(flip) + pi;
end