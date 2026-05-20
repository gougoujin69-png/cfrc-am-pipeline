% === DISPLAY 3D TOPOLOGY (ISO-VIEW) ===
function display_3D2(rho)
[nely, nelx, nelz] = size(rho);
hx = 1; hy = 1; hz = 1;  % 单元尺寸
face = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8]; % 立方体面定义
set(gcf, 'Name', 'ISO display', 'NumberTitle', 'off');

% 创建新图形并清除旧内容
%clf;

% 修正坐标系：
% MATLAB 默认坐标系：
%   X: 从左到右 (i 循环)
%   Y: 从下到上 (j 循环)
%   Z: 从内到外 (k 循环)
%
% 但拓扑优化通常使用：
%   X: 从左到右 (nelx)
%   Y: 从前到后 (nely) - 需要映射到 Z 轴
%   Z: 从下到上 (nelz) - 需要映射到 Y 轴

for k = 1:nelz  % Z 维度 (高度)
    for i = 1:nelx  % X 维度 (宽度)
        for j = 1:nely  % Y 维度 (深度)
            if rho(j, i, k) > 0.2
                % 计算物理坐标
                x = (i-1) * hx;       % X 从左到右
                y = (j-1) * hy;       % Y 从前到后（深度）
                z = (k-1) * hz;       % Z 从下到上（高度）
                
                % 定义立方体顶点（物理坐标系）
                % 注意：这里保持物理坐标关系
                vert = [...
                    x,     y,     z;      % 1: 前-左-下
                    x+hx,  y,     z;      % 2: 前-右-下
                    x+hx,  y,     z+hz;   % 3: 前-右-上
                    x,     y,     z+hz;   % 4: 前-左-上
                    x,     y+hy,  z;      % 5: 后-左-下
                    x+hx,  y+hy,  z;      % 6: 后-右-下
                    x+hx,  y+hy,  z+hz;   % 7: 后-右-上
                    x,     y+hy,  z+hz];  % 8: 后-左-上
                
                % 关键映射：将物理坐标转换为显示坐标
                % 物理坐标 -> 显示坐标：
                %   X物理 = X显示 (保持不变)
                %   Y物理 = Z显示 (深度变成Z轴)
                %   Z物理 = Y显示 (高度变成Y轴)
                display_vert = vert;
                display_vert(:, 1) = vert(:, 1);  % X 不变
                display_vert(:, 2) = vert(:, 3);  % 物理Z -> 显示Y (高度)
                display_vert(:, 3) = vert(:, 2);  % 物理Y -> 显示Z (深度)
                
                % 创建补片
                patch('Faces', face, 'Vertices', display_vert, ...
                      'FaceColor', [0.3, 0.3, 0.7 + 0.3 * (1 - rho(j, i, k))], ...
                      'EdgeColor', 'k', 'LineWidth', 0.5, ...
                      'FaceAlpha', 0.8);
                hold on;
            end
        end
    end
end

% 设置坐标系属性
axis equal;
axis tight;
axis off;
box on;
grid on;

% 设置视角 - 确保所有方向可见
view(3); % 使用MATLAB默认3D视图
rotate3d on; % 启用交互式旋转

% 设置光照
light('Position', [nelx/2, nely*2, nelz*2], 'Style', 'infinite', 'Color', [1, 1, 0.9]);
light('Position', [-nelx, -nely, nelz*2], 'Style', 'infinite', 'Color', [0.8, 0.8, 1]);
lighting gouraud;
material dull;

% 添加坐标轴标签（调试用）
%xlabel('X (Width)');
%ylabel('Y (Height)');
%zlabel('Z (Depth)');
%title('3D Topology Optimization Result');

% 设置合适的视角范围
axis([-0.5*nelx*hx 1.3*nelx*hx -0.5*nelz*hz 1.3*nelz*hz -0.5*nely*hy 1.3*nely*hy]);

% 添加颜色条说明密度
%colormap(jet);
%caxis([0 1]);
%colorbar;
%pause(1e-6);
end