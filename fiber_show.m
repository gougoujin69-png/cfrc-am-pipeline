%% 已知纤维方向角度矩阵 t，生成流线并显示方向分布

% 假设 t 是一个 m×n 的矩阵，每个元素表示纤维方向的角度（单位：弧度）
% 如果 t 是角度矩阵（单位为度），请先转换为弧度：t_rad = deg2rad(t);

[m, n] = size(t);

% 创建网格坐标
[x, y] = meshgrid(1:n, 1:m);

% 从角度计算方向向量分量
t_u = cos(t);  % x方向分量
t_v = sin(t);  % y方向分量

%% 可视化1：纤维方向分布图
figure('Position', [100, 100, 1400, 500]);

% 子图1：角度分布热图
subplot(1, 3, 1);
imagesc(t);
axis equal tight;
colorbar;
xlabel('X坐标');
ylabel('Y坐标');
title('纤维方向角度分布 (弧度)');
colormap(gca, 'hsv');  % 使用HSV色图，方便表示角度
caxis([-pi pi]);  % 如果是弧度

% 子图2：方向箭头图
subplot(1, 3, 2);
% 使用稀疏箭头显示，避免太密集
quiver(x(1:2:end, 1:2:end), y(1:2:end, 1:2:end), ...
       t_u(1:2:end, 1:2:end), t_v(1:2:end, 1:2:end), ...
       0.8, 'b', 'LineWidth', 1.2);
axis equal tight;
xlabel('X坐标');
ylabel('Y坐标');
title('纤维方向箭头图');
grid on;

% 子图3：流线图
subplot(1, 3, 3);
% 创建流线起始点（沿左边界）
seed_density = 1.5; % 控制种子点密度
startx = 1:seed_density:nelx; % X方向种子点
starty = 1:seed_density:nely; % Y方向种子点
[startX, startY] = meshgrid(startx, starty); % 生成网格状种子点
startX = startX(:); % 转换为列向量
startY = startY(:); % 转换为列向量
% 绘制流线
streamline(x, y, t_u, t_v, startX, startY);
hold on;
axis equal tight;
xlabel('X坐标');
ylabel('Y坐标');
title('纤维流线图');
grid on;

%% 可视化2：综合图 - 流线与方向叠加
figure('Position', [100, 100, 800, 800]);

% 先绘制流线
streamline(x, y, t_u, t_v, startx, starty);

% 叠加稀疏的方向箭头
quiver(x(1:3:end, 1:3:end), y(1:3:end, 1:3:end), ...
       t_u(1:3:end, 1:3:end), t_v(1:3:end, 1:3:end), ...
       0.8, 'r', 'LineWidth', 1);

% 标记起始点
plot(startx, starty, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 6);
plot(startx2, starty2, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 6);

axis equal tight;
xlabel('X坐标');
ylabel('Y坐标');
title('纤维方向与流线综合图');
legend('流线1', '流线2', '方向箭头', '起始点', 'Location', 'best');
grid on;

%% 可视化3：方向统计分布
figure('Position', [100, 100, 1000, 400]);

subplot(1, 2, 1);
% 方向角度直方图
histogram(t(:), 50, 'Normalization', 'probability');
xlabel('纤维方向角度 (弧度)');
ylabel('概率密度');
title('纤维方向角度分布直方图');
grid on;

subplot(1, 2, 2);
% 玫瑰图（极坐标直方图）
polarhistogram(t(:), 50, 'Normalization', 'probability');
title('纤维方向玫瑰图');