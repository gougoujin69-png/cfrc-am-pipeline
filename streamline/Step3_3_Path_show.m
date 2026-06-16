% 1. 读取数据
YM = xlsread('carbon_path_3_1.xlsx');

% 假设 YM 的列定义如下 (根据你提供的代码推断):
% YM(:,1): x, YM(:,2): y, YM(:,3): z
% YM(:,4): Angle4 (alpha), YM(:,5): Angle5 (beta)
% 这里的偏移量参考你提供的代码
x_offset = 0.55 * 1000; % 转换为mm方便绘图，或统一用m
firstl = 0; % 假设的初始高度偏移

% 提取路径点 (单位: mm)
px = (0.55 + (YM(:,1) + 0)/1000.0) * 1000; 
py = (YM(:,2) + 0) / 1000.0 * 1000;
pz = (firstl + (YM(:,3) + 0)/1000.0) * 1000;

num_pts = size(YM, 1);
vx = zeros(num_pts, 1);
vy = zeros(num_pts, 1);
vz = zeros(num_pts, 1);

% 2. 计算每个点的末端 Z 轴方向
for i = 1:num_pts
    a4 = deg2rad(YM(i,4)); % trotx(YM(i,4))
    a5 = deg2rad(YM(i,5)); % troty(YM(i,5))
    
    % 根据你的公式推导出的旋转矩阵 R = Ry(pi) * Rx(a4) * Ry(a5)
    % 我们只关心 Z 轴 (矩阵的第三列)
    % 这里的变换顺序必须与你提供的代码完全一致
    R = troty_local(pi) * trotx_local(a4) * troty_local(a5);
    
    z_axis = R(1:3, 3); % 提取第三列作为方向矢量
    vx(i) = z_axis(1);
    vy(i) = z_axis(2);
    vz(i) = z_axis(3);
end

% 3. 绘图
figure('Color', 'w');
plot3(px, py, pz, 'b-', 'LineWidth', 1.5); % 绘制蓝色路径线
hold on;

% 使用 quiver3 绘制方向矢量
% 这里的 20 是矢量长度因数，你可以根据图形大小调整
% 每隔 10 个点画一个方向箭头
skip = 1;
quiver3(px(1:skip:end), py(1:skip:end), pz(1:skip:end), ...
        vx(1:skip:end), vy(1:skip:end), vz(1:skip:end), 2, 'r');
%quiver3(px, py, pz, vx, vy, vz, 20, 'r', 'LineWidth', 1); 

grid on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('三维路径点与末端执行器 Z 轴方向');
legend('加工路径', '末端 Z 轴朝向');
view(45, 30);

% --- 辅助旋转矩阵函数 (避免缺少工具箱) ---
function T = trotx_local(theta)
    c = cos(theta); s = sin(theta);
    T = [1 0 0; 0 c -s; 0 s c];
end

function T = troty_local(theta)
    c = cos(theta); s = sin(theta);
    T = [c 0 s; 0 1 0; -s 0 c];
end