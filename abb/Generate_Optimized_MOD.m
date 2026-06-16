%% ========================================================================
%  ABB IRB-1200 生成优化后的.mod文件 (RZ偏移+35°)
%  ========================================================================
%  功能：
%    1. 使用RZ偏移+35°计算新的关节轨迹
%    2. 生成优化后的.mod文件（无MoveAbsJ）
%    3. 保存关节轨迹供仿真使用
%  ========================================================================

clear; clc; close all;

%% ======================== 参数设置 ======================================
firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;
tool_hc = -50.7321e-3; tool_yc = -50.0753e-3; tool_lc = 70.963e-3; tool_ac = 30/180*pi;
x_comp = -0.5; y_comp = -1.5; z_comp = 0.2;
base_x_offset = 0.55;
ql1_default = [0, 0, pi/6.0, 0, 0, 0];

% RZ偏移角度
RZ_OFFSET_DEG = -90;
RZ_OFFSET_RAD = RZ_OFFSET_DEG * pi / 180;

%% ======================== 读取路径 ======================================
excel_file = 'path_23_1.xlsx';
fprintf('========================================================\n');
fprintf('   生成优化后的.mod文件 (RZ偏移 = +%d°)\n', RZ_OFFSET_DEG);
fprintf('========================================================\n\n');

YM = xlsread(excel_file);
n = size(YM, 1);
fprintf('路径点数: %d\n', n);

tool_params = [tool_lc, tool_hc, tool_yc, tool_ac];

%% ======================== 计算原始轨迹 ==================================
fprintf('\n计算原始轨迹 (无RZ偏移)...\n');

q_orig = zeros(n, 6);
cfx_orig = zeros(n, 1);
ql = ql1_default;

for i = 1:n
    target_x = base_x_offset + (YM(i,1) + x_comp) / 1000.0;
    target_y = (YM(i,2) + y_comp) / 1000.0;
    target_z = firstl + (YM(i,3) + z_comp) / 1000.0;
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0; ry = 0;
    end
    
    T = transl_m(target_x, target_y, target_z) * troty_m(pi) * trotx_m(rx) * troty_m(ry);
    Q = ikabb_m(T, tool_params(1), tool_params(2), tool_params(3), tool_params(4));
    
    try
        qi = ikbest_m(ql, Q);
    catch
        qi = ql;
    end
    
    q_orig(i,:) = qi;
    rcf = rscfx_m(qi);
    cfx_orig(i) = rcf(4);
    ql = qi;
end

orig_switches = sum(diff(cfx_orig) ~= 0);
fprintf('  奇异点数: %d\n', orig_switches);

%% ======================== 计算优化轨迹 ==================================
fprintf('\n计算优化轨迹 (RZ偏移 = +%d°)...\n', RZ_OFFSET_DEG);

q_opt = zeros(n, 6);
cfx_opt = zeros(n, 1);
rcf_opt = zeros(n, 4);
ql = ql1_default;

for i = 1:n
    target_x = base_x_offset + (YM(i,1) + x_comp) / 1000.0;
    target_y = (YM(i,2) + y_comp) / 1000.0;
    target_z = firstl + (YM(i,3) + z_comp) / 1000.0;
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0; ry = 0;
    end
    
    % 添加RZ偏移
    T = transl_m(target_x, target_y, target_z) * troty_m(pi) * trotx_m(rx) * troty_m(ry) * trotz_m(RZ_OFFSET_RAD);
    Q = ikabb_m(T, tool_params(1), tool_params(2), tool_params(3), tool_params(4));
    
    try
        qi = ikbest_m(ql, Q);
    catch
        qi = ql;
    end
    
    q_opt(i,:) = qi;
    rcf = rscfx_m(qi);
    cfx_opt(i) = rcf(4);
    rcf_opt(i,:) = rcf;
    ql = qi;
end

opt_switches = sum(diff(cfx_opt) ~= 0);
fprintf('  奇异点数: %d\n', opt_switches);
fprintf('  J5范围: [%.2f°, %.2f°]\n', min(q_opt(:,5))*180/pi, max(q_opt(:,5))*180/pi);

%% ======================== 生成优化后的.mod文件 ==========================
fprintf('\n生成优化后的.mod文件...\n');

mod_filename = sprintf('path_23_2_optimized_RZ%d.mod', RZ_OFFSET_DEG);
fid = fopen(mod_filename, 'w');

% 写入模块头
fprintf(fid, 'MODULE MainModule\n\n');

% 写入工具定义
fprintf(fid, '  ! 工具定义 (碳纤维工具)\n');
fprintf(fid, '  PERS tooldata tool_carbon:=[TRUE,[[%.4f,%.4f,%.4f],[1,0,0,0]],[0.5,[0,0,50],[1,0,0,0],0,0,0]];\n\n', ...
        tool_hc*1000, tool_yc*1000, (tool_lc+0.082)*1000);

% 写入工件定义
fprintf(fid, '  ! 工件定义\n');
fprintf(fid, '  PERS wobjdata wobj_print:=[FALSE,TRUE,"",[[%.1f,0,%.4f],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];\n\n', ...
        base_x_offset*1000, firstl*1000);

% 写入速度定义
fprintf(fid, '  ! 速度定义\n');
fprintf(fid, '  CONST speeddata v_print:=[50,500,5000,1000];\n');
fprintf(fid, '  CONST speeddata v_move:=[200,500,5000,1000];\n\n');

% 写入目标点定义
fprintf(fid, '  ! 目标点定义 (共%d个点)\n', n);

for i = 1:n
    % robtarget格式: [[x,y,z],[q1,q2,q3,q4],[cf1,cf4,cf6,cfx],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]
    
    % 计算位置 (相对于工件坐标系)
    px = YM(i,1) + x_comp;
    py = YM(i,2) + y_comp;
    pz = YM(i,3) + z_comp;
    
    % 计算姿态四元数 (包含RZ偏移)
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0; ry = 0;
    end
    
    R = roty_m(pi) * rotx_m(rx) * roty_m(ry) * rotz_m(RZ_OFFSET_RAD);
    quat = rotm2quat_m(R);
    
    % 获取配置数据
    cf1 = rcf_opt(i, 1);
    cf4 = rcf_opt(i, 2);
    cf6 = rcf_opt(i, 3);
    cfx = rcf_opt(i, 4);
    
    fprintf(fid, '  CONST robtarget Target_%d:=[[%.3f,%.3f,%.3f],[%.6f,%.6f,%.6f,%.6f],[%d,%d,%d,%d],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n', ...
            i, px, py, pz, quat(1), quat(2), quat(3), quat(4), cf1, cf4, cf6, cfx);
end

% 写入主程序
fprintf(fid, '\n  ! 主程序\n');
fprintf(fid, '  PROC main()\n');
fprintf(fid, '    ! 优化后的路径 (RZ偏移 = +%d°, 无奇异点)\n', RZ_OFFSET_DEG);
fprintf(fid, '    ! 原始奇异点: %d, 优化后: %d\n\n', orig_switches, opt_switches);

% 移动到起始点
fprintf(fid, '    ! 移动到起始点\n');
fprintf(fid, '    MoveJ Target_1, v_move, fine, tool_carbon\\WObj:=wobj_print;\n\n');

% 打印路径 - 全部使用MoveL
fprintf(fid, '    ! 打印路径 (全部使用MoveL, 无MoveAbsJ)\n');
for i = 2:n
    fprintf(fid, '    MoveL Target_%d, v_print, z1, tool_carbon\\WObj:=wobj_print;\n', i);
end

fprintf(fid, '\n  ENDPROC\n');
fprintf(fid, '\nENDMODULE\n');

fclose(fid);
fprintf('  已保存: %s\n', mod_filename);

%% ======================== 生成原始.mod文件（用于对比）==================
fprintf('\n生成原始.mod文件（用于对比）...\n');

mod_orig_filename = 'path_23_2_original.mod';
fid = fopen(mod_orig_filename, 'w');

fprintf(fid, 'MODULE MainModule\n\n');
fprintf(fid, '  PERS tooldata tool_carbon:=[TRUE,[[%.4f,%.4f,%.4f],[1,0,0,0]],[0.5,[0,0,50],[1,0,0,0],0,0,0]];\n', ...
        tool_hc*1000, tool_yc*1000, (tool_lc+0.082)*1000);
fprintf(fid, '  PERS wobjdata wobj_print:=[FALSE,TRUE,"",[[%.1f,0,%.4f],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];\n', ...
        base_x_offset*1000, firstl*1000);
fprintf(fid, '  CONST speeddata v_print:=[50,500,5000,1000];\n');
fprintf(fid, '  CONST speeddata v_move:=[200,500,5000,1000];\n\n');

% 写入原始目标点
for i = 1:n
    px = YM(i,1) + x_comp;
    py = YM(i,2) + y_comp;
    pz = YM(i,3) + z_comp;
    
    if size(YM, 2) >= 5
        rx = YM(i, 4) * pi / 180;
        ry = YM(i, 5) * pi / 180;
    else
        rx = 0; ry = 0;
    end
    
    R = roty_m(pi) * rotx_m(rx) * roty_m(ry);
    quat = rotm2quat_m(R);
    
    rcf = rscfx_m(q_orig(i,:));
    
    fprintf(fid, '  CONST robtarget Target_%d:=[[%.3f,%.3f,%.3f],[%.6f,%.6f,%.6f,%.6f],[%d,%d,%d,%d],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n', ...
            i, px, py, pz, quat(1), quat(2), quat(3), quat(4), rcf(1), rcf(2), rcf(3), rcf(4));
end

% 写入带奇异点的程序
fprintf(fid, '\n  PROC main()\n');
fprintf(fid, '    ! 原始路径 (有%d个奇异点)\n\n', orig_switches);
fprintf(fid, '    MoveJ Target_1, v_move, fine, tool_carbon\\WObj:=wobj_print;\n\n');

sing_idx = find(diff(cfx_orig) ~= 0) + 1;
for i = 2:n
    if ismember(i, sing_idx)
        % 奇异点使用MoveAbsJ
        fprintf(fid, '    MoveAbsJ [[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v_print, z1, tool_carbon;  ! 奇异点\n', ...
                q_orig(i,:)*180/pi);
    else
        fprintf(fid, '    MoveL Target_%d, v_print, z1, tool_carbon\\WObj:=wobj_print;\n', i);
    end
end

fprintf(fid, '\n  ENDPROC\n');
fprintf(fid, '\nENDMODULE\n');

fclose(fid);
fprintf('  已保存: %s\n', mod_orig_filename);

%% ======================== 保存轨迹数据供仿真使用 ========================
fprintf('\n保存轨迹数据供仿真使用...\n');

% 保存为.mat文件
trajectory_data.q_orig = q_orig;
trajectory_data.q_opt = q_opt;
trajectory_data.cfx_orig = cfx_orig;
trajectory_data.cfx_opt = cfx_opt;
trajectory_data.YM = YM;
trajectory_data.n = n;
trajectory_data.RZ_OFFSET_DEG = RZ_OFFSET_DEG;
trajectory_data.orig_switches = orig_switches;
trajectory_data.opt_switches = opt_switches;
trajectory_data.sing_idx_orig = sing_idx;

save('trajectory_data.mat', 'trajectory_data');
fprintf('  已保存: trajectory_data.mat\n');

%% ======================== 对比总结 ======================================
fprintf('\n========================================================\n');
fprintf('   对比总结\n');
fprintf('========================================================\n');

fprintf('\n【原始路径】\n');
fprintf('  奇异点数: %d\n', orig_switches);
fprintf('  奇异点位置: %s\n', mat2str(sing_idx'));
fprintf('  J5范围: [%.2f°, %.2f°]\n', min(q_orig(:,5))*180/pi, max(q_orig(:,5))*180/pi);
fprintf('  需要MoveAbsJ: 是\n');

fprintf('\n【优化路径 (RZ偏移 +%d°)】\n', RZ_OFFSET_DEG);
fprintf('  奇异点数: %d\n', opt_switches);
fprintf('  J5范围: [%.2f°, %.2f°]\n', min(q_opt(:,5))*180/pi, max(q_opt(:,5))*180/pi);
fprintf('  需要MoveAbsJ: 否\n');

fprintf('\n【生成的文件】\n');
fprintf('  1. %s - 优化后的.mod文件\n', mod_filename);
fprintf('  2. %s - 原始.mod文件（用于对比）\n', mod_orig_filename);
fprintf('  3. trajectory_data.mat - 轨迹数据（供仿真使用）\n');

fprintf('\n【YZ_AMV1.m修改建议】\n');
fprintf('在第80行附近，修改末端姿态计算:\n');
fprintf('  E1 = transl(...) * troty(pi) * trotx(rx) * troty(ry) * trotz(%.6f);\n', RZ_OFFSET_RAD);

%% ========================================================================
%  辅助函数
%% ========================================================================

function T = transl_m(x, y, z)
    T = eye(4); T(1:3, 4) = [x; y; z];
end

function T = trotx_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4); T(1:3, 1:3) = [1, 0, 0; 0, c, -s; 0, s, c];
end

function T = troty_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4); T(1:3, 1:3) = [c, 0, s; 0, 1, 0; -s, 0, c];
end

function T = trotz_m(angle)
    c = cos(angle); s = sin(angle);
    T = eye(4); T(1:3, 1:3) = [c, -s, 0; s, c, 0; 0, 0, 1];
end

function R = rotx_m(angle)
    c = cos(angle); s = sin(angle);
    R = [1, 0, 0; 0, c, -s; 0, s, c];
end

function R = roty_m(angle)
    c = cos(angle); s = sin(angle);
    R = [c, 0, s; 0, 1, 0; -s, 0, c];
end

function R = rotz_m(angle)
    c = cos(angle); s = sin(angle);
    R = [c, -s, 0; s, c, 0; 0, 0, 1];
end

function quat = rotm2quat_m(R)
    % 旋转矩阵转四元数 [w, x, y, z]
    tr = R(1,1) + R(2,2) + R(3,3);
    if tr > 0
        s = sqrt(tr + 1) * 2;
        w = 0.25 * s;
        x = (R(3,2) - R(2,3)) / s;
        y = (R(1,3) - R(3,1)) / s;
        z = (R(2,1) - R(1,2)) / s;
    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        s = sqrt(1 + R(1,1) - R(2,2) - R(3,3)) * 2;
        w = (R(3,2) - R(2,3)) / s;
        x = 0.25 * s;
        y = (R(1,2) + R(2,1)) / s;
        z = (R(1,3) + R(3,1)) / s;
    elseif R(2,2) > R(3,3)
        s = sqrt(1 + R(2,2) - R(1,1) - R(3,3)) * 2;
        w = (R(1,3) - R(3,1)) / s;
        x = (R(1,2) + R(2,1)) / s;
        y = 0.25 * s;
        z = (R(2,3) + R(3,2)) / s;
    else
        s = sqrt(1 + R(3,3) - R(1,1) - R(2,2)) * 2;
        w = (R(2,1) - R(1,2)) / s;
        x = (R(1,3) + R(3,1)) / s;
        y = (R(2,3) + R(3,2)) / s;
        z = 0.25 * s;
    end
    quat = [w, x, y, z];
end

function Q = ikabb_m(To, tool_l, tool_h, tool_y, tool_a)
    T = To * trotx_m(-tool_a) * transl_m(-tool_h, -tool_y, -tool_l-0.082);
    T(3,4) = T(3,4) - 0.3991;
    a2 = 0.448; a3 = 0.042; d4 = 0.451;
    k = ((T(1,4)^2+T(2,4)^2+T(3,4)^2) - a2^2-a3^2-d4^2)/(2*a2);
    nx = T(1,1); ny = T(2,1); nz = T(3,1);
    ox = T(1,2); oy = T(2,2); oz = T(3,2);
    ax = T(1,3); ay = T(2,3); az = T(3,3);
    Q = zeros(8, 6);
    signs = [1,1,1; 1,1,-1; 1,-1,1; 1,-1,-1; -1,1,1; -1,1,-1; -1,-1,1; -1,-1,-1];
    for idx = 1:8
        Q(idx,:) = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, signs(idx,1), signs(idx,2), signs(idx,3));
    end
end

function ik = compute_ik(T, a2, a3, d4, k, nx,ny,nz,ox,oy,oz,ax,ay,az, sign2, sign3, sign5)
    theta3 = atan2(a3, d4) - atan2(k, sign3*sqrt(a3^2+d4^2-k^2));
    u1 = a3*cos(theta3) - d4*sin(theta3) + a2;
    u2 = a3*sin(theta3) + d4*cos(theta3);
    theta2 = atan2(-T(3,4), sign2*sqrt(u1^2+u2^2-T(3,4)^2)) - atan2(u2, u1);
    v1 = cos(theta2)*u1 - sin(theta2)*u2;
    if v1 > 0, theta1 = atan2(T(2,4), T(1,4));
    elseif v1 < 0, theta1 = atan2(-T(2,4), -T(1,4));
    else, theta1 = 0; end
    c1 = cos(theta1); s1 = sin(theta1);
    c23 = cos(theta2+theta3); s23 = sin(theta2+theta3);
    c5 = -ax*c1*s23 - ay*s1*s23 - az*c23;
    theta5 = atan2(sign5*sqrt(1-c5^2), c5);
    s5 = sin(theta5);
    if s5 > 0
        theta4 = atan2(-ax*s1+ay*c1, -ax*c1*c23-ay*s1*c23+az*s23);
        theta6 = atan2(-ox*c1*s23-oy*s1*s23-oz*c23, nx*c1*s23+ny*s1*s23+nz*c23);
    elseif s5 < 0
        theta4 = atan2(ax*s1-ay*c1, ax*c1*c23+ay*s1*c23-az*s23);
        theta6 = atan2(ox*c1*s23+oy*s1*s23+oz*c23, -nx*c1*s23-ny*s1*s23-nz*c23);
    else, theta4 = 0; theta6 = 0; end
    theta2 = theta2 + pi/2;
    ik = [theta1, theta2, theta3, theta4, theta5, theta6];
end

function qi = ikbest_m(qn, Q)
    s = zeros(1, 8);
    for i = 1:8
        for j = 1:6
            s(i) = s(i) + abs(Q(i,j) - qn(j));
        end
    end
    for i = 1:9
        [~, b] = min(s);
        output = 1;
        if Q(b,1) > (17*pi/18) || Q(b,1) < (-17*pi/18), output = 0; end
        if Q(b,2) > (13*pi/18) || Q(b,2) < (-10*pi/18), output = 0; end
        if Q(b,3) > (7*pi/18) || Q(b,3) < (-20*pi/18), output = 0; end
        if Q(b,4) > (27*pi/18) || Q(b,4) < (-27*pi/18), output = 0; end
        if Q(b,5) > (17*pi/18) || Q(b,5) < (-17*pi/18), output = 0; end
        if Q(b,6) > (13*pi/18) || Q(b,6) < (-13*pi/18), output = 0; end
        if output == 1, qi = Q(b,:); return;
        elseif i <= 8, s(b) = 10000;
        else, error('The solution does not exist!'); end
    end
end

function cfx = rscfx_m(q)
    cfx = zeros(1, 4); d = 1e-14;
    if q(1) > d-pi && q(1) <= d-pi/2, cfx(1) = -2;
    elseif q(1) > d-pi/2 && q(1) <= -d, cfx(1) = -1;
    elseif q(1) > -d && q(1) <= d+pi/2, cfx(1) = 0;
    elseif q(1) > d+pi/2 && q(1) <= d+pi, cfx(1) = 1; end
    if q(4) > d-3*pi/2 && q(4) <= d-pi, cfx(2) = -3;
    elseif q(4) > d-pi && q(4) <= d-pi/2, cfx(2) = -2;
    elseif q(4) > d-pi/2 && q(4) <= -d, cfx(2) = -1;
    elseif q(4) > -d && q(4) <= d+pi/2, cfx(2) = 0;
    elseif q(4) > d+pi/2 && q(4) <= d+pi, cfx(2) = 1;
    elseif q(4) > d+pi && q(4) <= d+3*pi/2, cfx(2) = 2; end
    if q(6) > d-3*pi/2 && q(6) <= d-pi, cfx(3) = -3;
    elseif q(6) > d-pi && q(6) <= d-pi/2, cfx(3) = -2;
    elseif q(6) > d-pi/2 && q(6) <= -d, cfx(3) = -1;
    elseif q(6) > -d && q(6) <= d+pi/2, cfx(3) = 0;
    elseif q(6) > d+pi/2 && q(6) <= d+pi, cfx(3) = 1;
    elseif q(6) > d+pi && q(6) <= d+3*pi/2, cfx(3) = 2; end
    cfx_a = 448*sin(q(2)) + 42*sin(q(2)+q(3)) + 451*sin(pi/2-(q(2)+q(3)));
    cfx_b = q(3); cfx_c = q(5); cfx_r = 0;
    if cfx_a >= 0 && cfx_b > -1.459095 && cfx_c >= 0, cfx_r = 0;
    elseif cfx_a >= 0 && cfx_b > -1.459095 && cfx_c < 0, cfx_r = 1;
    elseif cfx_a >= 0 && cfx_b < -1.50971 && cfx_c >= 0, cfx_r = 2;
    elseif cfx_a >= 0 && cfx_b < -1.50971 && cfx_c < 0, cfx_r = 3;
    elseif cfx_a < 0 && cfx_b > -1.459095 && cfx_c >= 0, cfx_r = 4;
    elseif cfx_a < 0 && cfx_b > -1.459095 && cfx_c < 0, cfx_r = 5;
    elseif cfx_a < 0 && cfx_b < -1.50971 && cfx_c >= 0, cfx_r = 6;
    elseif cfx_a < 0 && cfx_b < -1.50971 && cfx_c < 0, cfx_r = 7; end
    cfx(4) = cfx_r;
end