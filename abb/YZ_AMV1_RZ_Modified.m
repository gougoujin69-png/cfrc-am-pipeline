%%%此代码用于FP结构多材料制造
clear;
clc;
%建立信号传输程序
%YM(i,8)=111,碳纤维打印
%YM(i,8)=222,树脂打印
%Steppermotor总共有4个信号
%set DO0 DO1 DO2 DO3 DO5=1
signal1=30;  %DOO 0 DO1 0 DO2 0 DO3 0      %%%代表所有信号关闭
signal3=20;  %DOO 0 DO1 0 DO2 1 DO3 0      %%%碳纤维喷头%%%裁剪后进给
signal4=5;  %DOO 0 DO1 0 DO2 1 DO3 1      %%%碳纤维喷头%%%5mm/s打印速度匹配
signal5=10;  %DOO 0 DO1 1 DO2 0 DO3 0      %%%碳纤维喷头%%%10mm/s打印速度匹配
signal6=20;  %DOO 0 DO1 1 DO2 0 DO3 1      %%%碳纤维喷头%%%20mm/s打印速度匹配
signal7=0;  %DOO 0 DO1 1 DO2 1 DO3 0      %%%碳纤维喷头%%%
signal8=0;  %DOO 0 DO1 1 DO2 1 DO3 1      %%%碳纤维喷头%%%
signal9=0;  %DOO 1 DO1 0 DO2 0 DO3 0       %%%碳纤维喷头%%%
signal10=40; %DOO 1 DO1 0 DO2 0 DO3 1      %%%树脂喷头%%%打印正常层：40mm/s
signal11=60; %DOO 1 DO1 0 DO2 1 DO3 0      %%%树脂喷头%%%空驶：60mm/s
signal12=30; %DOO 1 DO1 0 DO2 1 DO3 1      %%%树脂喷头%%%打印首层：30mm/s
signal13=30; %DOO 1 DO1 1 DO2 0 DO3 0      %%%树脂喷头%%%树脂头回抽
signal14=30; %DOO 1 DO1 1 DO2 0 DO3 1      %%%树脂喷头%%%
signal15=30; %DOO 1 DO1 1 DO2 1 DO3 0      %%%树脂喷头%%%
signal16=30; %DOO 1 DO1 1 DO2 0 DO3 1      %%%树脂喷头%%%
%Servocode总共有1个信号
%set DO4=1   %%%启动裁剪功能%%%
%%%%对于碳纤维复合材料[111]%%%状态信号设定%%%%
%vcs=10;%空驶 
%vcs=11;%裁剪
%vcs=12:裁剪后进给
%vcs=13;%5mm/s打印速度匹配
%vcs=14;%10mm/s打印速度匹配
%vcs=15;%20mm/s打印速度匹配
%%%%对于纯树脂[222]%%%状态信号设定%%%%
%vcs=20;%空驶
%vcs=21;%回抽
%vcs=22;%40mm/s正常层打印
%vcs=23;%30mm/s首层打印
%----------------------------规定数据格式----------------------------------
rbtform = ['    CONST robtarget Target_%s_%d:=[[%9.9f,%9.9f,%9.9f],'...
    '[%9.9f,%9.9f,%9.9f,%9.9f],[%d,%d,%d,%d],'...
    '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n'];
jttform = ['    CONST jointtarget Target_%s_%d:='...
    '[[%9.9f,%9.9f,%9.9f,%9.9f,%9.9f,%9.9f],'...
    '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n'];
%*****************************END*****************************************
%关于停顿问题，此处重新定义pot为1代表停顿，pot为0代表不停顿
%-----------------------------初始参量-------------------------------------
tol = 1e-10;
nl = 0;%初始层高
phead = 'Main';%%Main
firstl =  57.36e-3+0.20e-3-1.03e-3;%%%%此处为测试底板高度
ql1 = [0 0 pi/6.0 0 0 0];  %一号机器人六个关节角的初始姿态
lw = 1e-3;%层宽
printv = 20; %打印速度 
tool_hc = -50.7321e-3; %x
tool_yc = -50.0753e-3; %y
tool_lc = 70.963e-3; %z
tool_ac = 30/180*pi;      %纤维工件角度
tool_hr = -50.3541e-3; %x
tool_yr = 50.9039e-3; %y
tool_lr = 71.4291e-3; %z
tool_ar = -30/180*pi;      %树脂工件角度
sc = 0.2; %纤维停顿点停止时间
sr = 0.001; %树脂停顿点停止时间
x=-0.5;%纤维头补偿值,单位mm
y=-1.5;
z=0.2;
%% ============ RZ偏移参数（用于避免奇异点）============
% 通过RZ_Offset_Analysis.m脚本分析得到最优值
% path_23_1: 推荐 delta_rz = -pi/2 (-90°)
% path_23_2: 推荐 delta_rz = 35*pi/180 (+35°)
delta_rz = -pi/2;  % RZ偏移角度（弧度），根据路径文件调整
%% ====================================================
filename = ['path_23_1',phead,'.mod'];
YM = xlsread('path_23_1.xlsx');
E1{1, 1} = transl(0.65, 0, 360e-3)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{1, 1} = E1{1, 1}; %世界坐标下机械臂末端坐标
TE1{1, 1} = E1{1, 1};  %相对于机械臂基坐标的机械臂末端坐标
rob1{1, 1} = abbrt(RE1{1, 1},TE1{1, 1},tool_lr,tool_hr,tool_yr,tool_ar,ql1);  %此处包含树脂工具的长、高以及角度,单位分别为米和弧度
rob1{1, 1}.pot = 1;
ql1 = rob1{1, 1}.qi;%记录此处各轴数据，为了配合下一点的位置来计算下一点最佳的各轴数据
Th = length(YM);
for hs = 0:1:0 %打印次数（0代表一次）
        for i = 1:1:Th
            if YM(i,8) == 111  %此处包含纤维工具的长、高以及角度,单位分别为米和弧度。
                E1{2 + hs, i} = transl(0.55 + (YM(i,1)+x)/1000.0, (YM(i,2)+y)/1000.0, firstl+(YM(i,3)+z)/1000.0)*troty(pi)*trotx(YM(i,4)/180*pi)*troty(YM(i,5)/180*pi)*trotz(delta_rz);
                RE1{2 + hs, i} = E1{2 + hs, i}; %世界坐标下机械臂末端坐标
                TE1{2 + hs, i} = E1{2 + hs, i}; %相对于机械臂基坐标的机械臂末端坐标
                rob1{2 + hs, i} = abbrt(RE1{2 + hs, i},TE1{2 + hs, i},tool_lc,tool_hc,tool_yc,tool_ac,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
                rob1{2 + hs, i}.tool = 111;
            end
            if YM(i,8) == 222  %此处包含树脂工具的长、高以及角度,单位分别为米和弧度。
                E1{2 + hs, i} = transl(0.55 + YM(i,1)/1000.0, YM(i,2)/1000.0, firstl+YM(i,3)/1000.0)*troty(pi)*trotx(YM(i,4)/180*pi)*troty(YM(i,5)/180*pi)*trotz(delta_rz);
                RE1{2 + hs, i} = E1{2 + hs, i}; %世界坐标下机械臂末端坐标
                TE1{2 + hs, i} = E1{2 + hs, i}; %相对于机械臂基坐标的机械臂末端坐标
                rob1{2 + hs, i} = abbrt(RE1{2 + hs, i},TE1{2 + hs, i},tool_lr,tool_hr,tool_yr,tool_ar,ql1);  
                rob1{2 + hs, i}.tool = 222;
            end
            if YM(i,6) == 1    %此处pot判断是否有停顿
            rob1{2 + hs, i}.pot = 1;
            else
            rob1{2 + hs, i}.pot = 0;
            end
            rob1{2 + hs, i}.vcs=YM(i,7);
            ql1 = rob1{2 + hs, i}.qi;
        end
end
imax = 2 + hs;
jmax = Th;
%-----------------------判断奇点并标记相应姿态点-----------------------------
i = imax + 1;
E1{i, 1} = transl(0.65, 0, 0.36)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{i, 1} = E1{i,1}; %世界坐标下机械臂末端坐标
TE1{i, 1} = E1{i,1};  %相对于机械臂基坐标的机械臂末端坐标
rob1{i, 1} = abbrt(RE1{i,1},TE1{i,1},tool_lr,tool_hr,tool_yr,tool_ar,ql1);  %此处有bug，奇异点判断并不准确。
rob1{i, 1}.pot = 1;
ql1 = rob1{i, 1}.qi;
imax = i;%总共涉及到的节点数
rob1{1, 1}.sing = 1;%sing为1表示该点为奇异点
rob1{1, 1}.qi = rad2deg(rob1{1, 1}.qi);
j = 2;
for i = 2:imax
    if rob1{i, 1}.rcf(4) ~=  rob1{i-1, j-1}.rcf(4)
        rob1{i, 1}.sing = 1;
        rob1{i, 1}.qi = rad2deg(rob1{i, 1}.qi);
    end
    for j = 2:jmax
        if isempty(rob1{i, j}) == 1
            continue;
        else
            if rob1{i, j}.rcf(4) ~=  rob1{i, j-1}.rcf(4)
                rob1{i, j}.sing = 1;
                rob1{i, j}.qi = rad2deg(rob1{i, j}.qi);%计算流程为得到轴数据从而确认轴配置，计算都用弧度值，而为了输出Rapid代码，此处用角度值覆盖掉
            end
        end
    end
end
%-----------------------输出为RobotStudio可读代码--------------------------
%Robot1
tick = 1;
fid = fopen(filename,'w');
fprintf(fid,'MODULE %sModule\n', phead);
fprintf(fid,'    TASK PERS tooldata toolr:=[TRUE,[[-50.3541,50.9039,71.4291],[0.965925826289068,-0.258819045102521,0,0]],[1,[0,0,5],[1,0,0,0],0,0,0]];\n');
fprintf(fid,'    TASK PERS tooldata toolc:=[TRUE,[[-50.7321,-50.0753,70.963],[0.965925826289068,0.258819045102521,0,0]],[1,[0,0,5],[1,0,0,0],0,0,0]];\n');
%输出路径点
for i = 1:imax%i表示循环数加首尾
    for j = 1:jmax
        if isempty(rob1{i, j}) == 1
            continue;
        else
            if rob1{i, j}.sing == 0
                fprintf(fid,rbtform,phead,tick,rob1{i, j}.rp(1),rob1{i, j}.rp(2),rob1{i, j}.rp(3),...
                    rob1{i, j}.rq(1),rob1{i, j}.rq(2),rob1{i, j}.rq(3),rob1{i, j}.rq(4),...
                    rob1{i, j}.rcf(1),rob1{i, j}.rcf(2),rob1{i, j}.rcf(3),rob1{i, j}.rcf(4));
            elseif rob1{i, j}.sing == 1
                fprintf(fid,jttform,phead,tick,rob1{i, j}.qi(1),rob1{i, j}.qi(2),rob1{i, j}.qi(3),...
                    rob1{i, j}.qi(4),rob1{i, j}.qi(5),rob1{i, j}.qi(6));
            end
%             j = j + 1;
            tick = tick + 1;
        end
    end
end
%输出移动轨迹主程序
fprintf(fid,'    PROC %s()\n',phead);
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 0;\n');
fprintf(fid,'        SetDo DO2, 0;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,1,20);
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 0;\n');
fprintf(fid,'        SetDo DO2, 0;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
fprintf(fid,'        WaitTime 2;\n');
tick = 2;
for i = 2:imax - 1
    for j = 1:jmax
    
%树脂打印头指令
         if rob1{i, j}.tool == 222
                     if isempty(rob1{i, j}) == 1
                        continue;
%无奇异点
                     elseif rob1{i, j}.pot == 1
                         if rob1{i, j}.sing == 0
                            if rob1{i,j}.vcs == 20%空驶
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal11);%60mm/s
                            elseif rob1{i,j}.vcs == 21%回抽完空驶
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 1;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime %d;\n',0.1);
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal11);
                            elseif rob1{i,j}.vcs == 22%正常层打印40mm/s
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal10);
                            elseif rob1{i,j}.vcs == 23%首层打印30mm/s
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 1;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal12);
                            end                  
                       %有奇异点
                     elseif rob1{i, j}.sing == 1
                            if rob1{i,j}.vcs == 20%空驶
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal11);%60mm/s
                            elseif rob1{i,j}.vcs == 21%回抽完空驶
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 1;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime %d;\n',0.1);
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal11);
                            elseif rob1{i,j}.vcs == 22%正常层打印40mm/s
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal10);
                            elseif rob1{i,j}.vcs == 23%首层打印30mm/s
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 1;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,signal12);
                           end
                         end
                         fprintf(fid,'        WaitTime %d;\n',sr);
                     end
                tick = tick + 1;             
%纤维打印头指令
         elseif rob1{i, j}.tool == 111
                     if isempty(rob1{i, j}) == 1
                        continue;
%无停顿无奇异点                     
                     elseif rob1{i, j}.pot == 0
                         if rob1{i, j}.sing == 0
                                if rob1{i,j}.vcs == 10%空驶
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                elseif rob1{i,j}.vcs == 11%路径末端裁剪操作
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 12%裁剪后进给:30mm/s:先空驶至目标位置，然后快速进丝
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',2.1);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 13%5mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,signal4);
                                 elseif rob1{i,j}.vcs == 14%10mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,signal5);
                                 elseif rob1{i,j}.vcs == 15%20mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,signal6);
                                end
%无停顿有奇异点 
                         elseif rob1{i, j}.sing == 1    
                                 if rob1{i,j}.vcs == 10%空驶
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                elseif rob1{i,j}.vcs == 11%路径末端裁剪操作
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,10);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 12%裁剪后进给:30mm/s:先空驶至目标位置，然后快速进丝
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',2.1);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 13%5mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,signal4);
                                 elseif rob1{i,j}.vcs == 14%10mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,signal5);
                                 elseif rob1{i,j}.vcs == 15%20mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,signal6);
                                 end
                         end
%有停顿无奇异点                     
                     elseif rob1{i, j}.pot == 1
                         if rob1{i, j}.sing == 0
                                if rob1{i,j}.vcs == 10%空驶
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                elseif rob1{i,j}.vcs == 11%路径末端裁剪操作
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 12%裁剪后进给:30mm/s:先空驶至目标位置，然后快速进丝
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',2.1);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 13%5mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,signal4);
                                 elseif rob1{i,j}.vcs == 14%10mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,signal5);
                                 elseif rob1{i,j}.vcs == 15%20mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,signal6);
                                end
%有停顿有奇异点 
                         elseif rob1{i, j}.sing == 1    
                                 if rob1{i,j}.vcs == 10%空驶
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                elseif rob1{i,j}.vcs == 11%路径末端裁剪操作
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,10);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 12%裁剪后进给:30mm/s:先空驶至目标位置，然后快速进丝
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,20);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',2.1);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 13%5mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,signal4);
                                 elseif rob1{i,j}.vcs == 14%10mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,signal5);
                                 elseif rob1{i,j}.vcs == 15%20mm/s打印
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 1;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,signal6);
                                end
                         end
                         fprintf(fid,'        WaitTime %d;\n',sc);
                     end
                     tick = tick + 1;
         end
    end
end
%此处设置最后抬起阶段
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 0;\n');
fprintf(fid,'        SetDo DO2, 0;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
fprintf(fid,'        AccSet 15,15;\n');
    if rob1{imax, 1}.sing == 0
        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,30);
    elseif rob1{imax, 1}.sing == 1
        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,30);
    end     
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 0;\n');
fprintf(fid,'        SetDo DO2, 0;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
%抬起完成之后，关闭所有信号
fprintf(fid,'        WaitTime %d;\n',100000);
fprintf(fid,'    ENDPROC\n');
fprintf(fid,'ENDMODULE');
fclose(fid);
%********************************END**************************************

fprintf('Finish!')
%********************************END**************************************