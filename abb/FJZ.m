%%%此代码用于FP结构多材料制造
clear;
clc;
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
firstl =  57.36e-3+0.0e-3;%%%%此处为测试底板高度
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
z=0.3;
filename = ['fjz_4_2',phead,'.mod'];
YM = xlsread('fjz_4_2.xlsx');
E1{1, 1} = transl(0.65, 0, 360e-3)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{1, 1} = E1{1, 1}; %世界坐标下机械臂末端坐标
TE1{1, 1} = E1{1, 1};  %相对于机械臂基坐标的机械臂末端坐标
rob1{1, 1} = abbrt(RE1{1, 1},TE1{1, 1},tool_lr,tool_hr,tool_yr,tool_ar,ql1);  %此处包含树脂工具的长、高以及角度,单位分别为米和弧度
rob1{1, 1}.pot = 1;
ql1 = rob1{1, 1}.qi;%记录此处各轴数据，为了配合下一点的位置来计算下一点最佳的各轴数据
Th = length(YM);
for hs = 0:1:0 %打印次数（0代表一次）
        for i = 1:1:Th
            if YM(i,8) == 111  %此处包含树脂工具的长、高以及角度,单位分别为米和弧度。
                E1{2 + hs, i} = transl(0.55 + YM(i,1)/1000.0, YM(i,2)/1000.0, firstl+YM(i,3)/1000.0)*troty(pi)*trotx(YM(i,4)/180*pi)*troty(YM(i,5)/180*pi);
                RE1{2 + hs, i} = E1{2 + hs, i}; %世界坐标下机械臂末端坐标
                TE1{2 + hs, i} = E1{2 + hs, i}; %相对于机械臂基坐标的机械臂末端坐标
                rob1{2 + hs, i} = abbrt(RE1{2 + hs, i},TE1{2 + hs, i},tool_lr,tool_hr,tool_yr,tool_ar,ql1);  
                rob1{2 + hs, i}.tool = 111;
            end
            if YM(i,8) == 222  %此处包含纤维工具的长、高以及角度,单位分别为米和弧度。
                E1{2 + hs, i} = transl(0.55 + (YM(i,1)+x)/1000.0, (YM(i,2)+y)/1000.0, firstl+(YM(i,3)+z)/1000.0)*troty(pi)*trotx(YM(i,4)/180*pi)*troty(YM(i,5)/180*pi);
                RE1{2 + hs, i} = E1{2 + hs, i}; %世界坐标下机械臂末端坐标
                TE1{2 + hs, i} = E1{2 + hs, i}; %相对于机械臂基坐标的机械臂末端坐标
                rob1{2 + hs, i} = abbrt(RE1{2 + hs, i},TE1{2 + hs, i},tool_lc,tool_hc,tool_yc,tool_ac,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
                rob1{2 + hs, i}.tool = 222;
            end
            if YM(i,6) == 1    %此处pot判断是否有停顿
            rob1{2 + hs, i}.pot = 1;
            else
            rob1{2 + hs, i}.pot = 0;
            end
            if YM(i,7) == 0     %空驶
               rob1{2 + hs, i}.vcs = 0;
               rob1{2 + hs, i}.moves = 30;
            elseif YM(i,7) ==1
               rob1{2 + hs, i}.vcs = 1;
               rob1{2 + hs, i}.moves = 30;
            elseif YM(i,7) ==2
               rob1{2 + hs, i}.vcs = 2;
               rob1{2 + hs, i}.moves = 30;
            elseif YM(i,7) ==3
               rob1{2 + hs, i}.vcs = 3;
               rob1{2 + hs, i}.moves = 30;
            elseif YM(i,7) ==4
               rob1{2 + hs, i}.vcs = 4;
               rob1{2 + hs, i}.moves = 30;
             elseif YM(i,7) ==5
               rob1{2 + hs, i}.vcs = 5;
               rob1{2 + hs, i}.moves = 30;
            end
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
vcsold=0000;
for i = 2:imax - 1
    for j = 1:jmax
%树脂打印头指令
         if rob1{i, j}.tool == 111
                     if isempty(rob1{i, j}) == 1
                        continue;
%有停顿无奇异点
                     elseif rob1{i, j}.pot == 1
                         if rob1{i, j}.sing == 0
                             if rob1{i,j}.vcs == vcsold
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            else
                            if rob1{i,j}.vcs == 0
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 1
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 1;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime %d;\n',0.2);
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 2
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        WaitTime %d;\n',0.5);
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 1;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 3
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 1;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 4
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime 2;\n');
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        WaitTime 1;\n');
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 5
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 1;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime 0.2;\n');
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            end
                            end
%                             fprintf(fid,'        WaitTime %d;\n',sr);
%有停顿有奇异点

                     elseif rob1{i, j}.sing == 1
                           if rob1{i,j}.vcs == vcsold
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            else
                            if rob1{i,j}.vcs == 0
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 1
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 1;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime %d;\n',0.2);
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 2
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        WaitTime %d;\n',0.5);
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 1;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 3
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 1;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 4
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime 2;\n');
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 1;\n');
                               fprintf(fid,'        WaitTime 1;\n');
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime 1;\n');
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            elseif rob1{i,j}.vcs == 5
                               fprintf(fid,'        SetDo DO0, 1;\n');
                               fprintf(fid,'        SetDo DO1, 1;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        WaitTime 0.2;\n');
                               fprintf(fid,'        SetDo DO0, 0;\n');
                               fprintf(fid,'        SetDo DO1, 0;\n');
                               fprintf(fid,'        SetDo DO2, 0;\n');
                               fprintf(fid,'        SetDo DO3, 0;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolr\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                            end
%                             fprintf(fid,'        WaitTime %d;\n',sr);
                           end
                         end
                         fprintf(fid,'        WaitTime %d;\n',sr);
                     end
                vcsold=rob1{i,j}.vcs;
                tick = tick + 1;
%纤维打印头指令
         elseif rob1{i, j}.tool == 222
                     if isempty(rob1{i, j}) == 1
                        continue;
%无停顿无奇异点                     
                     elseif rob1{i, j}.pot == 0
                         if rob1{i, j}.sing == 0
                            if rob1{i,j}.vcs == vcsold
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,5);
                            else
                                if rob1{i,j}.vcs == 0
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
                                elseif rob1{i,j}.vcs == 1
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,5);
%                                  fprintf(fid,'        WaitTime %d;\n',sc);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 2
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.90);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 3
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                end
                            end
%无停顿有奇异点 
                         elseif rob1{i, j}.sing == 1    
                            if rob1{i,j}.vcs == vcsold
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                            else
                                if rob1{i,j}.vcs == 0
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                elseif rob1{i,j}.vcs == 1
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
%                                  fprintf(fid,'        WaitTime %d;\n',sc);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 2
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.90);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 3
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                end
                            end
                         end
%有停顿无奇异点                     
                     elseif rob1{i, j}.pot == 1
                         if rob1{i, j}.sing == 0
                            if rob1{i,j}.vcs == vcsold
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,5);
                            else
                                if rob1{i,j}.vcs == 0
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
                                elseif rob1{i,j}.vcs == 1
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,5);
%                                  fprintf(fid,'        WaitTime %d;\n',sc);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 2
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.90);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 3
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                end
                            end
%有停顿有奇异点 
                         elseif rob1{i, j}.sing == 1    
                            if rob1{i,j}.vcs == vcsold
                               fprintf(fid,'        AccSet 15,15;\n');
                               fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                            else
                                if rob1{i,j}.vcs == 0
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                elseif rob1{i,j}.vcs == 1
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
%                                  fprintf(fid,'        WaitTime %d;\n',sc);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.5);
                                   fprintf(fid,'        SetDo DO4, 1;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                   fprintf(fid,'        SetDo DO4, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1);
                                elseif rob1{i,j}.vcs == 2
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                   fprintf(fid,'        WaitTime %d;\n',1.90);
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 0;\n');
                                   fprintf(fid,'        SetDo DO3, 0;\n');
                                elseif rob1{i,j}.vcs == 3
                                   fprintf(fid,'        SetDo DO0, 0;\n');
                                   fprintf(fid,'        SetDo DO1, 0;\n');
                                   fprintf(fid,'        SetDo DO2, 1;\n');
                                   fprintf(fid,'        SetDo DO3, 1;\n');
                                   fprintf(fid,'        AccSet 15,15;\n');
                                   fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,5);
                                end
                            end
                         end
                         fprintf(fid,'        WaitTime %d;\n',sc);
                     end
                     vcsold=rob1{i,j}.vcs;
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
            
            
            
            
            
            
            