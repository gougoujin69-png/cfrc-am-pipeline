%%%此代码用于连续纤维打印
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
%firstl =  59.8e-3;%%%%此处为测试底板高度
firstl =  60.2e-3+0.35e-3;
ql1 = [0 0 pi/6.0 0 0 0];  %一号机器人六个关节角的初始姿态
lw = 1e-3;%层宽
printv = 20; %打印速度 
tool_h = -50.7321e-3; %x
tool_y = -50.0753e-3; %y
tool_l = 70.963e-3; %z
tool_a = 30/180*pi;      %工件角度
s = 0.5; %停顿点停止时间
filename = ['yyl2',phead,'.mod'];
%********************************END**************************************
E1{1, 1} = transl(0.65, 0, 360e-3)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{1, 1} = E1{1, 1};%*troty(-tool_a)*transl(-tool_h,-tool_y,-tool_l); %世界坐标下机械臂末端坐标
TE1{1, 1} = E1{1, 1};  %相对于机械臂基坐标的机械臂末端坐标
rob1{1, 1} = abbrt(RE1{1, 1},TE1{1, 1},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
rob1{1, 1}.pot = 1;
ql1 = rob1{1, 1}.qi;%记录此处各轴数据，为了配合下一点的位置来计算下一点最佳的各轴数据
YM = xlsread('yyl2.xlsx');
Th = length(YM);
for hs = 0:1:0 %打印次数（0代表一次）
    for i = 1:1:Th
        E1{2 + hs, i} = transl(0.55 + YM(i, 1)/1000.0, YM(i, 2)/1000.0, firstl+YM(i,3)/1000.0)*troty(pi)*trotx(YM(i,4)/180*pi)*troty(YM(i,5)/180*pi);
        RE1{2 + hs, i} = E1{2 + hs, i};%*troty(-tool_a)*transl(-tool_h,-tool_y,-tool_l); %世界坐标下机械臂末端坐标
        TE1{2 + hs, i} = E1{2 + hs, i};  %相对于机械臂基坐标的机械臂末端坐标
        rob1{2 + hs, i} = abbrt(RE1{2 + hs, i},TE1{2 + hs, i},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
        if YM(i,6) == 1%此处判断从文件中得到的pot值，根据pot值来判断是否需要停顿
            rob1{2 + hs, i}.pot = 1;
        else
            rob1{2 + hs, i}.pot = 0;
        end
        if YM(i,7) == 0
            rob1{2 + hs, i}.vcs = 0;
        elseif YM(i,7) == 1
            rob1{2 + hs, i}.vcs = 1;
        elseif YM(i,7) == 2
            rob1{2 + hs, i}.vcs = 2;
        elseif YM(i,7) == 11
            rob1{2 + hs, i}.vcs = 11;
        elseif YM(i,7) == 12
            rob1{2 + hs, i}.vcs = 12;
        elseif YM(i,7) == 13
            rob1{2 + hs, i}.vcs = 13;
        else
            rob1{2 + hs, i}.vcs = 0;
        end
        ql1 = rob1{2 + hs, i}.qi;
        rob1{2 + hs, i}.moves = YM(i,8);
    end
end
imax = 2 + hs;
jmax = Th;

i = imax + 1;
E1{i, 1} = transl(0.65, 0, 0.36)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{i, 1} = E1{i,1}; %世界坐标下机械臂末端坐标
TE1{i, 1} = E1{i,1};  %相对于机械臂基坐标的机械臂末端坐标
rob1{i, 1} = abbrt(RE1{i,1},TE1{i,1},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
rob1{i, 1}.pot = 1;
ql1 = rob1{i, 1}.qi;
imax = i;%总共涉及到的节点数
%-----------------------判断奇点并标记相应姿态点-----------------------------
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
%fprintf(fid,'    TASK PERS tooldata toolc:=[TRUE,[[-50.5267,-51.39578,72.43274],[0.965925826289068,0.258819045102521,0,0]],[1,[0,0,5],[1,0,0,0],0,0,0]];\n');
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
            j = j + 1;
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
fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,1,20);
fprintf(fid,'        WaitTime 2;\n');
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 0;\n');
fprintf(fid,'        SetDo DO2, 1;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
fprintf(fid,'        WaitTime 2.8;\n');
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 0;\n');
fprintf(fid,'        SetDo DO2, 0;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
fprintf(fid,'        WaitTime 5;\n');
tick = 2;
vcsold=0000;
for i = 2:imax - 1
%     if rob1{i, 1}.sing == 0
%         fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,printv);
%         %fprintf(fid,'        WaitSyncTask sync%d, task_list\n',s);
%     elseif rob1{i, 1}.sing == 1
%         fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,printv);
%         %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
%     end
%     tick = tick + 1;
    for j = 1:jmax
        if isempty(rob1{i, j}) == 1
            continue;
%无停顿且无奇异点            
        elseif rob1{i, j}.pot == 0
            if rob1{i, j}.sing == 0
                if rob1{i,j}.vcs == vcsold
                    fprintf(fid,'        AccSet 15,15;\n');
                    fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                else
                    if rob1{i,j}.vcs == 0
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        %fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 1
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
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
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        WaitTime %d;\n',2.9);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                    elseif rob1{i,j}.vcs == 11
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    elseif rob1{i,j}.vcs == 12
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 1;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    elseif rob1{i,j}.vcs == 13
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,z1,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    end
                end
                
%无停顿有奇异点
            elseif rob1{i, j}.sing == 1
                if rob1{i,j}.vcs == vcsold
                    fprintf(fid,'        AccSet 15,15;\n');
                    fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                else                
                    if rob1{i,j}.vcs == 0
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 1
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
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
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        WaitTime %d;\n',2.9);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                    elseif rob1{i,j}.vcs == 11
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    elseif rob1{i,j}.vcs == 12
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 1;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    elseif rob1{i,j}.vcs == 13
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    end
                end
            end
%%%%%
%%%%%有停顿
%%%%%无奇异点
        elseif rob1{i, j}.pot == 1 
            if rob1{i, j}.sing == 0
               if rob1{i,j}.vcs == vcsold
                    fprintf(fid,'        AccSet 15,15;\n');
                    fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    fprintf(fid,'        WaitTime %d;\n',s);
                else 
                    if rob1{i,j}.vcs == 0
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 1
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
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
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        WaitTime %d;\n',2.9);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                    elseif rob1{i,j}.vcs == 11
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 12
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 1;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 13
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    end
               end
            elseif rob1{i, j}.sing == 1
                if rob1{i,j}.vcs == vcsold
                    fprintf(fid,'        AccSet 15,15;\n');
                    fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                    fprintf(fid,'        WaitTime %d;\n',s);
                else 
                    if rob1{i,j}.vcs == 0
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 1
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
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
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        WaitTime %d;\n',2.9);
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                    elseif rob1{i,j}.vcs == 11
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 12
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 1;\n');
                        fprintf(fid,'        SetDo DO2, 0;\n');
                        fprintf(fid,'        SetDo DO3, 0;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    elseif rob1{i,j}.vcs == 13
                        fprintf(fid,'        SetDo DO0, 0;\n');
                        fprintf(fid,'        SetDo DO1, 0;\n');
                        fprintf(fid,'        SetDo DO2, 1;\n');
                        fprintf(fid,'        SetDo DO3, 1;\n');
                        fprintf(fid,'        AccSet 15,15;\n');
                        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z10,toolc\\WObj:=wobj0;\n',phead,tick,rob1{i, j}.moves);
                        %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                        fprintf(fid,'        WaitTime %d;\n',s);
                    end
                end
            end
%                 if rob1{i,j-1}.laser~=rob1{i,j}.laser
%                     fprintf(fid,'        SetDo DO0, %d;\n',rob1{i,j}.laser);
%                 end
        end
        vcsold=rob1{i,j}.vcs;
        tick = tick + 1;
    end
end
%此处设置最后抬起阶段
%设定抬起高度为0.36m，抬起的速度为10mm/s,抬起时候正常挤出
fprintf(fid,'        SetDo DO0, 0;\n');
fprintf(fid,'        SetDo DO1, 1;\n');
fprintf(fid,'        SetDo DO2, 0;\n');
fprintf(fid,'        SetDo DO3, 0;\n');
fprintf(fid,'        AccSet 15,15;\n');
if rob1{imax, 1}.sing == 0
    fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
elseif rob1{imax, 1}.sing == 1
    fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,toolc\\WObj:=wobj0;\n',phead,tick,10);
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