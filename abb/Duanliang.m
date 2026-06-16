clear
clc
%----------------------------规定数据格式----------------------------------
rbtform = ['    CONST robtarget Target_%s_%d:=[[%9.9f,%9.9f,%9.9f],'...
    '[%9.9f,%9.9f,%9.9f,%9.9f],[%d,%d,%d,%d],'...
    '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n'];
jttform = ['    CONST jointtarget Target_%s_%d:='...
    '[[%9.9f,%9.9f,%9.9f,%9.9f,%9.9f,%9.9f],'...
    '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n'];
%*****************************END*****************************************

%-----------------------------初始参量-------------------------------------
tol = 1e-10;
nl = 1;
layerh = 0.2e-3; %单层高度，单位米d
status = 2;
phead = 'Main';
firstl =  1*nl*layerh + 51.9e-3;%初始层高度 * 31.3
ql1 = [0 0 pi/6.0 0 0 0];  %一号机器人六个关节角的初始姿态
lw = 0.8e-3;%层宽
printv = 50; %打印速度 
tool_h = 89.4e-3; %工件高度
tool_l = 74.53e-3; %工件长度
tool_y = 16.59e-3; %工件横向位移
tool_a = pi/2.0;      %工件角度
s = 1;
filename = ['B1013',phead,'.mod'];
%********************************END**************************************

%-----------------------生成机器人的末端轨迹-----------------------------
E1{1, 1} = transl(0.65, -0.13, firstl)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{1, 1} = E1{1, 1}*troty(-tool_a)*transl(-tool_h,-tool_y,-tool_l); %世界坐标下机械臂末端坐标
TE1{1, 1} = E1{1, 1};  %相对于机械臂基坐标的机械臂末端坐标
rob1{1, 1} = abbrt(RE1{1, 1},TE1{1, 1},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
rob1{1, 1}.pot = 1;
ql1 = rob1{1, 1}.qi;
i = 2;
j = 1;
imax = 2;
jmax = 1;
syms t;
syms h;
for ks = 0:1:1
    if mod(ks, 2) == 0
        hs = 0;
        hst = 1;
        hf = 2;
    else
        hs = 2;
        hst = -1;
        hf = 0;
    end
    for h = hs:hst:hf
        j = 1;
        if mod(i, 2) == 0
            for t = 0:10e-3:120e-3
                E1{i, j} = transl(0.65 + h*lw, -0.06 + t, firstl + ks*layerh)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
                RE1{i, j} = E1{i, j}*troty(-tool_a)*transl(-tool_h,-tool_y,-tool_l); %世界坐标下机械臂末端坐标
                TE1{i, j} = E1{i, j};  %相对于机械臂基坐标的机械臂末端坐标
                rob1{i, j} = abbrt(RE1{i, j},TE1{i, j},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
                if abs(t - 0)<tol || abs(t - 120e-3)<tol
                    rob1{i,j}.pot = 0;
                end
                ql1 = rob1{i, j}.qi;
                jmax_t = j;
                if(jmax <= jmax_t)
                    jmax = jmax_t;
                end
                j = j+1;
            end
        elseif mod(i, 2) ~= 0
            for t = 120e-3:-10e-3:0
                E1{i, j} = transl(0.65 + h*lw, -0.06 + t, firstl + ks*layerh)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
                RE1{i, j} = E1{i, j}*troty(-tool_a)*transl(-tool_h,-tool_y,-tool_l); %世界坐标下机械臂末端坐标
                TE1{i, j} = E1{i, j};  %相对于机械臂基坐标的机械臂末端坐标
                rob1{i, j} = abbrt(RE1{i, j},TE1{i, j},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
                if abs(t - 0)<tol || abs(t - 120e-3)<tol
                    rob1{i,j}.pot = 0;
                end
                ql1 = rob1{i, j}.qi;
                jmax_t = j;
                if(jmax <= jmax_t)
                    jmax = jmax_t;
                end
                j = j+1;
            end
        end
        imax = i;
        i = i+1;
    end
end
E1{i, 1} = transl(0.580-0.125*cos(0), -0.125*sin(0), 300e-3)*troty(pi); %世界坐标下带工具的末端坐标，单位为米
RE1{i, 1} = E1{i}*troty(-tool_a)*transl(-tool_h,-tool_y,-tool_l); %世界坐标下机械臂末端坐标
TE1{i, 1} = E1{i};  %相对于机械臂基坐标的机械臂末端坐标
rob1{i, 1} = abbrt(RE1{i},TE1{i},tool_l,tool_h,tool_y,tool_a,ql1);  %此处包含打印工具的长、高以及角度,单位分别为米和弧度。
rob1{i, 1}.pot = 1;
ql1 = rob1{i, 1}.qi;
imax = i;
%********************************END**************************************

%-----------------------判断奇点并标记相应姿态点-----------------------------
rob1{1, 1}.sing = 1;
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
                rob1{i, j}.qi = rad2deg(rob1{i, j}.qi);
            end
        end
    end
end

% for i  = 1:imax
%     for j = 1:jmax
%         if isempty(rob1{i, j}) == 1
%             continue;
%         elseif isempty(rob1{i, j+1}) == 1
%                 rob1{i,j}.pot = 2
%         elseif rob1{i,j}.pot = 1
%             if rob{i, j+1}.pot = 1
%                 rob1{i, j} = 2;
%             end
%         end
%     end
% end
                
% for i = 2:imax
%     if rob1{i}.rcf(4) ~=  rob1{i-1}.rcf(4)
%         rob1{i}.sing = 1;
%         rob1{i}.qi = rad2deg(rob1{i}.qi);
%     end
% end
%********************************END**************************************

%-----------------------标定奇点-----------------------------%单机械臂运动不需要
% s = 2;
% smax = 1;
% for i = 2:imax
%     if rob1{i}.sing == 1
%         smax = s;  %记录协同停顿点的个数
%         sing(s) = i;   %标记协同停顿点的编号
%         s = s+1;
%     end
% end
% sing(smax+1) = 0;
%********************************END**************************************

%-----------------------输出为RobotStudio可读代码--------------------------
%Robot1
tick = 1;
fid = fopen(filename,'w');
fprintf(fid,'MODULE %sModule\n', phead);
%输出路径点
for i = 1:imax
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
if(status == 0 || status == 2)
    fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,1,50);
    fprintf(fid,'        SetDo DO0, 1;\n');
    fprintf(fid,'        WaitTime 1;\n');
end
tick = 2;
for i = 2:imax - 1
%     if rob1{i, 1}.sing == 0
%         fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,printv);
%         %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
%     elseif rob1{i, 1}.sing == 1
%         fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,printv);
%         %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
%     end
%     tick = tick + 1;
    for j = 1:jmax
        if isempty(rob1{i, j}) == 1
            continue;
        elseif rob1{i, j}.pot == 1
            if rob1{i, j}.sing == 0
                fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,5);
                %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                fprintf(fid,'        WaitTime %d;\n',s);
            elseif rob1{i, j}.sing == 1
                fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,5);
                %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
                fprintf(fid,'        WaitTime %d;\n',s);
            end
        elseif rob1{i, j}.pot == 0
            if rob1{i, j}.sing == 0
                fprintf(fid,'        MoveL Target_%s_%d,v%d,z100,tool0\\WObj:=wobj0;\n',phead,tick,printv);
                %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
            elseif rob1{i, j}.sing == 1
                fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,z100,tool0\\WObj:=wobj0;\n',phead,tick,printv);
                %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
            end
        end
        j = j + 1;
        tick = tick + 1;
    end
%     if isempty(rob1{i, jmax}) ~= 1
%         if rob1{i, jmax}.sing == 0
%             fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,printv);
%             %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
%         elseif rob1{i, jmax}.sing == 1
%             fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,printv);
%             %fprintf(fid,'        WaitSyncTask sync%d, task_list;\n',s);
%         end
%         tick = tick + 1;
%     end
end
if(status == 1 || status == 2)
    fprintf(fid,'        WaitTime 1;\n');
    fprintf(fid,'        SetDo DO0, 0;\n');
    if rob1{imax, 1}.sing == 0
        fprintf(fid,'        MoveL Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,50);
    elseif rob1{imax, 1}.sing == 1
        fprintf(fid,'        MoveAbsJ Target_%s_%d,v%d,fine,tool0\\WObj:=wobj0;\n',phead,tick,50);
    end
end
fprintf(fid,'    ENDPROC\n');
fprintf(fid,'ENDMODULE');
fclose(fid);
%********************************END**************************************

fprintf('Finish!')
%********************************END**************************************