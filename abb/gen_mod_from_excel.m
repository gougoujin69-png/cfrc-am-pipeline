function out = gen_mod_from_excel(excel, modfile, opts)
% GEN_MOD_FROM_EXCEL  Robot path Excel -> ABB RAPID .mod (carbon).
%
% Consumes the 8-column table produced by gen_robot_excel.m
%   [ x y z(mm)  rx ry(deg)  pot  vcs  tool ]
% places it in the robot work frame (same constants as YZ_AMV1_RZ_Modified),
% chooses a per-point roll about the tool axis (avoids singularities + idle-
% nozzle collisions, opts.roll='opt'), solves the FIXED analytic IK
% (ikabb/ikbest/rscfx), flags singularities with the improved test
% (needs_moveabsj), and writes a valid RAPID module: CONST robtarget /
% jointtarget plus a PROC that emits MoveL (normal) or MoveAbsJ (singular)
% with the carbon tool. This is the end of the chain:
%   Manufacturing_printing_path.mat -> gen_robot_excel -> gen_mod_from_excel -> .mod
%
% opts.roll : 'opt' (per-point optimised, default) | 'fixed' (opts.delta) | 'zero'
% opts.delta: fixed roll (rad) when roll=='fixed'
% opts.print_speed / opts.travel_speed (mm/s)

if nargin < 2 || isempty(modfile), modfile = 'carbon_out.mod'; end
if nargin < 3, opts = struct(); end
if ~isfield(opts,'roll'),  opts.roll  = 'opt'; end
if ~isfield(opts,'delta'), opts.delta = 0;     end
if ~isfield(opts,'print_speed'),  opts.print_speed  = 10; end
if ~isfield(opts,'travel_speed'), opts.travel_speed = 20; end
addpath(fileparts(mfilename('fullpath')));

% ---- placement / tool constants (match YZ_AMV1_RZ_Modified.m) ----
firstl = 57.36e-3 + 0.20e-3 - 1.03e-3;
base_x = 0.55;
carbon = struct('h',-50.7321e-3,'y',-50.0753e-3,'l',70.963e-3,'a', 30/180*pi);
resin  = struct('h',-50.3541e-3,'y', 50.9039e-3,'l',71.4291e-3,'a',-30/180*pi);
cx=-0.5; cy=-1.5; cz=0.2;           % carbon xyz compensation (mm)
ql = [0 0 pi/6 0 0 0];

YM = readmatrix(excel);
n  = size(YM,1);

tgt = repmat(struct('rp',[0 0 0],'rq',[1 0 0 0],'rcf',[0 0 0 0], ...
                    'qi',zeros(1,6),'sing',false,'vcs',14,'pot',0), n, 1);
tip_hist = zeros(n,3);
prev_delta = 0; rcf_prev = []; qprev = [];
nSing = 0; nInf = 0;

ropts = opts;            % forward w_coll / coll_stride / weights to optimiser
ropts.rcf_prev = [];
for i = 1:n
    x=YM(i,1); y=YM(i,2); z=YM(i,3);
    rx=YM(i,4)*pi/180; ry=YM(i,5)*pi/180;
    tip = [base_x + (x+cx)/1000, (y+cy)/1000, firstl + (z+cz)/1000];

    switch opts.roll
        case 'opt'
            ropts.rcf_prev = rcf_prev;
            res = optimize_print_roll(tip, rx, ry, carbon, resin, ql, prev_delta, tip_hist(1:i-1,:), ropts);
            if ~res.feasible
                delta = 0; E1 = pose(tip,rx,ry,delta);
                Q = ikabb(E1,carbon.l,carbon.h,carbon.y,carbon.a);
                qi = trySolve(ql,Q); if isempty(qi), qi=ql; nInf=nInf+1; end
                rcf = rscfx(qi);
            else
                delta = res.delta; qi = res.qi; rcf = res.rcf; E1 = pose(tip,rx,ry,delta);
            end
        otherwise
            if strcmp(opts.roll,'fixed'), delta = opts.delta; else, delta = 0; end
            E1 = pose(tip,rx,ry,delta);
            Q = ikabb(E1,carbon.l,carbon.h,carbon.y,carbon.a);
            qi = trySolve(ql,Q); if isempty(qi), qi=ql; nInf=nInf+1; end
            rcf = rscfx(qi);
    end

    [sg,~] = needs_moveabsj(qi, rcf, qprev, rcf_prev);
    rp = (E1(1:3,4).')*1000;             % mm
    rq = rot2quat(E1(1:3,1:3));
    tgt(i).rp=rp; tgt(i).rq=rq; tgt(i).rcf=rcf; tgt(i).qi=qi*180/pi;
    tgt(i).sing=sg; tgt(i).vcs=YM(i,7); tgt(i).pot=YM(i,6);
    if sg, nSing=nSing+1; end

    tip_hist(i,:)=tip; ql=qi; prev_delta=delta; rcf_prev=rcf; qprev=qi;
end

[dd,nm,~] = fileparts(modfile);
if isfield(opts,'split') && opts.split > 0 && n > opts.split
    nparts = ceil(n/opts.split); files = cell(nparts,1);
    for p = 1:nparts
        a = (p-1)*opts.split+1; b = min(n, p*opts.split);
        pf = fullfile(dd, sprintf('%s_part%02d.mod', nm, p));
        write_mod_range(pf, tgt(a:b), opts, p);
        files{p} = pf;
    end
    fprintf('[gen_mod_from_excel] %d points -> %d module files %s_partNN.mod\n', n, nparts, nm);
    out = struct('files',{files},'points',n,'parts',nparts,'singularities',nSing,'infeasible',nInf);
else
    write_mod_range(modfile, tgt, opts, 0);
    out = struct('file',modfile,'points',n,'singularities',nSing,'infeasible',nInf);
end
fprintf('[gen_mod_from_excel] total: %d points, %d MoveAbsJ(sing), %d IK-infeasible\n', n, nSing, nInf);
end

% ---------------------------------------------------------------------------
function E1 = pose(tip,rx,ry,delta)
E1 = transl(tip(1),tip(2),tip(3))*troty(pi)*trotx(rx)*troty(ry)*trotz(delta);
end
function qi = trySolve(ql,Q)
qi=[]; try, qi=ikbest(ql,Q); catch, qi=[]; end
if ~isempty(qi) && any(~isfinite(qi)), qi=[]; end
end

function write_mod_range(modfile, tgt, opts, partno)
n = numel(tgt);
if partno > 0, mname = sprintf('Main%02d', partno); else, mname = 'Main'; end
rbt = ['    CONST robtarget Target_%s_%%d:=[[%%9.6f,%%9.6f,%%9.6f],' ...
       '[%%9.9f,%%9.9f,%%9.9f,%%9.9f],[%%d,%%d,%%d,%%d],' ...
       '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n'];
jtt = ['    CONST jointtarget Target_%s_%%d:=[[%%9.6f,%%9.6f,%%9.6f,%%9.6f,%%9.6f,%%9.6f],' ...
       '[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n'];
rbt = sprintf(rbt, mname); jtt = sprintf(jtt, mname);
fid = fopen(modfile,'w');
fprintf(fid,'MODULE %sModule\n', mname);
fprintf(fid,'    TASK PERS tooldata toolc:=[TRUE,[[-50.7321,-50.0753,70.963],[0.965925826289068,0.258819045102521,0,0]],[1,[0,0,5],[1,0,0,0],0,0,0]];\n');
% target declarations
for i=1:n
    if tgt(i).sing
        fprintf(fid, jtt, i, tgt(i).qi(1),tgt(i).qi(2),tgt(i).qi(3),tgt(i).qi(4),tgt(i).qi(5),tgt(i).qi(6));
    else
        fprintf(fid, rbt, i, tgt(i).rp(1),tgt(i).rp(2),tgt(i).rp(3), ...
            tgt(i).rq(1),tgt(i).rq(2),tgt(i).rq(3),tgt(i).rq(4), ...
            tgt(i).rcf(1),tgt(i).rcf(2),tgt(i).rcf(3),tgt(i).rcf(4));
    end
end
% procedure
fprintf(fid,'    PROC %s()\n', mname);
fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,0; SetDo DO3,0;\n');
if tgt(1).sing
    fprintf(fid,'        MoveAbsJ Target_%s_1,v20,fine,toolc\\WObj:=wobj0;\n', mname);
else
    fprintf(fid,'        MoveJ Target_%s_1,v20,fine,toolc\\WObj:=wobj0;\n', mname);
end
for i=2:n
    emit_carbon(fid, mname, i, tgt(i));   % full vcs->DO choreography
end
fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,0; SetDo DO3,0;\n');
fprintf(fid,'    ENDPROC\n');
fprintf(fid,'ENDMODULE\n');
fclose(fid);
end

function emit_carbon(fid, mname, i, t)
% Carbon (toolc) DO/speed choreography per vcs code, MoveL normally /
% MoveAbsJ at a singularity. Faithful to YZ_AMV1_RZ_Modified.m's 111 block:
%   10 travel(idle) | 11 cut | 12 feed | 13/14/15 print 5/10/20 mm/s
mv='MoveL'; zone='z1';
if t.sing, mv='MoveAbsJ'; zone='z10'; end
tn = sprintf('Target_%s_%d', mname, i);
switch t.vcs
  case 10   % travel: all print signals off
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,0; SetDo DO3,0;\n');
    fprintf(fid,'        AccSet 15,15;\n');
    fprintf(fid,'        %s %s,v20,%s,toolc\\WObj:=wobj0;\n', mv, tn, zone);
  case 11   % cut fiber at path end
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,1; SetDo DO2,0; SetDo DO3,0;\n');
    fprintf(fid,'        AccSet 15,15;\n');
    fprintf(fid,'        %s %s,v10,%s,toolc\\WObj:=wobj0;\n', mv, tn, zone);
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,0; SetDo DO3,0;\n');
    fprintf(fid,'        WaitTime 1.5;\n        SetDo DO4,1;\n        WaitTime 1;\n        SetDo DO4,0;\n        WaitTime 1;\n');
  case 12   % feed fiber after travelling in (segment start)
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,0; SetDo DO3,0;\n');
    fprintf(fid,'        AccSet 15,15;\n');
    fprintf(fid,'        %s %s,v20,%s,toolc\\WObj:=wobj0;\n', mv, tn, zone);
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,1; SetDo DO3,0;\n');
    fprintf(fid,'        WaitTime 2.1;\n        SetDo DO0,0; SetDo DO1,0; SetDo DO2,0; SetDo DO3,0;\n');
  case 13   % print 5 mm/s
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,0; SetDo DO2,1; SetDo DO3,1;\n');
    fprintf(fid,'        AccSet 15,15;\n');
    fprintf(fid,'        %s %s,v5,%s,toolc\\WObj:=wobj0;\n', mv, tn, zone);
  case 15   % print 20 mm/s
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,1; SetDo DO2,0; SetDo DO3,1;\n');
    fprintf(fid,'        AccSet 15,15;\n');
    fprintf(fid,'        %s %s,v20,%s,toolc\\WObj:=wobj0;\n', mv, tn, zone);
  otherwise % 14 print 10 mm/s (default)
    fprintf(fid,'        SetDo DO0,0; SetDo DO1,1; SetDo DO2,0; SetDo DO3,0;\n');
    fprintf(fid,'        AccSet 15,15;\n');
    fprintf(fid,'        %s %s,v10,%s,toolc\\WObj:=wobj0;\n', mv, tn, zone);
end
if t.pot==1, fprintf(fid,'        WaitTime 0.2;\n'); end
end
