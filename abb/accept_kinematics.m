function accept_kinematics(file, stride, delta)
% Acceptance: NEW (fixed) ikabb/ikbest/rscfx/rot2quat vs ORIGINAL (backup),
% on the real path, same poses + same continuity seed. Goal: prove no
% STRUCTURAL regression -- where the original succeeds, the new must reach
% the same end-effector pose; the new must NOT fail where the original
% succeeds; differences should only be original-bug points the new fixes.
if nargin<1||isempty(file), file='path_23_1.xlsx'; end
if nargin<2||isempty(stride), stride=1; end
if nargin<3||isempty(delta), delta=-pi/2; end
addpath(pwd);
bk = fullfile(pwd,'_backup_pre_fix_20260615_211528');
assert(exist(bk,'dir')==7, 'backup dir not found');

carbon=struct('h',-50.7321e-3,'y',-50.0753e-3,'l',70.963e-3,'a', 30/180*pi);
resin =struct('h',-50.3541e-3,'y', 50.9039e-3,'l',71.4291e-3,'a',-30/180*pi);
firstl=57.36e-3+0.20e-3-1.03e-3; base_x=0.55;
YM=readmatrix(file);
if stride>1, YM=YM(1:stride:end,:); end
if size(YM,2)<8, YM(:,end+1:8)=0; YM(:,8)=111; end
n=size(YM,1);

RC=CreateIRB1200('c',carbon.l,carbon.h,carbon.a); RC.tool=transl(carbon.h,carbon.y,carbon.l+0.082)*trotx(carbon.a);
RR=CreateIRB1200('r',resin.l,resin.h,resin.a);     RR.tool=transl(resin.h,resin.y,resin.l+0.082)*trotx(resin.a);

E=cell(n,1); tl=zeros(n,1);
for i=1:n
    if YM(i,8)==111, t=carbon; ox=-0.5;oy=-1.5;oz=0.2; tl(i)=1; else, t=resin; ox=0;oy=0;oz=0; tl(i)=2; end %#ok<NASGU>
    tip=[base_x+(YM(i,1)+ox)/1000,(YM(i,2)+oy)/1000,firstl+(YM(i,3)+oz)/1000];
    E{i}=transl(tip(1),tip(2),tip(3))*troty(pi)*trotx(YM(i,4)/180*pi)*troty(YM(i,5)/180*pi)*trotz(delta);
end

% ---------- NEW pass ----------
ql=[0 0 pi/6 0 0 0]; qiN=nan(n,6); stN=zeros(n,1); qlseq=zeros(n,6); fkN=nan(n,1); rqN=nan(n,4);
for i=1:n
    if tl(i)==1,t=carbon;Rk=RC;else,t=resin;Rk=RR;end
    qlseq(i,:)=ql;
    Q=ikabb(E{i},t.l,t.h,t.y,t.a);
    try qi=ikbest(ql,Q); catch, qi=[]; end
    if isempty(qi)||any(~isfinite(qi)), stN(i)=1;
    else, qiN(i,:)=qi; ql=qi; T=fk(Rk,qi); fkN(i)=perr(T,E{i}); rqN(i,:)=rot2quat(E{i}(1:3,1:3)); end
end

% ---------- ORIG pass (same seeds qlseq) ----------
addpath(bk,'-begin'); clear ikabb ikbest rscfx rot2quat;
qiO=nan(n,6); stO=zeros(n,1); rqO=nan(n,4); rscfxErr=0;
for i=1:n
    if tl(i)==1,t=carbon;else,t=resin;end
    Q=ikabb(E{i},t.l,t.h,t.y,t.a);
    try qi=ikbest(qlseq(i,:),Q); catch, qi=[]; end
    if isempty(qi)||any(~isfinite(qi)), stO(i)=1;
    else, qiO(i,:)=qi; try rscfx(qi); catch, rscfxErr=rscfxErr+1; end
          try rqO(i,:)=rot2quat(E{i}(1:3,1:3)); catch, end
    end
end
rmpath(bk); clear ikabb ikbest rscfx rot2quat;

% ---------- compare ----------
both = stN==0 & stO==0;
dq = max(abs(qiN(both,:)-qiO(both,:)),[],2);
% quaternion compare (mod sign), only where both produced one
qb = ~any(isnan(rqN),2) & ~any(isnan(rqO),2);
dquat = min(sqrt(sum((rqN(qb,:)-rqO(qb,:)).^2,2)), sqrt(sum((rqN(qb,:)+rqO(qb,:)).^2,2)));

fprintf('\n================ KINEMATICS ACCEPTANCE (%s, %d pts) ================\n', file, n);
fprintf('NEW  infeasible: %d   (max FK pose err over feasible = %.3e m)\n', sum(stN), max(fkN(stN==0)));
fprintf('ORIG infeasible(ikbest error/NaN): %d ;  ORIG rscfx dead-band errors: %d\n', sum(stO), rscfxErr);
fprintf('both-feasible: %d ; max|qiN-qiO| = %.4f rad ; joints identical(<1e-6): %d/%d\n', ...
        sum(both), max(dq), sum(dq<1e-6), sum(both));
fprintf('rot2quat: max|rqN-rqO| (mod sign) over %d pts = %.3e\n', sum(qb), max(dquat));
fprintf('ORIG-fail & NEW-ok  (bug-fix, expected >=0): %d\n', sum(stO==1 & stN==0));
fprintf('NEW-fail & ORIG-ok  (REGRESSION, must be 0): %d\n', sum(stN==1 & stO==0));
% for the "both feasible but joints differ" points, confirm NEW still reaches the pose
diffpts = find(both & dq>1e-6);
if ~isempty(diffpts)
    fprintf('joint-differ points: %d ; NEW FK err on them max=%.3e m (must be ~0 = still correct)\n', ...
        numel(diffpts), max(fkN(diffpts)));
end
fprintf('VERDICT: %s\n', verdict(sum(stN==1 & stO==0)==0 && max(fkN(stN==0))<1e-6));
end

function T=fk(R,q), t=R.fkine(q); if isa(t,'SE3'),T=t.T;else,T=double(t);end, end
function e=perr(A,B), e=norm(A(1:3,4)-B(1:3,4)) + norm(A(1:3,1:3)-B(1:3,1:3),'fro'); end
function s=verdict(b), if b, s='PASS (no regression; all new poses correct)'; else, s='CHECK'; end, end
