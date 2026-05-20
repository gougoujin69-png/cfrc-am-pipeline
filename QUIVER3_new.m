%quiver
%存在为了调整显示

Para_me=5;
%%%%%%外层约束显示
%%%%%%
XX=zeros(nele+1,1);
YY=zeros(nele+1,1);
ZZ=zeros(nele+1,1);
uu=zeros(nele+1,1);
vv=zeros(nele+1,1);
ww=zeros(nele+1,1);
uu(nele+1)=0.1;
vv(nele+1)=0.1;
ww(nele+1)=0.1;
t_xoy_tem=t_xoy(:).*180./pi;
t_xoz_tem=t_xoz(:).*180./pi;
xPhys2=zeros(nelz,nelx,nely);
for elz = 1:nelz
    for elx = 1:nelx
        for ely = 1:nely
            xPhys2(elz,elx,ely)=xPhys(ely,elx,elz);
        end
    end
end
xPhys_tem=xPhys(:);
xPhys_tem(xPhys_tem < 0.5) = 0.0000001; 
uu(1:nele)=cosd(t_xoz_tem).*cosd(t_xoy_tem).*xPhys_tem;
vv(1:nele)=1.*cosd(t_xoz_tem).*sind(t_xoy_tem).*xPhys_tem;%去掉-
ww(1:nele)=-1.*sind(t_xoz_tem).*xPhys_tem;%去掉-
k =1;
for k1 = 1:nelz
    for i1 = 1:nelx
        for j1 = 1:nely
            XX(k)=i1-0.5;
            YY(k)=j1-0.5;
            ZZ(k)=k1-0.5;
            k=k+1;
        end
    end
end
XX=XX.';
YY=YY.';
ZZ=ZZ.';
uu=uu.';
vv=vv.';
ww=ww.';
%%%%
xx=-1:0.5:nelx+1;
yy=-1:0.5:nely+1;
[x,y]=meshgrid(xx,yy);
a=SA;
b=SB;
c=SC;
d=SD;
e=SE;
f=SF;
g=SG;
h=SH;
X0=SX0;
Y0=SY0;
z=a.*(x-X0).^3+b.*(y-Y0).^3+c.*(x-X0).*(y-Y0)+d.*(x-X0).^2+e.*(y-Y0).^2+f.*(x-X0)+g.*(y-Y0)+h+Para_me;
display_3D2(xPhys2);
alpha(0.2);
hold on
%quiver3(XX,ZZ-nelz,-YY+nely,-uu,ww,vv,2,'.b','linewidth',1.8,'AutoScaleFactor', 1);
quiver3(XX,YY,ZZ,uu,vv,ww,2,'.w','linewidth',2,'AutoScaleFactor', 1);
%quiver3(XX,ZZ-nelz,-YY+nely,uu,-1*ww,-1*vv,2,'.b','linewidth',1.8,'AutoScaleFactor', 1);
hold on
%h1=surf(x,y,z);
%alpha(h1, 0.5);
h1=surf(x, y, z, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
colormap(jet);
alpha(h1, 0.5);



