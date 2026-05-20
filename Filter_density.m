
rmin=2;% PREPARE FILTER
step = ceil(rmin)-1;
iH = zeros(nele*(2*step+1)^3,1);
jH = zeros(size(iH)); vH = zeros(size(iH));
n = 0;
for el=1:nele
    [i,j,k] = ind2sub([nely,nelx,nelz],el);
    [ispan,jspan,kspan] = meshgrid(max(1,i-step):min(nely,i+step),max(1,j-step):min(nelx,j+step),max(1,k-step):min(nelz,k+step));
    dist = max(0,rmin-sqrt((ispan-i).^2 + (jspan-j).^2 + (kspan-k).^2));
    vH(n+(1:numel(dist))) = dist(:);
    iH(n+(1:numel(dist))) = el;
    jH(n+(1:numel(dist))) = sub2ind([nely nelx nelz],ispan,jspan,kspan);
    n = n + numel(dist);
end
iH(n+1:end)=[]; jH(n+1:end)=[]; vH(n+1:end)=[];
H = sparse(iH,jH,vH);
Hs = sum(H,2);             % 当前物理密度（列向量）
eps0 = 1e-9;                     % 数值容限，避免除0
xPhys(:) = (H*(xPhys(:)))./(Hs);
filename = 'test.mat';
save(filename,'nele','nelx','nely','nelz','xPhys','t_xoy','t_xoz','Iter');