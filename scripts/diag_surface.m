function diag_surface()
% DIAGNOSTIC for generate_reference_surface.m: measures (1) Y/X symmetry of the
% material, the median surface, and the fitted domain box, and (2) how much of the
% surface-fit / MFG evaluation falls on VOID (no-material) columns. No SA/GD.
    DENSITY_THRESHOLD=0.5; EDGE_MIN_COL_FRAC=0.05; MFG_EVAL_SPACING=0.5;

    fprintf('Loading voxel_refined_latest.mat ...\n');
    S=load('voxel_refined_latest.mat'); rd=S.refined_data;
    gd=rd.grid_data; vmask=rd.valid_grid_mask;
    nelx=rd.grid_size.nelx; nely=rd.grid_size.nely; nelz=rd.grid_size.nelz;
    sz=[nelx nely nelz];
    fprintf('  grid %dx%dx%d\n',nelx,nely,nelz);

    % vectorized field extraction
    X=reshape([gd.x],sz); Y=reshape([gd.y],sz); Z=reshape([gd.z],sz);
    XP=reshape([gd.xPhys],sz);
    eff = vmask & (XP>=DENSITY_THRESHOLD);

    phys_x=X(:,1,1); phys_y=squeeze(Y(1,:,1)); phys_y=phys_y(:);

    % per-column material
    col_mat=sum(XP.*eff,3);              % density sum per (i,j)
    v_cnt  =sum(eff,3);                   % voxel count per (i,j)
    populated=col_mat>0;
    col_keep=col_mat >= EDGE_MIN_COL_FRAC*median(col_mat(populated));
    col_has_mat=(v_cnt>=1)&col_keep;

    [ii,jj]=find(col_keep);
    x_min=min(phys_x(ii)); x_max=max(phys_x(ii));
    y_min=min(phys_y(jj)); y_max=max(phys_y(jj));
    Lx=x_max-x_min; Ly=y_max-y_min;

    % median surface (density-weighted mean z per column)
    z_wsum=sum(XP.*Z.*eff,3); d_sum=col_mat;
    zmed=nan(nelx,nely); zmed(col_has_mat)=z_wsum(col_has_mat)./d_sum(col_has_mat);

    %% ---- centers ----
    gi=(1:nelx)'; gj=(1:nely)';
    wcol=d_sum; wsum=sum(wcol(:));
    ic=sum(sum(wcol).*gj')/wsum;                 % weighted centroid (j idx)  [careful below]
    % proper centroids
    ic_x=sum(sum(wcol,2).*gi)/wsum;              % material centroid in i-index
    ic_y=sum(sum(wcol,1)'.*gj)/wsum;             % material centroid in j-index
    cx_phys=sum(sum(wcol,2).*phys_x)/wsum;       % material centroid in physical x
    cy_phys=sum(sum(wcol,1)'.*phys_y)/wsum;      % material centroid in physical y
    domx=(x_min+x_max)/2; domy=(y_min+y_max)/2;  % basis symmetry axis (cosine xi=0.5)

    fprintf('\n==== CENTERS ====\n');
    fprintf('  grid-index center      : i=%.2f  j=%.2f\n',(nelx+1)/2,(nely+1)/2);
    fprintf('  material centroid (idx): i=%.2f  j=%.2f\n',ic_x,ic_y);
    fprintf('  material centroid (phys): x=%.2f  y=%.2f\n',cx_phys,cy_phys);
    fprintf('  fitted-box center (phys): x=%.2f  y=%.2f   <-- cosine basis symmetry axis\n',domx,domy);
    fprintf('  box: x=[%.1f,%.1f] y=[%.1f,%.1f]\n',x_min,x_max,y_min,y_max);
    fprintf('  --> Y offset (centroid - box center) = %.2f mm  (nonzero => even-cosine basis is symmetric about the WRONG y axis)\n',cy_phys-domy);
    fprintf('  --> X offset (centroid - box center) = %.2f mm\n',cx_phys-domx);

    %% ---- symmetry of material (d_sum) about grid-index center (what the script tests) ----
    dY=flip(d_sum,2);  % reflect j -> nely+1-j
    dX=flip(d_sum,1);
    both_y=(d_sum>0)|(dY>0); both_x=(d_sum>0)|(dX>0);
    dsm=max(mean(d_sum(col_has_mat)),1e-6);
    ey=sum(abs(d_sum(both_y)-dY(both_y)))/max(nnz(both_y),1)/dsm;
    ex=sum(abs(d_sum(both_x)-dX(both_x)))/max(nnz(both_x),1)/dsm;
    fprintf('\n==== MATERIAL SYMMETRY (about GRID-INDEX center, as the script tests) ====\n');
    fprintf('  X asym err=%.3f (sym_x=%d at <0.15)\n',ex,ex<0.15);
    fprintf('  Y asym err=%.3f (sym_y=%d at <0.15)\n',ey,ey<0.15);

    %% ---- symmetry of the MEDIAN surface itself ----
    zr_y=flip(zmed,2); v=~isnan(zmed)&~isnan(zr_y);
    zr_x=flip(zmed,1); vx=~isnan(zmed)&~isnan(zr_x);
    fprintf('\n==== MEDIAN-SURFACE SYMMETRY (z reflected) ====\n');
    if any(v(:)),  fprintf('  Y: mean|z-z_reflZ|=%.3f mm, max=%.3f (over %d cols)\n',mean(abs(zmed(v)-zr_y(v))),max(abs(zmed(v)-zr_y(v))),nnz(v)); end
    if any(vx(:)), fprintf('  X: mean|z-z_reflX|=%.3f mm, max=%.3f (over %d cols)\n',mean(abs(zmed(vx)-zr_x(vx))),max(abs(zmed(vx)-zr_x(vx))),nnz(vx)); end

    %% ---- VOID coverage of the MFG eval grid (full bounding box, as in script) ----
    xe=x_min:MFG_EVAL_SPACING:x_max; ye=y_min:MFG_EVAL_SPACING:y_max;
    [Xe,Ye]=meshgrid(xe,ye);
    % nearest column index for each eval point
    [~,ie]=min(abs(Xe(:)'-phys_x),[],1); [~,je]=min(abs(Ye(:)'-phys_y),[],1);
    lin=sub2ind([nelx nely],ie(:),je(:));
    inmat=col_has_mat(lin);
    fprintf('\n==== MFG / FIT DOMAIN COVERAGE ====\n');
    fprintf('  columns with material: %d / %d grid columns (%.0f%%)\n',nnz(col_has_mat),nelx*nely,100*nnz(col_has_mat)/(nelx*nely));
    fprintf('  MFG eval grid points: %d total, %.0f%% land on VOID (no-material) columns\n',numel(inmat),100*mean(~inmat));
    fprintf('  --> the manufacturing cost is dominated by surface behaviour over EMPTY space.\n');

    %% ---- figure ----
    f=figure('Position',[60 60 1500 420],'Visible','off');
    subplot(1,3,1); imagesc(phys_x,phys_y,double(col_has_mat)'); set(gca,'YDir','normal'); axis image;
    title('col\_has\_mat (material columns)'); xlabel X; ylabel Y; colorbar;
    subplot(1,3,2); zz=zmed'; h=imagesc(phys_x,phys_y,zz); set(h,'AlphaData',~isnan(zz)); set(gca,'YDir','normal'); axis image;
    title('median surface z (NaN=void)'); xlabel X; ylabel Y; colorbar;
    subplot(1,3,3); dd=zmed-zr_y; dd=dd'; h=imagesc(phys_x,phys_y,dd); set(h,'AlphaData',~isnan(dd)); set(gca,'YDir','normal'); axis image;
    title('median - Y-reflected median'); xlabel X; ylabel Y; colorbar; caxis([-max(abs(dd(~isnan(dd))))-eps max(abs(dd(~isnan(dd))))+eps]);
    saveas(f,'diag_surface.png'); close(f);
    fprintf('\nSaved diag_surface.png\n');
end
