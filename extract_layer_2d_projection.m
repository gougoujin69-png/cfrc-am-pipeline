function [xold, t,nelx,nely,nelz] = extract_layer_2d_projection(activated_grids,grid_data)
[nelx,nely,nelz]=size(grid_data);
xold=zeros(nely,nelx);
t=zeros(nely,nelx);
num_grids = length(activated_grids);
for i = 1:num_grids
    g = activated_grids(i);
    %x_coords(i) = (g.x+0.25).*2;
    %y_coords(i) = (g.y+0.25).*2;
    x_coords(i) = g.x;
    y_coords(i) = g.y;
    densities(i) = 1;
    t_xoy_vals(i) = g.t_xoy;
    xold(y_coords(i),x_coords(i))=densities(i);
    t(y_coords(i),x_coords(i))=t_xoy_vals(i);    
end