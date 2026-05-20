function pointCloud_data = create_pointcloud_from_surface(X_surf, Y_surf, Z_surf)
    pointCloud_data = struct();
    pointCloud_data.X = X_surf;
    pointCloud_data.Y = Y_surf;
    pointCloud_data.Z = Z_surf;
end
