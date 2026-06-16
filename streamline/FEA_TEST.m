FEA_path=struct();
for i=1:length(all_layers_data)
    FEA_path(i).printing_path=all_layers_data(i).connected_paths;
end
save('FEA_path.mat', 'FEA_path','-v7.3');    