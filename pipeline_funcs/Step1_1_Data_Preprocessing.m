%数据前处理：DATA PREPROCESSING
%数据扩大或者缩小因子
mm=1;
%建立新的all_layers_path_carbon_resin2.mat
load('all_layers_path_carbon_resin.mat');
all_layers_data2=all_layers_data;
num_layers = length(all_layers_data);
fprintf('Loaded: %d layers\n\n', num_layers);
for layer = 1:num_layers
    paths_2d = all_layers_data(layer).paths_2d;
    paths_2d2=paths_2d;
    paths_3d = all_layers_data(layer).paths_3d;
    paths_3d2=paths_3d;
    paths_2dr = all_layers_data(layer).paths_2dr;  
    paths_2dr2=paths_2dr;
    paths_3dr = all_layers_data(layer).paths_3dr;
    paths_3dr2=paths_3dr;
    pointCloud_data=all_layers_data(layer).pointCloud_data;
    pointCloud_data2=pointCloud_data;
    for i=1:length(paths_2d)
        paths_2d2{i}=paths_2d{i}*mm;
    end
    for i=1:length(paths_3d)
        paths_3d2{i}=paths_3d{i}*mm;
    end
    for i=1:length(paths_2dr)
        paths_2dr2{i}=paths_2dr{i}*mm;
    end
    for i=1:length(paths_3dr)
        paths_3dr2{i}=paths_3dr{i}*mm;
    end
    pointCloud_data2.X=pointCloud_data.X*mm;
    pointCloud_data2.Y=pointCloud_data.Y*mm;
    pointCloud_data2.Z=pointCloud_data.Z*mm;    
    all_layers_data2(layer).paths_2d=paths_2d2;
    all_layers_data2(layer).paths_3d=paths_3d2;
    all_layers_data2(layer).paths_2dr=paths_2dr2;  
    all_layers_data2(layer).paths_3dr=paths_3dr2;
    all_layers_data2(layer).pointCloud_data=pointCloud_data2;   
end
all_layers_data=all_layers_data2;
save('all_layers_path_carbon_resin2.mat', 'all_layers_data','-v7.3');



