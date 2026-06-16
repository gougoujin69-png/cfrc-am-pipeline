function run_step3_1_with_save()
%% RUN_STEP3_1_WITH_SAVE  跑 Step3_1 + 补 save (你的脚本没自带 save)
%
% Step3_1_Robotic_Preprocess.m 只 load + 处理, 没有最后的 save.
% Step3_2 又需要从 Manufacturing_printing_path.mat 读, 所以这里补 save.

run('Step3_1_Robotic_Preprocess');
% 此时 all_layers_data 在本函数 workspace 中 (Step3_1 没有 clear)
save('Manufacturing_printing_path.mat', 'all_layers_data', '-v7.3');
fprintf('  [Wrap] 已 save -> Manufacturing_printing_path.mat\n');
end
