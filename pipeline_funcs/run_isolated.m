function run_isolated(script_name)
%% RUN_ISOLATED  在独立函数 workspace 中跑用户脚本, 避免 clear 污染主控
%
% 用户的 Step2_1/2_2/2_3 和 Step3_2 开头都有 `clear; clc; close all;`,
% 直接 run 会清掉 Run_Full_Pipeline 的 RUN/resin_data 等变量.
% 这个 wrapper 把执行隔离在函数 workspace 内, clear 只影响这个函数的局部变量.
%
% 用法:
%   run_isolated('Step2_1_add_drag_compensation')

run(script_name);
end
