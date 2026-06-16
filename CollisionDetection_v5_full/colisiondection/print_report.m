function print_report(results, num_points)
%% PRINT_REPORT 输出检测报告
%
% 输入:
%   results    - 检测结果结构体
%   num_points - 总路径点数

    counts = [sum(results.overall_status == 0), ...
              sum(results.overall_status == 1), ...
              sum(results.overall_status == 2)];
    
    fprintf('\n=================== 碰撞检测报告 ===================\n');
    fprintf('总路径点数: %d\n', num_points);
    fprintf('\n状态统计:\n');
    fprintf('  安全点: %d (%.1f%%)\n', counts(1), 100*counts(1)/num_points);
    fprintf('  风险点: %d (%.1f%%)\n', counts(2), 100*counts(2)/num_points);
    fprintf('  碰撞点: %d (%.1f%%)\n', counts(3), 100*counts(3)/num_points);
    fprintf('\n距离分析:\n');
    fprintf('  ToolR最小距离: %.4f mm\n', min(results.toolr_dist)*1000);
    fprintf('  锥面最小距离: %.4f mm\n', min(results.cone_dist)*1000);
    fprintf('\n姿态角范围:\n');
    fprintf('  Rx: %.2f° ~ %.2f°\n', rad2deg(min(results.Rx_rad)), rad2deg(max(results.Rx_rad)));
    fprintf('  Ry: %.2f° ~ %.2f°\n', rad2deg(min(results.Ry_rad)), rad2deg(max(results.Ry_rad)));
    fprintf('====================================================\n');
end
