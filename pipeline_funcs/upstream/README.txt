# upstream/ — 来自 cfrc-am-pipeline 仓库的依赖脚本

这个文件夹是空的。请从 https://github.com/gougoujin69-png/cfrc-am-pipeline 仓库
拷贝以下文件到本目录:

  必需 (Step 0 树脂路径生成依赖):
    - generate_offset_path2.m
    - resample_path.m
    - ensureClockwise.m
    - getNormalAtPoint.m

  可选 (调试/可视化时方便):
    - Path_show_carbon.m
    - Path_show_resin.m
    - polyshape_to_cell.m

主控脚本 Run_Full_Pipeline.m 启动时会自动 addpath 这个目录,
所以放进去之后无需手动配置 MATLAB path.

注意:
  - generate_offset_path2.m 内部已经把所有 helper (offset_contour_by_normals,
    Self_intersection_outer, remove_self_intersection_loops, polyshape2rings,
    do_lines_intersect, filter_intersecting_paths 等) 内嵌在同一文件里,
    所以只需要它单文件就够了, 不用拷其他 helper.
  - resample_path.m / ensureClockwise.m / getNormalAtPoint.m 是仓库根目录的
    独立函数文件, 单独拷过来.
