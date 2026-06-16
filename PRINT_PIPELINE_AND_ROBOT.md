# 打印后处理 + 机器人 MOD 生成（路径生成之后的全链路）

本目录在原 `cfrc-am-pipeline`（应力驱动切片 + 路径规划）之后，补齐了**从路径到真实 ABB IRB-1200 打印程序**的完整链路，并修复了运动学奇异点定义/生成与碰撞规避的若干 bug。

## 链路总览
```
all_layers_path_generation_v6.m  (碳纤维路径)
   │
   ▼  Run_Full_Pipeline_v8.m  +  pipeline_funcs/   (打印后处理)
   │     Step0 支撑 → 合并 → 变换 → Step2 拖拽补偿/螺旋组装 → Step3.1 重采样
   ▼  Manufacturing_printing_path.mat
   │
   ▼  gen_robot_excel.m            (法向→姿态角, 8 列机器人路径 Excel/CSV)
   │
   ▼  abb/gen_mod_from_excel.m     (放置 + 逐点 roll 优化 + 修好的解析 IK + 奇异判定)
   ▼  *.mod  (合法 RAPID, 可导入 RobotStudio)
```

## 关键文档
- [`END_TO_END_RUNBOOK.md`](END_TO_END_RUNBOOK.md) —— 全链路运行手册：每步命令、产物、实测数据、注意事项。
- [`abb/SINGULARITY_COLLISION_FIXES.md`](abb/SINGULARITY_COLLISION_FIXES.md) —— 奇异点定义/生成的 bug 修复 + 逐点 roll 避奇异/避碰新模块。

## 主要目录
| 目录 | 内容 |
|---|---|
| `pipeline_funcs/` | 打印后处理胶水 + 参考 Step 脚本（Step0 支撑、合并、Step2/Step3 组装、底座、碰撞序列、Excel） |
| `abb/` | ABB IRB-1200 运动学（`CreateIRB1200`/`ikabb`/`ikbest`/`rscfx`/`rot2quat`，已修）+ MOD 生成器 + 逐点 roll 优化器 + 验收脚本 |
| `CollisionDetection_v5_full/` | 独立锥/点碰撞检测模块（已修姿态合成顺序、锥内符号、锥深） |
| `streamline/` | 原 FJZV16 机器人路径参考脚本 |

## 修复 / 验收要点
- **运动学**：`ikbest` 关节限位对齐 qlim（J5±130/J6±242）；`rscfx` cfx 的 J3 死区（原会 `error` 中断）；`ikabb` 数值保护；`rot2quat` 并列最大覆盖。验收：在 path_23_1 / 件 demo / 全件采样上，**新旧运动学输出逐点一致，0 回归**（`abb/accept_kinematics.m`）。
- **避奇异/避碰**：逐点绕刀轴 roll 优化（`abb/optimize_print_roll.m`），全件 842k 点把奇异点从 70,458 降到 27,480；碰撞检测用空闲喷头几何（`abb/dual_tool_points.m`）。
- **变换顺序（B4）**：先在未旋转坐标系贴 Z 再整体旋转（`Run_Full_Pipeline_v8.m` Step1a/1b），修复原顺序导致碳纤维错位 ~1.5–2.2mm（`test_B4_order.m`）。

> 大体积产物（`*.mat` GB 级、`*.csv` 84 万行、`*.mod` 200MB+）**不入库**，均可由上述脚本再生（见 RUNBOOK）。`*.mat/*.csv/*.png/*.mod/*.xlsx/*.stl` 已在 `.gitignore`。
