# 全链路运行手册：路径生成 → 打印后处理 → Excel → MOD

> 从 `all_layers_path_generation_v6.m` 产出碳纤维路径之后，一直到 ABB IRB-1200 可读的 `.mod`。
> 本手册记录已**实测跑通**的完整流程、命令、产物与注意事项。MATLAB R2022a + Robotics Toolbox。

## 0. 本次数据概况
- `Manufacturing_printing_path.mat`：**115 层 / 123 段 / 841,999 个碳纤维点 / 无树脂**（该件体素无封闭空腔→无支撑）。
- 坐标为**零件坐标系 mm**（默认恒等变换）；机器人工作系放置由 MOD 生成端施加（`base_x=0.55`、`firstl≈56.5mm`，与 `YZ_AMV1_RZ_Modified` 一致）。

## 1. 前段（Step0–Step4，打印后处理）—— 已有有效产物
由 `Run_Full_Pipeline_v8.m` 串起 `pipeline_funcs/`。本机以下中间文件均已存在且有效（之前已成功跑过）：

| 产物 | 步 |
|---|---|
| `support_paths_step0.mat` | Step0 体素支撑（本件为空） |
| `all_layers_path_carbon_resin.mat`(+`_raw`) | 合并 |
| `Modified_printing_path.mat` | Step2.1–2.3 拖拽补偿+螺旋组装 |
| `Manufacturing_printing_path.mat` | Step3.1 重采样+贴点云（**FINAL**） |
| `base_support_paths.mat` + `Base_Reduced.stl` | Step4 底座 |

> 盘点脚本：`pipeline_inventory.m`。若要从零重跑前段：`Run_Full_Pipeline_v8`（注意先打上之前评审里给的安全修复：`connected_paths` 缺字段保护、Step0-off 分支等）。

## 2. Excel 生成（exl路径生成）—— `gen_robot_excel.m`
从 FINAL mat 计算每点法向（`getPathNormals`）→ 姿态角（与 `Step3_2_Robotic_carbon_path` 同约定：`rx=atan2(-ny,-nz)`，`ry=asin(-nx)`），输出 8 列机器人格式 `[x y z(mm) rx ry(deg) pot vcs tool]`。

```matlab
% 3 层 demo（验证用，秒级）
gen_robot_excel('Manufacturing_printing_path.mat','abb\carbon_demo.xlsx', struct('layers',1:3));
% 全件（实测 ~40 分钟，842k 行 → CSV）
gen_robot_excel('Manufacturing_printing_path.mat','abb\carbon_full.csv', struct());
% 可选下采样：struct('stride',4) 把点数降到 ~1/4
```
产物：`abb/carbon_demo.xlsx`(2693 行) / `abb/carbon_full.csv`(**841,999 行 / 123 段 / 115 层**)。

## 3. MOD 生成 —— `gen_mod_from_excel.m`
读 8 列 Excel/CSV → 放置到工作系 → 逐点选 roll（绕刀轴自转，避奇异/避碰）→ **修好的解析 IK**(`ikabb/ikbest/rscfx`) → 改进的奇异判定(`needs_moveabsj`，含 J5≈0+全配置翻转) → 输出合法 RAPID（`MoveL`+robtarget / `MoveAbsJ`+jointtarget），按 `split` 拆成多个 module。

```matlab
% demo（验证）
gen_mod_from_excel('carbon_demo.xlsx','carbon_demo_opt.mod', struct('roll','opt'));
% 全件：快速基线（roll=0），拆成每 2万点一个 module
gen_mod_from_excel('carbon_full.csv','carbon_full.mod', struct('roll','zero','split',20000));
% 全件：逐点 roll 优化（本件碰撞不吃紧→ w_coll=0 走 O(n) 快路径）
gen_mod_from_excel('carbon_full.csv','carbon_full_opt.mod', struct('roll','opt','w_coll',0,'split',20000));
```
- `roll`：`'opt'`(逐点优化,默认) / `'fixed'`(用 `opts.delta`) / `'zero'`。
- `split`：每个 module 最大点数（控制器友好；842k→43 个 `*_partNN.mod`）。
- `w_coll=0`：关闭碰撞项（本件空闲喷头离件 60–80mm，远大于安全带，可安全关闭以提速）；密件请保留默认并用 `coll_stride` 调速。

## 4. 实测结果
| 阶段 | 规模 | IK 不可解 | 奇异点(MoveAbsJ) |
|---|---|---|---|
| demo 3 层 roll=0 | 2693 点 | 0 | 188 |
| demo 3 层 **逐点优化** | 2693 点 | 0 | **19** (↓90%, 12s) |
| **全件** roll=0 | 841,999 点 | **0** | 70,458 (8.4%) — 45.8s, 43 module |
| **全件 逐点优化** | 841,999 点 | **0** | **27,480 (3.3%, ↓61%)** — 28.5min, 43 module (`carbon_full_opt_part*.mod`) |

`0 IK-infeasible` 说明修好的 `ikbest`(限位对齐)+`ikabb`(数值保护) 能解算整件每一个点。

> 性能：逐点 roll 优化器第一版在 842k 点上跑了 16 小时(卡死) —— 病因是热循环里 `ikbest` 的 `error()`+`try/catch` 异常风暴 + 每点完整扫描 73 个 roll(~6000万次 IK)。已重写 `optimize_print_roll.m`：**无异常 IK 选解 + 惰性自适应(只在奇异点才扫描) + 预算位姿基**，降到 28.5 分钟，奇异点结果不变。

## 5. 注意 / 后续可加强
1. **信号编排**：本 MOD 生成器只放了最小 DO（DO2/DO3 打印开关）骨架。你真机的完整信号/裁剪/回抽/多速度逻辑在 `YZ_AMV1_RZ_Modified.m` 里 —— 可把那套 `vcs`(10–23) 分支搬进 `write_mod_range` 的 PROC 段（Excel 第 7 列已带 `vcs`）。
2. **层间抬刀/空驶**：当前段与段之间是直接 `MoveL`；真实打印需在换段处插入抬刀+空驶点（可在 `gen_robot_excel` 段首插入 travel 点实现）。
3. **变换/放置**：若要换工作系，改 `gen_mod_from_excel.m` 顶部的 `base_x/firstl/comp`（与 `YZ_AMV1` 对齐）。
4. **树脂**：本件无树脂；多材料件 `gen_robot_excel`/`gen_mod_from_excel` 需对 `resin_connected_paths`(tool 222) 同样处理并按打印时序交错。

## 6. 关键脚本清单
- `gen_robot_excel.m`（根目录）、`pipeline_inventory.m`、`probe_final.m`
- `abb/gen_mod_from_excel.m`（MOD 生成）
- `abb/optimize_print_roll.m`、`abb/dual_tool_points.m`、`abb/needs_moveabsj.m`（避奇异+避碰核心）
- `abb/RZ_Optimize_PerPoint.m`（逐点 roll 分析/对比）
- `abb/{ikabb,ikbest,rscfx,rot2quat}.m`（已修的运动学核心，详见 `abb/SINGULARITY_COLLISION_FIXES.md`）
- `abb/validate_fixes.m`（运动学自检）
