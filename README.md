# CFRC-AM Pipeline · 连续纤维复合材料增材制造管线

> 面向高性能连续纤维复合材料（CFRC）的智能增材制造工艺优化与路径规划研究的完整代码库。  
> 涵盖**几何前处理 → 体素化 → Abaqus 应力分析 → 应力驱动切片 → 路径规划 → FEA 刚度对比 → 路径几何统计**全链路。
>
> Author: 冯镜泽（Jazz Feng）· 同济大学航空航天与力学学院

---

## 1. 这是什么

一套从"任意几何 + 工况"自动产出"连续纤维路径 + 性能验证报告"的完整管线，把以下五件事串成一条流水线：

| 阶段 | 解决的问题 | 关键产物 |
|---|---|---|
| **A. 前处理** | 把 STL 或 SIMP 拓扑优化结果 → Abaqus 可直接计算的等大六面体网格 | `voxel_grid.inp` + `voxel_grid.npz` |
| **B. 应力分析** | 在 Abaqus 里跑线弹性，得到每个体素的主应力方向 | `topo_stress_result.mat` |
| **C. 切片** | 应力驱动的曲面切片（vs 传统平面切片基线） | `refined_data.grid_data`（74 704 个 grid 点） |
| **D. 路径规划** | 在每一层切片面内生成 stream / offset 两种填充路径 | `all_layers_paths_only_*.mat` |
| **E. 验证** | 把 4 种组合（曲面/平面 × stream/offset）丢进 Abaqus 嵌入梁模型，对比结构刚度 K | F-U 曲线 / K 柱状图 / 路径统计 |

**核心结论**（你当前的实验数据）：MINE+Stream 组合的结构刚度 K = 19 452 N/mm，比 Planar+Offset 基线 **+69 %**。

---

## 2. 一图看懂总管线

```
                ┌──────────────┐         ┌──────────────────┐
                │   STL 几何    │   或    │  SIMP 拓扑优化    │
                └──────┬───────┘         │  .mat (xPhys)     │
                       │                 └────────┬─────────┘
                       └─────────┬────────────────┘
                                 │
                       ┌─────────▼─────────┐
                       │  voxelize.py      │   阶段 A · 前处理
                       │  (统一前端)        │
                       └─────────┬─────────┘
                                 │
                       voxel_grid.inp + .npz
                                 │
                       ┌─────────▼─────────┐
                       │  Abaqus / CAE     │   阶段 B · 应力分析
                       │  (手工加 BC/Load)  │
                       └─────────┬─────────┘
                                 │
                              job.odb
                                 │
                       ┌─────────▼─────────┐
                       │ abaqus_odb_to_mat │
                       └─────────┬─────────┘
                                 │
                       topo_stress_result.mat
                                 │
                       ┌─────────▼──────────────────────┐
                       │ voxel_refinement_from_test.m   │   阶段 C · 应力场细化
                       └─────────┬──────────────────────┘
                                 │
                       voxel_refined_latest.mat
                                 │
                ┌────────────────┼────────────────┐
                │                                 │
       ┌────────▼─────────┐              ┌────────▼─────────┐
       │ generate_         │              │ generate_planar_ │   阶段 D · 切片
       │ reference_surface │              │ slicing          │   （曲面 vs 平面）
       │ + slice_refined_  │              │                  │
       │ model_v6          │              │                  │
       └────────┬─────────┘              └────────┬─────────┘
                │                                 │
       refined_data.mat                  planar_slicing.mat
                │                                 │
       ┌────────▼──────────┐             ┌────────▼──────────┐
       │ all_layers_path_  │             │ path_generation_  │   阶段 D · 路径
       │ generation_v6     │             │ offset_only       │   （stream / offset）
       │ (stream)          │             │                   │
       └────────┬──────────┘             └────────┬──────────┘
                │                                 │
                └────────────┬────────────────────┘
                             │
              all_layers_paths_only_*.mat   (4 个 config)
                             │
                ┌────────────▼────────────────┐
                │ run_full_comparison.m       │   阶段 E · 4-way FEA 对比
                │   → abaqus_cfrc_compare.py  │
                │   → extract_fea_results.py  │
                │   → compare_fea_results.m   │
                │   → compute_path_statistics │
                └────────────┬────────────────┘
                             │
                F-U 曲线 / K 对比 / 路径统计图
```

---

## 3. 快速上手（30 分钟跑通最小用例）

### 3.1 环境

| 环节 | 软件 | 说明 |
|---|---|---|
| 体素化 | **Python 3** + `trimesh` `scipy` `numpy` `h5py`(可选) `manifold3d` | 仅用于 `voxelize.py` |
| 应力分析 | **Abaqus 2021**（Python 2.7 kernel） | 必须！本仓库所有 Abaqus 脚本都遵循 PY2 语法约束 |
| 切片 / 路径 / 后处理 | **MATLAB** R2021+ | 全部 `.m` 脚本 |

### 3.2 最小流程

```bash
# 1. 体素化 STL（或 SIMP 拓扑结果）
python3 voxelize.py from-stl part.stl -s 1.0

# 2. 在 Abaqus/CAE 里 Import voxel_grid.inp，加 BC/Load/Step，跑 job → job.odb

# 3. 回收应力到 .mat
abaqus python abaqus_odb_to_mat.py --odb job.odb --npz voxel_grid.npz \
    --output topo_stress_result.mat

# 4. MATLAB 里跑切片+路径+FEA 对比
matlab -batch "voxel_refinement_from_test; generate_reference_surface; \
               slice_refined_model_v6; all_layers_path_generation_v6; \
               run_full_comparison"
```

详细工作流见 `docs/01_前处理与体素化.md` 与 `docs/02_切片路径与FEA对比.md`。

---

## 4. 全部脚本一览（按功能分组）

### 4.1 阶段 A — 几何前处理 / 体素化

| 文件 | 说明 |
|---|---|
| `voxelize.py` | 统一前端：STL → 占用网格 / SIMP `.mat` → 阈值化网格，生成 `.inp` + `.npz` |
| `voxelize.m` | MATLAB 包装器（背后调 Python 3） |
| `abaqus_odb_to_mat.py` | Abaqus 应力分析完成后：ODB → `topo_stress_result.mat`（PY2.7） |

详见 [`docs/01_前处理与体素化.md`](docs/01_前处理与体素化.md)。

### 4.2 阶段 C — 应力场细化与可视化

| 文件 | 说明 |
|---|---|
| `voxel_refinement_from_test.m` | 把 `topo_stress_result.mat` 细化为 `refined_data.grid_data`，预算好方向向量 `uu/vv/ww` |
| `visualize_refinement_from_test_data.m` | 细化后应力场的三维可视化 |
| `visualize_topo_stress_field.m` | 原始拓扑应力场可视化 |
| `visualize_slicing_results.m` | 切片结果可视化 |
| `QUIVER3_new.m` | 改进版 quiver3，用于矢量场可视化 |
| `create_pointcloud_from_surface.m` | 曲面采样为点云 |

### 4.3 阶段 D₁ — 切片（曲面 / 平面）

**参考面 + 曲面切片**

| 文件 | 说明 |
|---|---|
| `generate_reference_surface.m` | Fourier/DCT 基函数 + SA+Adam 优化，自适应代价归一化，曲率三层约束 |
| `plot_reference_surface_diagram.m` | 参考面诊断图 |
| `slice_refined_model_v6.m` | 主力曲面切片器（v6，解析梯度 Z-only offset，无折叠） |
| `slice_refined_model_complete.m` | 完整切片版本（包含全部诊断） |
| `fix_surface_overlap.m` | 修复层间面互相穿插的情况 |
| `trim_offset_surface.m` | 偏移面裁剪到工件内部 |
| `extract_layer_2d_projection.m` | 把曲面层投影到 2D 用于路径规划 |
| `view_surface_layers.m` | 多层曲面叠放可视化 |

**平面切片基线**

| 文件 | 说明 |
|---|---|
| `generate_planar_slicing.m` | 沿 Z 轴等高切片（对比组用） |

### 4.4 阶段 D₂ — 路径规划

**Stream 路径（应力主方向流线）**

| 文件 | 说明 |
|---|---|
| `all_layers_path_generation_v6.m` | 多层 stream 路径主流程（v6） |
| `plot_streamline_partitioning_v6.m` | 流线分区与可视化 |
| `extendStreamlinesToContour.m` | 把流线两端外推到轮廓 |
| `filterStreamlinesInsideContours.m` | 过滤跑出轮廓的流线 |

**Offset 路径（轮廓等距偏移）**

| 文件 | 说明 |
|---|---|
| `path_generation_offset_only.m` | 多层 offset 路径主流程 |
| `generate_offset_path2.m` | 单层 offset 算法 |
| `plot_contour_offset_v2.m` | 轮廓偏移可视化 |
| `plot_offset_curved_layer_v3.m` | 曲面层 offset 可视化 |

**通用路径处理工具**

| 文件 | 说明 |
|---|---|
| `preprocess_contour.m` | 轮廓预处理（去重、合并） |
| `polyshape_to_cell.m` | `polyshape` ↔ cell 转换 |
| `split_region_points_improved.m` | 多连通区域分割 |
| `find_continuous_segments.m` | 路径连续段识别 |
| `resample_path.m` | 按弧长重采样路径（Abaqus 单元长度均匀化） |
| `ensureClockwise.m` | 强制顺时针朝向 |
| `getNormalAtPoint.m` | 任意点处的曲面法向 |
| `filter_orientation.m` / `filter_orientation_simple.m` | 方向场滤波 |
| `Filter_density.m` | 密度场滤波 |
| `calculate_straightness.m` | 路径直度量化 |
| `recover_v6_clobber.m` | 恢复被 v6 误覆盖的中间变量 |
| `single_layer_test.m` | 单层测试沙盒 |
| `debug_layer_coverage.m` | 层覆盖率诊断 |

### 4.5 阶段 E — 4-way FEA 刚度对比

| 文件 | 说明 |
|---|---|
| **`run_full_comparison.m`** | **MATLAB 主驱动**（10 个 stage，skip-if-exists） |
| `export_paths_to_fea.m` | 把 MATLAB 路径导出为 Abaqus 可吃的格式 |
| `fix_paths_for_abaqus.m` | 修复路径中的零长 segment / 共线点 |
| **`abaqus_cfrc_compare.py`** | **Abaqus 4-way job runner**（v15，PY2.7） |
| `extract_fea_results.py` | ODB → CSV（v2，history → field 自动 fallback） |
| `compare_fea_results.m` | 4-way K / F-U / 雷达图 |
| `run_compare.m` | `compare_fea_results` 的 launcher |
| `diagnose_loadpoint.py` | 加载点诊断（写文件版本） |
| **`compute_path_statistics.m`** | **路径几何统计**（长度 / 平滑段 / 应力对齐度，含 50% 阴影与截断 violin） |

详见 [`docs/02_切片路径与FEA对比.md`](docs/02_切片路径与FEA对比.md)。

### 4.6 通用 / 可视化辅助

| 文件 | 说明 |
|---|---|
| `display_3D2.m` | 通用 3D 显示 |
| `display_fiber_tubes_v3.m` | 纤维路径渲染成管状 3D |
| `display_fiber_tubes_interactive.m` | **交互式 4-way 路径对比 viewer**（共享体素背景与 BC 标记，含层范围/管径/Z 间距/视角滑条，"SAVE all 4" 用同一相机视角同时导出 4 张图） |
| `display_mesh_with_paths.m` | 网格+路径叠加渲染 |
| `fiber_show.m` | 纤维方向场可视化 |
| `Path_show_carbon.m` / `Path_show_resin.m` | 碳纤维 / 树脂路径可视化 |
| `Resin_test3.m` | 树脂相关测试 |
| `show_overview.m` | 项目总览图 |
| `plotTopologyWithMedialAxis.m` / `…2.m` | 拓扑+中轴可视化 |
| `bc_selector_gui.m` | BC 节点集图形化选取（GUI 工具） |
| `verify_outputs.m` | 各阶段输出完备性自检 |

---

## 5. 关键约定（所有脚本共用）

### 5.1 坐标与单位

| 量 | 单位 | 备注 |
|---|---|---|
| 长度 | **mm** | 所有几何 / 体素 / 路径 |
| 力 | **N** | Abaqus 载荷 |
| 应力 | **MPa** | Abaqus 输出 |
| 角度 | **°（degrees）** | `t_xoy`, `t_xoz` 都是度，不是弧度 |
| 体素索引 | `(j, i, k)` = `[nely, nelx, nelz]` | **MATLAB 的 row-major 习惯**，跟 `xPhys` 一致 |
| 切片方向 | **+Z** | STL 必须 Z-up；`voxelize.py` 提供 9 种轴旋转预设 |

### 5.2 方向向量公式（重要）

```
u = cos(t_xoz) · cos(t_xoy)
v = cos(t_xoz) · sin(t_xoy)
w = +sin(t_xoz)                ← 注意是 +，不是 −
```

如果 `.mat` 里已经存了 `uu/vv/ww`，下游脚本**直接用**，跳过角度换算（避免符号歧义）。

### 5.3 Abaqus 脚本规约

所有 `.py` 在 Abaqus 端执行的脚本（`abaqus_cfrc_compare.py`、`abaqus_odb_to_mat.py`、`extract_fea_results.py`、`diagnose_loadpoint.py`）必须：

1. 第一行声明 `# -*- coding: utf-8 -*-`
2. **源码内不含任何非 ASCII 字符**（包括中文注释、中文字符串）
3. 使用 Python 2.7 兼容语法（`print` 语句、`from __future__ import …` 可加）

### 5.4 单元类型

- Abaqus 母体网格：`C3D8R`（缩减积分六面体，避免 volumetric locking）
- 嵌入纤维梁：`B31`（一阶 Timoshenko 梁），椭圆截面 0.6 × 0.15 mm，E_ratio = 92×
- 嵌入容差：`absoluteTolerance = 2.5 mm`，方向 n1 = (0.309, 0.619, 0.722)

---

## 6. 已知 K 实验结果

| Config | K (N/mm) | vs Planar+Offset 基线 |
|---|---|---|
| **MINE + Stream** | **19 452** | **+69 %** |
| MINE + Offset | 17 922 | +56 % |
| Planar + Stream | 13 019 | +13 % |
| Planar + Offset | 11 487 | baseline |

注：F_total = 1 300 N = 20 N × 65 nodes（`*Cload` 是**逐节点**施加，不是总力）。K = F/U 与载荷大小无关，4 个 config 的相对比较始终正确。

---

## 7. 历史 bug 修复速查

| 版本 | 修了什么 |
|---|---|
| slice_refined_model v3c→v3d | 从 KNN+PCA 法向估计 → 解析梯度 Z-only offset（消除折叠） |
| voxel_refinement | 修了 0-based vs 1-based 体素索引 bug（拓扑保存度数，细化误乘 180/π） |
| abaqus_cfrc_compare v11 | `Config.BEAM_N1_RAW` 统一 |
| v13→v14 | `BEAMPATH_NNNN` 用原始 path index（不再受 blacklist 偏移） |
| v15 | drop `s3_mine`，全套 4-way |
| compute_path_statistics | w 符号修正为 `+sind(t_xoz)`；支持嵌套 `refined_data.grid_data` 结构数组 |
| compute_path_statistics | 应力查询用 mask 跳过网格外位置 |
| abaqus_odb_to_mat | `t_xoz` 用 `+asin(dz)`（匹配 `step3_extract_stress.py` 历史约定） |
| voxel_refinement + extract_layer_2d_projection | 启用 `REFINE_FACTOR=3`：fine grid 去掉 `-0.5` 偏移以对齐原始采样 `[0.5, nelx-0.5]`；2D 投影改用 `grid_index` 整数下标（物理坐标 `.x/.y` 在 refine≥2 时非整数无法作数组下标） |

---

## 8. License / 版权

**Private repository**——内部研究代码，未公开发布。  
如需引用或合作请联系 Jazz Feng（同济大学航空航天与力学学院，李岩教授课题组）。

---

## 9. 文档索引

- 本文（顶层 README）—— 全景与文件索引
- [`docs/01_前处理与体素化.md`](docs/01_前处理与体素化.md) —— 阶段 A/B 深入：`voxelize.py` CLI、9 种轴旋转、STL vs Topo 模式语义、`abaqus_odb_to_mat.py` 输出字段定义、常见坑 Q1–Q8
- [`docs/02_切片路径与FEA对比.md`](docs/02_切片路径与FEA对比.md) —— 阶段 C/D/E 深入：`run_full_comparison` 10 stage、Abaqus 自动重试 / blacklist 机制、`*Cload` 逐节点语义、`voxel_refined_latest.mat` 数据结构、4 张路径统计图的物理含义
