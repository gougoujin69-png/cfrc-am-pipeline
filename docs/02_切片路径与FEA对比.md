# CFRC 4-way 切片+路径 FEA 刚度对比 + 路径几何统计 — 完整流程

> **4 种组合对比**:
> - `mine_stream` &nbsp;&nbsp; 曲面切片 + stream 路径
> - `mine_offset` &nbsp;&nbsp; 曲面切片 + offset 路径
> - `planar_stream` &nbsp; 平面切片 + stream 路径
> - `planar_offset` &nbsp; 平面切片 + offset 路径
>
> **输出两类核心指标**:
> 1. **FEA 结构刚度 K** (= F / U, 衡量承力能力)
> 2. **路径几何统计** (长度 / 平滑段 / 路径-应力 / 曲面-应力对齐度)

---

## 1. 一句话总流程

```
MATLAB                 →   Abaqus              →   Abaqus              →   MATLAB
切片+路径+部署文件      →   跑 4 个 jobs        →   提取 ODB 结果        →   画 K 对比图 + 路径统计
run_full_comparison.m  →   run_with_auto_retry →   extract_fea_results  →   run_compare + 
                                                                            compute_path_statistics
```

每一步都有 skip-if-exists 逻辑，**重跑只做缺的步骤**。

---

## 2. 项目目录布局 (★关键架构)

**所有源代码 + 数据 mat 都在 script_dir。FEA 中间产物在 fea_dir（自动生成）。**

```
script_dir = E:\308\傅里叶\chair-leg\曲面切片_路径规划脚本\
│
├── # 主驱动脚本
├── run_full_comparison.m              ← MATLAB 主驱动 (10 个 stage)
├── compute_path_statistics.m          ← ★路径几何统计（不再复制到 fea_dir）
│
├── # FEA 端 helper（会被复制到 fea_dir）
├── abaqus_cfrc_compare.py             ← Abaqus 主脚本 v15 (4-way)
├── extract_fea_results.py             ← ODB → CSV 提取 v2
├── compare_fea_results.m              ← FEA 比较画图 (4-way)
├── run_compare.m                      ← MATLAB 比较 launcher
├── diagnose_loadpoint.py              ← 诊断工具 (写文件版)
│
├── # 切片/路径数据 (路径生成脚本输出)
├── all_layers_paths_only_v3.mat            ← mine_stream 路径
├── all_layers_paths_only_mine_offset.mat   ← mine_offset 路径
├── all_layers_paths_only_planar_stream.mat ← planar_stream 路径
├── all_layers_paths_only_planar_offset.mat ← planar_offset 路径
│
├── # 应力场（路径统计要用）
├── voxel_refined_latest.mat           ← ★应力场 (refined_data.grid_data 嵌套结构)
│
├── # 路径统计输出目录（compute_path_statistics 自动生成）
├── path_stats_output\
│   ├── mine_stream_path_stats.csv     ← 每条 path 一行的统计
│   ├── mine_offset_path_stats.csv
│   ├── planar_stream_path_stats.csv
│   ├── planar_offset_path_stats.csv
│   ├── all_configs_path_summary.csv   ← 4-way 汇总
│   ├── fig_path_length.png  +  fig_path_length_notext.png
│   ├── fig_angle_path_stress.png  +  _notext.png
│   ├── fig_angle_surface_stress.png  +  _notext.png
│   └── fig_smooth_run_distribution.png  +  _notext.png
│
└── ... 其他切片/路径生成函数
        (slice_refined_model.m, all_layers_path_generation.m 等)


fea_dir = C:\temp\cfrc_fea\        ← 自动 mkdir，FEA 工作目录
│
├── # 一次性手工准备
├── EmbeddedBeamModel.inp              ← host orphan-mesh (单独生成放这里)
├── template.cae                       ← 一次性手工搭, 含 Set-1/Set-2/Cload/BC/Step
│
├── # Stage 9 自动复制（5 个 helper）
├── abaqus_cfrc_compare.py
├── extract_fea_results.py
├── compare_fea_results.m
├── run_compare.m
├── diagnose_loadpoint.py
│
├── # Stage 8 自动生成（4 个 config 的 path）
├── mine_stream\beam_paths\path_0001.txt ...
├── mine_offset\beam_paths\ ...
├── planar_stream\beam_paths\ ...
├── planar_offset\beam_paths\ ...
│
├── # Abaqus 跑完后产生
├── Job_<cfg>.odb / .inp / .dat ...
│
└── results\
    ├── <cfg>_time_history.csv         ← extract.py 输出
    ├── summary.txt
    ├── fig1_FU_curves.png
    ├── fig2_K_comparison.png
    ├── fig3_secondary_metrics.png
    ├── fig4_radar.png
    └── comparison_table.md
```

**关键变化**: `compute_path_statistics.m` 不再复制到 fea_dir，直接在 script_dir 跑，输出到 `<script_dir>\path_stats_output\`。

---

## 3. 完整跑一次流程的命令

### 阶段 A — MATLAB 切片 + 路径 + 部署 helper

```matlab
>> cd E:\308\傅里叶\chair-leg\曲面切片_路径规划脚本
>> run_full_comparison                       % 跑完 10 个 stage
```

选项:
```matlab
>> run_full_comparison('force', true)        % 强制重跑全部 (会覆盖已有 mat)
>> run_full_comparison('stages', [8, 9])     % 只重新做导出和部署 (典型用例: 改 helper 后)
>> run_full_comparison('stages', 9)          % 只复制 helper 到 fea_dir
```

### 阶段 B — Abaqus 跑 4 个 job

打开 Abaqus CAE，在 Python 命令行:

```python
>>> execfile('C:/temp/cfrc_fea/abaqus_cfrc_compare.py')
# 看到 banner: SKIP-DONE+RUN-ONLY (2026-05-20-v15) [4-way, no s3_mine]

# 第一次跑要先建 template (一次性)
>>> step1_build_template()
# 然后在 CAE 里手工给 template.cae 加:
#   - *Cload (Set-1, 3, -X)        ← 加载
#   - BC (Set-2, ENCASTRE)         ← 固支
#   - Step (Static, General)
#   - Output requests
# 保存 template.cae

# 一键跑全部 4 个 + 自动重试
>>> run_with_auto_retry(['mine_stream', 'mine_offset', 'planar_stream', 'planar_offset'])

# 跑完后, 把内存里的 blacklist 持久化到源码
>>> dump_blacklist()
# 把输出粘贴回 abaqus_cfrc_compare.py 的 Config.BLACKLIST_PATH_IDX
```

### 阶段 C — 提取 ODB → CSV

Windows cmd:
```cmd
cd /d C:\temp\cfrc_fea
abaqus cae noGUI=extract_fea_results.py
```

期望看到:
```
--- Extracting: mine_stream ---
  [Stage 1] Trying History Output...
    [history] empty or zeros only -- falling back
  [Stage 2] Trying Field Output (per-frame)...
    [field] OK -- 21 frames, vars=['CF1',...,'U3']
  Data source: field
  K (linear fit)    = 19452 N/mm
  |U3|_max          = 0.067 mm
```

### 阶段 D — MATLAB 画 K 对比图

```matlab
>> cd C:\temp\cfrc_fea
>> run_compare                    % 跑 compare_fea_results
% 或:
>> run_compare('all')             % stats + compare 两个都跑
```

输出在 `C:\temp\cfrc_fea\results\`:
- `fig1_FU_curves.png` — 4 条 F-U 曲线
- `fig2_K_comparison.png` — K 柱状图 + 相对改进率
- `fig3_secondary_metrics.png` — max 位移 / max Mises / beam 数
- `fig4_radar.png` — 综合性能雷达图
- `comparison_table.md` — markdown 对比表

### 阶段 E — ★ 路径几何统计

```matlab
>> cd E:\308\傅里叶\chair-leg\曲面切片_路径规划脚本    % 重要: 在 script_dir 跑!
>> compute_path_statistics
```

可选参数:
```matlab
>> compute_path_statistics('smooth_turn_deg', 30)              % 折角阈值 (default 30°)
>> compute_path_statistics('box_y_max', 60)                    % violin y 上限
>> compute_path_statistics('box_y_min', -10)                   % violin y 下限
>> compute_path_statistics('stress_file', 'custom_stress.mat') % 显式应力场文件
>> compute_path_statistics('output_dir', 'D:\my_output')       % 自定义输出目录
```

输出 (默认 `<script_dir>\path_stats_output\`):
- 4 个 `<cfg>_path_stats.csv` (每条 path 一行)
- 1 个 `all_configs_path_summary.csv` (4-way 汇总)
- 4 张 PNG (每张同时生成 `_notext.png` 无文字版)

---

## 4. 关键技术说明

### 4.1 *Cload 语义 (必看, 容易踩坑)

Abaqus 的 `*Cload, Set-1, 3, -X` 是 **逐节点施加 -X N**, 不是总力!

- Set-1 有 65 个节点 → **总加载力 = -X × 65**
- 如果想要"总加载 = 20 N"：`*Cload, Set-1, 3, -0.308`（= -20/65）

但 **K = F / U 与载荷大小无关**, K 是结构属性。所以 4 个 config 的 K 比较始终正确。

诊断验证脚本:
```cmd
abaqus cae noGUI=diagnose_loadpoint.py
# 看 diagnose_output.txt
```
诊断输出会列出:
- 每个节点上的 CF3（应该 = *Cload 设的 X 值）
- Sum CF3（总加载）
- Sum RF3 over BC nodes（总反力，应满足 -Sum_CF3）
- RF balance ratio（应 ≈ 1.0）

### 4.2 应力场: voxel_refined_latest.mat 格式

```
voxel_refined_latest.mat
└── refined_data (struct, 1x1)
    ├── refined_data.metadata
    └── refined_data.grid_data (struct array, N=74704 元素)
        每个元素是一个 grid 点, 含字段:
        ├── x, y, z            ← voxel 中心坐标 (mm)
        ├── xPhys              ← 密度 (SIMP)
        ├── t_xoy, t_xoz       ← 应力角度 (degrees)
        ├── uu, vv, ww         ← ★ 预算好的方向向量分量
        └── is_valid           ← 掩膜 (1 = 进入 Abaqus 计算)
```

**方向公式** (与 step3_extract_stress.py 一致, README §Step 3 修正后):

```
u = cos(t_xoz) · cos(t_xoy)
v = cos(t_xoz) · sin(t_xoy)
w = +sin(t_xoz)              ← 注意是 +, 不是 -
```

如果方向场已存为 `uu/vv/ww`, `compute_path_statistics` 会**直接用它们**，跳过角度→方向的换算（避免符号歧义）。

### 4.3 4 个路径统计图的物理含义

#### A. `fig_path_length.png` (3 子图柱状图)

| 子图 | 含义 |
|------|------|
| Total Path Length | 每个 config 所有路径长度之和 (mm) |
| Sum of Smooth Runs | 连续 <30° 折角的最长段之和 |
| Smooth Ratio | sum_smooth_run / total_length, 0-100% |

→ smooth ratio 越高 = 路径连续性越好。

#### B. `fig_angle_path_stress.png` (路径-应力对齐)

- x 轴: path 切向 vs 该位置应力方向夹角 (0°-90°)
- **理想 = 0°** (path 沿应力方向)
- 4 条曲线 = 4 个 config 的角度分布直方图
- **半透明阴影** [0°, median]: 该 config 中 50% 段落在此区间
- 阴影越靠左 = 路径与应力对齐越好

#### C. `fig_angle_surface_stress.png` (曲面-应力对齐)

- x 轴: 切片面法向 vs 应力方向夹角 (0°-90°)
- **理想 = 90°** (应力完全在切片面内)
- **半透明阴影** [median, 90°]: 该 config 中 50% 点落在此区间
- 阴影越靠右 = 应力越能由该面承担

→ 这一图是评价"曲面切片优势"的核心证据：好的曲面切片应让应力大量落在面内（贴近 90°）。

#### D. `fig_smooth_run_distribution.png` (截断 violin)

- 每个 config 一个 violin 形状（KDE 密度估计）
- 内部白色细盒 = IQR (25-75 分位)
- 粗黑横线 = median
- **y 轴截断** (default: max(Q3) × 1.6)
- 超出截断的 max 在顶部注 `max=XX (off-chart)`
- 形状: 底部胖 = 短段集中; 顶部细尖 = 少数长段

### 4.4 无文字版本 (_notext.png)

每张图同时生成两个版本:
- `fig_xxx.png` — 完整版（标题/标签/legend/注释 全有）
- `fig_xxx_notext.png` — 无文字版（保留所有视觉元素，去掉所有 text/legend/标题/tick 标签）

无文字版用于 InDesign / PowerPoint / Latex 中自己加注释。

---

## 5. 自动诊断武器库 (Abaqus 端)

| 命令 | 用途 |
|------|------|
| `list_status()` | 看每个 config 的当前状态 |
| `inp_health_check('cfg')` | **不跑 job** 就扫 inp, 报段长/共线/重合节点 |
| `auto_diagnose('cfg')` | 跑过且失败时, 从 dat 反查问题 path |
| `run_with_auto_retry(['cfg'])` | 全自动: 跑→失败→diagnose→加 blacklist→重跑 |
| `dump_blacklist()` | 打印 in-memory blacklist (粘贴回源码用) |
| `clean_config('cfg')` | 清掉 lck/odb/sta 残留 |
| `step1_build_template()` | 一次性生成 template.cae 骨架 |

---

## 6. 常见 Abaqus 错误速查

| 报错 | 根因 | 修法 |
|------|------|------|
| `Normal cannot be computed in N elements` | path 内部几何 noise | `run_with_auto_retry` 自动加 blacklist |
| `zero length` (但 inp 里没零长) | Abaqus 内部容差问题 | 同上 |
| `direction vectors coincide` | n1 与 tangent 共线 | `Config.BEAM_N1_PARALLEL_COSINE` 调严 |
| `aspect ratio may exceed 1000` | resample 产生长 segment | v14+ 已修 (按弧长降采样) |
| `node does not lie in any host element` | 路径出工件 | `auto_diagnose` 捕获 + blacklist |
| `lock file detected` | 上次 job 没退干净 | `clean_config('cfg')` 然后重跑 |
| `duplicate AllBeamElements` 警告 | 已知, 无害 | 忽略 |

---

## 7. Stage 出错时怎么办

| 卡在哪一步 | 该看哪里 |
|-----------|---------|
| Stage 1-2 (切片) | `slice_refined_model` / `slice_planar_z` 的输出 |
| Stage 3-6 (路径) | `all_layers_path_generation` / `offset_only_path_generation` |
| Stage 8 (导出) | `export_paths_to_fea` helper, 确认 .mat 里有 `paths_only` 字段 |
| Stage 9 (复制) | 确认 5 个 helper 文件都在 script_dir 下 |
| Abaqus job 失败 | 跑 `inp_health_check('cfg')`, 然后 `run_with_auto_retry` |
| 提取 CSV 是空的 | 看阶段 C 的输出, 确认 "Data source" 不是 NONE |
| 比较图全是 0 | 确认 `results\*_time_history.csv` 不是空 (head 一下) |
| **compute_path_statistics 找不到 stress** | 用 `'stress_file'` 参数显式指定; 或确认 voxel_refined_latest.mat 在 script_dir |
| **compute_path_statistics 找不到 path .mat** | 把它们放到 script_dir; 或 addpath 进 MATLAB path |
| **violin 形状奇怪** | 调 `box_y_max` 参数; 看 N=XXX 是否合理 |

---

## 8. 关键文件版本号

| 文件 | 当前版本 | 关键变更 |
|------|---------|---------|
| `abaqus_cfrc_compare.py` | v15 (2026-05-20) | 4-way (drop s3_mine), 弧长 resample, n1 一致, BEAMPATH=原始 idx |
| `extract_fea_results.py` | v2 (2026-05-20) | 4-way, history → field fallback |
| `compare_fea_results.m` | 2026-05-20 | 4-way 版 (去 s3_mine) |
| `run_compare.m` | 2026-05-20 | addpath + cd, 自动找原比较脚本 |
| `run_full_comparison.m` | 2026-05-20 | 4-way, 10 stage, 自动复制 5 个 helper |
| `compute_path_statistics.m` | 2026-05-20 | ★ 长度+应力对齐+50%阴影+truncated violin+notext |
| `diagnose_loadpoint.py` | 2026-05-20 | 日志同时写文件 (diagnose_output.txt) |

---

## 9. 历史 bug 修复一览

| 版本 | 修了什么 |
|------|---------|
| v11 | Config.BEAM_N1_RAW 统一 |
| v12 | run_only / list_status / skip_done 加入 |
| v13 | resample_path 改按弧长降采样, n1 一致 (failed: blacklist 编号偏移 bug) |
| v14 | **BEAMPATH_NNNN 用原始 path index** (不再受 blacklist 偏移) |
| v15 | **drop s3_mine, 全套 4-way** |
| compute_path_stats | 关键: w 符号修正为 `+sind(t_xoz)`; 支持 refined_data 嵌套 struct array 格式 |
| compute_path_stats | 关键: 应力查询用 mask 跳过网格外位置 |
| compute_path_stats | 关键: 出图加 50% 阴影, violin 截断, 无文字版 |

---

## 10. K 实验结果 (你的当前数据)

| Config | K (N/mm) | vs Planar+Offset baseline |
|--------|----------|---------------------------|
| **MINE+Stream** | **19452** | **+69 %** |
| MINE+Offset | 17922 | +56 % |
| Planar+Stream | 13019 | +13 % |
| Planar+Offset | 11487 | (baseline) |

→ **MINE+Stream 是 Planar+Offset 的 1.69×**, 这是论文核心结论。

注意: 这里 F_total = 1300 N = 20 N × 65 nodes（*Cload 是逐节点的）。K = F/U 与载荷大小无关，所以 K 数值不会随你改 *Cload 而变。

---

## 11. 路径统计单位换算速查

| 输出字段 | 单位 | 含义 |
|---------|------|------|
| `total_length_mm` | mm | 每条 path 的总长 |
| `smooth_run_mm` | mm | 连续 <30° 折角的最长段长度 |
| `n_points` | 个 | 每条 path 的节点数 |
| `mean_seg_len_mm` | mm | 平均 segment 长度 |
| `mean_angle_stress_deg` | ° | 每条 path 上 segment 与应力方向夹角的均值, 0-90 |
| `median_angle_stress_deg` | ° | 同上, 中位数 |
| `in_plane_ratio` (summary) | 0-1 | sin(surface-stress 夹角), 1.0 = 应力完全在面内 |
| `smooth_ratio` (summary) | 0-1 | sum(smooth_run) / sum(total_length) |

