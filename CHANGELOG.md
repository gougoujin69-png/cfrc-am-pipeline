# Changelog

本项目的修改日志。最新的改动在最上面。

---

## 2026-06-04 — 纯偏置改用 v6 偏置机制 (offset_only 模式), 修复覆盖不全

### 背景 / 问题
偏置对照组 (mine_offset / planar_offset) 用的是早期独立实现 `path_generation_offset_only.m`
("造新轮子")。它整块区域只迭代 `max_iterations=30` 环 (≈30×线宽 mm), 对宽区域**覆盖不全**
(边界向内偏置到 ~12mm 就停, 中心留空) —— 重大错误。

而 `all_layers_path_generation_v6.m` 的偏置之所以覆盖好, 是因为流线把区域切成窄条, 每条
几环就填满。

### 方案
不再另起炉灶: 偏置对照组改走**同一个** `all_layers_path_generation_v6`, 新增 `offset_only`
模式 —— 强制主流线数量为 0, 整块有效区域纯偏置填充, 复用完全相同的区域+偏置机制。

### 改动
- **all_layers_path_generation_v6.m**: 新增第 4 参数 `opts.offset_only`。
  - 步骤 8: `offset_only` 时强制 `streamlines_raw = {}` (主流线=0)。下游步骤 11 无流线则
    不切区域, 整块走步骤 13 的 `generate_offset_path2` 纯偏置。
  - 不切区域时偏置环要从边界一直填到中轴才算覆盖完整, 故 `offset_only` 模式把
    `max_iterations` 提到 200 (`generate_offset_path2` 区域收缩到 `min_path_length` 后自动
    break, 高上限只是天花板, 薄区域仍早停, 不空跑)。
  - 输出 `paths_only.mode` ('offset_only'/'stream') + `source_slice_file`。
- **run_full_comparison.m**: `generate_offset_paths` 改为调用
  `all_layers_path_generation_v6(slice, out, full, struct('offset_only', true))`,
  不再调用 `path_generation_offset_only`。
- **path_generation_offset_only.m**: 标记为已弃用 (保留作历史参考, 管线不再调用)。

### 验证
- 曲面切片最密两层 (#63/#66) 跑 offset_only: stream=0, 整块区域偏置环从边界填到中轴,
  栅格化"沉积条带"(路径按线宽外扩) 占有效区域面积比 = **99.8%** (旧版覆盖不全)。出图确认
  X 形结构被同心偏置环填满。

### 注意 (速度/日志)
- 不切区域时 generate_offset_path2 在复杂整块结构上较重: ~105 s/层 (max_iter=200, 填到
  区域收缩才停), 且会打出大量"已捕获"的 polyshape 报错栈 (复杂几何深迭代所致, 不影响结果)。
  这是"完全覆盖"的一次性代价。`opts.offset_max_iter` 可调上限。

---

## 2026-06-04 — 修复 FEA 刚度提取 (K 混乱的根因)

### 背景 / 问题
`run_compare` / `compare_fea_results.m` 算出的刚度 K 极其混乱:
mine_stream K=3.5e7, mine_offset=3.5e5, planar_offset=4.9e4 —— 数量级离谱且与
beam 数量强相关 (beam 越多 K 越大)。根因在 `extract_fea_results.py`:

1. **LoadPoint 集缺失 -> 退化为"全节点平均"**: 模型里没有名为 `LoadPoint` 的
   装配节点集 (summary.txt 里 `num_load_nodes=0`)。提取脚本找不到它时, 旧逻辑
   `collect_from_field_output(step, [])` 用 **全部节点** 求位移平均, 使 u_loadpt≈0
   (大部分结构几乎不动), K=F/u 被放大成与节点数 (≈beam 数) 成反比的假值。
   这就是 4 个配置 K 互相"混乱"的真正原因。
2. **`sum` 被 Abaqus 覆盖**: `from abaqusConstants import *` 会覆盖内建 `sum`,
   在 `abaqus python` 下 `sum([...])` 抛 "illegal argument type for built-in
   operation" (在 `abaqus cae noGUI` 下恰好没覆盖, 所以旧版能跑出 K —— 那 K 仍是
   假值)。

### 改动 (extract_fea_results.py)
- **CF 自动定位载荷点**: 当 `LoadPoint` 集缺失时, 不再全节点平均, 而是从 CF
  (集中力) 场自动识别载荷补丁 = 所有施加了非零 CF 的节点; 每帧取该补丁的
  **U 平均**(载荷点位移) + **CF 求和**(总外载), K=总F/平均u 是真实的载荷点刚度,
  与 beam 数无关。(`collect_loadpoint_from_cf`, 替换原全节点回退)
- **恢复被覆盖的内建函数**: 顶部 `from __builtin__ import sum, min, max, abs,
  float, sorted, len, range, zip`, 保证脚本在 `abaqus python` 和
  `abaqus cae noGUI` 两种启动方式下都稳。

### 验证
- 单配置 (planar_offset) 实测: 旧 K=48622 (全节点平均假值) -> 新 K=6959 N/mm
  (= 180N / 0.0259mm, 真实载荷点刚度)。load patch 正确识别为 9 节点, 总 CF=180N。

### 仍待解决: planar_stream 作业失败 (与提取无关)
`Job_planar_stream.odb` 只有 9MB、0 帧, 无 `.sta`/`.msg` -> Abaqus 作业在 pre.exe
(输入处理) 阶段崩溃, 未产出任何增量。planar_stream 是退化几何最多的配置
(abaqus_cfrc_compare.py 里有 40+ 条 blacklist)。这是**模型几何**问题, 不是提取问题:
需让该 Abaqus 作业本身跑通 (重跑; 若反复崩溃则需扩 blacklist 或重新生成更干净的
planar_stream 路径)。提取脚本对 0 帧 ODB 已能优雅跳过 (status OK, K=0)。

---

## 2026-06-04 — Host 尺度一致性守卫 (ELEM_SIZE 拉大不再撞 Abaqus)

### 背景 / 问题
`voxel_refinement_from_test.m` 里把 `ELEM_SIZE` 拉大 (或改 `REFINE_FACTOR`) 后, 切片/路径
的物理坐标按 `ELEM_SIZE` 放大。但 `export_paths_to_fea.m` 的 host 网格
(`host/mesh_params.txt` + `valid_elements.txt`) 是**只写一次**的
(`need_write_host = ... && ~exist(valid_elements.txt)`)，于是改了 `ELEM_SIZE` 重跑后，
旧 host 仍是旧尺度 → `abaqus_cfrc_compare.py` 按旧 `dx` 建 host → **beam 路径范围远大于
host 结构范围 → 嵌入 (EmbeddedRegion) 计算出错**。`force=true` 也救不了 (同一存在性判断)。

### 改动
**run_full_comparison.m — Stage 8 新增 host 尺度守卫 + 导出后 bbox 核查**
- `host_scale_mismatch()`: 比对 `host/mesh_params.txt` 记录的 `(nelx,nely,nelz,dx,dy,dz)`
  与当前 `voxel_refined_latest.mat` 的网格尺度。不一致 (或旧格式缺 `dx` 字段且当前 `dx≠1`)
  → 判定 host 陈旧。
- 陈旧 (或 `force=true`) 时自动删除旧 `mesh_params.txt` + `valid_elements.txt`，
  使本 Stage 按当前 `ELEM_SIZE` 重写 host。
- `verify_path_host_bbox()`: 导出后打印 host 结构 bbox vs beam 路径 bbox，
  路径明显超出结构 (容差 = 跨度 20% 或 3mm) 时红字告警并提示 `force` 重跑。

### 对比脚本是否受网格尺寸影响 (已核查, 无需改)
- **compute_path_statistics.m (路径对比)**: 尺度安全。应力查表优先用
  `voxel_refined_latest.mat` (Format A)，按精化网格自身的 `dx/x_min` 定位，天然随 `ELEM_SIZE`。
  长度类指标是物理 mm，4 个配置同一 `ELEM_SIZE` → 相对对比有效。
  (注意: 同一次 run 内的 4 配置可比; 跨不同 `ELEM_SIZE` 的历史结果, 绝对长度会按比例不同。)
- **compare_fea_results.m (刚度对比)**: 尺度无关。只读 ODB 提取的 `U/F/RF` 算
  `K=Σ(Fu)/Σ(u²)`，模型本身尺度一致即可 (由上面 host 守卫保证)。

### 说明: voxelize.m 生成的 voxel_grid.inp 不需要随 ELEM_SIZE 改
`voxel_grid.inp` 是**初始应力分析**用的 host (原始网格, `VoxelSize` 默认 1.0)。它产出的
应力场 (方向角 + 密度) 是**尺度无关**的，`voxel_refinement` 会重新网格化并按 `ELEM_SIZE`
放大。4-way 对比用的 host 是 `abaqus_cfrc_compare.py` 从 `valid_elements.txt` **独立重建**的
(已随 `ELEM_SIZE`)，并不使用 `voxel_grid.inp`。故初始 host 与对比 host 互不影响。

---

## 2026-06-04 — 对比管线参数集中化 + 线宽统一

### 背景 / 问题
`run_full_comparison.m` 做 4-way 对比 —— (曲面 mine / 平面 planar) × (流线 stream / 偏置 offset)。
原本各脚本的几何与工艺参数**分散硬编码、彼此不一致**，导致 4 个配置并非在"完全相同条件"下生成：

- **线宽不一致**：流线路径 `offset_distance = 0.4`，偏置路径 `offset_distance = 0.3`，分别硬编码在两个文件里，无法从管线统一控制。
- **平面切片未接入管线**：`generate_planar_slicing.m` 硬编码 `LAYER_THICKNESS_ORIG = 1.0`（魔数），曲面分辨率用 `surf_res = h*0.5`，且不读取 `GRID_STEP`；注释还引用了早已不存在的旧 v6 公式 `OFFSET_STEP = OFFSET_STEP_ORIG * SCALE_FACTOR`（当前 v6 实为 `OFFSET_STEP = SCALE_FACTOR`）。

### 方案
建立**单一数据源**，让层高与线宽沿管线流动：

```
voxel_refinement_from_test.m  (LINE_WIDTH = 0.4 在此定义)
        │  写入 refined_data.parameters.LINE_WIDTH
        ├──► slice_refined_model_v6.m   ─┐ 读取并盖章进 slice_results.parameters
        └──► generate_planar_slicing.m  ─┘
                    │  slice_results.parameters.LINE_WIDTH
                    ├──► all_layers_path_generation_v6.m  (offset_distance ← LINE_WIDTH)
                    ├──► path_generation_offset_only.m    (offset_distance ← LINE_WIDTH)
                    └──► interactive_path_planning_v28.m  (GUI 默认线宽 ← LINE_WIDTH)
```

- **线宽统一为 0.4mm**：4 个对比配置共用同一线宽来源；改 `voxel_refinement_from_test.m` 里一个 `LINE_WIDTH` 即全部联动。
- **平面切片对齐曲面 v6**：从 `refined_data.parameters` 继承全部几何参数。

### 改动文件
1. **voxel_refinement_from_test.m**
   - 新增控制参数 `LINE_WIDTH = 0.4`（4-way 对比线宽的唯一来源）。
   - 写入 `refined_data.parameters.LINE_WIDTH`；更新顶部参数说明注释。
2. **slice_refined_model_v6.m**（曲面切片）
   - 读取 `refined_data.parameters.LINE_WIDTH`（缺失回退 0.4），透传盖章进 `slice_results.parameters.LINE_WIDTH`。
   - `slice_results.parameters` 增加 `GRID_STEP`、`LINE_WIDTH` 字段。
3. **generate_planar_slicing.m**（平面切片，核心整改）
   - 删除魔数 `LAYER_THICKNESS_ORIG = 1.0`；层高直接 `LAYER_THICKNESS_MM = SCALE_FACTOR`（≡ v6 `OFFSET_STEP`）。
   - 从管线继承 `GRID_STEP = ELEM_SIZE/REFINE_FACTOR`、`SURFACE_RESOLUTION = 0.35*GRID_STEP`、`DENSITY_THRESHOLD`。
   - 曲面网格分辨率 `surf_res` 由 `h*0.5` 改为 `SURFACE_RESOLUTION`（与 v6 同源）。
   - Z 高度场改用体素中心原始 min/max（不再 ±h/2 过度外扩），painting 半径随 `GRID_STEP`；边界容差统一交给下游 `z_margin`，与 v6 烧入语义一致。
   - `slice_results.parameters` 盖章 `LINE_WIDTH / GRID_STEP / SURFACE_RESOLUTION / OFFSET_STEP`。
4. **all_layers_path_generation_v6.m**（流线路径）
   - 加载切片后用 `slice_results.parameters.LINE_WIDTH` 覆盖 `offset_distance`（缺失回退 0.4）。
5. **path_generation_offset_only.m**（偏置路径）
   - 同上；默认值由 `0.3` 改为统一的 `0.4`。
6. **interactive_path_planning_v28.m**（交互式 GUI）
   - 启动加载切片后，每层默认线宽继承自 `slice_results.parameters.LINE_WIDTH`（回退 0.4）。
   - "偏置距离" 输入框初始值由硬编码 `0.3` 改为读每层默认线宽。
   - `default_params()` 回退值 `0.3 → 0.4`。

### 验证
- MATLAB `checkcode` 静态检查全部修改文件：无语法错误（仅余既有风格提示）。
- 合成体素（含 `LINE_WIDTH`）跑通整条平面切片：层高/`GRID_STEP`/`SURFACE_RESOLUTION`/`LINE_WIDTH` 正确盖章并被路径生成读到；魔数字段已消失。

### 升级 / 使用注意
- **需重跑才能生效**：`run_full_comparison()` 默认跳过已存在产物。磁盘上现有的 offset 路径 mat 是用旧线宽 0.3 生成的（陈旧）。
- 推荐刷新流程：
  ```matlab
  run('voxel_refinement_from_test.m')   % 让 LINE_WIDTH=0.4 写进体素 mat
  run_full_comparison('force', true)     % 强制重切 + 重生成 4 套路径
  ```
- 切片器在缺 `LINE_WIDTH` 时回退 0.4 并盖章，故即使不重跑 voxel_refinement，强制重跑 `run_full_comparison` 结果线宽也是 0.4（只是体素文件不显式记录线宽，可追溯性稍弱）。
