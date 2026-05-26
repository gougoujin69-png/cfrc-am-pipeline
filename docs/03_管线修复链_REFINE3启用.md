# CFRC AM Pipeline 修复记录

记录从 `run_full_comparison` Stage 2 报错开始，到 Abaqus 端 4 路对比能够正确出图的完整修复链。涉及 MATLAB 切片/路径生成、`run_full_comparison` 包装器、host 数据导出、Abaqus 端 host 重建、路径坐标映射等多处。

---

## 修复时间线

### 1. Stage 2 函数找不到

**症状**：`run_full_comparison` 在 Stage 2 报 `函数或变量 'slice_planar_z' 无法识别`。

**根因**：`run_full_comparison.m` 调用了 `slice_planar_z`，但仓库里的实际脚本叫 `generate_planar_slicing.m`。同类问题还有 4 处：

| Stage | wrapper 里调的名字 | 仓库里实际存在 |
|---|---|---|
| 1 | `slice_refined_model` | `slice_refined_model_v6.m` |
| 2 | `slice_planar_z` | `generate_planar_slicing.m` |
| 3、5 | `all_layers_path_generation(slice, target)` | `all_layers_path_generation_v6.m`（脚本，不接受参数）|
| 4、6 | `offset_only_path_generation(slice, target)` | `path_generation_offset_only.m`（已是函数）|

更糟的是这些脚本顶部都有 `clear; clc; close all;`，从 wrapper（function）里调用会把 wrapper 自己的局部变量清光。

**修复**（方案 B）：把 4 个核心脚本全部 function 化：
- 增加 `function ... end` 声明，参数化输入/输出 mat 文件路径
- 去掉顶部 `clear; clc; close all;`
- 对带 local subfunctions 的脚本（v6 系列）在主函数体后加 `end` 分隔
- `run_full_comparison.m` 里 5 处调用名对齐到新签名

### 2. Planar 切片层厚未应用 SCALE_FACTOR

**症状**：曲面切片和平面切片的层数不一致——曲面用了 `OFFSET_STEP = OFFSET_STEP_ORIG * SCALE_FACTOR` 处理细化，平面里 `LAYER_THICKNESS_MM = 1.0` 硬编码。

**修复**（`generate_planar_slicing.m`）：

```matlab
SCALE_FACTOR = refined_data.parameters.SCALE_FACTOR;
LAYER_THICKNESS_ORIG = 1.0;
LAYER_THICKNESS_MM = LAYER_THICKNESS_ORIG * SCALE_FACTOR;
```

与 v6 中的 `OFFSET_STEP = OFFSET_STEP_ORIG * SCALE_FACTOR` 完全对齐，保证 mine 和 planar 层数一致。新增的字段 `LAYER_THICKNESS_ORIG`、`SCALE_FACTOR` 写入 `slice_results.parameters` 便于追溯。

### 3. `export_paths_to_fea` brace 索引错误

**症状**：`Stage 8` 报 `此类型的变量不支持使用花括号进行索引。`，导致 4 个 cfg 目录全部为空。

**根因**：上一轮我在 `run_full_comparison.m` 里嵌的本地 helper `export_paths_to_fea(mat_file, cfg_name, fea_dir)` 假定 `paths_only` 是 cell，直接 `paths{k}` 取值；但实际 `paths_only` 是 struct：

```
paths_only (struct)
├── layer_paths_3d  (cell, 长度 = 层数)
│     └── {li}  (cell, 该层路径数)
│           └── {pj}  (Nx3 double, 一条 3D 路径)
├── layer_paths_2d
├── layer_offsets
├── ...
```

**修复**：重写 helper 按真实嵌套结构遍历——外层遍历层、内层遍历该层路径、统一展平为 4 位流水号 `path_NNNN.txt`。增加 struct 类型检查和空层/短路径过滤。

### 4. 本地 helper 屏蔽了独立的 `export_paths_to_fea.m`

**症状**：4 路 beam_paths 都正确写出后，Abaqus 调 `step1_build_template()` 报 `ERROR: host data directory not found`。

**根因**：仓库根目录有独立的 `export_paths_to_fea.m`（标准版），它会写 host 数据（`host/mesh_params.txt` + `host/valid_elements.txt`） + beam_paths + sentinel 文件 `beam_paths_summary.txt`。但我之前在 `run_full_comparison.m` 里嵌的本地 helper 同名且只写 beam_paths，MATLAB 解析规则下本地函数屏蔽全局，导致独立版从未被调用，host 数据从未生成。

**修复**：把本地 helper 整段（80 行）从 `run_full_comparison.m` 删掉，让 MATLAB 自然解析到仓库根目录的标准 `export_paths_to_fea.m`。

### 5. Stage 8 SKIP 判据与 Abaqus 不一致

**症状**：4 个 config 中只有 mine_stream 被 Abaqus 处理，其余 3 个被 `SKIP <cfg>: no beam_paths` 跳过。

**根因**：`abaqus_cfrc_compare.py:1389` 用 `beam_paths_summary.txt` 作 sentinel 判 SKIP，但旧版 wrapper 只看 `path_*.txt` 文件数。我那个本地 helper（已删除）只写 `path_*.txt` 不写 sentinel，所以 mine_offset / planar_stream / planar_offset 的 SKIP 状态在 MATLAB 和 Abaqus 两端不一致：MATLAB 觉得已经导好了所以 SKIP（不会触发标准 export 写 sentinel），Abaqus 觉得没导所以 SKIP。

**修复**：Stage 8 SKIP 判据改成"path 文件存在 **且** `beam_paths_summary.txt` sentinel 存在才跳过"，与 Abaqus 端完全对齐。当 sentinel 缺失但 path 文件存在时，触发 `[RUN]` 标签为 `(re-export: NNN path_*.txt present but summary missing)` 的强制重导出。

### 6. host 缺失导致 Stage 8 永远跳过

**症状**：当 4 路 beam_paths 都已经存在但 host 数据被删，Stage 8 全部 SKIP，host 永远不会被重建。

**根因**：标准 `export_paths_to_fea.m` 只在 `valid_elements.txt` 不存在时写 host 数据。如果 4 个 config 都 SKIP，整个 export 流程不被调用，host 缺失自我维持。

**修复**：Stage 8 前置检测 `host/valid_elements.txt`，缺失时强制 mine_stream（第一个 config）走 `force_this` 分支，无视 path 是否存在都重跑一次。日志多两行：

```
[INFO] C:\temp\cfrc_fea\host\valid_elements.txt missing
       will force-run first config to populate host/
[RUN]  ... -> mine_stream  (also writes host/)
```

### 7. Abaqus 端 host cell 尺寸硬编码（严重）

**症状**：Abaqus 里 host 部分的物理尺寸是真实尺寸的 REFINE_FACTOR 倍，beam 路径相对位置严重错位。从日志能直接看到：

```
Host center: (60.000, 30.000, 12.000)   <- 应该是 (20, 10, 4)
Beam center: (19.959, 10.039, 3.948)
```

host 是 beam 应在位置的 3 倍大。

**根因**：`abaqus_cfrc_compare.py:create_host_from_voxels` 在第 342 行：

```python
node_coords.append((i * dx, j * dy, k * dz))   # i,j,k 是细化网格索引
```

其中 `dx = Config.ELEMENT_SIZE_X = 1.0` 硬编码（原始网格的 mm），但 `i, j, k` 是细化后的索引（最大 120/60/24）。乘出来的物理范围 = nelx_fine × 1.0 = 120 mm，而真实物理 X = nelx_orig × 1.0 = 40 mm。

**修复**（双端）：

MATLAB 端 `export_paths_to_fea.m` 的 `write_host_mesh`：从 `grid_data` 相邻 voxel 中心反推真实物理 cell 尺寸，写进 `mesh_params.txt`：

```matlab
dx_phys = grid_data(2, 1, 1).x - grid_data(1, 1, 1).x;
dy_phys = grid_data(1, 2, 1).y - grid_data(1, 1, 1).y;
dz_phys = grid_data(1, 1, 2).z - grid_data(1, 1, 1).z;
```

输出新增 3 行：

```
dx 0.333333
dy 0.333333
dz 0.333333
```

Python 端 `create_host_from_voxels`：

```python
dx = params.get('dx', Config.ELEMENT_SIZE_X)
dy = params.get('dy', Config.ELEMENT_SIZE_Y)
dz = params.get('dz', Config.ELEMENT_SIZE_Z)
```

向后兼容：旧版 `mesh_params.txt` 没有 dx 行时退回到 Config 默认值。

### 8. BEAM_MANUAL_OFFSET 默认值有害

**症状**：beam 路径被强行"居中"，但拓扑优化结构本身不对称，自动居中会把 beam 整体推偏。

**根因**：原本 `BEAM_MANUAL_OFFSET = None` 触发自动居中（`beam bbox 中心对齐 host bbox 中心`），这在 host 错位时的兼容性兜底有意义；修复 cell 尺寸 bug 之后，MATLAB 端 beam 和 Python 端 host 都用以 `voxel(0,0,0)` 角为原点的同一套物理 mm 坐标系，本来就对齐，自动居中反而破坏对齐。

**修复**：`Config.BEAM_MANUAL_OFFSET` 默认值改为 `(0.0, 0.0, 0.0)`。

### 9. 路径生成 X 方向 scale 误用（最难定位）

**症状**：host cell 尺寸修好后，Y 和 Z 方向 path 跨度正常，但 X 方向 path 跨度只有应该值的 ~60%。

**根因**：`all_layers_path_generation_v6.m:464` 和 `path_generation_offset_only.m:246`：

```matlab
scale_x = (actual_x_max - actual_x_min) / max(nelx - 1, 1);
%         └─激活voxel物理x跨度─┘   └全网格nelx-1┘
```

这把"全网格 120 个 pixel 的 X 跨度"整体压缩到"激活 voxel 的物理 X 跨度"。Y 碰巧对，因为激活 voxel 在 Y 上跨满 99% 全网格；X 因为拓扑优化结构在 X 中段窄、激活 voxel 只占全网格 35%，scale_x 被低估 ~3 倍（恰好等于 1/REFINE_FACTOR）。

物理图像：流线种子点遍布 1–120 列，流线 endpoint 的 pixel col 能到 1 和 120。乘上低估的 scale_x 加上 `actual_x_min` 这个错的原点：

```
col = 1:    x_actual = 12.9 + 0    = 12.9   ← path X 最小值, 看似对
col = 120:  x_actual = 12.9 + 14   = 27.0   ← path X 最大值, 看似对
```

bbox 看着对（12.9–27.0），但中间所有 pixel 被压扁了 3 倍——physical x=12.9 应该对应 col=39（激活区最左边），现在所有 col 1~38 全部挤到 col=39 那个位置。

**修复**：从 `activated_grids` 自带的 `g.grid_index(1)` 反推真实物理 cell 尺寸和原点：

```matlab
[ix_min_val, p_ix_min] = min(actual_ix);
[ix_max_val, p_ix_max] = max(actual_ix);
if ix_max_val > ix_min_val
    dx_phys = (actual_x(p_ix_max) - actual_x(p_ix_min)) / ...
              (ix_max_val - ix_min_val);
    grid_x_origin = actual_x(p_ix_min) - (ix_min_val - 1) * dx_phys;
end
scale_x = dx_phys;
```

所有 `actual_x_min + (col-1)*scale_x` 改成 `grid_x_origin + (col-1)*scale_x`。同样的修复套到 Y 方向和 offset 脚本。不依赖 `grid_data` 直接访问、不依赖 SCALE_FACTOR 命名约定。

### 10. `run_compare('all')` 只生成 mine_stream 的统计

**症状**：从 `C:\temp\cfrc_fea\` 跑 `run_compare('all')`，
- 第一次报错 `Cannot find: compute_path_statistics`，
- 手动把脚本拷到 fea_dir 后再跑，4 联柱状图只画出 MINE+Stream 一根柱子，另外 3 个 config 都没数。

**根因**（两个）：

(a) `run_full_comparison.m` Stage 9 的 `helper_files` 列表里有 `run_compare.m` / `compare_fea_results.m` 等，**漏了 `compute_path_statistics.m`**。源码里还有一行注释说 "compute_path_statistics.m is NOT copied here - it lives in script_dir and reads/writes from there directly"——这是旧时代的假设，那时候 `run_compare` 还没出现，用户直接在 MATLAB 项目目录跑 `compute_path_statistics`。现在 `run_compare('all')` 会 `cd` 到 `C:\temp\cfrc_fea\` 再调，脚本必须能从 fea_dir 找到。

(b) `compute_path_statistics` 是从 `pwd` + MATLAB path 搜 path mat 文件的。`run_compare` cd 到 fea_dir 之后 pwd = fea_dir，那里并没有 4 个 path mat。`run_compare` 的 `script_dirs` 列表里有 `s3_compare` 和 `ablation_outputs`，这俩老目录里**碰巧**有同名的 `all_layers_paths_only_v3.mat`（之前做 3-way ablation 时留下的），所以 mine_stream 蒙对了；其他 3 个 mat（`*_mine_offset.mat`、`*_planar_stream.mat`、`*_planar_offset.mat`）只在拓扑结构主目录里有，没在搜索路径里，于是 `compute_path_statistics` 对它们 `[SKIP] no path mat found`。

**修复**（`run_full_comparison.m` Stage 9，两步）：

1. 把 `compute_path_statistics.m` 加进 `helper_files` 列表，删掉那行过时的注释。
2. 新增"复制 4 个 path mat 到 fea_dir"的步骤，紧跟在 helper 拷贝之后：

```matlab
path_mats = { ...
    'all_layers_paths_only_v3.mat', ...
    'all_layers_paths_only_mine_offset.mat', ...
    'all_layers_paths_only_planar_stream.mat', ...
    'all_layers_paths_only_planar_offset.mat', ...
};
for k = 1:numel(path_mats)
    src = fullfile(script_dir, path_mats{k});
    dst = fullfile(fea_dir, path_mats{k});
    if ~exist(src, 'file'), fprintf('  [WARN] ...'); continue; end
    copyfile(src, dst, 'f');
    fprintf('  [OK]   %s  (%.1f MB)\n', path_mats{k}, dir(dst).bytes/1e6);
end
```

总共 ~17 MB，复制秒级完成。这样 fea_dir 自给自足，`run_compare('all')` 不再依赖任何 MATLAB 项目目录在 path 里。

**关于应力对齐统计**：`compute_path_statistics` 还能算"路径切向 vs 局部应力方向"和"切片面法向 vs 应力方向"两类角度指标，前提是搜索路径里能找到 `topo_stress_result.mat`（或者它认识的几个别名）。如果你想在 fea_dir 也看到这两个图，需要手动把 `topo_stress_result.mat` 也拷过去：

```matlab
copyfile(fullfile(script_dir, 'topo_stress_result.mat'), ...
         fullfile(fea_dir, 'topo_stress_result.mat'), 'f');
```

我没在 Stage 9 里自动拷这个——它跟路径生成强相关性较弱、文件可能较大、且不是每次都要重算应力图。需要的时候手动一次到位。

---

## 修改的文件清单

| 文件 | 修改内容 | 涉及条目 |
|---|---|---|
| `run_full_comparison.m` | 函数调用名对齐、SKIP 判据改用 sentinel、新增 host 缺失检测、删掉 80 行错误的本地 helper、Stage 9 加 `compute_path_statistics.m` 和 4 个 path mat 到拷贝列表 | 1, 3, 4, 5, 6, 10 |
| `generate_planar_slicing.m` | 改成函数；从 `refined_data.parameters.SCALE_FACTOR` 计算 `LAYER_THICKNESS_MM`；元数据记录 SCALE_FACTOR | 1, 2 |
| `slice_refined_model_v6.m` | 改成函数；参数化输入/输出 mat 路径；保留 local subfunctions | 1 |
| `all_layers_path_generation_v6.m` | 改成函数；参数化输入/输出 mat 路径；**X/Y 方向 scale 改用真实 dx_phys** | 1, 9 |
| `path_generation_offset_only.m` | （原本就是函数）**X/Y 方向 scale 改用真实 dx_phys** | 9 |
| `export_paths_to_fea.m` | `write_host_mesh` 计算 dx/dy/dz 并写入 `mesh_params.txt` | 7 |
| `abaqus_cfrc_compare.py` | `create_host_from_voxels` 从 `mesh_params.txt` 读 dx/dy/dz；`BEAM_MANUAL_OFFSET` 默认值改成 `(0,0,0)` | 7, 8 |

`path_generation_offset_only.m` 本来就是带 `(slice_file, output_file)` 签名的标准函数，前面没改过结构，只在条目 9 改了 scale_x/y 逻辑。

---

## 数据流约定（修复后）

```
topo_stress_result.mat
   │
   ▼
voxel_refinement_from_test.m            (REFINE_FACTOR=3 → 120×60×24 网格)
   │  refined_data.parameters.SCALE_FACTOR = 0.333
   │  grid_data(i,j,k).x/.y/.z 已是物理 mm
   ▼
voxel_refined_latest.mat
   │
   ├──────── slice_refined_model_v6 ───────► slice_results_refined_latest.mat
   │         (OFFSET_STEP = 1.0 * SCALE_FACTOR)
   │
   └──────── generate_planar_slicing ───────► slice_results_refined_latest_PLANAR.mat
             (LAYER_THICKNESS_MM = 1.0 * SCALE_FACTOR)
   │
   ▼
all_layers_path_generation_v6  /  path_generation_offset_only
   (X/Y pixel→physical 用真实 dx_phys，从 g.grid_index 反推)
   │
   ▼
all_layers_paths_only_*.mat
   │
   ▼
export_paths_to_fea  (标准独立版, 不再被 wrapper 内嵌 helper 屏蔽)
   │  写 host/mesh_params.txt (含 dx dy dz)
   │  写 host/valid_elements.txt
   │  写 <cfg>/beam_paths/path_NNNN.txt
   │  写 <cfg>/beam_paths_summary.txt  (sentinel)
   ▼
C:/temp/cfrc_fea/
   │
   ▼
abaqus_cfrc_compare.py
   create_host_from_voxels  (用 mesh_params.txt 里的真实 dx/dy/dz)
   BEAM_MANUAL_OFFSET = (0,0,0)
```

---

## 验证流程

完整重跑一次（路径坐标在 mat 内部，必须重新生成，光重导出 txt 不行）：

```matlab
%% 1. 清掉所有下游产物
for f = {'all_layers_paths_only_v3.mat', ...
         'all_layers_paths_only_mine_offset.mat', ...
         'all_layers_paths_only_planar_stream.mat', ...
         'all_layers_paths_only_planar_offset.mat'}
    if exist(f{1}, 'file'), delete(f{1}); end
end

for c = {'mine_stream','mine_offset','planar_stream','planar_offset'}
    d = fullfile('C:/temp/cfrc_fea', c{1});
    if exist(d, 'dir'), rmdir(d, 's'); end
end
if exist('C:/temp/cfrc_fea/host', 'dir')
    rmdir('C:/temp/cfrc_fea/host', 's');
end

%% 2. 跑完整流程
run_full_comparison

%% 3. 验证 beam path X span (mine_stream 为例)
P = load('all_layers_paths_only_v3.mat');
allx = []; ally = []; allz = [];
for li = 1:numel(P.paths_only.layer_paths_3d)
    lp = P.paths_only.layer_paths_3d{li};
    if iscell(lp)
        for pj = 1:numel(lp)
            p = lp{pj};
            if ~isempty(p)
                allx = [allx; p(:,1)]; %#ok<AGROW>
                ally = [ally; p(:,2)];
                allz = [allz; p(:,3)];
            end
        end
    end
end
fprintf('Beam path coord ranges:\n');
fprintf('  X span: %.4f mm\n', max(allx)-min(allx));
fprintf('  Y span: %.4f mm\n', max(ally)-min(ally));
fprintf('  Z span: %.4f mm\n', max(allz)-min(allz));
```

预期 X span 从修复前的 14.12 mm 增大到 ~23 mm（贴近 host valid voxel X span 23.34 mm）。

Abaqus 端的关键日志检查点：

```
Cell size (mm): dx=0.3333 dy=0.3333 dz=0.3333  (from mesh_params.txt)   ← 必须是 from mesh_params, 不是 Config default
Bounding box: X[8.33, 31.67] Y[0.33, 19.67] Z[0.33, 7.67]               ← 真实物理尺寸
[Beam offset] Using manual offset from Config: (0.0, 0.0, 0.0)          ← 不再 auto-shift
```

---

## 仍需手动操作的步骤

`step1_build_template()` 只搭好 host part + step + material。在 `step2_run_batch_comparison()` 之前必须在 CAE GUI 里手动做完：

1. Load module
2. 设置 BC（约束 / rigid body restraint）
3. 在 load point 加 Concentrated Force（-Z）
4. **关键**: 创建 Assembly Set "LoadPoint"
5. File → Save

否则后续 4 路 job 都会跑出 `WARNING: Assembly Set "LoadPoint" not found`，结果是无约束 + 无外力的自由体，COMPLETED 但物理无意义。

---

## 还可以做的优化（未实施）

- `path_generation_offset_only.m` 顶部那两行关 warning 只对主端生效，parfor worker 上 polyshape 修复 warning 仍会刷屏。要彻底关掉，需要把 `warning off` 移到 `process_layer_offset_only` 函数体最开头。
- `Config.BLACKLIST_PATH_IDX` 现在还是 in-memory 的，每次跑完 `dump_blacklist()` 输出的 list 需要手动粘回源码才能持久化。可以改成自动写到 `blacklist.json` 然后读取。
- 5-way 对比中的 s3_mine 第 5 路还没整合进 `run_full_comparison`，目前框架是 4-way。
