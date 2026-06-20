# CFRC 管线控制台 · CFRC-AM Pipeline Console

一个把整条「连续纤维复合材料增材制造」研究管线串起来的 MATLAB App：
每个阶段一键运行、实时状态灯、自动提示下一步、可视化直达。

---

## 1. 启动

```matlab
>> cd F:\308\DCT\CFRC_App
>> launch_CFRC
```

`launch_CFRC` 会自动把 `scripts/ functions/ viz/ python/` 加入 MATLAB 路径，
把工作目录切到 `data/`（这样所有脚本的 `load('x.mat')/save('x.mat')` 都落在 `data/`），
然后打开控制台窗口。**你只需要运行这一条命令。**

> 要求：MATLAB R2020a+（用到 uifigure / uigridlayout / scroll）。
> Python / Abaqus 阶段需要 `python`、`abaqus` 在系统 PATH 中。

---

## 2. 文件夹划分（脚本 / 数据 / 功能 分开）

```
F:\308\DCT\
├── CFRC_App\            ← 应用本体
│   ├── launch_CFRC.m            启动器（设路径 + 切目录 + 开窗）
│   ├── CFRC_Pipeline_App.m      App 主体（uifigure）
│   ├── app_helpers\cfrc_layout.m  唯一定义文件夹布局的地方
│   └── README_APP.md
├── scripts\             ← 流程“阶段”入口脚本/函数
├── functions\           ← 被阶段调用的辅助函数
├── viz\                 ← 可视化脚本
├── python\             ← voxelize.py / abaqus_*.py / extract_fea_results.py …
├── data\                ← 所有 .mat 数据（运行时的工作目录 cwd）
├── output\
│   ├── figures\         ← 出图（含 path_stats）
│   └── logs\            ← console.log 运行日志
（Abaqus 工作目录不在项目树内：固定为 C:\temp\cfrc_fea ——ASCII 路径，Abaqus 无法处理含中文的项目路径）
├── docs\                ← 仓库文档 + CHANGELOG + REPO_README
├── _gh_repo\            ← GitHub 克隆（**不在路径上**，用于同步/对比）
└── _backup_<时间戳>\    ← 改动前的本地 .m 备份（含旧版 generate_reference_surface.m）
```

**为什么这样能跑通**：源代码按类型分到 4 个文件夹并加入路径；数据集中在 `data/` 并作为工作目录。
脚本之间靠路径互相找到，脚本读写数据靠工作目录落到 `data/`——所以**不用改任何脚本里写死的文件名**。

---

## 3. 控制台用法

- **左侧** = 全部 16 个阶段，按 A 前处理 / B 切片 / C 路径 / D FEA 对比 分组。
  每个阶段前的小圆点是**状态灯**：

  | 颜色 | 含义 |
  |---|---|
  | 🔘 灰 | 缺输入（前置阶段没跑） |
  | 🟠 橙 | **可运行 = 下一步** |
  | 🟢 绿 | 已完成（产物已存在） |
  | 🟡 黄 | 输入比产物新，建议重跑 |
  | 🔴 红 | 上次运行出错 |
  | 🟣 紫 | Abaqus 手动步骤 |

- **中间** = 选中阶段的详情：说明、输入/产物是否就绪（✓/✗）、命令、运行/打开脚本/可视化按钮。
- **顶部「下一步 NEXT ▸」** 永远提示你现在该做什么。
- **顶部「▶ 自动串联 Run-all」** 会按顺序连续跑 MATLAB 阶段，**直到遇到需要 Abaqus 的手动步骤**才停下并提示。
- **底部** = 实时运行日志（同时写入 `output/logs/console.log`）。

---

## 4. 16 个阶段一览

| # | 阶段 | 类型 | 产物 |
|---|---|---|---|
| ① | 体素化 voxelize | Python | `voxel_grid.inp/.npz` |
| ② | Abaqus 应力分析 | **手动** | `topo_stress_result.mat` |
| ③ | 体素细化 | MATLAB 脚本 | `voxel_refined_latest.mat` |
| ④ | 参考曲面（中面） | MATLAB 脚本 | `Pre_surface.mat` |
| ⑤ | 曲面切片 v6 | MATLAB 函数 | `slice_results_refined_latest.mat` |
| ⑥ | 平面切片（基线） | MATLAB 函数 | `..._PLANAR.mat` |
| ⑦ | 路径 mine_stream | MATLAB 函数 | `all_layers_paths_only_v3.mat` |
| ⑧ | 路径 mine_offset | MATLAB 函数 | `..._mine_offset.mat` |
| ⑨ | 路径 planar_stream | MATLAB 函数 | `..._planar_stream.mat` |
| ⑩ | 路径 planar_offset | MATLAB 函数 | `..._planar_offset.mat` |
| ⑪ | 校验 4 组产物 | MATLAB 脚本 | — |
| ⑫ | 导出到 FEA ×4 | MATLAB ×4 | `C:\temp\cfrc_fea\<cfg>\beam_paths\` (+ helper 脚本) |
| ⑬ | Abaqus 4-way 作业 | **手动** | 4 个 `.odb` |
| ⑭ | 提取 ODB→CSV | Abaqus | `results/<cfg>_time_history.csv` |
| ⑮ | 刚度对比 K | MATLAB 函数 | `results/comparison_table.md` |
| ⑯ | 路径几何统计 | MATLAB 函数 | `output/figures/path_stats/` |

阶段 ② 和 ⑬⑭ 是 Abaqus 步骤：控制台会把要在 Abaqus/CAE 里敲的命令完整列在该阶段详情里（点击即可复制）。

---

## 5. 注意事项

- **参数微调**：本版本对 MATLAB 函数阶段用既定参数运行；对 `clear` 开头的脚本阶段（③④）用脚本内默认参数运行，点「📝 打开脚本」可直接编辑参数后再 Run。
- **运行时 UI 会短暂无响应**：MATLAB App 回调是同步的，长阶段（切片/路径）运行期间窗口会“忙”，属正常，日志结束即恢复。
- **`generate_reference_surface.m` 已替换为 GitHub 最新版**；你之前本地版（含我加的 `[EDGE-FIX]` 多项式趋势 + 你的中面/parity 工作若在本地）已备份在 `_backup_<时间戳>\`。如需要找回，从那里取。
- **`diag_surface.m` / `Run_Full_Pipeline*.m`** 等本地独有脚本已保留在 `scripts/`。注意 `Run_Full_Pipeline_v8.m` 属于另一条「制造后处理」管线，依赖 `pipeline_funcs/`（仓库里没有），未接入本控制台。
- **`_gh_repo/` 与 `_backup_*/` 不在 MATLAB 路径上**，避免函数重名冲突。

---

## 6. 以后再从 GitHub 同步

```bash
cd F:\308\DCT\_gh_repo && git pull
# 然后把更新的 .m 复制回 scripts/ functions/ viz/，.py 回 python/
```
（或直接告诉我“同步最新”，我来做差异替换。）
