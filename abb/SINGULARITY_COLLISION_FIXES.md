# 奇异点定义/生成 + 碰撞规避 —— 修复与新模块说明

> 针对 ABB IRB-1200 双喷头 FP 打印。原始文件已备份到 `abb/_backup_pre_fix_*/`。
> 全部改动均为纯 ASCII 源码，MATLAB R2022a + Robotics Toolbox (Peter Corke) 实测通过。

## 一、修掉的"奇异点定义/生成"真 bug

| 文件 | Bug | 修复 |
|---|---|---|
| `rscfx.m` | cfx 的 J3 子标志用了**两个不同阈值** `>-1.459095` 与 `<-1.50971`，中间 (−1.50971,−1.459095) 约 −86.5°~−83.6° 是**空带**：8 个分支全不命中 → 落到末尾 `error('The Cfx dose not exist!')`，**直接中断 MOD 生成**（边界附近还会产生虚假/漏报的奇异标志）。 | 统一成单一阈值 `J3_THR=-1.484`，分类连续且全覆盖，永不报错。 |
| `rscfx.m` | cf1/cf4/cf6 首/末分支有多余外边界，关节恰在/略超限时全不命中 → 标志静默保持 0（配置判错）。 | 首/末分支改为开区间，覆盖整个关节量程；量程内分类不变。 |
| `ikbest.m` | 关节限位门**与 `CreateIRB1200` qlim 不符**：J5 用 ±170°（真限位 ±130°）→ 可能**选中不可达解并下发**；J6 用 ±130°（真限位 ±242°）→ **拒绝合法解**、虚假抛 "solution does not exist"。 | 限位门改为与 qlim 完全一致：J1±170 / J2[-100,130] / J3[-200,70] / J4±270 / **J5±130** / **J6±242**。并跳过 NaN（不可达）分支。 |
| `ikabb.m` | 8 个复制块**无数值保护**：不可达目标时 `sqrt(负)` 出复数、`|c5|>1` 出复数、`v1==0` 时 `theta1` 未定义（泄漏上一个解）。这些都会静默污染下游 IK/配置/下发。 | 重构为一个带保护的分支函数：不可达 → 该解返回 NaN（供 ikbest 跳过）；c5 钳到 [−1,1]；v1==0 有默认 theta1。可达目标结果与原版逐位一致。 |
| `rot2quat.m` | 用 4 个独立 `if d==w/a/b/c`（非 elseif）+ 浮点精确相等：并列最大时**多分支执行、后者覆盖**，且可能 `sqrt(负)`。 | 取最大分量唯一选支（switch），半径钳到 ≥0；各分支公式不变。 |

### 改进的奇异点"判定"（生成端）—— `needs_moveabsj.m`
原 `YZ_AMV1 / Duanliang` 只在 `rcf(4)` (cfx) 变化时判奇异。新判据同时覆盖：
- **真·腕部奇异 J5≈0**（J4/J6 共线，IK 病态）——原逻辑完全漏掉；
- **任意** cf1/cf4/cf6/cfx 配置翻转（不止 cfx）；
- 相邻点关节大跳变。
返回是否需要 `MoveAbsJ` 及原因（`init/cf/j5/jump`）。

## 二、成熟的"碰撞规避"——绕刀具轴 roll（你选的自由度）

原 `YZ_AMV1_RZ_Modified.m` 只有**一个手调全局常数** `delta_rz=-pi/2`（且引用的 `RZ_Offset_Analysis.m` 在仓库里不存在），全程对每个点施加同一 roll，从不搜索、从不校验，也**没有任何碰撞几何**。

新模块做**逐点** roll 搜索，同时优化奇异 + 碰撞 + 连续性：

| 文件 | 作用 |
|---|---|
| `dual_tool_points.m` | 由刀尖位姿 E1 几何计算法兰、**空闲喷头尖端**、刀轴、以及空闲喷头这段"运动碰撞体"采样点（与 Duanliang 的 `flange=tip*troty(-a)*transl(-h,-y,-l)` 一致）。 |
| `optimize_print_roll.m` | 单点核心：在 `*trotz(delta)` 上扫 roll（粗 5°→细 1°），每个候选跑修好的 IK，代价 = `w_j5·(J5 离 0)` + `w_lim·(关节限位余量)` + `w_cfg·配置翻转` + `w_coll·(空闲喷头对已打印点云的间隙)` + `w_cont·(roll/关节连续性)`，取最优。roll 绕法向自转**不改变喷嘴朝向**，不损打印质量。 |
| `RZ_Optimize_PerPoint.m` | 驱动：读路径 Excel → 逐点优化 → 输出每点 `delta/qi/rcf/sing/clearance`，并与两基线（roll=0、roll=−90°）对比，存 `<path>_RZopt.mat`。 |

### 实测（`path_23_1.xlsx`，1451 点）
| 策略 | 奇异点(MoveAbsJ) | 空闲喷头最小间隙 |
|---|---|---|
| roll = 0° | 14 | 79.4 mm |
| roll = −90°（你原来的手调值） | 12 | 58.8 mm |
| **逐点优化** | **8** | 61.0 mm |

逐点优化把奇异点数压到最低（8 vs 14/12），间隙保持健康（本路径碰撞不吃紧，>> 8mm 安全带，故主要省在奇异上；更密的件上碰撞项会主动起作用）。

## 三、怎么接进你的 MOD 生成器

在 `YZ_AMV1_RZ_Modified.m` 里只需 3 处改动：
1. 读完 `YM` 后加一行：`R = RZ_Optimize_PerPoint('path_23_1.xlsx');`
2. 把两处姿态里的 `*trotz(delta_rz)` 换成 `*trotz(R.delta(i))`（碳纤维行/树脂行各一处，line 86 / 93）。
3. 把奇异判定块（line 122–137 的 `rcf(4)~=` 那段）换成用 `R.sing(i)`（已含 J5≈0 与全配置翻转），并用 `R.qi(i,:)` 作 `MoveAbsJ` 的 jointtarget。

> 验证脚本：`validate_fixes.m`（IK 正逆往返、rscfx 空带、rot2quat 往返）；`RZ_Optimize_PerPoint('path_23_1.xlsx')`（对比三策略）。

## 四、顺带修掉的独立碰撞检测模块（`CollisionDetection_v5_full/`）
- `compute_tool_positions.m`：姿态合成顺序 `Rx·Ry·troty(π)` → 改为 `troty(π)·trotx·troty`（与角度生成端一致），原来刀轴 **Y 分量符号反了**。
- `detect_cone_collision.m`：锥**内部**距离返回正值（越深越"安全"）→ 改为负穿透量，正确触发碰撞。
- `get_default_config.m`：`cone.depth` 50mm → 5mm（与注释/README 一致，原值过大几乎全判碰撞）。
（两份拷贝 `CollisionDetection_v5_full/` 与其 `colisiondection/` 子目录均已同步。）
