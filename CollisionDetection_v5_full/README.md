# 双喷头CFRC 3D打印碰撞检测系统 v3.3

## 版本更新 (v3.3)

**修正内容**：
1. 列4和列5都是**度数**
2. 锥面方向修正：**顶点在喷嘴尖端附近，锥面向远离喷嘴的方向展开**
3. **法兰盘位置修正**：法兰盘现在正确地位于两个工具尖端的上方（Z更大）

## 项目结构

```
CollisionDetection_v3_full/
├── main_collision_detection.m   # 主程序入口 ← 运行这个
├── get_default_config.m         # 默认配置参数
├── read_excel_path.m            # Excel数据读取
├── compute_tool_positions.m     # 工具位置和姿态计算 (核心)
├── detect_point_collision.m     # 点碰撞检测 (ToolR)
├── detect_cone_collision.m      # 锥面碰撞检测 (ToolC)
├── run_collision_detection.m    # 检测主循环
├── visualize_results.m          # 结果可视化
├── print_report.m               # 报告输出
└── README.md                    # 本文档
```

**共9个MATLAB文件 + 1个README = 10个文件**

## Excel输入格式

你的Excel格式 (8列，无表头):

| 列 | 含义 | 单位 | 示例值 |
|----|------|------|--------|
| 1 | X坐标 (ToolC尖端) | mm | 0.876 |
| 2 | Y坐标 (ToolC尖端) | mm | -0.143 |
| 3 | Z坐标 (ToolC尖端) | mm | 149.9 |
| 4 | **Rx** (绕X轴旋转) | **度** | 0.26 |
| 5 | **Ry** (绕Y轴旋转) | **度** | -41.47 |
| 6 | 信号1 | - | 1 |
| 7 | 信号2 | - | 14 |
| 8 | 信号3 | - | 111 |

## 锥面检测模型

```
               /         \
              /           \     ← 锥面向远离喷嘴的方向展开
             /             \
            /               \
           --------▲--------  ← 锥面顶点 (cone_apex)，在尖端上方0.5mm
                   |
            喷嘴尖端 (toolc_tip = 路径点)
                   |
                   ↓ 
            tool_z_axis (喷嘴出口方向)
```

## 工具结构模型

```
                 ┌───────┐
                 │ 法兰盘 │  ← Z最高 (flange)
                 └───┬───┘
            ┌────────┴────────┐
            │                 │
            ▼                 ▼
        ToolC尖端         ToolR尖端
       (=路径点)         (Y偏移相反)
        Z较低             Z较低
```

法兰盘位置根据Duanliang.m的变换公式计算：
```matlab
RE1 = E1 * troty(-tool_a) * transl(-tool_h, -tool_y, -tool_l)
```

**关键点**：
- 锥面**轴向与工具Z轴平行**，代表喷嘴的锥形外壳区域
- 工具Z轴方向由**Rx和Ry姿态角**计算得出
- 锥面顶点在喷嘴尖端**上方**（沿工具Z轴负方向偏移）

## 使用方法

```matlab
cd CollisionDetection_v3_full
main_collision_detection
```

运行后会弹出文件选择对话框，选择你的Excel文件即可。

## 坐标变换逻辑

```matlab
% 基础姿态: troty(pi) - 工具垂直向下
% 应用姿态调整: Rx(绕X轴) 和 Ry(绕Y轴)
R_world = Rx * Ry * R_base

% 工具Z轴方向
tool_z_axis = R_world * [0; 0; 1]

% 锥面顶点 (在尖端上方)
cone_apex = toolc_tip - apex_offset * tool_z_axis
```

## 可视化说明

关键帧图中会显示：
- **蓝色线**: 已打印路径
- **绿色点**: ToolC尖端 (= 路径点)
- **青色方块**: 法兰盘
- **品红色点**: ToolR尖端
- **红色箭头**: 工具Z轴方向
- **橙色半透明锥体**: 检测锥面 (与工具Z轴平行)

## 参数配置

编辑 `get_default_config.m`:

```matlab
% 锥面参数
config.cone.half_angle = deg2rad(15);  % 半角 (全角30度)
config.cone.depth = 5e-3;              % 深度 5mm
config.cone.apex_offset = 0.5e-3;      % 顶点到尖端的安全距离 0.5mm

% 碰撞阈值
config.collision.safe_dist = 5e-3;     % 安全距离 5mm
config.collision.warn_dist = 0.1e-3;   % 碰撞距离 0.1mm
```
