# P3: ABB GoFa CRB15000 机械臂模型

从 ABB GoFa CRB15000-10/1.52 的 URDF/XACRO 源文件出发，构建 MuJoCo 物理仿真模型。

## 目录结构

```
p3_arm_model/
├── gofa_crb15000.xml                  # MuJoCo 模型 (6-DOF, 8 body, 6 actuator)
├── step1_verify_model.py              # 静态验证 (5 项测试 + 渲染截图)
├── step2_interactive_viewer.py        # MuJoCo 官方交互查看器
├── urdf/                              # URDF/XACRO 源文件 (参考)
│   ├── gofa_crb15000_10_152_macro.xacro  # 🔑 核心 — 6 link + 6 joint + 质量惯量
│   ├── gofa_crb15000_10_152.xacro        # 顶层入口
│   ├── gofa_crb15000.ros2_control.xacro
│   ├── gofa_crb15000.muJoCo.ros2_control.xacro
│   ├── common_materials.xacro
│   └── common_colours.xacro
├── meshes/
│   ├── collision/                     # 7 个 STL 碰撞模型
│   │   ├── base_link.stl
│   │   └── link_1.stl ... link_6.stl
│   └── visual/                        # 7 个 DAE 视觉模型
│       ├── base_link.dae
│       └── link_1.dae ... link_6.dae
├── config/
│   └── joint_names_gofa_crb15000_10_152.yaml
└── README.md
```

## 模型信息

| 属性 | 值 |
|------|-----|
| 型号 | ABB GoFa CRB15000-10/1.52 |
| 自由度 | 6 (全旋转关节) |
| 负载 | 10 kg |
| 臂展 | 1.52 m |
| 控制器 | OmniCore |
| 源文件来源 | `abb_omnicore_ros2/robot_specific_config/abb_gofa_crb15000_support` |

### 关节参数

| 关节 | 轴 | 角度范围 | 最高速度 |
|------|-----|---------|---------|
| joint_1 | Z (垂直) | ±270° | 120°/s |
| joint_2 | Y | -140° ~ +90° | 120°/s |
| joint_3 | Y | -225° ~ +85° | 125°/s |
| joint_4 | X | ±200° | 200°/s |
| joint_5 | Y | ±130° | 200°/s |
| joint_6 | X | ±270° | 200°/s |

### 运动学链

```
world → base_link
  → joint_1(Z) → link_1
    → joint_2(Y) → link_2
      → joint_3(Y) → link_3
        → joint_4(X) → link_4
          → joint_5(Y) → link_5
            → joint_6(X) → link_6 → tool0
```

零位末端位置: `[0.888, 0.000, 1.297]`（垂直向上伸展）

## 运行

### Step1: 模型验证（静态测试，无需显示器）

验证模型加载、运动学、重力仿真，并保存渲染截图。

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p3_arm_model/step1_verify_model.py
```

输出：
- 终端：5 项测试结果（模型结构、渲染、关节运动、FK、重力）
- 图片：`resources/p3_step1_zeropose.png`（零位渲染图）

### Step2: 官方交互查看器（MuJoCo viewer）

使用 MuJoCo 官方的 `viewer.launch()` 进行实时渲染和交互。
所有交互由 MuJoCo 内置支持，鼠标操作即可旋转/平移/缩放视角。
（WSL2 下 `viewer.launch_passive()` 存在 GL context 线程间传递的 segfault 问题，已改用 `viewer.launch()` 替代，功能完全相同。）

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p3_arm_model/step2_interactive_viewer.py
```

**鼠标操作（官方内置）：**

| 操作 | 功能 |
|------|------|
| 左键拖拽 | 旋转视角 |
| 右键拖拽 | 平移视角 |
| 滚轮 | 缩放 |
| 双击 body | 跟踪该 body |
| Ctrl + 右键拖拽 | 调整 FOV |

**快捷键（官方内置）：**

| 按键 | 功能 |
|------|------|
| `Tab` | 切换右侧面板 |
| `Space` | 暂停 / 恢复仿真 |
| `F1` | 帮助 |
| `Esc` | 退出 |

**注意：** WSL2 下如果窗口黑屏，请确认 WSLg 已正确安装运行。

## 前置条件

- `mujoco` conda 环境：`conda create -n mujoco -y && conda activate mujoco && pip install mujoco numpy glfw pillow`
- Step2 交互查看器需要 X server（WSLg 或 VcXsrv）

## 模型来源

URDF/XACRO 源文件和 STL/DAE 网格文件复制自：

```
/home/yunhao2204/ros2_ws_abb/abb_gofa_battery/src/abb_omnicore_ros2/
├── abb_resources/urdf/common_materials.xacro
└── robot_specific_config/abb_gofa_crb15000_support/
```

MuJoCo XML 由 XACRO 手动转换，保留了完整的质量/惯量参数和关节限位。
