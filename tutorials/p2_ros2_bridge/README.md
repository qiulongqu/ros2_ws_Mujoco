# P2: ROS2 + Mujoco 桥接

让 ROS2 和 Mujoco 通过 Topic 双向通信。

## 目录结构（官方标准）

```
p2_ros2_bridge/
├── models/                          # Mujoco XML 模型文件
│   └── pendulum_actuated.xml        # 带执行器的单摆模型
├── scripts/                         # Python 脚本（加载模型）
│   ├── step1_openloop_control.py    # 开环力矩控制
│   └── step2_ros2_subscriber.py     # ROS2 订阅者驱动
├── ros2_pkg/                        # ROS2 功能包
│   └── mujoco_bridge/
│       ├── package.xml
│       ├── setup.py
│       └── mujoco_bridge/
│           └── bridge_node.py       # 完整桥接节点
└── README.md
```

## 运行

### Step1: 开环控制（纯 Mujoco，无需 ROS2）

```bash
conda activate mujoco
python3 tutorials/p2_ros2_bridge/scripts/step1_openloop_control.py
```

### Step2: ROS2 订阅者（ROS2 → Mujoco）

**推荐先用 standalone 模式验证物理:**
```bash
conda activate mujoco
python3 tutorials/p2_ros2_bridge/scripts/step2_ros2_subscriber.py --test
```

**ROS2 模式（需要两个 terminal）:**

**Terminal 1**（发送力矩指令）:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /mujoco_joint_cmd std_msgs/Float64MultiArray "{data: [2.0]}"
```

**Terminal 2**（运行桥接节点）:
```bash
conda activate mujoco
python3 tutorials/p2_ros2_bridge/scripts/step2_ros2_subscriber.py
```

### ROS2 功能包（完整桥接）

```bash
cd tutorials/p2_ros2_bridge/ros2_pkg
colcon build --packages-select mujoco_bridge
source install/setup.bash
ros2 run mujoco_bridge bridge_node
```

## Topic 接口

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/mujoco_joint_cmd` | Float64MultiArray | → Mujoco | 力矩指令 |
| `/mujoco_joint_state` | JointState | ← Mujoco | 关节状态 |

## 前置条件

- ROS2 Humble 已安装
- `mujoco` conda 环境：`conda create -n mujoco -y && conda activate mujoco && pip install mujoco numpy glfw pillow`
- ROS2 Python 包（仅 ros2_pkg 需要）：`pip install mujoco rclpy std_msgs sensor_msgs`