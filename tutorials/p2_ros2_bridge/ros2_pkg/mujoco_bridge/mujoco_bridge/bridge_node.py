"""
ROS2Mujoco 桥接节点
================================================

订阅 /mujoco_joint_cmd (Float64MultiArray)
驱动 Mujoco 仿真
发布 /mujoco_joint_state (JointState)

用法:
  cd ros2_ws
  colcon build --packages-select mujoco_bridge
  source install/setup.bash
  ros2 run mujoco_bridge bridge_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

try:
    import mujoco as mj
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


class MujocoBridge(Node):
    def __init__(self, model_path: str):
        super().__init__('mujoco_bridge')

        self.model = mj.MjModel.from_xml_path(model_path)
        self.data  = mj.MjData(self.model)

        # 关节名称（与 XML 中 joint name 一致）
        self.joint_names = ['pivot_joint']

        # 订阅力矩指令
        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            'mujoco_joint_cmd',
            self.cmd_callback,
            10
        )

        # 发布关节状态
        self.pub_state = self.create_publisher(
            JointState,
            'mujoco_joint_state',
            10
        )

        # 仿真参数
        self.dt = 0.002
        self.hz = 500
        self.steps_per_cycle = int(1.0 / self.hz / self.dt)
        self.latest_torque = [0.0]

        self.get_logger().info(f'Mujoco Bridge started: {model_path}')
        self.get_logger().info(f'Publishing to: mujoco_joint_state')

    def cmd_callback(self, msg: Float64MultiArray):
        self.latest_torque = list(msg.data)

    def step(self):
        torque = self.latest_torque[0] if self.latest_torque else 0.0
        self.data.ctrl[0] = torque
        for _ in range(self.steps_per_cycle):
            mj.mj_step(self.model, self.data)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(self.data.qpos[0])]
        msg.velocity = [float(self.data.qvel[0])]
        self.pub_state.publish(msg)


def main():
    import os

    if not MUJOCO_AVAILABLE:
        print("ERROR: mujoco not installed in ROS2 environment")
        return

    # 模型路径（相对于 ros2_pkg 目录）
    pkg_dir = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    model_path = os.path.join(pkg_dir, 'models', 'pendulum_actuated.xml')

    rclpy.init()
    node = MujocoBridge(model_path)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            node.step()
            if int(node.data.time * 10) % 5 == 0:  # 每0.5秒打印
                node.publish_state()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()