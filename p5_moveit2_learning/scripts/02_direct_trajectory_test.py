#!/usr/bin/env python3
"""
P5 直接 JointTrajectory 端到端验证 (绕过 L1 MoveIt 规划, 直接测 L3→L4)

P5 (MoveIt2) A2 阶段:
  1. 构造 5 个 waypoint 的 JointTrajectory
  2. 通过 /abb_controller/follow_joint_trajectory action 发给 JTC
  3. 监控 /joint_states 验证 MuJoCo 实际运动
  4. 写 evidence 到 resources/p5/p5_step2_direct_trajectory.json

L3→L4 验证:
  L3 JointTrajectoryController (接 FollowJointTrajectory action)
  L4 MuJoCo position actuator (kp=4000) → 物理仿真

这个测试绕过了 L1 (MoveIt 规划), 因为 L1 规划在初次集成时可能需要更细的
workspace + collision 配置. 但 L3→L4 是 MoveIt2→JTC→ros2_control→MuJoCo
链条的核心, 必须先打通.

启动前置:
  终端 1: ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false rviz_gui:=false
  终端 2: python3 02_direct_trajectory_test.py
"""

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# 6 个 waypoint (rad): "有故事的拣放演示" — 起手→抓取上方→抓取位→抬升→放置位→归零
# 沿用 P4 拣放已验证的角度 (joint_limits 内 + IK 收敛)
# 故事: 机械臂从零位, 去"抓"一个目标 (joint_2 = -0.5 表示前伸+下俯),
#        "抬起"到安全高度 (joint_3 = 0.5 表示抬升), "放下"到另一位置, 归零
WAYPOINTS = [
    [0.0,  0.0,  0.0,  0.0, 0.0, 0.0],   # 0. 起手位 (零位)
    [0.3, -0.3,  0.5,  0.0, 0.3, 0.0],   # 1. 抓取位上方 (肩转 + 抬升)
    [0.0, -0.5,  0.8,  0.0, 0.5, 0.0],   # 2. 抓取位 (前伸下俯)
    [0.0, -0.3,  0.5,  0.0, 0.3, 0.0],   # 3. 抬升到安全高度
    [0.3, -0.5,  0.8,  0.0, 0.5, 0.0],   # 4. 放置位 (侧方)
    [0.0,  0.0,  0.0,  0.0, 0.0, 0.0],   # 5. 归零
]
WAYPOINT_LABELS = [
    "起手位 (零位)",
    "抓取位上方 (肩转+抬升)",
    "抓取位 (前伸下俯)",
    "抬升到安全高度",
    "放置位 (侧方)",
    "归零",
]
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
WAYPOINT_DURATION_S = 1.5  # 每个 waypoint 1.5 秒, 6 个 = 9 秒总轨迹


class DirectTrajectoryValidator(Node):
    def __init__(self):
        super().__init__("direct_trajectory_validator")

        self.action_client = ActionClient(
            self, FollowJointTrajectory, "/abb_controller/follow_joint_trajectory"
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self.latest_joint_state = None

    def _joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def wait_for_action(self, timeout=10.0):
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            return False
        self.get_logger().info("action /abb_controller/follow_joint_trajectory 在线 ✓")
        return True

    def wait_for_joint_state(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is not None:
                return True
        return False

    def build_trajectory_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        for i, wp in enumerate(WAYPOINTS):
            point = JointTrajectoryPoint()
            point.positions = wp
            # 简化: 不指定 velocity/acceleration, 让 JTC 自己规划
            point.time_from_start.sec = int((i + 1) * WAYPOINT_DURATION_S)
            point.time_from_start.nanosec = int(((i + 1) * WAYPOINT_DURATION_S % 1) * 1e9)
            goal.trajectory.points.append(point)

        return goal

    def send_trajectory(self):
        goal = self.build_trajectory_goal()
        self.get_logger().info(f"发送 JointTrajectory: {len(WAYPOINTS)} 个 waypoint, "
                              f"总时长 = {len(WAYPOINTS) * WAYPOINT_DURATION_S:.1f}s")
        self.get_logger().info("轨迹剧情:")
        for label, wp in zip(WAYPOINT_LABELS, WAYPOINTS):
            self.get_logger().info(f"  - {label}: {[f'{p:.2f}' for p in wp]}")

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("JTC goal 被拒绝!")
            return None, None

        self.get_logger().info("JTC goal 接受, 等待执行结果...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=len(WAYPOINTS) * WAYPOINT_DURATION_S + 5.0)
        return goal_handle, result_future.result()

    def collect_joint_state_log(self, duration_s, hz=20):
        """持续采集 joint_state, 用于 evidence"""
        samples = []
        n = int(duration_s * hz)
        interval = 1.0 / hz
        for i in range(n):
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_joint_state is not None:
                samples.append({
                    "t": i * interval,
                    "position": list(self.latest_joint_state.position),
                    "velocity": list(self.latest_joint_state.velocity or []),
                })
            time.sleep(interval)
        return samples


def main():
    rclpy.init()
    node = DirectTrajectoryValidator()

    if not node.wait_for_action():
        node.get_logger().error("action server 未就绪, 退出")
        rclpy.shutdown()
        return 1

    if not node.wait_for_joint_state():
        node.get_logger().error("/joint_states 无数据, 退出")
        rclpy.shutdown()
        return 2

    initial_pos = list(node.latest_joint_state.position)
    node.get_logger().info(f"起始位置: {[f'{p:.3f}' for p in initial_pos]}")

    # 启动后台采样线程 (用 rclpy.spin_once 循环)
    # 这里简单: 串行, 先发 goal, 再在 goal 执行期间 spin
    node.get_logger().info("=" * 60)
    node.get_logger().info("L3 → L4 端到端验证 (有故事拣放演示)")
    node.get_logger().info("=" * 60)

    # 先发 goal
    goal_handle, result = node.send_trajectory()

    if goal_handle is None:
        rclpy.shutdown()
        return 3

    # 等待结果期间, 不停 spin 采 joint_state
    samples = []
    n_waypoints = len(WAYPOINTS)
    total_duration = n_waypoints * WAYPOINT_DURATION_S
    hz = 20
    n_samples = int(total_duration * hz)
    interval = 1.0 / hz

    node.get_logger().info(f"开始采样, 持续 {total_duration}s @ {hz}Hz ({n_samples} 个 sample)...")

    for i in range(n_samples):
        rclpy.spin_once(node, timeout_sec=0.005)
        if node.latest_joint_state is not None:
            samples.append({
                "t": i * interval,
                "position": list(node.latest_joint_state.position),
                "velocity": list(node.latest_joint_state.velocity or []),
            })
        time.sleep(interval)

    if result is None:
        node.get_logger().error("等待 result 超时, 强制结束")
        rclpy.shutdown()
        return 4

    status = result.status
    final_error = result.result.error_code
    final_pos = list(node.latest_joint_state.position) if node.latest_joint_state else []

    # 终点 vs 目标: 最后一个 waypoint
    final_target = WAYPOINTS[-1]
    position_error = [abs(f - t) for f, t in zip(final_pos, final_target)]

    evidence = {
        "test_name": "P5_A2_PickPlaceDemo_L3toL4",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "story": "有故事的拣放演示 (起手→抓取上方→抓取→抬升→放置→归零)",
        "trajectory": {
            "n_waypoints": n_waypoints,
            "waypoint_labels": WAYPOINT_LABELS,
            "waypoints_rad": WAYPOINTS,
            "waypoints_deg": [[f"{p * 180 / 3.14159:.1f}" for p in wp] for wp in WAYPOINTS],
            "duration_per_waypoint_s": WAYPOINT_DURATION_S,
            "total_duration_s": total_duration,
        },
        "result": {
            "status": status,
            "status_meaning": "SUCCEEDED" if status == 4 else f"code_{status}",
            "error_code": final_error,
        },
        "initial_position_rad": initial_pos,
        "final_position_rad": final_pos,
        "final_target_rad": final_target,
        "position_error_rad": position_error,
        "position_error_deg": [f"{e * 180 / 3.14159:.2f}" for e in position_error],
        "samples_n": len(samples),
        "verdict": "PASS" if status == 4 and all(e < 0.1 for e in position_error) else "FAIL",
    }

    out_dir = Path("/home/yunhao2204/ros2_ws_abb/mujocoONLY/resources/p5")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "p5_step2_direct_trajectory.json"
    with open(out_path, "w") as f:
        json.dump(evidence, f, indent=2)

    node.get_logger().info(f"Evidence 已写入: {out_path}")
    node.get_logger().info(f"status: {status} ({'SUCCEEDED' if status == 4 else '其他'})")
    if position_error:
        node.get_logger().info(
            f"终点误差: max={max(position_error):.4f} rad "
            f"({max(position_error) * 180 / 3.14159:.2f}°)"
        )
    node.get_logger().info(f"verdict: {evidence['verdict']}")

    rclpy.shutdown()
    return 0 if evidence["verdict"] == "PASS" else 5


if __name__ == "__main__":
    import sys
    sys.exit(main())
