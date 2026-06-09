#!/usr/bin/env python3
"""
P5 MoveIt2 端到端 Plan+Execute 验证脚本

P5 (MoveIt2) A2 阶段:
  1. 通过 move_group 服务规划从零位到目标关节角的轨迹
  2. 通过 /execute_trajectory action 执行轨迹
  3. 监控 /joint_states 验证实际运动
  4. 写 evidence 到 resources/p5/p5_step1_plan_execute.json

L1→L2→L3→L4 全链路验证:
  L1 MoveIt2 plan_kinematic_path 服务
  L2 JTC 内部 splines 插值
  L3 ros2_control position interface → MuJoCo position actuator (kp=4000)
  L4 MuJoCo 物理引擎

启动前置:
  终端 1: ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false rviz_gui:=false
  终端 2: ros2 launch gofa_moveit_config move_group.launch.py
  终端 3: python3 01_plan_execute_demo.py
"""

import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    MotionPlanRequest,
    MotionPlanResponse,
    RobotState,
    Constraints,
    JointConstraint,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK
from moveit_msgs.action import ExecuteTrajectory


# 目标关节角 (rad): [joint_1, ..., joint_6] — 折中位姿, 验证可达
TARGET_JOINTS = [0.0, -0.5, 0.8, 0.0, 0.5, 0.0]


class PlanExecuteValidator(Node):
    def __init__(self):
        super().__init__("plan_execute_validator")

        # 服务客户端
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # action 客户端
        self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        # 状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self.latest_joint_state = None

        self.get_logger().info("PlanExecuteValidator 节点已创建")

    def _joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def wait_for_services(self, timeout=10.0):
        """等待所有服务可用"""
        for client, name in [
            (self.plan_client, "/plan_kinematic_path"),
            (self.fk_client, "/compute_fk"),
            (self.ik_client, "/compute_ik"),
        ]:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"服务 {name} 不可用")
                return False
            self.get_logger().info(f"服务 {name} 在线 ✓")

        if not self.exec_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("action /execute_trajectory 不可用")
            return False
        self.get_logger().info("action /execute_trajectory 在线 ✓")

        # 等一次 joint_states
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None:
                break
        if self.latest_joint_state is None:
            self.get_logger().error("未收到 /joint_states")
            return False
        self.get_logger().info(f"当前 joint_states: {list(self.latest_joint_state.position)[:3]}...")
        return True

    def build_plan_request(self, target_joints):
        """构造 MotionPlanRequest: 起点 = current state, 终点 = target_joints"""
        from moveit_msgs.msg import WorkspaceParameters

        req = MotionPlanRequest()
        req.group_name = "gofa_arm"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        # workspace bounds: 覆盖 GoFa 工作空间
        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner.x = -2.0
        ws.min_corner.y = -2.0
        ws.min_corner.z = 0.0
        ws.max_corner.x = 2.0
        ws.max_corner.y = 2.0
        ws.max_corner.z = 3.0
        req.workspace_parameters = ws

        # 起点: is_diff=True + 空 joint_state = MoveIt 完全用当前 planning scene state
        # 这样 move_group 自己用 moveit_ros_planning_interface 拿的 current state
        req.start_state = RobotState()
        req.start_state.is_diff = True

        # 终点: 目标关节角, 容差放宽到 0.05 rad
        goal = Constraints()
        for i, joint_name in enumerate([
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ]):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_joints[i]
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        req.goal_constraints.append(goal)

        # pipeline_id = "" 用 move_group 默认 pipeline
        req.pipeline_id = ""
        req.planner_id = "RRTConnectkConfigDefault"

        return req

    def call_plan(self, target_joints):
        """调用 /plan_kinematic_path 服务"""
        srv = GetMotionPlan.Request()
        srv.motion_plan_request = self.build_plan_request(target_joints)

        self.get_logger().info(
            f"调用 /plan_kinematic_path: 目标 = {[f'{j:.2f}' for j in target_joints]}"
        )
        future = self.plan_client.call_async(srv)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        response = future.result()

        if response is None:
            self.get_logger().error("plan 服务调用失败 (no response)")
            return None

        result = response.motion_plan_response
        self.get_logger().info(
            f"plan result: error_code={result.error_code.val}, "
            f"fraction={result.planning_time}"
        )

        if result.error_code.val != 1:  # SUCCESS=1
            self.get_logger().warn(f"规划失败: error_code={result.error_code.val}")
            return None

        traj = result.trajectory.joint_trajectory
        self.get_logger().info(
            f"规划成功! {len(traj.points)} 个轨迹点, "
            f"总时长 = {traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9:.2f}s"
        )
        return result

    def call_execute(self, motion_plan_response):
        """调用 /execute_trajectory action"""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = motion_plan_response.trajectory

        self.get_logger().info("调用 /execute_trajectory action...")
        future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("execute_trajectory action 被拒绝")
            return False

        self.get_logger().info("execute_trajectory action 接受, 等待结果...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()

        if result is None:
            self.get_logger().error("execute_trajectory action 超时")
            return False

        status = result.status
        self.get_logger().info(f"execute_trajectory result: status={status}")
        return status == 4  # SUCCEEDED

    def collect_joint_state_snapshot(self, n_samples=10, interval=0.1):
        """连续采集 n_samples 次 joint_state, 用于 evidence"""
        snapshots = []
        for _ in range(n_samples):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is not None:
                snapshots.append({
                    "t": time.time(),
                    "name": list(self.latest_joint_state.name),
                    "position": list(self.latest_joint_state.position),
                    "velocity": list(self.latest_joint_state.velocity or []),
                })
            time.sleep(interval)
        return snapshots


def main():
    rclpy.init()
    validator = PlanExecuteValidator()

    if not validator.wait_for_services():
        validator.get_logger().error("服务未就绪, 退出")
        rclpy.shutdown()
        return 1

    validator.get_logger().info("=" * 60)
    validator.get_logger().info("P5 Plan+Execute 端到端验证")
    validator.get_logger().info(f"目标关节角: {TARGET_JOINTS}")
    validator.get_logger().info("=" * 60)

    # Step 1: Plan
    initial_state_snapshot = validator.collect_joint_state_snapshot(n_samples=3)
    plan_response = validator.call_plan(TARGET_JOINTS)

    if plan_response is None:
        validator.get_logger().error("规划失败, 退出")
        rclpy.shutdown()
        return 2

    # Step 2: Execute
    t_execute_start = time.time()
    success = validator.call_execute(plan_response)
    t_execute_end = time.time()

    # Step 3: 收集执行后的 joint state
    time.sleep(1.0)  # 让物理稳定
    final_state_snapshot = validator.collect_joint_state_snapshot(n_samples=3)

    # Step 4: 计算末端误差
    final_pos = final_state_snapshot[-1]["position"] if final_state_snapshot else []
    position_error = [
        abs(f - t) for f, t in zip(final_pos, TARGET_JOINTS)
    ] if final_pos else []

    evidence = {
        "test_name": "P5_A2_PlanExecute_End2End",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "target_joints_rad": TARGET_JOINTS,
        "target_joints_deg": [f"{j * 180 / 3.14159:.1f}" for j in TARGET_JOINTS],
        "plan_response": {
            "n_points": len(plan_response.trajectory.joint_trajectory.points),
            "duration_s": (
                plan_response.trajectory.joint_trajectory.points[-1].time_from_start.sec
                + plan_response.trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
            ),
            "error_code": plan_response.error_code.val,
        },
        "execute": {
            "success": success,
            "duration_s": t_execute_end - t_execute_start,
        },
        "initial_state": initial_state_snapshot,
        "final_state": final_state_snapshot,
        "position_error_rad": position_error,
        "position_error_deg": [f"{e * 180 / 3.14159:.2f}" for e in position_error],
        "verdict": "PASS" if success and all(e < 0.05 for e in position_error) else "FAIL",
    }

    # 保存 evidence
    out_dir = Path("/home/yunhao2204/ros2_ws_abb/mujocoONLY/resources/p5")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "p5_step1_plan_execute.json"
    with open(out_path, "w") as f:
        json.dump(evidence, f, indent=2)

    validator.get_logger().info(f"Evidence 已写入: {out_path}")
    validator.get_logger().info(
        f"位置误差: max={max(position_error) if position_error else 0:.4f} rad "
        f"({max(position_error) * 180 / 3.14159 if position_error else 0:.2f}°)"
    )
    validator.get_logger().info(f"verdict: {evidence['verdict']}")

    rclpy.shutdown()
    return 0 if evidence["verdict"] == "PASS" else 3


if __name__ == "__main__":
    sys.exit(main())
