#!/usr/bin/env python3
"""
GoFa MuJoCo + ros2_control + RViz 联合启动

支持独立控制 MuJoCo GUI 和 RViz GUI 的显示：
  - mujoco_gui:=true/false  — 控制 MuJoCo Simulate 窗口
  - rviz_gui:=true/false     — 控制 RViz 窗口

四种组合模式：
  1. 只开 MuJoCo:  ros2 launch gofa_moveit_config demo.launch.py rviz_gui:=false
  2. 只开 RViz:    ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false
  3. 两个都开:     ros2 launch gofa_moveit_config demo.launch.py
  4. 两个都不开:   ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false rviz_gui:=false

基于官方 mujoco_ros2_control_demos 模式:
  - ros2_control_node 使用 ParameterFile 加载控制器配置
  - Spawner 使用 --param-file 加载控制器参数
  - robot_description 通过 remap 传递给 ros2_control_node
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    bringup_share = FindPackageShare("gofa_mujoco_bringup")
    moveit_share = FindPackageShare("gofa_moveit_config")

    # Parse boolean arguments
    mujoco_gui = LaunchConfiguration("mujoco_gui").perform(context) == "true"
    rviz_gui = LaunchConfiguration("rviz_gui").perform(context) == "true"

    # Build robot description from xacro, pass headless parameter
    headless_value = "false" if mujoco_gui else "true"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([bringup_share, "urdf", "gofa.ros2_control.urdf.xacro"]),
            " headless:=", headless_value,
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    controllers_file = PathJoinSubstitution([bringup_share, "config", "gofa_controllers.yaml"])
    rviz_config = PathJoinSubstitution([moveit_share, "config", "gofa.rviz"])

    nodes = []

    # 1. robot_state_publisher — publishes TF from robot_description
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # 2. ros2_control_node with MuJoCo system interface
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file),
            ],
            remappings=[("~/robot_description", "/robot_description")],
            on_exit=Shutdown(),
        )
    )

    # 3. Controller spawners
    # ros2_control 硬约束: 同一 hardware interface (joint_X/position) 只能被一个 controller 持有
    # - joint_state_broadcaster: 发布 /joint_states (只读 state, 不冲突)
    # - abb_controller:          P5 MoveIt2 集成 (JTC 接 FollowJointTrajectory action)
    # 注意: P4 baseline (position_controller + topic pub) 走 gofa_mujoco_bringup/gofa_mujoco.launch.py
    #       demo.launch.py 专注 MoveIt2 路线,避免接口冲突
    controllers_to_spawn = ["joint_state_broadcaster", "abb_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controllers_file],
                output="both",
            )
        )

    # 4. RViz (conditional)
    if rviz_gui:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="both",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            )
        )

    return nodes


def generate_launch_description():
    mujoco_gui = DeclareLaunchArgument(
        "mujoco_gui",
        default_value="true",
        description="If true, shows the MuJoCo Simulate GUI window. If false, runs headless.",
    )

    rviz_gui = DeclareLaunchArgument(
        "rviz_gui",
        default_value="true",
        description="If true, launches RViz2. If false, skips RViz.",
    )

    return LaunchDescription(
        [
            mujoco_gui,
            rviz_gui,
            OpaqueFunction(function=launch_setup),
        ]
    )
