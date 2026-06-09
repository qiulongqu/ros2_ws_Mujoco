#!/usr/bin/env python3
"""
P5 Demo with Pick-and-Place Scene (B 路径深挖)
================================================

相对 demo.launch.py 的差别:
  1. URDF 用 gofa_with_scene.ros2_control.urdf.xacro (含 table + block)
  2. RViz 加载 scene 版的 SRDF 配置 (gofa_with_scene.srdf)
  3. move_group 加载 ompl_with_scene.yaml (workspace bounds + planner configs)

启动链:
  robot_state_publisher
  ros2_control_node (mujoco_ros2_control → gofa_table_block.xml)
  joint_state_broadcaster
  abb_controller (JTC 接 FollowJointTrajectory)
  move_group (MoveIt2)
  rviz2 (MotionPlanning)

用法:
  ros2 launch gofa_moveit_config demo_with_scene.launch.py mujoco_gui:=false rviz_gui:=false
  ros2 launch gofa_moveit_config demo_with_scene.launch.py  # 全开
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

    mujoco_gui = LaunchConfiguration("mujoco_gui").perform(context) == "true"
    rviz_gui = LaunchConfiguration("rviz_gui").perform(context) == "true"
    move_group = LaunchConfiguration("move_group").perform(context) == "true"

    headless_value = "false" if mujoco_gui else "true"

    # B 路径核心: 用 gofa_with_scene URDF, 把 table+block 加载进 TF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([bringup_share, "urdf", "gofa_with_scene.ros2_control.urdf.xacro"]),
            " headless:=", headless_value,
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    # 同步: MoveIt 用 scene 版 SRDF (含 table+block collision matrix)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([moveit_share, "config", "gofa_with_scene.srdf"]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            value=robot_description_semantic_content.perform(context), value_type=str
        )
    }

    controllers_file = PathJoinSubstitution([bringup_share, "config", "gofa_controllers.yaml"])
    rviz_config = PathJoinSubstitution([moveit_share, "config", "gofa.rviz"])

    nodes = []

    # 1. robot_state_publisher — 发布 arm + table + block 的 TF
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # 2. ros2_control_node with MuJoCo system interface (加载 scene 版 MuJoCo XML)
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

    # 3. Controller spawners (与 demo.launch.py 一致)
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

    # 4. move_group (B 路径核心: 加载 scene 版 OMPL 配置)
    if move_group:
        # yaml 文件用 load_yaml 解析为 dict, 避免 ROS2 直接解析 raw file (会因缺 ros__parameters 报错)
        # 注: 沿用 move_group.launch.py 的成熟模式, ROS2 接受 Python 原生 bool/dict
        import yaml as _yaml

        def _load(pkg, subpath):
            return _yaml.safe_load(open(os.path.join(pkg.perform(context), subpath)))

        kinematics_yaml = _load(moveit_share, "config/kinematics.yaml")
        ompl_yaml = _load(moveit_share, "config/ompl_planning.yaml")
        joint_limits_yaml = _load(moveit_share, "config/joint_limits.yaml")
        moveit_controllers_yaml = _load(moveit_share, "config/moveit_controllers.yaml")

        nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="both",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_yaml,
                    joint_limits_yaml,
                    moveit_controllers_yaml,
                    {"use_sim_time": True},
                    {"publish_robot_description_semantic": True},
                    {"publish_robot_description": True},
                    {"publish_planning_scene": True},
                    {"allow_trajectory_execution": True},
                ],
            )
        )

    # 5. RViz
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("mujoco_gui", default_value="true",
                                  description="If true, shows MuJoCo Simulate window"),
            DeclareLaunchArgument("rviz_gui", default_value="true",
                                  description="If true, launches RViz2"),
            DeclareLaunchArgument("move_group", default_value="true",
                                  description="If true, launches move_group (MoveIt2 planner)"),
            OpaqueFunction(function=launch_setup),
        ]
    )
