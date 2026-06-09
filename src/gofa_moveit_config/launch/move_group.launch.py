#!/usr/bin/env python3
"""
MoveIt2 MoveGroup Launch for GoFa CRB15000

启动 move_group 节点,接管规划请求。需要 MuJoCo + ros2_control + RViz
(ros2_control_node + controllers) 已经在另一个进程里运行。

P5 修复: 加载 ompl_planning.yaml + joint_limits.yaml, xacro 解析风格
与 demo.launch.py 统一 (Command + FindExecutable + PathJoinSubstitution)

用法:
    ros2 launch gofa_mujoco_bringup gofa_mujoco.launch.py &
    ros2 launch gofa_moveit_config move_group.launch.py
    rviz2 -d <path>/moveit.rviz
"""

import os

import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    bringup_share = FindPackageShare("gofa_mujoco_bringup")
    moveit_share = FindPackageShare("gofa_moveit_config")

    # 与 demo.launch.py 统一: 用 launch_ros.substitutions 而不是 subprocess
    # robot_description 通过 xacro 解析
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([bringup_share, "urdf", "gofa.ros2_control.urdf.xacro"]),
            " headless:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    # semantic + kinematics + planning + limits + controllers
    robot_description_semantic = {
        "robot_description_semantic": open(
            os.path.join(moveit_share.perform(context), "config", "gofa.srdf")
        ).read()
    }

    kinematics_yaml = load_yaml(
        os.path.join(moveit_share.perform(context), "config", "kinematics.yaml")
    )
    ompl_yaml = load_yaml(
        os.path.join(moveit_share.perform(context), "config", "ompl_planning.yaml")
    )
    joint_limits_yaml = load_yaml(
        os.path.join(moveit_share.perform(context), "config", "joint_limits.yaml")
    )
    moveit_controllers_yaml = load_yaml(
        os.path.join(moveit_share.perform(context), "config", "moveit_controllers.yaml")
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
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

    return [move_group_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
