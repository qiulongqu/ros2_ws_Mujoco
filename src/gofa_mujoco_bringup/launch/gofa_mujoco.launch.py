#!/usr/bin/env python3
"""
ABB GoFa CRB15000 MuJoCo + ros2_control Bridge

启动 ros2_control_node，加载 MujocoSystemInterface 插件，
将 MuJoCo 仿真的 GoFa 机械臂接入 ROS2 生态。

使用:
    ros2 launch gofa_mujoco_bringup gofa_mujoco.launch.py
    ros2 launch gofa_mujoco_bringup gofa_mujoco.launch.py headless:=true

发送控制命令:
    ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.2, 0, 0, 0]"

查看关节状态:
    ros2 topic echo /joint_states
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
    pkg_share = FindPackageShare("gofa_mujoco_bringup")

    # Build robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "gofa.ros2_control.urdf.xacro"]),
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    controllers_file = PathJoinSubstitution([pkg_share, "config", "gofa_controllers.yaml"])

    nodes = []

    # Robot state publisher — publishes TF from robot_description
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # ros2_control_node with MuJoCo system interface
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
            remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            on_exit=Shutdown(),
        )
    )

    # Controller spawners
    controllers_to_spawn = ["joint_state_broadcaster", "position_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controllers_file],
                output="both",
            )
        )

    return nodes


def generate_launch_description():
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )

    return LaunchDescription(
        [
            headless,
            OpaqueFunction(function=launch_setup),
        ]
    )
