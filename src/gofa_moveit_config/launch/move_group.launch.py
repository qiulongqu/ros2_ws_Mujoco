#!/usr/bin/env python3
"""
MoveIt2 MoveGroup Launch for GoFa CRB15000

Usage (requires MuJoCo bridge already running):
    ros2 launch gofa_moveit_config move_group.launch.py

Then in another terminal:
    rviz2 -d <path>/moveit.rviz
"""

import os
import subprocess
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def xacro_to_urdf(path):
    result = subprocess.run(["xacro", path], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed:\n{result.stderr}")
    return result.stdout


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("gofa_mujoco_bringup")
    moveit_share = get_package_share_directory("gofa_moveit_config")

    urdf_path = os.path.join(bringup_share, "urdf", "gofa.ros2_control.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "gofa.srdf")
    kinematics_path = os.path.join(moveit_share, "config", "kinematics.yaml")
    controllers_path = os.path.join(moveit_share, "config", "moveit_controllers.yaml")

    robot_description = {"robot_description": xacro_to_urdf(urdf_path)}
    robot_description_semantic = {"robot_description_semantic": open(srdf_path).read()}
    robot_description_kinematics = load_yaml(kinematics_path)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            load_yaml(controllers_path),
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
