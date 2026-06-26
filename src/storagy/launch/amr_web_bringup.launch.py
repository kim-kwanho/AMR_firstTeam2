#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    storagy_share = get_package_share_directory("storagy")
    ws_root = os.path.abspath(os.path.join(storagy_share, "..", "..", "..", ".."))
    scripts_dir = os.path.join(ws_root, "src", "storagy", "scripts")
    venv_python = os.path.join(ws_root, ".venv", "bin", "python")
    python_executable = venv_python if os.path.exists(venv_python) else "python3"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz2 = LaunchConfiguration("use_rviz2")
    enable_yolo = LaunchConfiguration("enable_yolo")
    enable_agent = LaunchConfiguration("enable_agent")
    enable_web = LaunchConfiguration("enable_web")
    map_file = LaunchConfiguration("map")
    web_map = LaunchConfiguration("web_map")

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(storagy_share, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz2": use_rviz2,
            "map": map_file,
        }.items(),
    )

    yolo_detector = ExecuteProcess(
        cmd=[python_executable, os.path.join(scripts_dir, "yolo_detector.py")],
        cwd=ws_root,
        output="screen",
        condition=IfCondition(enable_yolo),
    )

    agent_service = Node(
        package="storagy_llm",
        executable="agent_service",
        prefix=python_executable,
        output="screen",
        condition=IfCondition(enable_agent),
    )

    web_dashboard = Node(
        package="storagy_llm",
        executable="web_dashboard",
        prefix=python_executable,
        parameters=[
            {"map_file": web_map},
            {"map_offset_x": -5.9},
            {"map_offset_y": -0.6},
        ],
        output="screen",
        condition=IfCondition(enable_web),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz2", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(storagy_share, "map", "2026_amr.yaml"),
                description="Map file used by Nav2",
            ),
            DeclareLaunchArgument(
                "web_map",
                default_value=os.path.join(storagy_share, "map", "1206_sim_1.yaml"),
                description="Map file used by the web dashboard",
            ),
            DeclareLaunchArgument("enable_yolo", default_value="true"),
            DeclareLaunchArgument("enable_agent", default_value="true"),
            DeclareLaunchArgument("enable_web", default_value="true"),
            robot_bringup,
            TimerAction(period=3.0, actions=[yolo_detector]),
            TimerAction(period=5.0, actions=[agent_service, web_dashboard]),
        ]
    )
