#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_guide_share = get_package_share_directory('storagy_guide')
    default_params = os.path.join(pkg_guide_share, 'params', 'guide_nav.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    demo_person_arrived = LaunchConfiguration('demo_person_arrived')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_yaw = LaunchConfiguration('target_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('demo_person_arrived', default_value='false'),
        DeclareLaunchArgument('target_x', default_value='1.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        
        # 1. guide_nav_node (FSM Coordinator)
        Node(
            package='storagy_guide',
            executable='guide_nav_node',
            name='guide_nav_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'demo_person_arrived': demo_person_arrived,
                    'target_x': target_x,
                    'target_y': target_y,
                    'target_yaw': target_yaw,
                },
            ],
        ),
        
        # 2. guide_controller (PD controller / Interceptor)
        Node(
            package='storagy_llm',
            executable='guide_controller',
            name='guide_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
