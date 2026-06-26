# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TODO: 수정
MOBILE_MODEL = "storagy"

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory(MOBILE_MODEL),
            'map',
            '1206_sim_1.yaml'))

    param_file_name = MOBILE_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory(MOBILE_MODEL),
            'param',
            'navigation2',
            param_file_name))

    default_nav_to_pose_bt_xml_dir = os.path.join(
            get_package_share_directory(MOBILE_MODEL),
            'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'
            ) 

    default_nav_through_poses_bt_xml_dir = os.path.join(
            get_package_share_directory(MOBILE_MODEL),
            'behavior_trees',
            'navigate_through_poses_w_replanning_and_recovery.xml'
            ) 

    nav2_launch_file_dir = os.path.join(get_package_share_directory(MOBILE_MODEL), 'launch', 'navigation2')

    initial_pose_x = LaunchConfiguration('initial_pose_x', default='1.54')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='1.0')
    initial_pose_z = LaunchConfiguration('initial_pose_z', default='0.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.0')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml', default = default_nav_to_pose_bt_xml_dir)
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml', default = default_nav_through_poses_bt_xml_dir)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='1.54',
            description='X coordinate of the initial robot pose in the map frame"'),

        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='1.0',
            description='Y coordinate of the initial robot pose in the map frame"'),

        DeclareLaunchArgument(
            'initial_pose_z',
            default_value='0.0',
            description='Z coordinate of the initial robot pose in the map frame"'),

        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Yaw of the initial robot pose in the map frame"'),    

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_respawn': 'false',
                'params_file': param_dir,
                'initial_pose_x':initial_pose_x,
                'initial_pose_y':initial_pose_y,
                'initial_pose_z':initial_pose_z,
                'initial_pose_yaw':initial_pose_yaw,
                'default_nav_to_pose_bt_xml' : default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml' : default_nav_through_poses_bt_xml,
                }.items(),
        ),

        # Node(
        #     package='robot_navigation',
        #     executable='robot_navigation',
        #     name='robot_navigation',
        #     output='screen'),

        # Node(
        #     package='robot_cruise',
        #     executable='robot_cruise',
        #     name='robot_cruise',
        #     output='screen'),    

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
