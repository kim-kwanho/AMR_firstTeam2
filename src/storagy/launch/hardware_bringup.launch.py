import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    front_2d_lidar_config = os.path.join(
        get_package_share_directory("storagy"),
        "config",
        "sick_scan2",
        "sick_tim_5xx_front.yaml",
    )

    front_depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("orbbec_camera"),
                        "launch",
                        "astra_stereo_u3.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "publish_tf": "false",
            "depth_registration": "true",
            "enable_colored_point_cloud": "true",
        }.items(),
    )

    front_2d_lidar = Node(
        package="sick_scan2",
        name="sick_scan2",  # 'sick_scan2_tim_5xx', # For compatibility with ros versions previous to foxy, node name changed to sick_scan2 for all supported scanner. The type of scanner is configured by scanner_name in the yaml config file.
        # node_executable='sick_generic_caller',       # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable".
        executable="sick_generic_caller",  # Please use executable='sick_generic_caller', if ROS2 can't launch sick_generic_caller.
        output="screen",
        parameters=[front_2d_lidar_config],
    )

    motor_driver = Node(
        package="motor_driver2",
        executable="motor_driver2",
        name="motor_driver2",
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            # Launch
            motor_driver,
            front_depth_camera,
            front_2d_lidar,
        ]
    )
