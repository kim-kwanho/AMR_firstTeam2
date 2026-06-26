from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_driver2',
            # namespace='marker_docking',
            executable='motor_driver2',
            name='motor_driver2',

        )
    ])

