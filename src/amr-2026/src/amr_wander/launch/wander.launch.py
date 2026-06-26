from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_wander',
            executable='wander',
            name='wander_node',
            output='screen',
            parameters=[{
                'safe_distance': 0.6,
                'linear_speed': 0.15,
                'angular_speed': 0.5,
                'front_angle_deg': 30.0,
            }],
            # Remap here if your robot uses different topic names.
            remappings=[
                ('scan', '/scan'),
                ('cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
