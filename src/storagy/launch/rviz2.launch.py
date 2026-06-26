from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    use_rviz2 = LaunchConfiguration("use_rviz2", default="true")

    ROBOT_MODEL = "storagy"
    rviz2_config_file = "storagy.rviz"

    rviz2_config_dir = os.path.join(
        get_package_share_directory(ROBOT_MODEL), "rviz", rviz2_config_file
    )

    # Launch
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config_dir, "--ros-args", "--log-level", "ERROR"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=LaunchConfigurationEquals("use_rviz2", "true"),
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "use_rviz2", default_value=use_rviz2, description="Use rviz2 if true"
            ),
            # Launch
            rviz2,
        ]
    )
