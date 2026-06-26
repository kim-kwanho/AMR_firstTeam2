from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz2 = LaunchConfiguration("use_rviz2", default="true")
    map_file = LaunchConfiguration("map")

    ROBOT_MODEL = "storagy"

    this_launch_file_dir = FindPackageShare(ROBOT_MODEL)

    # Launch
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        this_launch_file_dir,
                        "launch",
                        "robot_state_publisher.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
        }.items(),
    )

    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        this_launch_file_dir,
                        "launch",
                        "hardware_bringup.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        this_launch_file_dir,
                        "launch",
                        "navigation2",
                        "navigation2.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
        }.items(),
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_launch_file_dir, "launch", "rviz2.launch.py"])]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz2": use_rviz2,
        }.items(),
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
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [this_launch_file_dir, "map", "1206_sim_1.yaml"]
                ),
                description="Full path to map file to load",
            ),
            # Node
            navigation2,
            robot_state_publisher,
            robot_hardware,
            rviz2,
        ]
    )
