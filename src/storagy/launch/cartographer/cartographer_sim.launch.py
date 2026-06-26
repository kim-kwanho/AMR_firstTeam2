import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_storagy = get_package_share_directory('storagy')
    
    # Path to original cartographer.launch.py
    cartographer_launch_file = os.path.join(
        pkg_storagy, 'launch', 'cartographer', 'cartographer.launch.py'
    )
    
    return LaunchDescription([
        # Run cartographer.launch.py with use_sim_time set to true by default
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_file),
            launch_arguments={
                'use_sim_time': 'true',
            }.items()
        )
    ])
