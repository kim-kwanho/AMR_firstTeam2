import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get packages paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_storagy = get_package_share_directory('storagy')

    # Set resource path for Gazebo Sim to find meshes (model://storagy/...)
    install_share_path = os.path.dirname(pkg_storagy)
    models_path = os.path.join(pkg_storagy, 'models')
    worlds_path = os.path.join(pkg_storagy, 'worlds')

    # Update GZ_SIM_RESOURCE_PATH / IGN_GAZEBO_RESOURCE_PATH
    gz_resource_path = (
        install_share_path + os.pathsep + models_path + os.pathsep + worlds_path)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + os.pathsep + gz_resource_path

    ign_resource_path = install_share_path + os.pathsep + models_path + os.pathsep + worlds_path
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        ign_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + ign_resource_path

    # 2. Include Gazebo Sim Server (2026 AMR 교실 월드)
    world_sdf_path = os.path.join(pkg_storagy, 'worlds', '2026_amr.sdf')
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r {world_sdf_path}'}.items()
    )

    # 3. Include Gazebo Sim GUI (client only)
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g'}.items()
    )

    # 4. Include Robot State Publisher (using simulation time)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_storagy, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 5. Spawn Robot Entity in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'storagy',
            '-topic', 'robot_description',
            '-x', '-3.92',
            '-y', '0.12',
            '-z', '0.1',
        ],
        output='screen'
    )

    # 6. Bridge communication between ROS 2 and Gazebo Sim
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/storagy/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/storagy/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/model/storagy/odometry', '/odom'),
            ('/model/storagy/tf', '/tf'),
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/camera/points', '/camera/depth/points')
        ],
        output='screen'
    )

    # 7. Include RViz2
    use_rviz2 = LaunchConfiguration('use_rviz2', default='true')
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_storagy, 'launch', 'rviz2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz2': use_rviz2,
        }.items()
    )

    # 8. Include SLAM (slam_toolbox - online async mapping)
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    use_slam = LaunchConfiguration('use_slam', default='true')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_slam),
    )

    # 9. Include Navigation2
    use_nav2 = LaunchConfiguration('use_nav2', default='false')
    map_path = LaunchConfiguration('map', default=os.path.join(pkg_storagy, 'map', '1206_sim_1.yaml'))
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_storagy, 'launch', 'navigation2', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
        }.items(),
        condition=IfCondition(use_nav2),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz2',
            default_value='true',
            description='Whether to launch RViz2'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Whether to launch slam_toolbox SLAM'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Whether to launch Navigation2'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_storagy, 'map', '1206_sim_1.yaml'),
            description='Full path to map file to load'
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        spawn_entity,
        bridge,
        rviz2,
        slam,
        nav2,
    ])
