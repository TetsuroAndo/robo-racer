import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('race_manager')
    frames_config_path = os.path.join(pkg_share, 'config', 'frames.yaml')
    
    lidar_dev_arg = DeclareLaunchArgument('lidar_dev', default_value='/dev/ttyUSB0')
    shm_path_arg = DeclareLaunchArgument('shm_path', default_value='/dev/shm/lidar_scan')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='laser')
    range_min_arg = DeclareLaunchArgument('range_min', default_value='0.1')
    range_max_arg = DeclareLaunchArgument('range_max', default_value='12.0')

    tf_node = Node(package='mc_tf_static', executable='mc_tf_static', name='mc_tf_static', parameters=[frames_config_path])
    bridge_node = Node(package='mc_bridge', executable='mc_bridge', name='mc_bridge')
    lidar_node = Node(
        package='lidar_bridge', executable='lidar_bridge', name='lidar_bridge',
        parameters=[{
            'shm_path': LaunchConfiguration('shm_path'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
        }]
    )
    driver_node = Node(
        package='race_manager', executable='lap_manager', name='race_manager',
        arguments=[LaunchConfiguration('lidar_dev'), '115200', '/tmp/seriald']
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'use_sim_time': 'False'}.items()
    )
    manager_node = Node(package='race_manager', executable='lap_manager', name='lap_manager', output='screen')

    return LaunchDescription([
        LogInfo(msg=f"Using frames config: {frames_config_path}"),
        lidar_dev_arg, shm_path_arg, lidar_frame_arg, range_min_arg, range_max_arg,
        tf_node, bridge_node, lidar_node, driver_node, slam_launch, manager_node
    ])