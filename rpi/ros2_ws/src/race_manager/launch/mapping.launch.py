import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('race_manager')
    frames_config_path = os.path.join(pkg_share, 'config', 'frames.yaml')
    
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
            'range_min': ParameterValue(LaunchConfiguration('range_min'), value_type=float),
            'range_max': ParameterValue(LaunchConfiguration('range_max'), value_type=float),
        }]
    )
    lap_manager_node = Node(
        package='race_manager', executable='lap_manager', name='race_manager'
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'use_sim_time': 'False'}.items()
    )


    return LaunchDescription([
        LogInfo(msg=f"Using frames config: {frames_config_path}"),
        shm_path_arg, lidar_frame_arg, range_min_arg, range_max_arg,
        tf_node, bridge_node, lidar_node, lap_manager_node, slam_launch
    ])
