import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Localization Mode Launch File (Gazebo Simulation)
    - Localizes the robot against a pre-built map
    - Loads a global map PCD file for localization
    - Gazebo provides robot state publishing and base_link->sensor TFs via URDF/plugins
    - use_sim_time is enabled by default for Gazebo compatibility
    """

    share_dir = get_package_share_directory('livox_lio_sam')
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    parameter_file = LaunchConfiguration('params_file')
    rviz_config_file = os.path.join(src_dir, 'rviz2', 'lio_sam.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params_localization_gazebo.yaml'),
        description='Path to the ROS2 parameters file for Localization mode.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (for Gazebo simulation)')

    global_map_path_declare = DeclareLaunchArgument(
        'global_map_path',
        default_value=os.path.expanduser('~/Study/ros2_3dslam_ws/maps/lio_sam/GlobalMap.pcd'),
        description='Path to the pre-built global map PCD file')

    print("======================================")
    print("  LIO-SAM Localization Mode (Gazebo Simulation)")
    print("  Loading pre-built map...")
    print("======================================")

    return LaunchDescription([
        params_declare,
        use_sim_time_declare,
        global_map_path_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'),
        Node(package='livox_lio_sam', executable='livox_lio_sam_imuPreintegration',
             name='livox_lio_sam_imuPreintegration',
             parameters=[parameter_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),
        Node(package='livox_lio_sam', executable='livox_lio_sam_imageProjection',
             name='livox_lio_sam_imageProjection',
             parameters=[parameter_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),
        Node(package='livox_lio_sam', executable='livox_lio_sam_featureExtraction',
             name='livox_lio_sam_featureExtraction',
             parameters=[parameter_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),
        Node(package='livox_lio_sam', executable='livox_lio_sam_mapOptimization',
             name='livox_lio_sam_mapOptimization',
             parameters=[parameter_file, {'use_sim_time': LaunchConfiguration('use_sim_time'),
                                          'globalMapPath': LaunchConfiguration('global_map_path')}],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config_file],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen')
    ])
