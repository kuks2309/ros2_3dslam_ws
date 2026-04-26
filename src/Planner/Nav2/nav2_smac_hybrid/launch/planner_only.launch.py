import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_smac_hybrid')
    default_params = os.path.join(pkg_share, 'config', 'smac_hybrid_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['planner_server'],
            }],
        ),
    ])
