"""
Smac Hybrid-A* planner experiment over the existing RTAB-Map 3D localization.

Assumptions (verified at design time):
  * RTAB-Map publishes /rtabmap/map (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL)
  * TF chain map -> odom_rtabmap -> base_footprint -> lidar_link is published
  * /scan (sensor_msgs/LaserScan) is bridged from Gazebo

Launches only:
  * planner_server with global_costmap (static_layer remapped to /rtabmap/map)
  * lifecycle_manager to autostart planner_server

Trigger a path:
  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \\
      "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0},
        orientation: {w: 1.0}}}, planner_id: 'GridBased'}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_smac_hybrid')
    default_params = os.path.join(
        pkg_share, 'config', 'smac_hybrid_localization_params.yaml'
    )

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
            name='lifecycle_manager_planner_test',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['planner_server'],
            }],
        ),
    ])
