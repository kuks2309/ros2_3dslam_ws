from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('hough_thresh',    default_value='30'),
        DeclareLaunchArgument('min_line_len_m',  default_value='0.8'),
        DeclareLaunchArgument('max_line_gap_m',  default_value='0.15'),
        DeclareLaunchArgument('scan_topic',      default_value='scan'),
        DeclareLaunchArgument('output_topic',    default_value='wall_detector/longest_wall'),

        Node(
            package='wall_detector',
            executable='wall_detector_node',
            name='wall_detector',
            output='screen',
            parameters=[{
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'hough_thresh':    LaunchConfiguration('hough_thresh'),
                'min_line_len_m':  LaunchConfiguration('min_line_len_m'),
                'max_line_gap_m':  LaunchConfiguration('max_line_gap_m'),
                'scan_topic':      LaunchConfiguration('scan_topic'),
                'output_topic':    LaunchConfiguration('output_topic'),
                'use_sim_time':    True,
            }],
        ),
    ])
