import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_smac_hybrid')

    default_params = os.path.join(pkg_share, 'config', 'smac_hybrid_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_map_server = LaunchConfiguration('use_map_server')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically configure and activate lifecycle nodes'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameters file'
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to map yaml (used only when use_map_server is true)'
    )
    declare_use_map_server = DeclareLaunchArgument(
        'use_map_server', default_value='false',
        description='Whether this launch should also start map_server'
    )

    lifecycle_nodes = [
        'planner_server',
        'smoother_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    map_lifecycle_nodes = ['map_server']

    common_params = [params_file, {'use_sim_time': use_sim_time}]

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('cmd_vel', 'cmd_vel_nav')],
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_params,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    map_server_node = Node(
        condition=IfCondition(use_map_server),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    lifecycle_manager_map = Node(
        condition=IfCondition(use_map_server),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': map_lifecycle_nodes,
        }],
    )

    nav_group = GroupAction([
        planner_node,
        smoother_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        waypoint_node,
        velocity_smoother_node,
        lifecycle_manager_navigation,
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params,
        declare_map,
        declare_use_map_server,
        map_server_node,
        lifecycle_manager_map,
        nav_group,
    ])
