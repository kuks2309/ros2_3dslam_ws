import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'Gazebo')
    world_file = os.path.join(pkg_src, 'worlds', 'my_world.sdf')
    models_path = os.path.join(pkg_src, 'models')
    rviz_config = os.path.join(pkg_src, 'rviz2', 'gazebo.rviz')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file'
    )

    odom_tf_arg = DeclareLaunchArgument(
        'odom_tf',
        default_value='true',
        description='Enable odom to base_link TF publishing'
    )

    # Set IGN_GAZEBO_RESOURCE_PATH for Ignition Gazebo to find models
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path
    )

    # Ignition Gazebo (Fortress)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': models_path}
    )

    # ROS-Gazebo Bridge
    # Bridge: cmd_vel (ROS2→Gazebo), clock/odom/scan/camera (Gazebo→ROS2)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Odom to TF: Publish odom -> base_link TF from /odom topic
    odom_to_tf_script = os.path.join(pkg_src, 'scripts', 'odom_to_tf.py')
    odom_to_tf = ExecuteProcess(
        cmd=['python3', odom_to_tf_script],
        output='screen',
        condition=IfCondition(LaunchConfiguration('odom_tf'))
    )

    # Static TF: base_footprint -> base_link (z=0.16 per SDF)
    static_tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_to_base',
        arguments=['0', '0', '0.16', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static TF: base_link -> lidar_link (lidar is 0.19m above base_link per SDF)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.19', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static TF: base_link -> camera_link (camera is 0.25m forward, 0.10m up from base_link)
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.25', '0', '0.10', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static TF: camera_link -> camera_color_optical_frame
    # Converts from ROS convention (X forward) to optical convention (Z forward)
    # Roll=-90deg, Pitch=0, Yaw=-90deg
    static_tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # rqt_robot_steering for manual control
    rqt_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        world_arg,
        odom_tf_arg,
        gazebo,
        bridge,
        odom_to_tf,
        static_tf_footprint_to_base,
        static_tf_lidar,
        static_tf_camera,
        static_tf_camera_optical,
        rviz,
        rqt_steering,
    ])
