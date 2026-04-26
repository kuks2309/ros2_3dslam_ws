import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_utils import setup_gpu_offload


def generate_launch_description():
    # src 폴더 경로 사용 (CLAUDE.md 규칙)
    pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'Gazebo')
    world_file = os.path.join(pkg_src, 'worlds', 'my_world.sdf')
    models_path = os.path.join(pkg_src, 'models')
    rviz_config = os.path.join(pkg_src, 'rviz2', 'gazebo.rviz')
    urdf_file = os.path.join(pkg_src, 'urdf', 'pioneer2dx.urdf')

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
    # Bridge: cmd_vel (ROS2→Gazebo), clock/odom/scan/camera/joint_state (Gazebo→ROS2)
    # joint_state: SDF의 ignition-gazebo-joint-state-publisher-system 플러그인이
    #              /world/my_world/model/robot_scan/joint_state로 발행 → /joint_states로 리맵
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
            '/world/my_world/model/robot_scan/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/my_world/model/robot_scan/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # Odom to TF: Publish odom -> base_link TF from /odom topic
    odom_to_tf_script = os.path.join(pkg_src, 'scripts', 'odom_to_tf.py')
    odom_to_tf = ExecuteProcess(
        cmd=['python3', odom_to_tf_script],
        output='screen',
        condition=IfCondition(LaunchConfiguration('odom_tf'))
    )

    # Robot State Publisher — publishes /robot_description for RViz RobotModel
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # NOTE: /joint_states 는 ros_gz_bridge 가 Gazebo SDF의 JointStatePublisher 플러그인
    # 토픽(/world/my_world/model/robot_scan/joint_state)을 리맵해 발행한다.
    # robot_state_publisher 가 이를 구독해 wheel link TF (continuous joint)를 생성.
    # 별도 joint_state_publisher 노드 불필요.

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
    # Qt 이벤트 루프가 SIGINT를 가로채므로 짧은 timeout + OnShutdown 훅으로 강제 종료 보장
    rqt_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        parameters=[{'use_sim_time': True}],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='3',
    )

    # launch 종료 시 rqt_robot_steering 프로세스가 Qt 트랩으로 남는 경우 pkill로 강제 정리
    rqt_cleanup_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'rqt_robot_steering'],
                shell=False,
            ),
        ]),
    )

    return LaunchDescription([
        *setup_gpu_offload(),
        set_gz_resource_path,
        world_arg,
        odom_tf_arg,
        gazebo,
        bridge,
        odom_to_tf,
        robot_state_publisher,
        rviz,
        rqt_steering,
        rqt_cleanup_on_shutdown,
    ])
