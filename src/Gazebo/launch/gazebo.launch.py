import math
import os
import re
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                             ExecuteProcess, OpaqueFunction,
                             RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _kill_orphan_rqt(context):
    """Kill orphan rqt_robot_steering child on launch shutdown.

    `ros2 run ... rqt_robot_steering` parent dies on SIGINT but its child
    rqt GUI process can outlive the cascade (Qt event loop blocks signals).
    This handler force-kills any remaining rqt_robot_steering on shutdown.
    """
    try:
        subprocess.run(['pkill', '-KILL', '-f', 'rqt_robot_steering'],
                       check=False, timeout=3)
    except Exception:
        pass
    return None


PKG_SRC     = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'Gazebo')
WORLD_FILE  = os.path.join(PKG_SRC, 'worlds', 'my_world.sdf')
MODELS_PATH = os.path.join(PKG_SRC, 'models')
RVIZ_CONFIG = os.path.join(PKG_SRC, 'rviz2', 'gazebo.rviz')
URDF_FILE   = os.path.join(PKG_SRC, 'urdf', 'pioneer2dx.urdf')

# NVIDIA EGL vendor file candidates (distro-dependent path)
_NVIDIA_EGL_VENDOR_CANDIDATES = [
    '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
    '/etc/glvnd/egl_vendor.d/10_nvidia.json',
]
# NVIDIA userspace libraries - presence implies dGPU + driver installed
_NVIDIA_LIB_GLOBS = [
    '/usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.0',
    '/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0',
    '/proc/driver/nvidia/version',
]


def _detect_nvidia_offload() -> tuple:
    """Return (enable: bool, egl_vendor_file: str|None).

    Only enables PRIME offload when NVIDIA userspace is present AND an EGL
    vendor file exists. Prevents breakage on NVIDIA-free systems.
    """
    has_nv_lib = any(os.path.exists(p) for p in _NVIDIA_LIB_GLOBS)
    if not has_nv_lib:
        return (False, None)
    for vf in _NVIDIA_EGL_VENDOR_CANDIDATES:
        if os.path.exists(vf):
            return (True, vf)
    # NVIDIA libs exist but no EGL vendor file — safer to skip
    return (False, None)


def _make_world_with_spawn(context):
    """spawn_x/y/yaw 인자로 SDF의 robot_scan 초기 pose를 치환, 임시 파일 반환."""
    x   = float(context.launch_configurations.get('spawn_x',   '-13.10'))
    y   = float(context.launch_configurations.get('spawn_y',   '2.64'))
    yaw = float(context.launch_configurations.get('spawn_yaw', '0.0'))
    yaw_rad = math.radians(yaw)

    with open(WORLD_FILE) as f:
        sdf = f.read()

    new_pose = f'<pose> {x} {y} 0 0 0 {yaw_rad:.6f} </pose>'
    # robot_scan 모델의 <pose> 한 줄만 교체
    sdf = re.sub(r'<pose>\s*-13\.10\s+2\.64\s+0\s+0\s+0\s+0\s*</pose>',
                 new_pose, sdf)

    tmp = tempfile.NamedTemporaryFile(suffix='.sdf', delete=False, mode='w')
    tmp.write(sdf)
    tmp.flush()

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', tmp.name],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': MODELS_PATH}
    )
    return [gazebo]


def generate_launch_description():
    with open(URDF_FILE) as f:
        robot_desc = f.read()

    # NVIDIA PRIME offload: dGPU 있으면 자동 적용, 없으면 skip (배포 안전).
    # 수동 오버라이드: `ros2 launch ... use_nvidia:=false` 로 끄기.
    nv_enable, nv_egl_vendor = _detect_nvidia_offload()
    actions = [
        # ── 환경 변수 ────────────────────────────────────────────
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', MODELS_PATH),
    ]
    if nv_enable:
        actions.extend([
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable(
                '__EGL_VENDOR_LIBRARY_FILENAMES', nv_egl_vendor),
        ])
        print(f'[gazebo.launch] NVIDIA PRIME offload enabled '
              f'(egl vendor: {nv_egl_vendor})')
    else:
        print('[gazebo.launch] NVIDIA libs not detected — using default GPU')

    return LaunchDescription(actions + [
        # ── Launch 인자 ──────────────────────────────────────────
        DeclareLaunchArgument('odom_tf',   default_value='true',    description='Enable odom→base_link TF'),
        DeclareLaunchArgument('spawn_x',   default_value='-13.10',  description='Robot spawn X (m)'),
        DeclareLaunchArgument('spawn_y',   default_value='2.64',    description='Robot spawn Y (m)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',     description='Robot spawn yaw (deg)'),
        DeclareLaunchArgument('rqt',       default_value='true',    description='Enable rqt_robot_steering GUI'),

        # ── Ignition Gazebo (spawn 위치 반영) ────────────────────
        OpaqueFunction(function=_make_world_with_spawn),

        # ── ROS-Gazebo Bridge ────────────────────────────────────
        Node(
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
            output='screen',
        ),

        # ── Odom → TF ────────────────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(PKG_SRC, 'scripts', 'odom_to_tf.py')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('odom_tf')),
        ),

        # ── Robot State Publisher ────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
            output='screen',
        ),

        # ── Static TF ────────────────────────────────────────────
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_footprint_to_base',
             arguments=['0', '0', '0.16', '0', '0', '0', 'base_footprint', 'base_link'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_lidar',
             arguments=['0', '0', '0.19', '0', '0', '0', 'base_link', 'lidar_link'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_camera',
             arguments=['0.25', '0', '0.10', '0', '0', '0', 'base_link', 'camera_link'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_camera_to_optical',
             arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                        'camera_link', 'camera_color_optical_frame'],
             parameters=[{'use_sim_time': True}], output='screen'),

        # ── RViz2 ────────────────────────────────────────────────
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', RVIZ_CONFIG],
             parameters=[{'use_sim_time': True}], output='screen'),

        # ── rqt_robot_steering ───────────────────────────────────
        # ExecuteProcess + 짧은 SIGTERM→SIGKILL 타임아웃 + OnShutdown pkill
        # 안전망. Qt 이벤트 루프가 SIGINT 를 block 하므로 Node 액션으로
        # 띄우면 종료 시 창이 남는 문제를 방지한다.
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_robot_steering', 'rqt_robot_steering'],
            output='screen',
            sigterm_timeout='2',
            sigkill_timeout='3',
            condition=IfCondition(LaunchConfiguration('rqt')),
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[OpaqueFunction(function=_kill_orphan_rqt)]
            )
        ),
    ])
