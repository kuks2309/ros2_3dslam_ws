#!/usr/bin/env python3
"""
SLAM Manager 3D Node - ROS2 node for managing 3D SLAM processes

This module handles:
- Process management for 3D SLAM launch files (RTAB-Map 3D)
- ROS2 node with 3D odometry subscriptions
- 6-DOF pose tracking (x, y, z, roll, pitch, yaw)
"""

import os
import subprocess
import signal
import time
import tempfile
import threading
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import math
from mapper_interfaces.srv import SlamControl


def get_workspace_path():
    """Detect workspace path from slam_manager_3d package installation"""
    try:
        pkg_path = get_package_share_directory('slam_manager_3d')
        workspace = Path(pkg_path).parents[3]
        return workspace
    except Exception:
        return Path.home()


def quaternion_to_euler(q):
    """Convert quaternion to roll, pitch, yaw (radians)

    Args:
        q: Quaternion with x, y, z, w attributes

    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ProcessTracker:
    """Track a detached process by PID"""
    def __init__(self, pid):
        self.pid = pid

    def poll(self):
        """Check if process is still running"""
        try:
            os.kill(self.pid, 0)
            return None  # Still running
        except OSError:
            return 0  # Process ended


# QoS Profiles
SENSOR_DATA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class SlamManager3DNode(Node):
    """ROS2 Node for 3D SLAM Manager"""

    def __init__(self, ui_window=None):  # None 허용 → headless 모드
        super().__init__('slam_manager_3d_node')
        self.ui = ui_window  # None이면 headless 모드

        # Dictionary to store running processes
        self.processes = {
            'gazebo': None,
            'motion_control': None,
            'orbbec_camera': None,
            'rgbd_mapping': None,
            'rgbd_loc': None,
            'rgbd_lidar_mapping': None,
            'rgbd_lidar_loc': None,
            'lio_sam_mapping': None,
            'lio_sam_loc': None,
            'rtabmap_3dlidar_mapping': None,
            'rtabmap_3dlidar_loc': None,
        }

        # Store launch file paths
        self.launch_files = {
            'gazebo': None,
            'motion_control': None,
            'orbbec_camera': None,
            'rgbd_mapping': None,
            'rgbd_loc': None,
            'rgbd_lidar_mapping': None,
            'rgbd_lidar_loc': None,
            'lio_sam_mapping': None,
            'lio_sam_loc': None,
            'rtabmap_3dlidar_mapping': None,
            'rtabmap_3dlidar_loc': None,
        }

        # Workspace path
        self.workspace_path = get_workspace_path()

        # Current 3D position (6-DOF) - RTAB-Map
        self.rtabmap3d_x = 0.0
        self.rtabmap3d_y = 0.0
        self.rtabmap3d_z = 0.0
        self.rtabmap3d_roll = 0.0
        self.rtabmap3d_pitch = 0.0
        self.rtabmap3d_yaw = 0.0

        # Current 3D position (6-DOF) - LIO-SAM
        self.liosam_x = 0.0
        self.liosam_y = 0.0
        self.liosam_z = 0.0
        self.liosam_roll = 0.0
        self.liosam_pitch = 0.0
        self.liosam_yaw = 0.0

        # QoS for odometry subscription
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # RTAB-Map 3D odometry subscriber
        self.rtabmap3d_pose_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self._rtabmap3d_odom_callback,
            odom_qos
        )

        # LIO-SAM odometry subscriber
        self.liosam_pose_sub = self.create_subscription(
            Odometry,
            '/lio_sam/mapping/odometry',
            self._liosam_odom_callback,
            odom_qos
        )

        # SlamControl service server
        self._slam_control_srv = self.create_service(
            SlamControl,
            'slam_manager_3d/slam_control',
            self._handle_slam_control
        )

        self.get_logger().info('SLAM Manager 3D Node initialized')

    def _log(self, msg: str):
        """UI가 있으면 UI에, 없으면 ROS logger에 로그"""
        if self.ui:
            self.ui.log(msg)
        else:
            self.get_logger().info(msg)

    def _handle_slam_control(self, request, response):
        """SlamControl.srv 핸들러"""
        if request.command == SlamControl.Request.CMD_START_3D:
            thread = threading.Thread(
                target=self._start_rtabmap3d_async,
                daemon=True
            )
            thread.start()
            response.success = True
            response.message = "rtabmap3d start initiated"

        elif request.command == SlamControl.Request.CMD_STOP:
            thread = threading.Thread(
                target=self._stop_rtabmap3d_async,
                daemon=True
            )
            thread.start()
            response.success = True
            response.message = "rtabmap3d stop initiated"

        elif request.command == SlamControl.Request.CMD_SAVE_MAP:
            map_name = request.map_name if request.map_name else ""
            save_dir = request.save_directory if request.save_directory else ""
            self._log(f"save_map called: {map_name}, {save_dir}")
            response.success = False
            response.saved_path = ""
            response.message = "Map save not implemented for 3D"

        else:
            response.success = False
            response.message = f"Unknown command: {request.command}"

        return response

    def _start_rtabmap3d_async(self):
        """thread 내부에서 실행 — blocking sleep 허용"""
        try:
            self.start_launch_file(
                'rgbd_mapping',
                'rtabmap_ros',
                'rtabmap.launch.py'
            )
        except Exception as e:
            self.get_logger().error(f"rtabmap3d start failed: {e}")

    def _stop_rtabmap3d_async(self):
        """thread 내부에서 실행"""
        try:
            self.stop_launch_file('rgbd_mapping')
        except Exception as e:
            self.get_logger().error(f"rtabmap3d stop failed: {e}")

    def _rtabmap3d_odom_callback(self, msg):
        """Handle RTAB-Map 3D odometry messages (RGB-D, RGB-D+LiDAR, 3D LiDAR)"""
        self.rtabmap3d_x = msg.pose.pose.position.x
        self.rtabmap3d_y = msg.pose.pose.position.y
        self.rtabmap3d_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        self.rtabmap3d_roll, self.rtabmap3d_pitch, self.rtabmap3d_yaw = quaternion_to_euler(q)

        if self.ui:
            self.ui.update_rtabmap3d_position(
                self.rtabmap3d_x, self.rtabmap3d_y, self.rtabmap3d_z,
                self.rtabmap3d_roll, self.rtabmap3d_pitch, self.rtabmap3d_yaw
            )

    def _liosam_odom_callback(self, msg):
        """Handle LIO-SAM odometry messages"""
        self.liosam_x = msg.pose.pose.position.x
        self.liosam_y = msg.pose.pose.position.y
        self.liosam_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        self.liosam_roll, self.liosam_pitch, self.liosam_yaw = quaternion_to_euler(q)

        if self.ui:
            self.ui.update_liosam_position(
                self.liosam_x, self.liosam_y, self.liosam_z,
                self.liosam_roll, self.liosam_pitch, self.liosam_yaw
            )

    def _is_gazebo_running(self):
        """Check if Gazebo simulation is running.

        Detection methods:
        1. Check for 'ign gazebo' or 'gz sim' process
        2. Check for /clock topic (Gazebo publishes simulation time)

        Returns:
            bool: True if Gazebo is detected, False otherwise
        """
        try:
            # Method 1: Check for Ignition Gazebo process
            result = subprocess.run(
                ['pgrep', '-f', 'ign gazebo'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return True

            # Method 2: Check for Gazebo Sim (newer versions)
            result = subprocess.run(
                ['pgrep', '-f', 'gz sim'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return True

            # Method 3: Check /clock topic (Gazebo publishes this)
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and '/clock' in result.stdout:
                return True

        except Exception as e:
            self.get_logger().debug(f'Gazebo detection failed: {e}')

        return False

    def _kill_gazebo_odom_to_tf(self):
        """Kill Gazebo's odom_to_tf process to avoid TF duplication with rgbd_odometry.

        When using RTAB-Map with visual odometry (rgbd_odometry), it publishes
        odom -> base_link TF. If Gazebo's odom_to_tf is also running, both publish
        the same TF causing point cloud duplication in RViz.
        """
        try:
            result = subprocess.run(
                ['pkill', '-f', 'odom_to_tf'],
                capture_output=True,
                timeout=5
            )
            if result.returncode == 0:
                self.get_logger().info('Killed Gazebo odom_to_tf (TF conflict prevention)')
                if self.ui:
                    self.ui.log('Gazebo odom_to_tf disabled (using visual odometry TF)')
        except Exception as e:
            self.get_logger().debug(f'odom_to_tf not running or already killed: {e}')

    def start_launch_file(self, process_name, package_name, launch_file, use_sim_time=False, extra_args=None):
        """Start a ROS2 launch file using package name"""
        # Kill Gazebo's odom_to_tf when starting RTAB-Map in Gazebo (TF conflict prevention)
        # Only needed for Gazebo simulation where odom_to_tf publishes odom -> base_link
        if process_name in ['rgbd_mapping', 'rgbd_loc', 'rgbd_lidar_mapping', 'rgbd_lidar_loc',
                           'lio_sam_mapping', 'lio_sam_loc']:
            if self._is_gazebo_running():
                self._kill_gazebo_odom_to_tf()

        if self.processes.get(process_name) is not None:
            if self.ui:
                self.ui.log(f"Launch '{process_name}' is already running!")
            self.get_logger().warn(f'{process_name} is already running')
            return False

        if not package_name or not launch_file:
            if self.ui:
                self.ui.log(f"Launch not configured: {process_name}")
            self.get_logger().error(f"Launch not configured: {process_name}")
            return False

        try:
            # Build command
            cmd = ['ros2', 'launch', package_name, launch_file]

            if use_sim_time:
                cmd.extend(['use_sim_time:=true'])

            if extra_args:
                if isinstance(extra_args, str):
                    cmd.extend(extra_args.split())
                else:
                    cmd.extend(extra_args)

            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'

            # PID tracking
            pid_file = tempfile.NamedTemporaryFile(mode='w', suffix='.pid', delete=False)
            pid_file_path = pid_file.name
            pid_file.close()

            # Create launch script
            script_content = f"""#!/bin/bash
set -e
source {self.workspace_path}/install/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
echo $$ > {pid_file_path}
cd {os.path.expanduser('~')}
exec {' '.join(cmd)}
"""
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                script_path = f.name
                f.write(script_content)

            os.chmod(script_path, 0o755)

            process = subprocess.Popen(
                ['setsid', 'bash', script_path],
                env=env,
                stdin=subprocess.DEVNULL,
                cwd=os.path.expanduser('~'),
                preexec_fn=os.setpgrp
            )

            time.sleep(0.5)

            actual_pid = None
            try:
                with open(pid_file_path, 'r') as f:
                    actual_pid = int(f.read().strip())
            except:
                actual_pid = process.pid

            # Cleanup temp files
            def cleanup_files():
                time.sleep(5)
                try:
                    os.unlink(script_path)
                    os.unlink(pid_file_path)
                except:
                    pass
            threading.Thread(target=cleanup_files, daemon=True).start()

            self.processes[process_name] = ProcessTracker(actual_pid)
            self.launch_files[process_name] = (package_name, launch_file)

            if self.ui:
                self.ui.log(f"Started: {package_name} / {launch_file}")
                if use_sim_time or extra_args:
                    args_str = []
                    if use_sim_time:
                        args_str.append('use_sim_time:=true')
                    if extra_args:
                        if isinstance(extra_args, str):
                            args_str.append(extra_args)
                        else:
                            args_str.extend(extra_args)
                    self.ui.log(f"  args: {' '.join(args_str)}")

            self.get_logger().info(f"Started {process_name}: PID={actual_pid}")
            return True

        except Exception as e:
            if self.ui:
                self.ui.log(f"Failed to start: {str(e)}")
            self.get_logger().error(f"Failed to start {process_name}: {str(e)}")
            return False

    def _kill_rtabmap_processes(self):
        """Kill all RTAB-Map related processes directly.

        ros2 launch spawns child processes that may not be killed when
        the parent launch process is terminated. This method kills them directly.
        """
        self.get_logger().info('Killing RTAB-Map related processes...')

        # Method 1: Kill by exact executable names
        # Note: Do NOT kill static_transform_publisher globally
        # - In Gazebo mode: static_transform_publishers are from Gazebo bridge (keep them)
        # - In real robot mode: they are killed via process group termination
        executables_to_kill = [
            'rgbd_odometry',       # Visual odometry node
            'rtabmap',             # Main RTAB-Map SLAM node
        ]

        for exe in executables_to_kill:
            try:
                # Use pkill with exact executable name match
                result = subprocess.run(
                    ['pkill', '-9', exe],
                    capture_output=True,
                    timeout=3
                )
                if result.returncode == 0:
                    self.get_logger().info(f'Killed: {exe}')
            except Exception:
                pass

        # Method 2: Kill by command line pattern (-f flag)
        patterns_to_kill = [
            'rtabmap_odom',        # Package name pattern
            'rtabmap_slam',        # Package name pattern
            'ros2 launch.*rtabmap',  # ros2 launch processes
        ]

        for pattern in patterns_to_kill:
            try:
                subprocess.run(
                    ['pkill', '-9', '-f', pattern],
                    capture_output=True,
                    timeout=3
                )
            except Exception:
                pass

        # Method 3: Kill RViz (only if rtabmap rviz config)
        try:
            # Find rviz2 processes using rtabmap config
            result = subprocess.run(
                ['pgrep', '-f', 'rviz2'],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        try:
                            # Check if this rviz is using rtabmap config
                            cmdline_result = subprocess.run(
                                ['cat', f'/proc/{pid}/cmdline'],
                                capture_output=True,
                                text=True,
                                timeout=2
                            )
                            if 'rtabmap' in cmdline_result.stdout:
                                os.kill(int(pid), signal.SIGKILL)
                                self.get_logger().info(f'Killed RViz PID: {pid}')
                        except Exception:
                            pass
        except Exception:
            pass

        # Method 4: Clean up using ros2 daemon reset (clears node cache)
        try:
            subprocess.run(
                ['ros2', 'daemon', 'stop'],
                capture_output=True,
                timeout=5
            )
            subprocess.run(
                ['ros2', 'daemon', 'start'],
                capture_output=True,
                timeout=5
            )
            self.get_logger().info('ROS2 daemon restarted')
        except Exception:
            pass

        time.sleep(0.5)
        self.get_logger().info('RTAB-Map process cleanup complete')

    def _kill_liosam_processes(self):
        """Kill all LIO-SAM related processes directly."""
        self.get_logger().info('Killing LIO-SAM related processes...')

        executables_to_kill = [
            'livox_lio_sam_imuPreintegration',
            'livox_lio_sam_imageProjection',
            'livox_lio_sam_featureExtraction',
            'livox_lio_sam_mapOptimization',
        ]

        for exe in executables_to_kill:
            try:
                result = subprocess.run(
                    ['pkill', '-9', '-f', exe],
                    capture_output=True,
                    timeout=3
                )
                if result.returncode == 0:
                    self.get_logger().info(f'Killed: {exe}')
            except Exception:
                pass

        # Kill ros2 launch processes for LIO-SAM
        try:
            subprocess.run(
                ['pkill', '-9', '-f', 'ros2 launch.*livox_lio_sam'],
                capture_output=True,
                timeout=3
            )
        except Exception:
            pass

        # Kill RViz with lio_sam config
        try:
            result = subprocess.run(
                ['pgrep', '-f', 'rviz2'],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        try:
                            cmdline_result = subprocess.run(
                                ['cat', f'/proc/{pid}/cmdline'],
                                capture_output=True,
                                text=True,
                                timeout=2
                            )
                            if 'lio_sam' in cmdline_result.stdout:
                                os.kill(int(pid), signal.SIGKILL)
                                self.get_logger().info(f'Killed RViz PID: {pid}')
                        except Exception:
                            pass
        except Exception:
            pass

        # ROS2 daemon restart
        try:
            subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True, timeout=5)
            subprocess.run(['ros2', 'daemon', 'start'], capture_output=True, timeout=5)
        except Exception:
            pass

        time.sleep(0.5)
        self.get_logger().info('LIO-SAM process cleanup complete')

    def _kill_gazebo_processes(self):
        """Kill all Gazebo related processes."""
        self.get_logger().info('Killing Gazebo related processes...')

        patterns_to_kill = [
            'ign gazebo',
            'gz sim',
            'parameter_bridge',
            'odom_to_tf',
            'rqt_robot_steering',
        ]

        for pattern in patterns_to_kill:
            try:
                result = subprocess.run(
                    ['pkill', '-9', '-f', pattern],
                    capture_output=True,
                    timeout=3
                )
                if result.returncode == 0:
                    self.get_logger().info(f'Killed: {pattern}')
            except Exception:
                pass

        # Kill static_transform_publishers from Gazebo launch
        try:
            subprocess.run(
                ['pkill', '-9', '-f', 'ros2 launch.*tm_gazebo'],
                capture_output=True,
                timeout=3
            )
        except Exception:
            pass

        # Kill RViz with gazebo config
        try:
            result = subprocess.run(
                ['pgrep', '-f', 'rviz2'],
                capture_output=True,
                text=True,
                timeout=3
            )
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        try:
                            cmdline_result = subprocess.run(
                                ['cat', f'/proc/{pid}/cmdline'],
                                capture_output=True,
                                text=True,
                                timeout=2
                            )
                            if 'gazebo' in cmdline_result.stdout.lower():
                                os.kill(int(pid), signal.SIGKILL)
                                self.get_logger().info(f'Killed Gazebo RViz PID: {pid}')
                        except Exception:
                            pass
        except Exception:
            pass

        time.sleep(1.0)
        self.get_logger().info('Gazebo process cleanup complete')

    def stop_launch_file(self, process_name):
        """Stop a running launch file process and all child processes"""
        process = self.processes.get(process_name)
        if process is None:
            self.get_logger().warn(f'{process_name} is not running')
            return False

        self.get_logger().info(f'Stopping {process_name}...')

        try:
            # Get PID
            if isinstance(process, ProcessTracker):
                pid = process.pid
            else:
                pid = process.pid

            # Method 1: Send SIGINT first (ros2 launch graceful shutdown)
            try:
                os.kill(pid, signal.SIGINT)
                self.get_logger().info(f'Sent SIGINT to PID {pid}')
            except Exception:
                pass

            time.sleep(1)

            # Get session ID (setsid creates new session)
            try:
                sid = os.getsid(pid)
                self.get_logger().info(f'Session ID: {sid}')
            except Exception:
                sid = None

            # Method 2: Kill by session ID (most effective for setsid processes)
            if sid:
                try:
                    subprocess.run(
                        ['pkill', '-INT', '-s', str(sid)],
                        timeout=5
                    )
                    self.get_logger().info(f'Sent SIGINT to session {sid}')
                except Exception as e:
                    self.get_logger().warn(f'pkill failed: {e}')

            time.sleep(1)

            # Method 3: Kill process group
            try:
                pgid = os.getpgid(pid)
                os.killpg(pgid, signal.SIGTERM)
                self.get_logger().info(f'Sent SIGTERM to process group {pgid}')
            except Exception:
                pass

            time.sleep(1)

            # Method 4: Force kill session if still running
            if sid:
                try:
                    subprocess.run(
                        ['pkill', '-KILL', '-s', str(sid)],
                        timeout=5
                    )
                    self.get_logger().info(f'Sent SIGKILL to session {sid}')
                except Exception:
                    pass

            # Method 5: Force kill process group
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except Exception:
                pass

            # Method 6: Kill main process directly
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass

            # Method 7: Kill child processes directly (ros2 launch child processes)
            if process_name in ['rgbd_mapping', 'rgbd_loc', 'rgbd_lidar_mapping', 'rgbd_lidar_loc',
                                'rtabmap_3dlidar_mapping', 'rtabmap_3dlidar_loc']:
                self._kill_rtabmap_processes()
                self.get_logger().info('Killed RTAB-Map child processes')
            elif process_name in ['lio_sam_mapping', 'lio_sam_loc']:
                self._kill_liosam_processes()
                self.get_logger().info('Killed LIO-SAM child processes')
            elif process_name == 'gazebo':
                self._kill_gazebo_processes()
                self.get_logger().info('Killed Gazebo child processes')

            self.processes[process_name] = None
            self.get_logger().info(f'{process_name} stopped')

            if self.ui:
                self.ui.log(f"Stopped: {process_name}")

            return True

        except Exception as e:
            self.get_logger().error(f'Error stopping {process_name}: {str(e)}')
            self.processes[process_name] = None
            return False

    def is_process_running(self, process_name):
        """Check if a process is running"""
        process = self.processes.get(process_name)
        if process is None:
            return False
        if isinstance(process, ProcessTracker):
            return process.poll() is None
        return process.poll() is None

    def cleanup(self):
        """Stop all running processes"""
        for process_name in list(self.processes.keys()):
            if self.is_process_running(process_name):
                self.stop_launch_file(process_name)


def main(args=None):
    """Main entry point"""
    import sys
    from PyQt5 import QtWidgets

    rclpy.init(args=args)

    # Import UI class (will be created in TODO 4)
    try:
        from slam_manager_3d.slam_manager_3d_ui import SlamManager3DUI
    except ImportError:
        print("UI module not found. Running node only.")
        node = SlamManager3DNode(None)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.cleanup()
            node.destroy_node()
            rclpy.shutdown()
        return

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create UI
    ui = SlamManager3DUI()
    ui.show()

    # Create node with UI reference
    node = SlamManager3DNode(ui)
    ui.set_node(node)

    # ROS2 spin in Qt timer
    from PyQt5.QtCore import QTimer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
