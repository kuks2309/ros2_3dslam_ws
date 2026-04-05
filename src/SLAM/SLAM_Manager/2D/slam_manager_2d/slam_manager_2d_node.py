#!/usr/bin/env python3
"""
SLAM Manager Node - ROS2 node for managing 2D SLAM processes

This module handles:
- Process management for SLAM launch files
- ROS2 node with odometry subscriptions
- ROS Bag playback control
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import math
from mapper_interfaces.srv import SlamControl


def get_workspace_path():
    """Detect workspace path from slam_manager_2d package installation"""
    try:
        # Get slam_manager_2d package path: /path/to/ws/install/slam_manager_2d/share/slam_manager_2d
        pkg_path = get_package_share_directory('slam_manager_2d')
        # Extract workspace: /path/to/ws
        workspace = Path(pkg_path).parents[3]
        return workspace
    except Exception:
        # Fallback to home directory
        return Path.home()


def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle (radians)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


class SlamManagerNode(Node):
    """ROS2 Node for SLAM Manager"""

    def __init__(self, ui_window=None):  # None 허용 → headless 모드
        super().__init__('slam_manager_2d_node')
        self.ui = ui_window  # None이면 headless 모드

        # Dictionary to store running processes
        self.processes = {
            'livox': None,
            'slamtoolbox_mapping': None,
            'slamtoolbox_loc': None,
            'carto_mapping': None,
            'carto_loc': None,
            'kslam': None,
            'rtabmap2d_mapping': None,
            'rtabmap2d_loc': None,
            'rtabmap2d_camera_mapping': None,
            'rtabmap2d_camera_loc': None,
        }

        # Store launch file paths
        self.launch_files = {
            'livox': None,
            'slamtoolbox_mapping': None,
            'slamtoolbox_loc': None,
            'carto_mapping': None,
            'carto_loc': None,
            'kslam': None,
            'rtabmap2d_mapping': None,
            'rtabmap2d_loc': None,
            'rtabmap2d_camera_mapping': None,
            'rtabmap2d_camera_loc': None,
        }

        # Workspace path (auto-detected from package installation)
        self.workspace_path = get_workspace_path()

        # Current positions
        self.slamtoolbox_x = 0.0
        self.slamtoolbox_y = 0.0
        self.slamtoolbox_yaw = 0.0

        self.carto_x = 0.0
        self.carto_y = 0.0
        self.carto_yaw = 0.0

        self.kslam_x = 0.0
        self.kslam_y = 0.0
        self.kslam_yaw = 0.0

        self.rtabmap2d_x = 0.0
        self.rtabmap2d_y = 0.0
        self.rtabmap2d_yaw = 0.0

        self.rtabmap2d_camera_x = 0.0
        self.rtabmap2d_camera_y = 0.0
        self.rtabmap2d_camera_yaw = 0.0

        # QoS profile for odometry
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to SLAM Toolbox pose (it publishes to /pose)
        self.slamtoolbox_pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.slamtoolbox_odom_callback,
            odom_qos
        )

        # Subscribe to Cartographer odometry (when available)
        self.carto_odom_sub = self.create_subscription(
            Odometry,
            '/carto/odom',
            self.carto_odom_callback,
            odom_qos
        )

        # Subscribe to K-SLAM pose
        self.kslam_pose_sub = self.create_subscription(
            Odometry,
            '/kslam/odom',
            self.kslam_odom_callback,
            odom_qos
        )

        # Subscribe to RTAB-Map 2D pose
        self.rtabmap2d_pose_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.rtabmap2d_odom_callback,
            odom_qos
        )

        # SlamControl service server
        self._slam_control_srv = self.create_service(
            SlamControl,
            'slam_manager_2d/slam_control',
            self._handle_slam_control
        )

        self.get_logger().info('SLAM Manager 2D Node initialized')

    def set_use_sim_time(self, enabled: bool):
        """Set use_sim_time parameter"""
        try:
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, enabled)])
            self.get_logger().info(f'use_sim_time set to {enabled}')
        except Exception as e:
            self.get_logger().warn(f'Failed to set use_sim_time: {e}')

    def _log(self, msg: str):
        """UI가 있으면 UI에, 없으면 ROS logger에 로그"""
        if self.ui:
            self.ui.log(msg)
        else:
            self.get_logger().info(msg)

    def slamtoolbox_odom_callback(self, msg):
        """Callback for SLAM Toolbox odometry"""
        self.slamtoolbox_x = msg.pose.pose.position.x
        self.slamtoolbox_y = msg.pose.pose.position.y
        self.slamtoolbox_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.ui and hasattr(self.ui, 'update_slamtoolbox_position'):
            self.ui.update_slamtoolbox_position(
                self.slamtoolbox_x, self.slamtoolbox_y, self.slamtoolbox_yaw)

    def carto_odom_callback(self, msg):
        """Callback for Cartographer odometry"""
        self.carto_x = msg.pose.pose.position.x
        self.carto_y = msg.pose.pose.position.y
        self.carto_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.ui and hasattr(self.ui, 'update_carto_position'):
            self.ui.update_carto_position(
                self.carto_x, self.carto_y, self.carto_yaw)

    def kslam_odom_callback(self, msg):
        """Callback for K-SLAM odometry"""
        self.kslam_x = msg.pose.pose.position.x
        self.kslam_y = msg.pose.pose.position.y
        self.kslam_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.ui and hasattr(self.ui, 'update_kslam_position'):
            self.ui.update_kslam_position(
                self.kslam_x, self.kslam_y, self.kslam_yaw)

    def rtabmap2d_odom_callback(self, msg):
        """Callback for RTAB-Map 2D odometry"""
        self.rtabmap2d_x = msg.pose.pose.position.x
        self.rtabmap2d_y = msg.pose.pose.position.y
        self.rtabmap2d_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.ui and hasattr(self.ui, 'update_rtabmap2d_position'):
            self.ui.update_rtabmap2d_position(
                self.rtabmap2d_x, self.rtabmap2d_y, self.rtabmap2d_yaw)
        # Also update RTAB-Map 2D Camera position (same odom topic)
        self.rtabmap2d_camera_x = self.rtabmap2d_x
        self.rtabmap2d_camera_y = self.rtabmap2d_y
        self.rtabmap2d_camera_yaw = self.rtabmap2d_yaw
        if self.ui and hasattr(self.ui, 'update_rtabmap2d_camera_position'):
            self.ui.update_rtabmap2d_camera_position(
                self.rtabmap2d_camera_x, self.rtabmap2d_camera_y, self.rtabmap2d_camera_yaw)

    def start_launch_file(self, launch_key, package_name, launch_file, extra_args=None):
        """Start a ROS2 launch file using package name"""
        if self.processes[launch_key] is not None:
            self._log(f"Launch '{launch_key}' is already running!")
            return False

        if not package_name or not launch_file:
            self._log(f"Launch not configured: {launch_key}")
            return False

        try:
            cmd = ['ros2', 'launch', package_name, launch_file]

            if extra_args:
                cmd.extend(extra_args)

            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'

            # PID tracking
            pid_file = tempfile.NamedTemporaryFile(mode='w', suffix='.pid', delete=False)
            pid_file_path = pid_file.name
            pid_file.close()

            # Create launch script
            # Add /opt/ros/humble/lib/x86_64-linux-gnu to LD_LIBRARY_PATH for g2o libraries
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

            self.processes[launch_key] = ProcessTracker(actual_pid)
            self._log(f"Started: {package_name} / {launch_file}")
            if extra_args:
                self._log(f"  args: {' '.join(extra_args)}")
            self.get_logger().info(f"Started {launch_key}: PID={actual_pid}")
            return True

        except Exception as e:
            self._log(f"Failed to start: {str(e)}")
            self.get_logger().error(f"Failed to start {launch_key}: {str(e)}")
            return False

    def _get_node_filter_keywords(self, launch_key: str):
        """Get node name filter keywords for a given launch_key"""
        keywords_map = {
            'carto_mapping': ['cartographer'],
            'carto_loc': ['cartographer'],
            'rtabmap2d_mapping': ['rtabmap', 'icp_odometry', 'rgbd_odometry'],
            'rtabmap2d_loc': ['rtabmap', 'icp_odometry', 'rgbd_odometry'],
            'rtabmap2d_camera_mapping': ['rtabmap', 'icp_odometry', 'rgbd_odometry'],
            'rtabmap2d_camera_loc': ['rtabmap', 'icp_odometry', 'rgbd_odometry'],
            'slamtoolbox_mapping': ['slam_toolbox', 'async_slam'],
            'slamtoolbox_loc': ['slam_toolbox', 'async_slam'],
            'kslam': ['kslam', 'k_slam'],
        }
        return keywords_map.get(launch_key, [])

    def _extract_process_pattern(self, node_name: str) -> str:
        """Extract process pattern from ROS2 node name.

        Example: /rtabmap/rtabmap -> rtabmap
                 /cartographer_node -> cartographer_node
        """
        if not node_name:
            return ''
        return node_name.split('/')[-1]

    def _get_running_ros2_nodes(self):
        """Get list of currently running ROS2 nodes.

        Returns list of node names (e.g., ['/rtabmap/rtabmap', '/cartographer_node'])
        """
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                nodes = [line.strip() for line in result.stdout.split('\n') if line.strip()]
                return nodes
            else:
                self.get_logger().info(f"ros2 node list returned non-zero: {result.stderr}")
                return []
        except subprocess.TimeoutExpired:
            self.get_logger().info("ros2 node list timed out")
            return []
        except Exception as e:
            self.get_logger().info(f"Failed to get node list: {e}")
            return []

    def _cleanup_remaining_nodes(self, launch_key: str) -> int:
        """Cleanup remaining nodes for a given launch_key using dynamic detection.

        Stage 2 of 2-stage termination strategy.
        Returns number of cleanup attempts made.
        """
        keywords = self._get_node_filter_keywords(launch_key)
        if not keywords:
            return 0

        running_nodes = self._get_running_ros2_nodes()
        if not running_nodes:
            return 0

        # Filter nodes matching keywords
        matching_nodes = []
        for node in running_nodes:
            node_lower = node.lower()
            for keyword in keywords:
                if keyword.lower() in node_lower:
                    matching_nodes.append(node)
                    break

        if not matching_nodes:
            return 0

        self._log(f"[Stage 2] Cleaning up {len(matching_nodes)} remaining nodes: {matching_nodes}")

        cleanup_count = 0
        for node in matching_nodes:
            process_pattern = self._extract_process_pattern(node)
            if process_pattern:
                try:
                    subprocess.run(
                        ['pkill', '-SIGTERM', '-f', process_pattern],
                        timeout=2
                    )
                    cleanup_count += 1
                    self._log(f"  Sent SIGTERM to: {process_pattern}")
                except Exception:
                    pass  # Ignore - process may already be dead

        return cleanup_count

    def stop_launch_file(self, launch_key):
        """Stop a running launch file"""
        if self.processes[launch_key] is None:
            self._log(f"Launch '{launch_key}' is not running!")
            return False

        try:
            process = self.processes[launch_key]

            def get_process_tree(pid):
                """Get all child processes of a given PID"""
                try:
                    result = subprocess.run(
                        ['pgrep', '-P', str(pid)],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    child_pids = [int(p) for p in result.stdout.strip().split('\n') if p]
                    all_pids = child_pids.copy()
                    for child_pid in child_pids:
                        all_pids.extend(get_process_tree(child_pid))
                    return all_pids
                except:
                    return []

            all_pids = [process.pid] + get_process_tree(process.pid)
            self._log(f"Stopping process tree: {all_pids}")

            # Try to kill the entire process group first (setsid creates new group)
            try:
                pgid = os.getpgid(process.pid)
                self._log(f"Sending SIGINT to process group {pgid}")
                os.killpg(pgid, signal.SIGINT)
            except (ProcessLookupError, OSError) as e:
                self._log(f"Process group kill failed: {e}, trying individual PIDs")
                # Fallback: Send SIGINT to individual processes
                for pid in reversed(all_pids):
                    try:
                        os.kill(pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass

            time.sleep(2)

            # Force kill surviving processes
            surviving_pids = []
            for pid in all_pids:
                try:
                    os.kill(pid, 0)
                    surviving_pids.append(pid)
                except ProcessLookupError:
                    pass

            if surviving_pids:
                self._log(f"Force killing: {surviving_pids}")
                # Try process group SIGKILL first
                try:
                    pgid = os.getpgid(process.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    # Fallback: kill individual processes
                    for pid in reversed(surviving_pids):
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass

                time.sleep(1)

            # Stage 2: Dynamic cleanup of remaining nodes
            cleaned = self._cleanup_remaining_nodes(launch_key)
            if cleaned > 0:
                self._log(f"[Stage 2] Cleaned {cleaned} remaining nodes")
                time.sleep(0.5)

            self.processes[launch_key] = None
            self._log(f"Stopped: {launch_key}")
            self.get_logger().info(f"Stopped {launch_key}")

            time.sleep(1)
            return True

        except Exception as e:
            self._log(f"Failed to stop: {str(e)}")
            self.get_logger().error(f"Failed to stop {launch_key}: {str(e)}")
            return False

    def stop_all_launches(self):
        """Stop all running launch files"""
        for key in self.processes.keys():
            if self.processes[key] is not None:
                self.stop_launch_file(key)
        self._log("All launches stopped")

    def is_running(self, launch_key):
        """Check if a launch file is currently running"""
        if self.processes[launch_key] is None:
            return False

        poll = self.processes[launch_key].poll()
        if poll is not None:
            self.processes[launch_key] = None
            return False

        return True

    def save_slamtoolbox_map(self, map_name):
        """Save SLAM Toolbox map using service call"""
        try:
            # Create map directory
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = self.workspace_path / 'maps' / 'slam_toolbox' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            map_path = str(map_dir / map_name)

            # Call slam_toolbox save map service
            result = subprocess.run(
                ['ros2', 'service', 'call',
                 '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{"filename": "{map_path}"}}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self._log(f"Map saved: {map_path}")
                self.get_logger().info(f"Map saved: {map_path}")
                return True, map_path
            else:
                self._log(f"Failed to save map: {result.stderr}")
                return False, result.stderr

        except Exception as e:
            self._log(f"Failed to save map: {str(e)}")
            return False, str(e)

    def save_carto_map(self, map_name):
        """Save Cartographer map (placeholder for future implementation)"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = self.workspace_path / 'maps' / 'cartographer' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            map_path = str(map_dir / map_name)

            # Cartographer uses finish_trajectory and write_state services
            # This is a placeholder - actual implementation depends on Cartographer setup
            self._log(f"Cartographer map save not implemented yet")
            self._log(f"Map would be saved to: {map_path}")
            return False, "Not implemented"

        except Exception as e:
            self._log(f"Failed to save map: {str(e)}")
            return False, str(e)

    def save_kslam_map(self, map_name):
        """Save K-SLAM map using K-SLAM's save_map service"""
        try:
            # Call K-SLAM's save_map service (saves to ~/Study/ros2_3dslam_ws/maps/)
            result = subprocess.run(
                ['ros2', 'service', 'call',
                 '/k_slam/save_map',
                 'std_srvs/srv/Trigger',
                 '{}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0 and 'success=true' in result.stdout.lower():
                self._log(f"K-SLAM map saved successfully")
                self.get_logger().info(f"K-SLAM map saved")
                return True, "Map saved to ~/Study/ros2_3dslam_ws/maps/"
            else:
                # Fallback: use nav2_map_server (now works with TRANSIENT_LOCAL QoS)
                self._log("Trying nav2_map_server...")
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                map_dir = self.workspace_path / 'maps' / 'kslam'
                os.makedirs(map_dir, exist_ok=True)
                map_path = str(map_dir / f"kslam_map_{timestamp}")

                result2 = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                    capture_output=True,
                    text=True,
                    timeout=30
                )

                if result2.returncode == 0:
                    self._log(f"K-SLAM map saved: {map_path}")
                    self.get_logger().info(f"K-SLAM map saved: {map_path}")
                    return True, map_path
                else:
                    self._log(f"Failed to save K-SLAM map: {result2.stderr}")
                    return False, result2.stderr

        except Exception as e:
            self._log(f"Failed to save K-SLAM map: {str(e)}")
            return False, str(e)

    def save_rtabmap2d_map(self, map_name: str = "", save_directory: str = "") -> str | None:
        """Save RTAB-Map 2D map - copy database and save 2D grid map"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if save_directory:
                map_dir = Path(save_directory)
            else:
                map_dir = self.workspace_path / 'maps' / 'rtabmap_2d' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            effective_name = map_name if map_name else f"rtabmap2d_{timestamp}"
            map_path = str(map_dir / effective_name)

            # 1. Copy rtabmap database file
            import shutil
            db_source = os.path.expanduser('~/.ros/rtabmap.db')
            db_dest = f"{map_path}.db"

            if os.path.exists(db_source):
                shutil.copy2(db_source, db_dest)
                self._log(f"RTAB-Map database saved: {db_dest}")
            else:
                self._log("Warning: rtabmap.db not found, skipping database backup")

            # 2. Save 2D occupancy grid map using nav2_map_server
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path,
                 '-t', '/rtabmap/map',
                 '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self._log(f"RTAB-Map 2D grid map saved: {map_path}.pgm/.yaml")
                self.get_logger().info(f"RTAB-Map 2D map saved: {map_path}")
                return map_path
            else:
                # Even if grid map fails, database was saved
                if os.path.exists(db_dest):
                    self._log(f"2D grid map save failed, but database saved: {db_dest}")
                    return db_dest
                self._log(f"Failed to save RTAB-Map 2D map: {result.stderr}")
                return None

        except Exception as e:
            self._log(f"Failed to save RTAB-Map 2D map: {str(e)}")
            return None


    def _handle_slam_control(self, request, response):
        """SlamControl.srv 핸들러"""
        if request.command == SlamControl.Request.CMD_START_2D:
            thread = threading.Thread(
                target=self._start_rtabmap2d_async,
                daemon=True
            )
            thread.start()
            response.success = True
            response.message = "rtabmap2d start initiated"

        elif request.command == SlamControl.Request.CMD_STOP:
            thread = threading.Thread(
                target=self._stop_rtabmap2d_async,
                daemon=True
            )
            thread.start()
            response.success = True
            response.message = "rtabmap2d stop initiated"

        elif request.command == SlamControl.Request.CMD_SAVE_MAP:
            map_name = request.map_name if request.map_name else ""
            save_dir = request.save_directory if request.save_directory else ""
            saved = self.save_rtabmap2d_map(map_name=map_name, save_directory=save_dir)
            response.success = saved is not None
            response.saved_path = saved or ""
            response.message = f"Map saved: {saved}" if saved else "Map save failed"

        else:
            response.success = False
            response.message = f"Unknown command: {request.command}"

        return response

    def _start_rtabmap2d_async(self):
        """thread 내부에서 실행 — blocking sleep 허용"""
        try:
            self.start_launch_file(
                'rtabmap2d_mapping',
                'rtab_map',
                'rtabmap_mapping.launch.py'
            )
        except Exception as e:
            self.get_logger().error(f"rtabmap2d start failed: {e}")

    def _stop_rtabmap2d_async(self):
        """thread 내부에서 실행"""
        try:
            self.stop_launch_file('rtabmap2d_mapping')
        except Exception as e:
            self.get_logger().error(f"rtabmap2d stop failed: {e}")


def main(args=None):
    """Main entry point"""
    import sys
    from PyQt5 import QtWidgets
    from PyQt5.QtCore import QTimer
    from slam_manager_2d.slam_manager_2d_ui import SlamManagerUI

    # Initialize ROS2
    rclpy.init(args=args)

    # Create Qt Application
    app = QtWidgets.QApplication(sys.argv)

    # Create UI
    ui = SlamManagerUI()

    # Create ROS2 Node
    node = SlamManagerNode(ui)
    ui.set_node(node)

    # Show UI
    ui.show()

    # Timer for ROS2 spinning
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
