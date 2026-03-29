"""QThread-based ROS2 bridge for PyQt5 GUI — topic Hz, TF pose, action clients."""

import math
import time
from collections import deque
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from PyQt5.QtCore import QThread, pyqtSignal

from tf2_ros import Buffer, TransformListener

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from amr_interfaces.action import (
    AMRMotionSpin,
    AMRMotionTurn,
    AMRMotionTranslate,
    AMRMotionYawControl,
)

# Topics to monitor: topic -> (msg_class, label, use_sensor_qos)
MONITORED_TOPICS = {
    '/clock':                  (Clock,       'Gazebo Clock',  True),
    '/scan/points':            (PointCloud2, '3D LiDAR',      True),
    '/imu/data':               (Imu,         'IMU',           True),
    '/rtabmap/odom':           (Odometry,    'SLAM Odom',     True),
    '/cmd_vel':                (Twist,       'Cmd Vel',       False),
    '/camera/color/image_raw': (Image,       'RGB Camera',    True),
}

# Action servers: key -> (server_name, action_type)
ACTION_SERVERS = {
    'spin':              ('spin',                   AMRMotionSpin),
    'turn':              ('turn',                   AMRMotionTurn),
    'translate':         ('amr_motion_translate',   AMRMotionTranslate),
    'translate_reverse': ('amr_translate_reverse_action', AMRMotionTranslate),
    'yaw_control':       ('amr_motion_yaw_control', AMRMotionYawControl),
}

PHASE_NAMES = {0: 'WAIT', 1: 'ACCEL', 2: 'CRUISE', 3: 'DECEL'}
STATUS_DESC = {
    0: 'success', -1: 'cancelled', -2: 'invalid_param',
    -3: 'timeout', -4: 'tf_fail',
}

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class RosBridge(QThread):
    """Runs rclpy in a background thread; emits Qt signals for GUI updates."""

    topic_hz_updated = pyqtSignal(dict)              # {topic_name: hz}
    pose_updated = pyqtSignal(float, float, float)   # x, y, yaw_deg
    action_feedback = pyqtSignal(str, str)            # action_key, text
    action_result = pyqtSignal(str, str, int)         # action_key, text, status
    log_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._node = None
        self._action_clients: Dict[str, ActionClient] = {}
        self._active_goal_handle = None
        self._cmd_vel_pub = None
        self._topic_stamps: Dict[str, deque] = {
            t: deque(maxlen=50) for t in MONITORED_TOPICS
        }

    # ── Thread lifecycle ──────────────────────────────────────────────

    def run(self):
        try:
            self._node = Node('amr_motion_test_gui')
            try:
                self._node.declare_parameter('use_sim_time', True)
            except Exception:
                pass  # already declared via launch override
            self._running = True

            # TF2
            self._tf_buf = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, self._node)

            # cmd_vel publisher (for E-STOP)
            self._cmd_vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)

            # Topic subscriptions for Hz monitoring
            for topic, (msg_cls, _label, sensor_qos) in MONITORED_TOPICS.items():
                qos = _SENSOR_QOS if sensor_qos else 10
                self._node.create_subscription(
                    msg_cls, topic,
                    lambda _msg, t=topic: self._topic_stamps[t].append(time.monotonic()),
                    qos,
                )

            # Action clients
            for key, (srv_name, action_type) in ACTION_SERVERS.items():
                self._action_clients[key] = ActionClient(
                    self._node, action_type, srv_name)

            self.log_message.emit(f'ROS bridge ready ({len(self._action_clients)} actions)')

            # Timers
            self._node.create_timer(1.0, self._emit_hz)
            self._node.create_timer(0.2, self._emit_pose)

            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.02)

            self._node.destroy_node()
        except Exception as e:
            self.log_message.emit(f'[FATAL] ROS bridge crashed: {e}')

    def stop(self):
        self._running = False
        self.wait(5000)

    # ── Topic Hz ──────────────────────────────────────────────────────

    def _emit_hz(self):
        now = time.monotonic()
        hz = {}
        for topic, stamps in self._topic_stamps.items():
            recent = [t for t in stamps if now - t < 3.0]
            if len(recent) < 2:
                hz[topic] = 0.0
            else:
                dt = recent[-1] - recent[0]
                hz[topic] = (len(recent) - 1) / dt if dt > 0 else 0.0
        self.topic_hz_updated.emit(hz)

    # ── Pose ──────────────────────────────────────────────────────────

    def _emit_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.pose_updated.emit(t.x, t.y, math.degrees(math.atan2(siny, cosy)))
        except Exception:
            pass

    # ── Action API ────────────────────────────────────────────────────

    def send_goal(self, action_key: str, goal):
        """Send an action goal asynchronously."""
        client = self._action_clients.get(action_key)
        if client is None:
            self.log_message.emit(f'[ERROR] Unknown action: {action_key}')
            return
        if not client.wait_for_server(timeout_sec=3.0):
            self.log_message.emit(f'[ERROR] Server "{action_key}" not available')
            return

        future = client.send_goal_async(
            goal,
            feedback_callback=lambda msg, k=action_key: self._on_feedback(k, msg),
        )
        future.add_done_callback(
            lambda f, k=action_key: self._on_goal_response(k, f))

    def cancel_goal(self):
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self.log_message.emit('Cancel requested')

    def publish_zero_cmd_vel(self):
        if self._cmd_vel_pub:
            self._cmd_vel_pub.publish(Twist())

    # ── Action callbacks ──────────────────────────────────────────────

    def _on_goal_response(self, key, future):
        try:
            gh = future.result()
        except Exception as e:
            self.log_message.emit(f'[{key}] Goal send failed: {e}')
            self.action_result.emit(key, f'Goal send failed: {e}', -2)
            return
        if not gh or not gh.accepted:
            self.log_message.emit(f'[{key}] Goal rejected')
            self.action_result.emit(key, 'Goal rejected', -2)
            return
        self._active_goal_handle = gh
        self.log_message.emit(f'[{key}] Goal accepted')
        gh.get_result_async().add_done_callback(
            lambda f, k=key: self._on_result(k, f))

    def _on_feedback(self, key, msg):
        fb = msg.feedback
        phase = PHASE_NAMES.get(getattr(fb, 'phase', 0), '?')
        if key == 'spin':
            txt = (f'angle={fb.current_angle:.1f}\u00b0 '
                   f'speed={fb.current_speed:.1f}\u00b0/s phase={phase}')
        elif key == 'turn':
            txt = (f'angle={fb.current_angle:.1f}\u00b0 '
                   f'remain={fb.remaining_angle:.1f}\u00b0 '
                   f'v={fb.current_linear_speed:.3f}m/s '
                   f'\u03c9={fb.current_angular_speed:.1f}\u00b0/s phase={phase}')
        elif key in ('translate', 'translate_reverse'):
            txt = (f'dist={fb.current_distance:.3f}m '
                   f'lat={fb.current_lateral_error:.4f}m '
                   f'hdg={fb.current_heading_error:.1f}\u00b0 '
                   f'vx={fb.current_vx:.3f}m/s phase={phase}')
        elif key == 'yaw_control':
            txt = (f'dist={fb.current_distance:.3f}m '
                   f'hdg={fb.current_heading_error:.1f}\u00b0 '
                   f'vx={fb.current_vx:.3f}m/s '
                   f'\u03c9={fb.current_omega:.4f}rad/s phase={phase}')
        else:
            txt = str(fb)
        self.action_feedback.emit(key, txt)

    def _on_result(self, key, future):
        self._active_goal_handle = None
        try:
            r = future.result().result
        except Exception as e:
            self.log_message.emit(f'[{key}] Result error: {e}')
            self.action_result.emit(key, f'Result error: {e}', -4)
            return
        s = STATUS_DESC.get(getattr(r, 'status', 0), 'unknown')
        if key == 'spin':
            txt = (f'{s}  angle={r.actual_angle:.2f}\u00b0 '
                   f'time={r.elapsed_time:.2f}s')
        elif key == 'turn':
            txt = (f'{s}  angle={r.actual_angle:.2f}\u00b0 '
                   f'time={r.elapsed_time:.2f}s')
        else:
            txt = (f'{s}  dist={r.actual_distance:.3f}m '
                   f'lat={r.final_lateral_error:.4f}m '
                   f'hdg={r.final_heading_error:.2f}\u00b0 '
                   f'time={r.elapsed_time:.2f}s')
        self.action_result.emit(key, txt, getattr(r, 'status', 0))
