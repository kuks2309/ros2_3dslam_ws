"""ACS Waypoint GUI Node -- PyQt5 GUI for waypoint mission management.

Minimal GUI with waypoint table, mission controls, and robot position display.
Uses QTimer(10ms) + spin_once pattern (same as slam_manager_3d).
"""

import math
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from waypoint_interfaces.action import WaypointMission
from waypoint_interfaces.msg import Waypoint
from waypoint_interfaces.srv import PauseMission

import tf2_ros

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QTimer

from acs_waypoint_gui.acs_test_node import parse_job_file, _DRIVE_MODE_MAP
from acs_waypoint_gui.acs_settings import load_settings, save_settings
from acs_waypoint_gui.performance_logger import PerformanceLogger


_DRIVE_MODE_NAMES = {v: k for k, v in _DRIVE_MODE_MAP.items()}

_WP_COLUMNS = ['ID', 'Type', 'X', 'Y', 'Heading', 'DriveMode',
               'Speed', 'Radius', 'Status', 'Time', 'Error']


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class AcsGuiRosNode(Node):

    def __init__(self):
        super().__init__('acs_gui_node')
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass  # already declared via launch override

        cb_group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(
            self, WaypointMission, '/waypoint_mission',
            callback_group=cb_group)
        self._pause_client = self.create_client(
            PauseMission, '/pause_mission', callback_group=cb_group)

        # TF2
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._goal_handle = None
        self._perf_log = None

    def lookup_robot_pose(self):
        """Return (x, y, yaw_deg) or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (x, y, math.degrees(yaw))
        except Exception:
            return None

    def cleanup(self):
        if self._perf_log:
            self._perf_log.close()


# ---------------------------------------------------------------------------
# GUI Dialog
# ---------------------------------------------------------------------------
class AcsGuiDialog(QtWidgets.QDialog):

    def __init__(self, node: AcsGuiRosNode):
        super().__init__()
        self._node = node
        self._mission_active = False
        self._paused = False
        self._job_file_path = ''

        self.setWindowTitle('ACS Waypoint GUI')
        self.resize(900, 600)
        self._build_ui()
        self._load_settings()

        # Robot position update (2 Hz)
        self._pos_timer = QTimer(self)
        self._pos_timer.timeout.connect(self._update_robot_pos)
        self._pos_timer.start(500)

    # ---- UI construction ---------------------------------------------------

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # -- Top: Robot position
        pos_group = QtWidgets.QGroupBox('Robot Position')
        pos_lay = QtWidgets.QHBoxLayout(pos_group)
        self._lbl_x = QtWidgets.QLabel('X: ---')
        self._lbl_y = QtWidgets.QLabel('Y: ---')
        self._lbl_yaw = QtWidgets.QLabel('Yaw: ---')
        for lbl in (self._lbl_x, self._lbl_y, self._lbl_yaw):
            lbl.setMinimumWidth(120)
            pos_lay.addWidget(lbl)
        pos_lay.addStretch()
        layout.addWidget(pos_group)

        # -- Job file bar
        file_lay = QtWidgets.QHBoxLayout()
        file_lay.addWidget(QtWidgets.QLabel('Job File:'))
        self._txt_job = QtWidgets.QLineEdit()
        self._txt_job.setReadOnly(True)
        file_lay.addWidget(self._txt_job, 1)
        btn_open = QtWidgets.QPushButton('Open')
        btn_open.clicked.connect(self._on_open_job)
        file_lay.addWidget(btn_open)
        btn_save = QtWidgets.QPushButton('Save')
        btn_save.clicked.connect(self._on_save_job)
        file_lay.addWidget(btn_save)
        layout.addLayout(file_lay)

        # -- Waypoint table
        self._table = QtWidgets.QTableWidget(0, len(_WP_COLUMNS))
        self._table.setHorizontalHeaderLabels(_WP_COLUMNS)
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._table.setContextMenuPolicy(Qt.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_table_context_menu)
        layout.addWidget(self._table, 1)

        # -- Parameters
        param_group = QtWidgets.QGroupBox('Parameters')
        param_lay = QtWidgets.QHBoxLayout(param_group)
        param_lay.addWidget(QtWidgets.QLabel('Max Speed:'))
        self._spin_speed = QtWidgets.QDoubleSpinBox()
        self._spin_speed.setRange(0.01, 2.0)
        self._spin_speed.setValue(0.3)
        self._spin_speed.setSingleStep(0.05)
        param_lay.addWidget(self._spin_speed)
        param_lay.addWidget(QtWidgets.QLabel('Accel:'))
        self._spin_accel = QtWidgets.QDoubleSpinBox()
        self._spin_accel.setRange(0.01, 2.0)
        self._spin_accel.setValue(0.3)
        self._spin_accel.setSingleStep(0.05)
        param_lay.addWidget(self._spin_accel)
        param_lay.addWidget(QtWidgets.QLabel('Start WP:'))
        self._spin_start = QtWidgets.QSpinBox()
        self._spin_start.setRange(1, 9999)
        self._spin_start.setValue(1)
        param_lay.addWidget(self._spin_start)
        param_lay.addWidget(QtWidgets.QLabel('End WP:'))
        self._spin_end = QtWidgets.QSpinBox()
        self._spin_end.setRange(0, 9999)
        self._spin_end.setValue(0)
        self._spin_end.setSpecialValueText('Last')
        param_lay.addWidget(self._spin_end)
        param_lay.addStretch()
        layout.addWidget(param_group)

        # -- Mission controls
        ctrl_lay = QtWidgets.QHBoxLayout()
        self._btn_run = QtWidgets.QPushButton('Run Mission')
        self._btn_run.clicked.connect(self._on_run_clicked)
        ctrl_lay.addWidget(self._btn_run)
        self._btn_pause = QtWidgets.QPushButton('Pause')
        self._btn_pause.clicked.connect(self._on_pause_clicked)
        self._btn_pause.setEnabled(False)
        ctrl_lay.addWidget(self._btn_pause)
        self._btn_cancel = QtWidgets.QPushButton('Cancel')
        self._btn_cancel.clicked.connect(self._on_cancel_clicked)
        self._btn_cancel.setEnabled(False)
        ctrl_lay.addWidget(self._btn_cancel)
        ctrl_lay.addStretch()
        layout.addLayout(ctrl_lay)

        # -- Status bar
        status_group = QtWidgets.QGroupBox('Status')
        status_lay = QtWidgets.QHBoxLayout(status_group)
        self._lbl_wp = QtWidgets.QLabel('WP: -')
        status_lay.addWidget(self._lbl_wp)
        self._progress = QtWidgets.QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        status_lay.addWidget(self._progress, 1)
        self._lbl_elapsed = QtWidgets.QLabel('0.0s')
        status_lay.addWidget(self._lbl_elapsed)
        self._lbl_status = QtWidgets.QLabel('IDLE')
        self._lbl_status.setMinimumWidth(100)
        status_lay.addWidget(self._lbl_status)
        layout.addWidget(status_group)

    # ---- Settings ----------------------------------------------------------

    def _load_settings(self):
        s = load_settings()
        if s is None:
            return
        if s.get('job_file') and os.path.exists(s['job_file']):
            self._load_job_file(s['job_file'])
        try:
            self._spin_speed.setValue(float(s.get('max_speed', 0.3)))
            self._spin_accel.setValue(float(s.get('accel', 0.3)))
            self._spin_start.setValue(int(s.get('start_wp', 1)))
            self._spin_end.setValue(int(s.get('end_wp', 0)))
        except (ValueError, TypeError):
            pass

    def _save_current_settings(self):
        save_settings({
            'job_file': self._job_file_path,
            'max_speed': str(self._spin_speed.value()),
            'accel': str(self._spin_accel.value()),
            'start_wp': str(self._spin_start.value()),
            'end_wp': str(self._spin_end.value()),
        })

    # ---- Job file ----------------------------------------------------------

    def _on_open_job(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, 'Open Job File', '', 'Job Files (*.txt);;All Files (*)')
        if path:
            self._load_job_file(path)

    def _load_job_file(self, path: str):
        try:
            entries = parse_job_file(path)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Error', f'Failed to parse job file:\n{e}')
            return
        self._job_file_path = path
        self._txt_job.setText(path)
        self._table.setRowCount(0)
        for i, (typ, x, y, yaw_deg, timeout, dm_str, turn_r, speed) in enumerate(entries):
            self._add_table_row(i, typ, x, y, yaw_deg, dm_str, speed, turn_r)
        self._save_current_settings()

    def _on_save_job(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, 'Save Job File', self._job_file_path,
            'Job Files (*.txt);;All Files (*)')
        if not path:
            return
        try:
            with open(path, 'w') as f:
                f.write('# ACS Waypoint Job File\n')
                f.write('# Format: Type  X(m)  Y(m)  Yaw(deg)  Timeout(sec)  '
                        'DriveMode  TurnRadius  Speed\n')
                for row in range(self._table.rowCount()):
                    typ = self._get_cell(row, 1) or 'AMR'
                    x = self._get_cell(row, 2) or '0.0'
                    y = self._get_cell(row, 3) or '0.0'
                    yaw = self._get_cell(row, 4) or '0'
                    dm = self._get_cell(row, 5) or 'AUTO'
                    spd = self._get_cell(row, 6) or '0.0'
                    rad = self._get_cell(row, 7) or '0.0'
                    f.write(f'{typ}    {x}   {y}   {yaw}   30.0   {dm}   {rad}   {spd}\n')
            self._job_file_path = path
            self._txt_job.setText(path)
            self._save_current_settings()
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Error', f'Failed to save: {e}')

    # ---- Table manipulation ------------------------------------------------

    def _add_table_row(self, wp_id, typ='AMR', x=0.0, y=0.0,
                       yaw_deg=0.0, dm='AUTO', speed=0.0, radius=0.0):
        row = self._table.rowCount()
        self._table.insertRow(row)
        values = [str(wp_id), typ, f'{x:.3f}', f'{y:.3f}',
                  f'{yaw_deg:.1f}', dm, f'{speed:.2f}', f'{radius:.2f}',
                  '', '', '']
        for col, val in enumerate(values):
            item = QtWidgets.QTableWidgetItem(val)
            if col in (8, 9, 10):  # Status, Time, Error are read-only
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self._table.setItem(row, col, item)

    def _on_add_row(self):
        wp_id = self._table.rowCount()
        self._add_table_row(wp_id)

    def _on_del_row(self):
        rows = set(idx.row() for idx in self._table.selectedIndexes())
        for row in sorted(rows, reverse=True):
            self._table.removeRow(row)
        # Re-number IDs
        for r in range(self._table.rowCount()):
            self._table.item(r, 0).setText(str(r))

    def _on_insert_pos(self):
        pose = self._node.lookup_robot_pose()
        if pose is None:
            QtWidgets.QMessageBox.information(
                self, 'TF Error', 'Cannot read robot position from TF2.')
            return
        x, y, yaw_deg = pose
        wp_id = self._table.rowCount()
        self._add_table_row(wp_id, 'AMR', x, y, yaw_deg)

    def _on_table_context_menu(self, pos):
        menu = QtWidgets.QMenu(self)
        row = self._table.rowAt(pos.y())

        act_add = menu.addAction('Add Row')
        act_insert_pos = menu.addAction('Insert Current Pos')
        act_del = menu.addAction('Delete Row')
        act_del.setEnabled(row >= 0)

        action = menu.exec_(self._table.viewport().mapToGlobal(pos))
        if action == act_add:
            self._on_add_row()
        elif action == act_insert_pos:
            self._on_insert_pos()
        elif action == act_del:
            if row >= 0:
                self._table.selectRow(row)
            self._on_del_row()

    def _get_cell(self, row, col) -> str:
        item = self._table.item(row, col)
        return item.text() if item else ''

    def _set_cell(self, row, col, text: str):
        item = self._table.item(row, col)
        if item:
            item.setText(text)

    # ---- Robot position ----------------------------------------------------

    def _update_robot_pos(self):
        pose = self._node.lookup_robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        self._lbl_x.setText(f'X: {x:.3f}')
        self._lbl_y.setText(f'Y: {y:.3f}')
        self._lbl_yaw.setText(f'Yaw: {yaw:.1f}\u00b0')

    # ---- Mission control ---------------------------------------------------

    def _on_run_clicked(self):
        if self._mission_active:
            return

        # Build waypoints from table
        row_count = self._table.rowCount()
        if row_count < 2:
            QtWidgets.QMessageBox.warning(
                self, 'Error', 'Need at least 2 waypoints.')
            return

        start_idx = self._spin_start.value() - 1  # 1-based to 0-based
        end_idx = self._spin_end.value()
        if end_idx == 0:
            end_idx = row_count
        end_idx = min(end_idx, row_count)

        waypoints = []
        for row in range(start_idx, end_idx):
            try:
                wp = Waypoint()
                wp.id = int(self._get_cell(row, 0))
                wp.x = float(self._get_cell(row, 2))
                wp.y = float(self._get_cell(row, 3))
                wp.heading = math.radians(float(self._get_cell(row, 4)))
                dm_str = self._get_cell(row, 5).upper() or 'AUTO'
                wp.drive_mode = _DRIVE_MODE_MAP.get(dm_str, 0)
                wp.max_speed = float(self._get_cell(row, 6) or '0.0')
                wp.turn_radius = float(self._get_cell(row, 7) or '0.0')
                wp.wait_duration = wp.max_speed if wp.drive_mode == 6 else 0.0
                waypoints.append(wp)
                self._set_cell(row, 8, 'PENDING')
                self._set_cell(row, 9, '')
                self._set_cell(row, 10, '')
            except (ValueError, IndexError) as e:
                self._node.get_logger().warn(f'Row {row} parse error: {e}')
                continue

        if len(waypoints) < 2:
            QtWidgets.QMessageBox.warning(
                self, 'Error', 'Need at least 2 valid waypoints.')
            return

        # Check action server
        if not self._node._action_client.wait_for_server(timeout_sec=2.0):
            QtWidgets.QMessageBox.warning(
                self, 'Error', 'waypoint_mission action server not available.')
            return

        # Setup perf logger
        self._node._perf_log = PerformanceLogger('logs/missions')

        # Build goal
        goal = WaypointMission.Goal()
        goal.waypoints = waypoints
        goal.default_max_speed = self._spin_speed.value()
        goal.default_acceleration = self._spin_accel.value()
        goal.loop = False

        self._mission_active = True
        self._paused = False
        self._btn_run.setEnabled(False)
        self._btn_pause.setEnabled(True)
        self._btn_cancel.setEnabled(True)
        self._lbl_status.setText('SENDING')
        self._progress.setValue(0)

        self._save_current_settings()

        self._node.get_logger().info(
            f'Sending mission: {len(waypoints)} WPs, '
            f'speed={goal.default_max_speed}, accel={goal.default_acceleration}')

        future = self._node._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error('Mission REJECTED')
            self._lbl_status.setText('REJECTED')
            self._mission_done()
            return
        self._node._goal_handle = goal_handle
        self._node.get_logger().info('Mission ACCEPTED')
        self._lbl_status.setText('RUNNING')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        if self._node._perf_log:
            self._node._perf_log.log_feedback(fb)

        self._lbl_wp.setText(f'WP: {fb.current_waypoint_id}')
        self._progress.setValue(int(fb.progress_percent))
        self._lbl_elapsed.setText(f'{fb.mission_elapsed_time:.1f}s')

        # Update table row status
        for row in range(self._table.rowCount()):
            try:
                row_id = int(self._get_cell(row, 0))
            except ValueError:
                continue
            if row_id == fb.current_waypoint_id:
                phase_names = {0: 'IDLE', 1: 'ROTATING', 2: 'DRIVING',
                               3: 'ARRIVING', 4: 'DONE'}
                phase = phase_names.get(fb.segment_phase, str(fb.segment_phase))
                self._set_cell(row, 8, phase)
                self._set_cell(row, 9, f'{fb.mission_elapsed_time:.1f}')
                self._table.selectRow(row)
                self._table.scrollToItem(self._table.item(row, 0))
            elif row_id < fb.current_waypoint_id:
                if self._get_cell(row, 8) != 'DONE':
                    self._set_cell(row, 8, 'DONE')

    def _on_result(self, future):
        result = future.result().result
        if self._node._perf_log:
            self._node._perf_log.log_result(result)
            summary = self._node._perf_log.print_summary(result)
            self._node.get_logger().info(summary)

        status_map = {0: 'SUCCESS', -1: 'CANCELED', -2: 'PARAM_ERR',
                      -3: 'TIMEOUT', -4: 'SAFETY'}
        status_str = status_map.get(result.status, f'ERR({result.status})')
        self._lbl_status.setText(status_str)
        self._progress.setValue(100 if result.status == 0 else self._progress.value())

        # Mark remaining rows
        for row in range(self._table.rowCount()):
            try:
                row_id = int(self._get_cell(row, 0))
            except ValueError:
                continue
            if row_id < result.completed_waypoints:
                self._set_cell(row, 8, 'DONE')
            elif self._get_cell(row, 8) not in ('DONE', ''):
                if result.status != 0:
                    self._set_cell(row, 8, status_str)

        self._mission_done()

    def _on_pause_clicked(self):
        if not self._mission_active:
            return
        self._paused = not self._paused
        req = PauseMission.Request()
        req.pause = self._paused
        if self._node._pause_client.wait_for_service(timeout_sec=1.0):
            future = self._node._pause_client.call_async(req)
            future.add_done_callback(self._on_pause_result)
        self._btn_pause.setText('Resume' if self._paused else 'Pause')
        self._lbl_status.setText('PAUSED' if self._paused else 'RUNNING')

    def _on_pause_result(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self._node.get_logger().warn(f'Pause failed: {resp.message}')
        except Exception as e:
            self._node.get_logger().warn(f'Pause service error: {e}')

    def _on_cancel_clicked(self):
        if not self._mission_active or self._node._goal_handle is None:
            return
        self._node.get_logger().info('Canceling mission...')
        self._lbl_status.setText('CANCELING')
        cancel_future = self._node._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda f: self._node.get_logger().info('Cancel request sent'))

    def _mission_done(self):
        self._mission_active = False
        self._paused = False
        self._node._goal_handle = None
        self._btn_run.setEnabled(True)
        self._btn_pause.setEnabled(False)
        self._btn_pause.setText('Pause')
        self._btn_cancel.setEnabled(False)
        if self._node._perf_log:
            self._node._perf_log.close()
            self._node._perf_log = None

    # ---- Close event -------------------------------------------------------

    def closeEvent(self, event):
        if self._mission_active and self._node._goal_handle is not None:
            reply = QtWidgets.QMessageBox.question(
                self, 'Mission Active',
                'A mission is running. Cancel and exit?',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)
            if reply == QtWidgets.QMessageBox.No:
                event.ignore()
                return
            self._node._goal_handle.cancel_goal_async()
        self._save_current_settings()
        event.accept()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)

    node = AcsGuiRosNode()
    dialog = AcsGuiDialog(node)
    dialog.show()

    # ROS2 spin in Qt timer (slam_manager_3d pattern)
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    exit_code = app.exec_()

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
