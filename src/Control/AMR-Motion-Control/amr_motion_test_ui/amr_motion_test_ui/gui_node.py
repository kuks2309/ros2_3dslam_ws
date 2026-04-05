"""PyQt5 GUI for AMR motion control testing with launch management."""

import sys
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QLabel, QPushButton, QTextEdit, QFormLayout,
    QDoubleSpinBox, QCheckBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

from amr_motion_test_ui.ros_bridge import RosBridge, MONITORED_TOPICS
from amr_motion_test_ui.process_manager import ProcessManager, LAUNCH_CONFIGS

from amr_interfaces.action import (
    AMRMotionSpin,
    AMRMotionTurn,
    AMRMotionTranslate,
    AMRMotionYawControl,
    AMRMotionStanley,
)
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MotionTestGUI(QMainWindow):
    """Main window with 8 tabs: Connection, Status, and 6 action tests."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('AMR Motion Test UI')
        self.resize(880, 720)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Tab widget
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        # Bottom bar: E-STOP + log
        bottom = QHBoxLayout()

        self.estop_btn = QPushButton('E-STOP')
        self.estop_btn.setStyleSheet(
            'background-color:#cc0000; color:white; font-weight:bold; '
            'font-size:16px; padding:10px 20px;')
        self.estop_btn.setFixedWidth(130)
        self.estop_btn.clicked.connect(self._on_estop)
        bottom.addWidget(self.estop_btn)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        bottom.addWidget(self.log_text)
        layout.addLayout(bottom)

        # Internal state
        self._pose = (0.0, 0.0, 0.0)
        self._fb_labels = {}   # action_key -> QLabel
        self._res_labels = {}  # action_key -> QLabel

        # Process manager + ROS bridge (must init before tabs reference them)
        self.proc = ProcessManager(log_callback=self._log)
        self.bridge = RosBridge()

        # Build tabs
        self._build_connection_tab()
        self._build_status_tab()
        self._build_spin_tab()
        self._build_turn_tab()
        self._build_translate_tab()
        self._build_translate_reverse_tab()
        self._build_yaw_control_tab()
        self._build_stanley_tab()

        # Connect bridge signals and start
        self.bridge.topic_hz_updated.connect(self._on_hz)
        self.bridge.pose_updated.connect(self._on_pose)
        self.bridge.action_feedback.connect(self._on_fb)
        self.bridge.action_result.connect(self._on_res)
        self.bridge.log_message.connect(self._log)
        self.bridge.start()

        # Poll process status every 2 s
        self._ptimer = QTimer(self)
        self._ptimer.timeout.connect(self._poll_procs)
        self._ptimer.start(2000)

    # ── Connection Tab ────────────────────────────────────────────────

    def _build_connection_tab(self):
        tab = QWidget()
        vbox = QVBoxLayout(tab)

        grp = QGroupBox('Launch / Stop')
        grid = QGridLayout()

        self._proc_labels = {}
        for row, (key, cfg) in enumerate(LAUNCH_CONFIGS.items()):
            grid.addWidget(QLabel(cfg['label']), row, 0)

            btn_start = QPushButton('Start')
            btn_start.clicked.connect(lambda _, k=key: self.proc.start(k))
            grid.addWidget(btn_start, row, 1)

            btn_stop = QPushButton('Stop')
            btn_stop.clicked.connect(lambda _, k=key: self.proc.stop(k))
            grid.addWidget(btn_stop, row, 2)

            status = QLabel('\u25cb Stopped')
            grid.addWidget(status, row, 3)
            self._proc_labels[key] = status

        grp.setLayout(grid)
        vbox.addWidget(grp)

        row_btns = QHBoxLayout()
        btn_all = QPushButton('Start All (\uc21c\uc11c\ub300\ub85c)')
        btn_all.clicked.connect(self._start_all)
        row_btns.addWidget(btn_all)

        btn_stop_all = QPushButton('Stop All')
        btn_stop_all.clicked.connect(self.proc.stop_all)
        row_btns.addWidget(btn_stop_all)
        vbox.addLayout(row_btns)

        vbox.addStretch()
        self.tabs.addTab(tab, 'Connection')

    def _start_all(self):
        self.proc.start('gazebo')
        QTimer.singleShot(8000, lambda: self.proc.start('slam'))
        QTimer.singleShot(15000, lambda: self.proc.start('motion_control'))
        self._log('Start All: Gazebo \u2192 SLAM(+8s) \u2192 MotionCtrl(+15s)')

    def _poll_procs(self):
        for key, lbl in self._proc_labels.items():
            if self.proc.is_running(key):
                lbl.setText('\u25cf Running')
                lbl.setStyleSheet('color:green; font-weight:bold;')
            else:
                lbl.setText('\u25cb Stopped')
                lbl.setStyleSheet('color:gray;')

    # ── Status Tab ────────────────────────────────────────────────────

    def _build_status_tab(self):
        tab = QWidget()
        vbox = QVBoxLayout(tab)

        # Topic table
        topics = list(MONITORED_TOPICS.keys())
        self._topic_table = QTableWidget(len(topics), 3)
        self._topic_table.setHorizontalHeaderLabels(['Topic', 'Hz', 'Status'])
        hdr = self._topic_table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.Stretch)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self._topic_table.setEditTriggers(QTableWidget.NoEditTriggers)

        self._topic_rows = {}
        for r, topic in enumerate(topics):
            _label = MONITORED_TOPICS[topic][1]
            self._topic_table.setItem(r, 0, QTableWidgetItem(f'{topic}  ({_label})'))
            self._topic_table.setItem(r, 1, QTableWidgetItem('--'))
            self._topic_table.setItem(r, 2, QTableWidgetItem('\u25cb Idle'))
            self._topic_rows[topic] = r

        vbox.addWidget(self._topic_table)

        # Pose
        grp = QGroupBox('Current Pose  (map \u2192 base_footprint)')
        h = QHBoxLayout()
        self._lbl_x = QLabel('X: --')
        self._lbl_y = QLabel('Y: --')
        self._lbl_yaw = QLabel('Yaw: --')
        for w in (self._lbl_x, self._lbl_y, self._lbl_yaw):
            w.setStyleSheet('font-size:14px; font-weight:bold;')
            h.addWidget(w)
        grp.setLayout(h)
        vbox.addWidget(grp)

        vbox.addStretch()
        self.tabs.addTab(tab, 'Status')

    def _on_hz(self, hz_map):
        for topic, hz in hz_map.items():
            r = self._topic_rows.get(topic)
            if r is None:
                continue
            self._topic_table.item(r, 1).setText(f'{hz:.1f}')
            if hz > 0.5:
                self._topic_table.item(r, 2).setText('\u25cf Active')
                self._topic_table.item(r, 2).setForeground(QColor('green'))
            else:
                self._topic_table.item(r, 2).setText('\u25cb Idle')
                self._topic_table.item(r, 2).setForeground(QColor('gray'))

    def _on_pose(self, x, y, yaw):
        self._pose = (x, y, yaw)
        self._lbl_x.setText(f'X: {x:.3f} m')
        self._lbl_y.setText(f'Y: {y:.3f} m')
        self._lbl_yaw.setText(f'Yaw: {yaw:.1f}\u00b0')

    # ── Generic tab builders ──────────────────────────────────────────

    def _make_action_tab(self, title, key, fields, goal_fn):
        """Simple action tab (Spin, Turn) with numeric inputs only."""
        tab = QWidget()
        vbox = QVBoxLayout(tab)
        form = QFormLayout()
        widgets = {}

        for label, param, default, lo, hi, suffix in fields:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(2)
            sb.setValue(default)
            sb.setSuffix(f' {suffix}')
            widgets[param] = sb
            form.addRow(f'{label}:', sb)

        vbox.addLayout(form)
        self._add_action_buttons(vbox, key, widgets, goal_fn)
        vbox.addStretch()
        self.tabs.addTab(tab, title)

    def _make_pose_action_tab(self, title, key, extra_fields, goal_fn,
                              has_next_checkbox=False):
        """Action tab with start/end XY + 'Use Current Pose' button."""
        tab = QWidget()
        vbox = QVBoxLayout(tab)
        widgets = {}

        # Position group
        grp = QGroupBox('Position (map frame, meters)')
        g = QGridLayout()
        for i, (lbl, param, default) in enumerate([
            ('Start X', 'start_x', 0.0), ('Start Y', 'start_y', 0.0),
            ('End X', 'end_x', 1.0),     ('End Y', 'end_y', 0.0),
        ]):
            row, col = divmod(i, 2)
            g.addWidget(QLabel(lbl + ':'), row, col * 2)
            sb = QDoubleSpinBox()
            sb.setRange(-100.0, 100.0)
            sb.setDecimals(3)
            sb.setValue(default)
            sb.setSuffix(' m')
            widgets[param] = sb
            g.addWidget(sb, row, col * 2 + 1)

        btn_pose = QPushButton('\u2190 Current Pose')
        btn_pose.clicked.connect(lambda: self._fill_pose(widgets))
        g.addWidget(btn_pose, 0, 4, 2, 1)
        grp.setLayout(g)
        vbox.addWidget(grp)

        # Extra numeric fields
        form = QFormLayout()
        for label, param, default, lo, hi, suffix in extra_fields:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(3)
            sb.setValue(default)
            sb.setSuffix(f' {suffix}')
            widgets[param] = sb
            form.addRow(f'{label}:', sb)
        vbox.addLayout(form)

        # Optional has_next checkbox
        if has_next_checkbox:
            cb = QCheckBox('has_next (\ub2e4\uc74c \uc138\uadf8\uba3c\ud2b8 \uc874\uc7ac \u2192 \uac10\uc18d \uc0dd\ub7b5)')
            cb.setChecked(False)
            widgets['has_next'] = cb
            vbox.addWidget(cb)

        self._add_action_buttons(vbox, key, widgets, goal_fn)
        vbox.addStretch()
        self.tabs.addTab(tab, title)

    def _add_action_buttons(self, layout, key, widgets, goal_fn):
        """Add Send/Cancel buttons + feedback/result labels."""
        h = QHBoxLayout()
        btn_send = QPushButton('Send Goal')
        btn_send.setStyleSheet(
            'background-color:#2196F3; color:white; padding:8px 16px;')
        btn_send.clicked.connect(
            lambda: self._send(key, widgets, goal_fn))
        h.addWidget(btn_send)

        btn_cancel = QPushButton('Cancel')
        btn_cancel.clicked.connect(self.bridge.cancel_goal)
        h.addWidget(btn_cancel)
        layout.addLayout(h)

        fb = QLabel('Feedback: --')
        fb.setWordWrap(True)
        layout.addWidget(fb)
        self._fb_labels[key] = fb

        res = QLabel('Result: --')
        res.setWordWrap(True)
        layout.addWidget(res)
        self._res_labels[key] = res

    def _fill_pose(self, widgets):
        x, y, _ = self._pose
        widgets['start_x'].setValue(x)
        widgets['start_y'].setValue(y)

    def _send(self, key, widgets, goal_fn):
        self._fb_labels[key].setText('Feedback: sending...')
        self._fb_labels[key].setStyleSheet('')
        self._res_labels[key].setText('Result: --')
        self._res_labels[key].setStyleSheet('')
        goal = goal_fn(widgets)
        self.bridge.send_goal(key, goal)

    def _on_fb(self, key, text):
        lbl = self._fb_labels.get(key)
        if lbl:
            lbl.setText(f'Feedback: {text}')

    def _on_res(self, key, text, status):
        lbl = self._res_labels.get(key)
        if lbl:
            color = 'green' if status == 0 else 'red'
            lbl.setText(f'Result: {text}')
            lbl.setStyleSheet(f'color:{color}; font-weight:bold;')

    # ── 5 Action Tabs ─────────────────────────────────────────────────

    def _build_spin_tab(self):
        def goal(w):
            g = AMRMotionSpin.Goal()
            g.target_angle = w['target_angle'].value()
            g.max_angular_speed = w['max_angular_speed'].value()
            g.angular_acceleration = w['angular_acceleration'].value()
            return g

        self._make_action_tab('Spin', 'spin', [
            ('Target Angle',  'target_angle',        90.0, -360, 360, 'deg'),
            ('Max Speed',     'max_angular_speed',   30.0,  1,  57.3, 'deg/s'),
            ('Acceleration',  'angular_acceleration', 30.0, 1,  180,  'deg/s\u00b2'),
        ], goal)

    def _build_turn_tab(self):
        def goal(w):
            g = AMRMotionTurn.Goal()
            g.target_angle = w['target_angle'].value()
            g.turn_radius = w['turn_radius'].value()
            g.max_linear_speed = w['max_linear_speed'].value()
            g.accel_angle = w['accel_angle'].value()
            return g

        self._make_action_tab('Turn', 'turn', [
            ('Target Angle', 'target_angle',    90.0, -360, 360, 'deg'),
            ('Turn Radius',  'turn_radius',      0.5,  0.1,  5,  'm'),
            ('Max Speed',    'max_linear_speed',  0.2, 0.05, 0.4, 'm/s'),
            ('Accel Angle',  'accel_angle',      20.0,  1,   90,  'deg'),
        ], goal)

    def _build_translate_tab(self):
        def goal(w):
            g = AMRMotionTranslate.Goal()
            g.start_x = w['start_x'].value()
            g.start_y = w['start_y'].value()
            g.end_x = w['end_x'].value()
            g.end_y = w['end_y'].value()
            g.max_linear_speed = w['max_linear_speed'].value()
            g.acceleration = w['acceleration'].value()
            g.exit_speed = w['exit_speed'].value()
            g.has_next = w['has_next'].isChecked()
            return g

        self._make_pose_action_tab('Translate', 'translate', [
            ('Max Speed',    'max_linear_speed', 0.2, 0.05, 0.4, 'm/s'),
            ('Acceleration', 'acceleration',     0.2, 0.05, 1.5, 'm/s\u00b2'),
            ('Exit Speed',   'exit_speed',       0.0, 0.0,  0.4, 'm/s'),
        ], goal, has_next_checkbox=True)

    def _build_translate_reverse_tab(self):
        def goal(w):
            g = AMRMotionTranslate.Goal()
            g.start_x = w['start_x'].value()
            g.start_y = w['start_y'].value()
            g.end_x = w['end_x'].value()
            g.end_y = w['end_y'].value()
            g.max_linear_speed = -abs(w['max_linear_speed'].value())
            g.acceleration = w['acceleration'].value()
            g.exit_speed = w['exit_speed'].value()
            g.has_next = w['has_next'].isChecked()
            return g

        self._make_pose_action_tab('TranslateRev', 'translate_reverse', [
            ('Max Speed',    'max_linear_speed', 0.2, 0.05, 0.4, 'm/s'),
            ('Acceleration', 'acceleration',     0.2, 0.05, 1.5, 'm/s\u00b2'),
            ('Exit Speed',   'exit_speed',       0.0, 0.0,  0.4, 'm/s'),
        ], goal, has_next_checkbox=True)

    def _build_yaw_control_tab(self):
        tab = QWidget()
        vbox = QVBoxLayout(tab)

        widgets = {}
        params = [
            ('Target Heading', 'target_heading',   0.0, -180.0, 180.0, 'deg'),
            ('Max Speed',      'max_linear_speed',  0.2, 0.05,  0.4,  'm/s'),
            ('Acceleration',   'acceleration',      0.2, 0.05,  1.5,  'm/s\u00b2'),
            ('Deceleration',   'deceleration',      0.3, 0.05,  2.0,  'm/s\u00b2'),
        ]
        for label, name, default, lo, hi, unit in params:
            h = QHBoxLayout()
            h.addWidget(QLabel(f'{label} ({unit}):'))
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(0.05)
            sb.setDecimals(2)
            sb.setValue(default)
            widgets[name] = sb
            h.addWidget(sb)
            vbox.addLayout(h)

        # Use Current Heading button
        btn_cur = QPushButton('Use Current Heading')
        btn_cur.clicked.connect(
            lambda: widgets['target_heading'].setValue(self._pose[2]))
        vbox.addWidget(btn_cur)

        def goal_fn(w):
            g = AMRMotionYawControl.Goal()
            g.target_heading = w['target_heading'].value()
            g.max_linear_speed = w['max_linear_speed'].value()
            g.acceleration = w['acceleration'].value()
            g.deceleration = w['deceleration'].value()
            return g

        # Buttons: Send Goal | Stop (decel) | Cancel (immediate)
        h = QHBoxLayout()
        btn_send = QPushButton('Send Goal')
        btn_send.setStyleSheet(
            'background-color:#2196F3; color:white; padding:8px 16px;')
        btn_send.clicked.connect(
            lambda: self._send('yaw_control', widgets, goal_fn))
        h.addWidget(btn_send)

        btn_stop = QPushButton('Stop (\uac10\uc18d)')
        btn_stop.setStyleSheet(
            'background-color:#FF9800; color:white; padding:8px 16px;')
        btn_stop.clicked.connect(self.bridge.stop_yaw_control)
        h.addWidget(btn_stop)

        btn_cancel = QPushButton('Cancel (\uc989\uc2dc)')
        btn_cancel.setStyleSheet(
            'background-color:#f44336; color:white; padding:8px 16px;')
        btn_cancel.clicked.connect(self.bridge.cancel_goal)
        h.addWidget(btn_cancel)
        vbox.addLayout(h)

        fb = QLabel('Feedback: --')
        fb.setWordWrap(True)
        vbox.addWidget(fb)
        self._fb_labels['yaw_control'] = fb

        res = QLabel('Result: --')
        res.setWordWrap(True)
        vbox.addWidget(res)
        self._res_labels['yaw_control'] = res

        vbox.addStretch()
        self.tabs.addTab(tab, 'YawControl')

    # ── Stanley Tab ────────────────────────────────────────────────────

    def _build_stanley_tab(self):
        tab = QWidget()
        vbox = QVBoxLayout(tab)
        widgets = {}

        # Waypoint table
        grp = QGroupBox('Waypoint Path (map frame)')
        gv = QVBoxLayout()

        wp_table = QTableWidget(0, 2)
        wp_table.setHorizontalHeaderLabels(['X (m)', 'Y (m)'])
        wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        wp_table.setMinimumHeight(140)
        widgets['wp_table'] = wp_table
        gv.addWidget(wp_table)

        btn_row = QHBoxLayout()
        btn_add = QPushButton('Add WP')
        btn_add.clicked.connect(lambda: self._stanley_add_wp(wp_table, 0.0, 0.0))
        btn_row.addWidget(btn_add)

        btn_cur = QPushButton('Add Current Pose')
        btn_cur.clicked.connect(
            lambda: self._stanley_add_wp(wp_table, self._pose[0], self._pose[1]))
        btn_row.addWidget(btn_cur)

        btn_rm = QPushButton('Remove Selected')
        btn_rm.clicked.connect(lambda: wp_table.removeRow(wp_table.currentRow()))
        btn_row.addWidget(btn_rm)

        btn_clr = QPushButton('Clear')
        btn_clr.clicked.connect(lambda: wp_table.setRowCount(0))
        btn_row.addWidget(btn_clr)

        gv.addLayout(btn_row)
        grp.setLayout(gv)
        vbox.addWidget(grp)

        # Controller parameters
        form = QFormLayout()
        params = [
            ('K Stanley',       'k_stanley',        2.0,  0.1,  10.0, ''),
            ('K Soft',          'k_soft',            0.8,  0.01,  5.0, ''),
            ('Max Speed',       'max_linear_speed',  0.3,  0.05,  0.5, 'm/s'),
            ('Acceleration',    'acceleration',      0.2,  0.05,  1.0, 'm/s\u00b2'),
            ('Goal Tolerance',  'goal_tolerance',    0.10, 0.01,  1.0, 'm'),
        ]
        for label, name, default, lo, hi, unit in params:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(0.1 if hi > 1.0 else 0.01)
            sb.setDecimals(2)
            sb.setValue(default)
            if unit:
                sb.setSuffix(f' {unit}')
            widgets[name] = sb
            form.addRow(f'{label}:', sb)

        cb = QCheckBox('Align Final Heading')
        cb.setChecked(False)
        widgets['align_final_heading'] = cb
        form.addRow(cb)
        vbox.addLayout(form)

        def goal_fn(w):
            table = w['wp_table']
            path = Path()
            path.header.frame_id = 'map'
            for row in range(table.rowCount()):
                x_item = table.item(row, 0)
                y_item = table.item(row, 1)
                if x_item is None or y_item is None:
                    continue
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = float(x_item.text())
                ps.pose.position.y = float(y_item.text())
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)

            g = AMRMotionStanley.Goal()
            g.path = path
            g.max_linear_speed = w['max_linear_speed'].value()
            g.acceleration = w['acceleration'].value()
            g.k_stanley = w['k_stanley'].value()
            g.k_soft = w['k_soft'].value()
            g.goal_tolerance = w['goal_tolerance'].value()
            g.align_final_heading = w['align_final_heading'].isChecked()
            return g

        self._add_action_buttons(vbox, 'stanley', widgets, goal_fn)
        vbox.addStretch()
        self.tabs.addTab(tab, 'Stanley')

    @staticmethod
    def _stanley_add_wp(table, x, y):
        row = table.rowCount()
        table.insertRow(row)
        table.setItem(row, 0, QTableWidgetItem(f'{x:.3f}'))
        table.setItem(row, 1, QTableWidgetItem(f'{y:.3f}'))

    # ── E-STOP & Logging ──────────────────────────────────────────────

    def _on_estop(self):
        self.bridge.cancel_goal()
        self.bridge.publish_zero_cmd_vel()
        self._log('[E-STOP] cancelled + zero cmd_vel')

    def _log(self, msg):
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f'[{ts}] {msg}')

    def closeEvent(self, event):
        self.bridge.stop()
        self.proc.stop_all()
        event.accept()


def main():
    import rclpy
    rclpy.init()
    try:
        app = QApplication(sys.argv)
        win = MotionTestGUI()
        win.show()
        ret = app.exec_()
    finally:
        rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
