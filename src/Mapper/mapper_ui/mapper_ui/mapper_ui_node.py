#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from mapper_interfaces.msg import MapperStatus
from mapper_interfaces.srv import MapperCommand
from .ros_bridge import RosBridge, STATE_NAMES, STATE_COLORS


class MapperMainWindow(QMainWindow):
    def __init__(self, ros_bridge: RosBridge):
        super().__init__()
        ui_path = os.path.join(
            get_package_share_directory('mapper_ui'), 'ui', 'mapper_main.ui')
        uic.loadUi(ui_path, self)
        self.bridge = ros_bridge

        # 시그널 연결
        self.bridge.status_received.connect(self._on_status)
        self.bridge.log_received.connect(self._on_log)

        # 버튼 연결
        self.btn_estop.clicked.connect(self._on_estop)
        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_resume.clicked.connect(self._on_resume)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_explore.clicked.connect(self._on_explore)

        self._update_buttons(MapperStatus.STATE_IDLE)

    def _on_status(self, msg: MapperStatus):
        state = msg.state
        name  = STATE_NAMES.get(state, "UNKNOWN")
        color = STATE_COLORS.get(state, "#888888")
        self.lbl_state.setText(name)
        self.lbl_state.setStyleSheet(
            f"background-color: {color}; color: white; "
            "font-weight: bold; border-radius: 4px; padding: 4px;")
        self.progress_coverage.setValue(int(msg.coverage_percent))
        self.lbl_heading_error.setText(
            f"헤딩 오차: {msg.current_heading_error_deg:.2f}°")
        self._update_buttons(state)

    def _on_log(self, msg: str):
        self.txt_log.append(f"> {msg}")

    def _update_buttons(self, state: int):
        S = MapperStatus
        self.btn_estop.setEnabled(True)   # 항상 활성
        self.btn_start.setEnabled(state == S.STATE_IDLE)
        self.btn_pause.setEnabled(state in (
            S.STATE_MAPPING_MANUAL, S.STATE_MAPPING_AUTO,
            S.STATE_EXPLORING_UNKNOWN))
        self.btn_resume.setEnabled(state == S.STATE_PAUSED)
        self.btn_stop.setEnabled(state not in (S.STATE_IDLE, S.STATE_COMPLETED))
        self.btn_explore.setEnabled(state == S.STATE_MAPPING_MANUAL)
        for btn in [self.btn_forward, self.btn_left, self.btn_right]:
            btn.setEnabled(state == S.STATE_MAPPING_MANUAL)
        self.combo_slam_mode.setEnabled(state == S.STATE_IDLE)
        self.combo_drive_mode.setEnabled(state == S.STATE_IDLE)

    def _on_estop(self):
        self.bridge.send_command(MapperCommand.Request.CMD_STOP)

    def _on_start(self):
        self.bridge.send_command(
            MapperCommand.Request.CMD_START_MAPPING,
            slam_mode=self.combo_slam_mode.currentIndex(),
            drive_mode=self.combo_drive_mode.currentIndex())

    def _on_pause(self):
        self.bridge.send_command(MapperCommand.Request.CMD_PAUSE)

    def _on_resume(self):
        self.bridge.send_command(MapperCommand.Request.CMD_RESUME)

    def _on_stop(self):
        self.bridge.send_command(MapperCommand.Request.CMD_STOP)

    def _on_explore(self):
        self.bridge.send_command(MapperCommand.Request.CMD_EXPLORE)


def main():
    rclpy.init()
    node = rclpy.create_node('mapper_ui_node')
    bridge = RosBridge(node)

    app = QApplication(sys.argv)
    window = MapperMainWindow(bridge)
    window.setWindowTitle("SLAM Mapper")
    window.show()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    timer = QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0))
    timer.start(100)  # 100ms

    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)
