from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mapper_interfaces.msg import MapperStatus
from mapper_interfaces.srv import MapperCommand


STATE_NAMES = {
    MapperStatus.STATE_IDLE:              "IDLE",
    MapperStatus.STATE_ALIGNING:          "ALIGNING",
    MapperStatus.STATE_STARTING_SLAM:     "STARTING SLAM",
    MapperStatus.STATE_VERIFYING_MAP:     "VERIFYING MAP",
    MapperStatus.STATE_MAPPING_MANUAL:    "MAPPING (MANUAL)",
    MapperStatus.STATE_MAPPING_AUTO:      "MAPPING (AUTO)",
    MapperStatus.STATE_EXPLORING_UNKNOWN: "EXPLORING",
    MapperStatus.STATE_LOOP_CLOSING:      "LOOP CLOSING",
    MapperStatus.STATE_COMPLETED:         "COMPLETED",
    MapperStatus.STATE_ERROR:             "ERROR",
    MapperStatus.STATE_PAUSED:            "PAUSED",
}

STATE_COLORS = {
    MapperStatus.STATE_IDLE:              "#888888",
    MapperStatus.STATE_ALIGNING:          "#2196F3",
    MapperStatus.STATE_STARTING_SLAM:     "#2196F3",
    MapperStatus.STATE_VERIFYING_MAP:     "#2196F3",
    MapperStatus.STATE_MAPPING_MANUAL:    "#2196F3",
    MapperStatus.STATE_MAPPING_AUTO:      "#2196F3",
    MapperStatus.STATE_EXPLORING_UNKNOWN: "#2196F3",
    MapperStatus.STATE_LOOP_CLOSING:      "#9C27B0",
    MapperStatus.STATE_COMPLETED:         "#4CAF50",
    MapperStatus.STATE_ERROR:             "#F44336",
    MapperStatus.STATE_PAUSED:            "#FF9800",
}


class RosBridge(QObject):
    status_received = pyqtSignal(object)   # MapperStatus
    log_received    = pyqtSignal(str)

    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        self._last_log = ""

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._status_sub = node.create_subscription(
            MapperStatus, 'mapper/status',
            self._on_status, qos)

        self._cmd_client = node.create_client(
            MapperCommand, 'mapper/command')

    def _on_status(self, msg: MapperStatus):
        self.status_received.emit(msg)
        if msg.log_message and msg.log_message != self._last_log:
            self.log_received.emit(msg.log_message)
            self._last_log = msg.log_message

    def send_command(self, command: int, slam_mode: int = 0,
                     drive_mode: int = 0, map_name: str = "",
                     save_directory: str = ""):
        req = MapperCommand.Request()
        req.command        = command
        req.slam_mode      = slam_mode
        req.drive_mode     = drive_mode
        req.map_name       = map_name
        req.save_directory = save_directory
        self._cmd_client.call_async(req)
