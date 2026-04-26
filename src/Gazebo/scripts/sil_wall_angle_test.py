#!/usr/bin/env python3
"""
SIL (Software-in-the-Loop) 벽 각도 탐지 테스트.

로봇 초기 위치·각도를 바꾸며 최장 벽의 월드 프레임 각도 일관성을 검증한다.

검증 원리:
  같은 물리 벽 → 로봇 프레임 벽 각도 + 로봇 yaw = 월드 프레임 벽 각도 (일정)

사전 조건:
  ros2 launch <gazebo_pkg> gazebo.launch.py  (Ignition Gazebo + Bridge 기동)

실행:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/Gazebo/scripts/sil_wall_angle_test.py
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

ROBOT_MODEL   = "robot_scan"
WORLD_NAME    = "my_world"
SETTLE_SEC    = 1.5      # 텔레포트 후 안정화 대기 시간
SCAN_TIMEOUT  = 5.0      # 스캔 수신 타임아웃
ANGLE_TOL_DEG = 5.0      # 월드 프레임 각도 허용 오차 (°)
GAP_M         = 0.25     # 세그먼트 분리 거리 임계값 (m)
MIN_PTS       = 10       # 세그먼트 최소 점 수


# ── 알고리즘 헬퍼 ──────────────────────────────────────────────────────────

def polar_to_cartesian(msg: LaserScan) -> np.ndarray:
    pts = []
    for i, r in enumerate(msg.ranges):
        if msg.range_min < r < msg.range_max:
            theta = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(theta), r * math.sin(theta)))
    return np.array(pts) if pts else np.empty((0, 2))


def split_segments(pts: np.ndarray) -> list[np.ndarray]:
    if len(pts) < MIN_PTS:
        return []
    segs, seg = [], [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - seg[-1]) < GAP_M:
            seg.append(p)
        else:
            if len(seg) >= MIN_PTS:
                segs.append(np.array(seg))
            seg = [p]
    if len(seg) >= MIN_PTS:
        segs.append(np.array(seg))
    return segs


def fit_segment(seg: np.ndarray) -> tuple[float, float]:
    """(길이_m, 각도_deg) 반환. 각도 범위: [-90, 90)."""
    centroid = seg.mean(axis=0)
    _, _, vh = np.linalg.svd(seg - centroid)
    direction = vh[0]
    proj = (seg - centroid) @ direction
    length = float(proj.max() - proj.min())
    angle  = math.degrees(math.atan2(direction[1], direction[0]))
    return length, angle


def normalize_angle(deg: float) -> float:
    """[-90, 90) 범위로 정규화 (직선은 방향 부호 무관)."""
    deg = deg % 180.0
    return deg - 180.0 if deg >= 90.0 else deg


def angle_diff(a: float, b: float) -> float:
    """두 정규화 각도 간 최소 차이 (절댓값)."""
    d = abs(normalize_angle(a - b))
    return min(d, 180.0 - d)


def yaw_to_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2
    return 0.0, 0.0, math.sin(half), math.cos(half)


# ── Ignition Gazebo 텔레포트 ───────────────────────────────────────────────

def teleport(x: float, y: float, yaw_deg: float) -> bool:
    qx, qy, qz, qw = yaw_to_quat(yaw_deg)
    req = (f'name: "{ROBOT_MODEL}", '
           f'position: {{x: {x}, y: {y}, z: 0.16}}, '
           f'orientation: {{x: {qx}, y: {qy}, z: {qz:.6f}, w: {qw:.6f}}}')
    cmd = [
        'ign', 'service',
        '-s', f'/world/{WORLD_NAME}/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    return 'true' in result.stdout.lower()


# ── ROS2 헬퍼 노드 ─────────────────────────────────────────────────────────

class SilNode(Node):
    def __init__(self):
        super().__init__('sil_wall_angle_test')
        self._scan: LaserScan | None = None
        self._yaw:  float | None     = None
        self.create_subscription(LaserScan, '/scan',  self._on_scan,  10)
        self.create_subscription(Odometry,  '/odom',  self._on_odom,  10)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        q  = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.degrees(math.atan2(siny, cosy))

    def wait_fresh(self, timeout: float = SCAN_TIMEOUT) -> bool:
        """텔레포트 후 새 스캔과 odometry를 기다린다."""
        self._scan = None
        self._yaw  = None
        deadline   = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._scan is not None and self._yaw is not None:
                return True
        return False

    def detect_longest_wall(self) -> tuple[float, float] | None:
        """최장 벽의 (robot_frame_angle_deg, length_m) 반환. 실패 시 None."""
        if self._scan is None:
            return None
        pts  = polar_to_cartesian(self._scan)
        segs = split_segments(pts)
        if not segs:
            return None
        results = sorted([fit_segment(s) for s in segs],
                         key=lambda x: x[0], reverse=True)
        length, angle = results[0]
        return angle, length


# ── 테스트 시나리오 ────────────────────────────────────────────────────────
# 형식: (이름, x, y, yaw_deg)
# 로봇을 이 자세에 놓고 최장 벽을 검출, 월드 프레임 각도를 비교한다.

# 초기 spawn 위치: SDF <pose> -13.10 2.64 0 0 0 0 </pose>
SPAWN_X, SPAWN_Y = -13.10, 2.64

SCENARIOS = [
    ("초기 spawn 위치 (0°)",           SPAWN_X,       SPAWN_Y,       0.0),
    ("X +2m 이동 (0°)",          SPAWN_X + 2.0, SPAWN_Y,       0.0),
    ("X -1m 이동 (0°)",          SPAWN_X - 1.0, SPAWN_Y,       0.0),
    ("Y +1m 이동 (0°)",          SPAWN_X,       SPAWN_Y + 1.0, 0.0),
    ("회전 30°",                 SPAWN_X,       SPAWN_Y,      30.0),
    ("회전 45°",                 SPAWN_X,       SPAWN_Y,      45.0),
    ("회전 -30°",                SPAWN_X,       SPAWN_Y,     -30.0),
    ("회전 60°",                 SPAWN_X,       SPAWN_Y,      60.0),
    ("이동+회전 (2m, 45°)",       SPAWN_X + 2.0, SPAWN_Y,      45.0),
    ("이동+회전 (-1m, -45°)",     SPAWN_X - 1.0, SPAWN_Y + 0.5,-45.0),
]


# ── 메인 ──────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = SilNode()

    reference_world_angle: float | None = None
    results = []
    W = 82

    print(f"\n{'='*W}")
    print(f"  SIL 벽 각도 일관성 테스트  |  허용 오차: ±{ANGLE_TOL_DEG}°")
    print(f"{'='*W}")
    print(f"{'시나리오':<26} {'로봇yaw':>7} {'벽(robot)':>9} {'벽(world)':>9} "
          f"{'기준대비':>8} {'길이(m)':>8}  결과")
    print(f"{'-'*W}")

    passed = failed = skipped = 0

    for name, x, y, yaw_cmd in SCENARIOS:
        # 텔레포트
        ok = teleport(x, y, yaw_cmd)
        if not ok:
            print(f"{'  ' + name:<26}  SKIP (teleport 실패)")
            skipped += 1
            continue

        time.sleep(SETTLE_SEC)

        # 스캔 + odometry 수신
        if not node.wait_fresh():
            print(f"{'  ' + name:<26}  SKIP (스캔/odometry 수신 실패)")
            skipped += 1
            continue

        detected = node.detect_longest_wall()
        if detected is None:
            print(f"{'  ' + name:<26}  SKIP (세그먼트 없음)")
            skipped += 1
            continue

        robot_angle, length = detected
        robot_yaw           = node._yaw or 0.0   # odometry에서 읽은 실제 yaw
        world_angle         = normalize_angle(robot_angle + robot_yaw)

        # 첫 번째 시나리오를 기준으로 설정
        if reference_world_angle is None:
            reference_world_angle = world_angle
            diff  = 0.0
            status = "BASE"
        else:
            diff   = angle_diff(world_angle, reference_world_angle)
            status = "PASS" if diff <= ANGLE_TOL_DEG else "FAIL"
            if status == "PASS":
                passed += 1
            else:
                failed += 1

        results.append((name, robot_yaw, robot_angle, world_angle, diff, length, status))
        print(f"  {name:<24} {robot_yaw:>7.1f}° {robot_angle:>8.1f}° "
              f"{world_angle:>8.1f}° {diff:>7.1f}° {length:>8.3f}m  {status}")

    print(f"{'='*W}")
    print(f"  결과: {passed} PASS / {failed} FAIL / {skipped} SKIP  "
          f"(기준 월드 각도: {reference_world_angle:.1f}°)\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
