#!/usr/bin/env python3
"""
SIL 벽 탐지 일관성 테스트 (Gazebo 재시작 방식).

각 시나리오마다 Gazebo를 종료 후 다른 spawn 위치/각도로 재시작하여
Hough 변환으로 최장 벽을 탐지하고 월드 프레임 각도 일관성을 검증한다.

검증 원리:
  같은 물리 벽 → (로봇 프레임 각도 + spawn_yaw) 는 spawn 위치에 무관하게 일정

사전 조건:
  다른 Gazebo 인스턴스가 종료되어 있어야 함.

실행:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/Gazebo/scripts/sil_hough_spawn_test.py
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── 경로 ──────────────────────────────────────────────────────────────────
LAUNCH_FILE = os.path.join(
    os.path.expanduser('~'),
    'Study/ros2_3dslam_ws/src/Gazebo/launch/gazebo.launch.py')

# ── 설정 ──────────────────────────────────────────────────────────────────
STARTUP_SEC      = 15.0   # Gazebo 기동 대기 시간
SHUTDOWN_SEC     = 3.0    # 종료 대기 시간
SCAN_TIMEOUT     = 8.0    # 스캔 수신 타임아웃
ANGLE_TOL_DEG    = 3.0    # 월드 프레임 각도 허용 오차
LENGTH_TOL_RATIO = 0.15   # 길이 일관성 허용 오차 (±15%)

# ── Hough 파라미터 (find_walls_hough.py와 동일) ───────────────────────────
GRID_RANGE_M   = 20.0
GRID_RES_M     = 0.05
HOUGH_THRESH   = 30
MIN_LINE_LEN   = 0.8
MAX_LINE_GAP   = 0.15


# ── Hough 탐지 함수 (인라인) ──────────────────────────────────────────────

def polar_to_xy(msg: LaserScan) -> np.ndarray:
    pts = []
    for i, r in enumerate(msg.ranges):
        if msg.range_min < r < msg.range_max:
            theta = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(theta), r * math.sin(theta)))
    return np.array(pts) if pts else np.empty((0, 2))


def xy_to_image(pts, grid_range, res):
    size   = int(2 * grid_range / res) + 1
    center = size // 2
    img    = np.zeros((size, size), dtype=np.uint8)
    for x, y in pts:
        px = int(center + x / res)
        py = int(center - y / res)
        if 0 <= px < size and 0 <= py < size:
            img[py, px] = 255
    return img, center


def normalize_angle(deg: float) -> float:
    deg = deg % 180.0
    return deg - 180.0 if deg >= 90.0 else deg


def angle_diff(a: float, b: float) -> float:
    d = abs(normalize_angle(a - b))
    return min(d, 180.0 - d)


def detect_walls_hough(pts: np.ndarray) -> list[dict]:
    img, center = xy_to_image(pts, GRID_RANGE_M, GRID_RES_M)
    img = cv2.dilate(img, np.ones((2, 2), np.uint8), iterations=1)

    lines = cv2.HoughLinesP(
        img, rho=1, theta=np.pi / 180,
        threshold=HOUGH_THRESH,
        minLineLength=int(MIN_LINE_LEN / GRID_RES_M),
        maxLineGap   =int(MAX_LINE_GAP / GRID_RES_M))
    if lines is None:
        return []

    walls = []
    for line in lines:
        x1p, y1p, x2p, y2p = line[0]
        x1 = (x1p - center) * GRID_RES_M
        y1 = (center - y1p) * GRID_RES_M
        x2 = (x2p - center) * GRID_RES_M
        y2 = (center - y2p) * GRID_RES_M
        dx, dy = x2 - x1, y2 - y1
        walls.append({
            "length": math.hypot(dx, dy),
            "angle":  normalize_angle(math.degrees(math.atan2(dy, dx))),
        })
    walls.sort(key=lambda w: w["length"], reverse=True)
    return walls


# ── Gazebo 프로세스 관리 ──────────────────────────────────────────────────

def kill_gazebo_processes() -> None:
    """이전 Gazebo/브리지 프로세스를 종료한다."""
    patterns = ['ign gazebo', 'parameter_bridge', 'odom_to_tf.py',
                'robot_state_publisher', 'rviz2', 'rqt_robot_steering',
                'ruby.*gz gazebo']
    for pat in patterns:
        subprocess.run(['pkill', '-f', pat],
                       capture_output=True, check=False)
    time.sleep(SHUTDOWN_SEC)


def launch_gazebo(spawn_x: float, spawn_y: float, spawn_yaw_deg: float):
    """Gazebo를 백그라운드로 기동하고 프로세스 핸들을 반환."""
    env = os.environ.copy()
    cmd = [
        'ros2', 'launch', LAUNCH_FILE,
        f'spawn_x:={spawn_x}',
        f'spawn_y:={spawn_y}',
        f'spawn_yaw:={spawn_yaw_deg}',
        'odom_tf:=true',
    ]
    log = open('/tmp/sil_gazebo.log', 'w')
    proc = subprocess.Popen(
        cmd, stdout=log, stderr=subprocess.STDOUT,
        env=env, preexec_fn=os.setsid)
    return proc, log


# ── 스캔 수신 노드 ────────────────────────────────────────────────────────

class ScanWaiter(Node):
    def __init__(self):
        super().__init__('sil_scan_waiter')
        self._scan: LaserScan | None = None
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg: LaserScan) -> None:
        self._scan = msg

    def wait_for_scan(self, timeout: float = SCAN_TIMEOUT
                      ) -> LaserScan | None:
        self._scan = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._scan is not None:
                return self._scan
        return None


# ── 테스트 시나리오 ───────────────────────────────────────────────────────
# (이름, spawn_x, spawn_y, spawn_yaw_deg)

SCENARIOS = [
    ("기본 spawn (-13.10, 2.64, 0°)",   -13.10,  2.64,   0.0),
    ("X +2m (-11.10, 2.64, 0°)",        -11.10,  2.64,   0.0),
    ("Y +1m (-13.10, 3.64, 0°)",        -13.10,  3.64,   0.0),
    ("회전 30° (-13.10, 2.64, 30°)",    -13.10,  2.64,  30.0),
    ("회전 -45° (-13.10, 2.64, -45°)",  -13.10,  2.64, -45.0),
    ("회전 90° (-13.10, 2.64, 90°)",    -13.10,  2.64,  90.0),
    ("복합 (-11.10, 3.64, 45°)",        -11.10,  3.64,  45.0),
]


# ── 메인 ──────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = ScanWaiter()

    results = []
    W = 90
    print(f"\n{'='*W}")
    print(f"  SIL Hough 벽 탐지 일관성 테스트  |  시나리오: {len(SCENARIOS)}  "
          f"|  각도 허용 오차: ±{ANGLE_TOL_DEG}°")
    print(f"{'='*W}")
    print(f"{'시나리오':<36} {'spawn_yaw':>10} {'벽(robot)':>10} "
          f"{'벽(world)':>10} {'길이(m)':>8}  결과")
    print(f"{'-'*W}")

    reference_world_angle: float | None = None
    reference_length:      float | None = None
    passed = failed = skipped = 0
    log_file = None

    try:
        for name, sx, sy, syaw in SCENARIOS:
            # 이전 Gazebo 종료 + 새로 기동
            kill_gazebo_processes()
            _, log_file = launch_gazebo(sx, sy, syaw)

            # 기동 대기
            time.sleep(STARTUP_SEC)

            scan = node.wait_for_scan()
            if scan is None:
                print(f"  {name:<34}  SKIP (스캔 수신 실패)")
                skipped += 1
                continue

            pts   = polar_to_xy(scan)
            walls = detect_walls_hough(pts)
            if not walls:
                print(f"  {name:<34}  SKIP (벽 탐지 실패)")
                skipped += 1
                continue

            best = walls[0]
            robot_angle = best["angle"]
            length      = best["length"]
            world_angle = normalize_angle(robot_angle + syaw)

            if reference_world_angle is None:
                reference_world_angle = world_angle
                reference_length      = length
                diff   = 0.0
                status = "BASE"
            else:
                diff   = angle_diff(world_angle, reference_world_angle)
                # 길이도 크게 다르면 다른 벽이 탐지된 것
                len_ratio = abs(length - reference_length) / reference_length
                ang_ok    = diff <= ANGLE_TOL_DEG
                len_ok    = len_ratio <= LENGTH_TOL_RATIO
                if ang_ok and len_ok:
                    status = "PASS"
                    passed += 1
                else:
                    status = f"FAIL({'각도' if not ang_ok else ''}"\
                             f"{'/' if not ang_ok and not len_ok else ''}"\
                             f"{'길이' if not len_ok else ''})"
                    failed += 1

            results.append((name, syaw, robot_angle, world_angle, length, diff, status))
            print(f"  {name:<34} {syaw:>9.1f}° {robot_angle:>9.1f}° "
                  f"{world_angle:>9.1f}° {length:>8.3f}  {status}")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        kill_gazebo_processes()
        if log_file:
            log_file.close()

    print(f"{'='*W}")
    print(f"  결과: {passed} PASS / {failed} FAIL / {skipped} SKIP  "
          f"(기준 월드 각도: {reference_world_angle:.1f}° / 길이: {reference_length:.2f}m)\n")

    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
