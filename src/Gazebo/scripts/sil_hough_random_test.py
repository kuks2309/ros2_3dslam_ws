#!/usr/bin/env python3
"""
SIL 랜덤 위치 벽 탐지 일관성 테스트 (Gazebo 재시작 방식).

각 시나리오마다:
  1. kill_all_ros2.sh 로 기존 ROS2/Gazebo 프로세스 정리
  2. 랜덤 (x, y, yaw) spawn 위치로 Gazebo 기동
  3. /scan 수신 → Hough 변환 → 최장 벽 탐지
  4. Gazebo 창 캡처 (capture_screen.py)
  5. 월드 프레임 각도 일관성 검증

검증 원리:
  같은 물리 벽 → (로봇 프레임 각도 + spawn_yaw) 는 spawn 위치 무관하게 일정

실행:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/Gazebo/scripts/sil_hough_random_test.py
"""
import math
import os
import random
import subprocess
import sys
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── 경로 ──────────────────────────────────────────────────────────────────
HOME          = os.path.expanduser('~')
PKG_ROOT      = os.path.join(HOME, 'Study/ros2_3dslam_ws/src/Gazebo')
LAUNCH_FILE   = os.path.join(PKG_ROOT, 'launch/gazebo.launch.py')
KILL_SCRIPT   = os.path.join(HOME, 'parking_robot_ros2_ws/kill_all_ros2.sh')
CAPTURE_TOOL  = os.path.join(HOME, '.claude/capture_screen.py')
PROJECT_ROOT  = os.path.join(HOME, 'Study/ros2_3dslam_ws')

# ── 설정 ──────────────────────────────────────────────────────────────────
STARTUP_SEC      = 25.0   # Gazebo GUI 렌더까지 충분히 대기
SHUTDOWN_SEC     = 2.5
SCAN_TIMEOUT     = 8.0
ANGLE_TOL_DEG    = 3.0    # 축(0° 또는 90°) 정렬 허용 오차

# Warehouse 안전 spawn 범위 (월드 프레임)
X_RANGE = (-13.0, 4.0)
Y_RANGE = (-4.0, 5.0)
YAW_RANGE_DEG = (-180.0, 180.0)
N_RANDOM_POSITIONS = 12          # 최소 10개 이상

RANDOM_SEED = 2026               # 재현성을 위해 고정

# ── Hough 파라미터 ────────────────────────────────────────────────────────
GRID_RANGE_M = 20.0
GRID_RES_M   = 0.05
HOUGH_THRESH = 30
MIN_LINE_LEN = 0.8
MAX_LINE_GAP = 0.15


# ── Hough 탐지 ────────────────────────────────────────────────────────────

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


def axis_aligned_diff(world_angle: float) -> tuple[float, str]:
    """
    창고 벽은 모두 월드 축 정렬(0° 또는 90°)이므로,
    world_angle이 0°축 또는 90°축에 얼마나 가까운지 반환.
    Returns: (min_diff_deg, 'H' or 'V')
    """
    h_diff = min(abs(world_angle), abs(abs(world_angle) - 180.0))   # 0°
    v_diff = abs(abs(world_angle) - 90.0)                            # ±90°
    if h_diff <= v_diff:
        return h_diff, 'H'
    return v_diff, 'V'


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


# ── 프로세스 관리 ─────────────────────────────────────────────────────────

def kill_all_ros2() -> None:
    """
    ROS2/Gazebo 프로세스 정리. kill_all_ros2.sh의 pkill -f "ros2"는
    경로에 ros2 를 포함한 테스트 스크립트 자신도 kill하므로,
    자신(os.getpid())을 제외하고 개별 패턴별로 처리한다.
    """
    print("  [정리] ROS2/Gazebo 프로세스 종료 중...")
    my_pid = os.getpid()

    # ros2 daemon 정리
    subprocess.run(['ros2', 'daemon', 'stop'],
                   capture_output=True, check=False, timeout=5)

    # 정밀 패턴 (자신과 겹치지 않음)
    safe_patterns = [
        'ign gazebo',
        'ruby.*gz gazebo',
        'gzserver', 'gzclient',
        'parameter_bridge',
        'odom_to_tf.py',
        'robot_state_publisher',
        'rviz2',
        'rqt_robot_steering',
        'mapper_orchestrator',
        'wall_aligner', 'map_alignment_checker', 'exploration_planner',
        'mapper_ui_node', 'amr_motion_control',
        'ros2-daemon',
    ]
    for pat in safe_patterns:
        subprocess.run(['pkill', '-9', '-f', pat],
                       capture_output=True, check=False)

    # /opt/ros/ 경로 프로세스 (자신 제외)
    try:
        result = subprocess.run(
            ['pgrep', '-f', '/opt/ros/'],
            capture_output=True, text=True, check=False, timeout=3)
        for pid_str in result.stdout.split():
            try:
                pid = int(pid_str)
                if pid != my_pid:
                    subprocess.run(['kill', '-9', str(pid)],
                                   capture_output=True, check=False)
            except ValueError:
                pass
    except Exception:
        pass

    time.sleep(SHUTDOWN_SEC)


def launch_gazebo(x: float, y: float, yaw_deg: float):
    """Gazebo 기동, (process, log_fd) 반환."""
    cmd = [
        'ros2', 'launch', LAUNCH_FILE,
        f'spawn_x:={x}', f'spawn_y:={y}', f'spawn_yaw:={yaw_deg}',
        'odom_tf:=true',
    ]
    log = open('/tmp/sil_gazebo.log', 'w')
    proc = subprocess.Popen(
        cmd, stdout=log, stderr=subprocess.STDOUT,
        env=os.environ.copy(), preexec_fn=os.setsid)
    return proc, log


def capture_window(label: str, window_title_hint: str = "Gazebo") -> str:
    """
    capture_screen.py 로 Gazebo 창을 캡처. 실패 시 전체 화면 캡처.
    반환: 저장된 이미지 경로 (or "")
    """
    # 창 리스트에서 Gazebo 찾기
    try:
        result = subprocess.run(
            ['python3', CAPTURE_TOOL, '--mode', 'list'],
            capture_output=True, text=True, timeout=8)
        import json
        windows = json.loads(result.stdout)
        target = next(
            (w for w in windows if window_title_hint.lower() in w['title'].lower()),
            None)
        if target:
            cap_result = subprocess.run(
                ['python3', CAPTURE_TOOL,
                 '--project', PROJECT_ROOT,
                 '--mode', 'window', '--window-id', target['id'],
                 '--label', label],
                capture_output=True, text=True, timeout=10)
            # 저장 경로 추출 (stdout 마지막 줄에 경로 포함)
            for line in cap_result.stdout.splitlines():
                if '.png' in line:
                    return line.strip()
    except Exception as e:
        print(f"  [캡처 경고] window 모드 실패: {e}")

    # 폴백: 전체 화면
    try:
        cap_result = subprocess.run(
            ['python3', CAPTURE_TOOL,
             '--project', PROJECT_ROOT,
             '--mode', 'full', '--label', f'{label}_full'],
            capture_output=True, text=True, timeout=10)
        for line in cap_result.stdout.splitlines():
            if '.png' in line:
                return line.strip()
    except Exception as e:
        print(f"  [캡처 경고] full 모드 실패: {e}")
    return ""


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


# ── 시나리오 생성 ─────────────────────────────────────────────────────────

def generate_random_scenarios(n: int = N_RANDOM_POSITIONS
                               ) -> list[tuple[str, float, float, float]]:
    random.seed(RANDOM_SEED)
    scenarios = [
        # 기준점: 기본 spawn
        ("기본 spawn",          -13.10, 2.64, 0.0),
    ]
    for i in range(n):
        x   = round(random.uniform(*X_RANGE),       2)
        y   = round(random.uniform(*Y_RANGE),       2)
        yaw = round(random.uniform(*YAW_RANGE_DEG), 1)
        scenarios.append((f"랜덤 #{i+1:02d}", x, y, yaw))
    return scenarios


# ── 메인 ──────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = ScanWaiter()

    scenarios = generate_random_scenarios()

    W = 110
    print(f"\n{'='*W}")
    print(f"  SIL 랜덤 위치 Hough 벽 탐지 일관성 테스트  "
          f"|  시나리오: {len(scenarios)}  |  각도 허용 오차: ±{ANGLE_TOL_DEG}°")
    print(f"{'='*W}")
    print(f"{'시나리오':<14} {'X':>7} {'Y':>6} {'yaw':>7} "
          f"{'벽(robot)':>10} {'벽(world)':>10} {'길이(m)':>8}  "
          f"{'캡처':<35}  결과")
    print(f"{'-'*W}")

    passed = failed = skipped = 0
    h_count = v_count = 0
    log_file = None
    all_results = []

    try:
        for scn_idx, (name, sx, sy, syaw) in enumerate(scenarios, 1):
            # 이전 Gazebo 정리 + 새로 기동
            kill_all_ros2()
            _, log_file = launch_gazebo(sx, sy, syaw)

            time.sleep(STARTUP_SEC)
            scan = node.wait_for_scan()

            if scan is None:
                print(f"  {name:<12} {sx:>7.2f} {sy:>6.2f} {syaw:>7.1f}°  "
                      f"SKIP (스캔 실패)")
                skipped += 1
                continue

            pts   = polar_to_xy(scan)
            walls = detect_walls_hough(pts)
            if not walls:
                print(f"  {name:<12} {sx:>7.2f} {sy:>6.2f} {syaw:>7.1f}°  "
                      f"SKIP (벽 탐지 실패)")
                skipped += 1
                continue

            best = walls[0]
            robot_angle = best["angle"]
            length      = best["length"]
            world_angle = normalize_angle(robot_angle + syaw)

            # 캡처
            label = f"scn{scn_idx:02d}_x{sx}_y{sy}_yaw{syaw}".replace('.', '_')
            capture_path = capture_window(label)
            cap_short = os.path.basename(capture_path) if capture_path else "없음"

            # 축 정렬 검증 — 0° 또는 ±90° 중 더 가까운 쪽 선택
            diff, axis = axis_aligned_diff(world_angle)
            if diff <= ANGLE_TOL_DEG:
                status = f"PASS({axis})"
                passed += 1
                if axis == 'H':
                    h_count += 1
                else:
                    v_count += 1
            else:
                status = f"FAIL ({diff:.1f}°)"
                failed += 1

            all_results.append({
                "idx": scn_idx, "name": name, "x": sx, "y": sy, "yaw": syaw,
                "robot_angle": robot_angle, "world_angle": world_angle,
                "length": length, "diff": diff, "axis": axis, "status": status,
                "capture": capture_path,
            })

            print(f"  {name:<12} {sx:>7.2f} {sy:>6.2f} {syaw:>7.1f}° "
                  f"{robot_angle:>9.1f}° {world_angle:>9.1f}° {length:>8.3f}  "
                  f"{cap_short[:33]:<35}  {status}")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        kill_all_ros2()
        if log_file:
            log_file.close()

    print(f"{'='*W}")
    total = passed + failed
    rate  = passed / total * 100 if total > 0 else 0.0
    print(f"  결과: {passed} PASS ({h_count} 가로/H + {v_count} 세로/V) / "
          f"{failed} FAIL / {skipped} SKIP  (축 정렬 통과율 {rate:.1f}%)\n")

    # CSV 요약 저장
    csv_path = os.path.join(PROJECT_ROOT, 'experiments/capture/sil_results.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w') as f:
        f.write("idx,name,x,y,yaw_deg,robot_angle,world_angle,length_m,diff_deg,axis,status,capture\n")
        for r in all_results:
            f.write(f"{r['idx']},{r['name']},{r['x']},{r['y']},{r['yaw']},"
                    f"{r['robot_angle']:.2f},{r['world_angle']:.2f},{r['length']:.3f},"
                    f"{r['diff']:.2f},{r['axis']},{r['status']},{r['capture']}\n")
    print(f"  CSV 결과: {csv_path}\n")

    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
