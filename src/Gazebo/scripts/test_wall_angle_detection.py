#!/usr/bin/env python3
"""
최장 벽 각도 탐지 알고리즘 단위 테스트.

합성 포인트 클라우드를 생성하여 fit_segment()의 각도 추정 정확도를 검증한다.
실행: python3 src/Gazebo/scripts/test_wall_angle_detection.py
"""
import math
import sys
import numpy as np

# ── 피테스트 함수 인라인 (ROS2 의존성 없음) ────────────────────────────────

def fit_segment(seg: np.ndarray) -> tuple[float, float, float, float, int]:
    centroid = seg.mean(axis=0)
    _, _, vh = np.linalg.svd(seg - centroid)
    direction = vh[0]
    proj = (seg - centroid) @ direction
    length = float(proj.max() - proj.min())
    angle = math.degrees(math.atan2(direction[1], direction[0]))
    return length, angle, float(centroid[0]), float(centroid[1]), len(seg)


def normalize_angle(deg: float) -> float:
    """각도를 [-90, 90) 범위로 정규화 (직선은 방향 부호 무관)."""
    deg = deg % 180.0
    if deg >= 90.0:
        deg -= 180.0
    return deg


def make_wall(angle_deg: float, length: float, cx: float, cy: float,
              n: int = 100, noise: float = 0.0) -> np.ndarray:
    """주어진 각도·길이·중심으로 벽 점군 생성. noise(m) 추가 가능."""
    theta = math.radians(angle_deg)
    dx, dy = math.cos(theta), math.sin(theta)
    t = np.linspace(-length / 2, length / 2, n)
    pts = np.column_stack([cx + t * dx, cy + t * dy])
    if noise > 0:
        pts += np.random.default_rng(42).normal(0, noise, pts.shape)
    return pts


# ── 테스트 케이스 ──────────────────────────────────────────────────────────

CASES = [
    # (이름, 각도, 길이, cx, cy, noise, 허용오차)
    ("수평 벽 (0°)",         0.0,   5.0,  0.0,  0.0, 0.000, 0.1),
    ("수직 벽 (90°)",       90.0,   5.0,  0.0,  0.0, 0.000, 0.1),
    ("대각 벽 (45°)",       45.0,   5.0,  0.0,  0.0, 0.000, 0.1),
    ("대각 벽 (-45°)",     -45.0,   5.0,  0.0,  0.0, 0.000, 0.1),
    ("대각 벽 (30°)",       30.0,   4.0,  1.0, -2.0, 0.000, 0.1),
    ("대각 벽 (135°≡-45°)",135.0,   4.0,  0.0,  0.0, 0.000, 0.1),
    ("노이즈 5mm (0°)",      0.0,  10.0,  3.0,-10.0, 0.005, 1.0),
    ("노이즈 5mm (90°)",    90.0,   5.0, -2.0,  0.0, 0.005, 1.0),
    ("노이즈 5mm (45°)",    45.0,   6.0,  0.0,  3.0, 0.005, 1.0),
    ("노이즈 1cm (0°)",      0.0,  10.0,  2.8,-10.3, 0.010, 2.0),
    ("노이즈 1cm (90°)",    90.0,   4.6, -1.9, -1.6, 0.010, 2.0),
    ("짧은 벽 (0°, 0.5m)",   0.0,   0.5,  0.0,  0.0, 0.000, 1.0),
    ("긴 벽 (0°, 20m)",      0.0,  20.0,  5.0,  0.0, 0.000, 0.1),
    ("오프셋 중심 (45°)",   45.0,   8.0, 15.0,-10.0, 0.000, 0.1),
]


def run_tests() -> int:
    passed = failed = 0
    W = 72
    print(f"\n{'='*W}")
    print(f"  벽 각도 탐지 알고리즘 테스트  |  총 {len(CASES)}개 케이스")
    print(f"{'='*W}")
    print(f"{'케이스':<28} {'기대(°)':>7} {'검출(°)':>7} {'오차(°)':>7} {'허용(°)':>7}  결과")
    print(f"{'-'*W}")

    for name, angle_deg, length, cx, cy, noise, tol in CASES:
        seg = make_wall(angle_deg, length, cx, cy, noise=noise)
        _, detected_angle, _, _, _ = fit_segment(seg)

        expected_norm = normalize_angle(angle_deg)
        detected_norm = normalize_angle(detected_angle)
        error = abs(normalize_angle(detected_norm - expected_norm))
        # 180° 대칭 처리
        error = min(error, abs(error - 180.0))

        ok = error <= tol
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        else:
            failed += 1

        print(f"{name:<28} {expected_norm:>7.1f} {detected_norm:>7.1f} "
              f"{error:>7.2f} {tol:>7.1f}  {status}")

    print(f"{'='*W}")
    print(f"  결과: {passed} PASS / {failed} FAIL / {len(CASES)} 총계")
    print(f"{'='*W}\n")
    return failed


# ── 라이브 스캔 검증 (Gazebo 실행 중일 때만) ──────────────────────────────

def run_live_validation():
    """
    /scan 토픽에서 실제 데이터를 받아 검출 결과를 출력한다.
    --live 플래그로 실행 시 호출됨.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan

    def polar_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max):
        pts = []
        for i, r in enumerate(ranges):
            if range_min < r < range_max:
                theta = angle_min + i * angle_increment
                pts.append((r * math.cos(theta), r * math.sin(theta)))
        return np.array(pts) if pts else np.empty((0, 2))

    def split_segments(pts, gap=0.25, min_pts=10):
        if len(pts) < min_pts:
            return []
        segs, seg = [], [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - seg[-1]) < gap:
                seg.append(p)
            else:
                if len(seg) >= min_pts:
                    segs.append(np.array(seg))
                seg = [p]
        if len(seg) >= min_pts:
            segs.append(np.array(seg))
        return segs

    class LiveValidator(Node):
        def __init__(self):
            super().__init__('wall_angle_validator')
            self.done = False
            self.create_subscription(LaserScan, '/scan', self._cb, 10)
            self.get_logger().info("Waiting for /scan ...")

        def _cb(self, msg):
            if self.done:
                return
            self.done = True
            pts = polar_to_cartesian(
                msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max)
            segs = split_segments(pts)
            if not segs:
                print("세그먼트 없음")
                return
            results = sorted(
                [fit_segment(s) for s in segs], key=lambda x: x[0], reverse=True)
            W = 62
            print(f"\n{'='*W}")
            print(f"  라이브 검증  |  유효점: {len(pts)}  |  세그먼트: {len(segs)}")
            print(f"{'='*W}")
            print(f"{'순위':>4}  {'길이(m)':>8}  {'각도(°)':>8}  {'중심X':>8}  {'중심Y':>8}  {'점수':>5}")
            print(f"{'-'*W}")
            for rank, (length, angle, cx, cy, npts) in enumerate(results[:10], 1):
                marker = " ◀ 최장" if rank == 1 else ""
                norm = normalize_angle(angle)
                print(f"{rank:>4}  {length:>8.3f}  {norm:>8.1f}  {cx:>8.3f}  {cy:>8.3f}  {npts:>5}{marker}")
            print(f"{'='*W}\n")

    rclpy.init()
    node = LiveValidator()
    rclpy.spin_once(node, timeout_sec=6.0)
    if not node.done:
        print("ERROR: /scan 수신 실패 (6 s 타임아웃)")
    node.destroy_node()
    rclpy.shutdown()


# ── 진입점 ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    if '--live' in sys.argv:
        run_live_validation()
    else:
        fail_count = run_tests()
        sys.exit(fail_count)
