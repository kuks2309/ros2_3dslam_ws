#!/usr/bin/env python3
"""
벽 탐지 검증용 시각화 도구.

LiDAR 포인트 + Hough 검출 라인을 한 이미지에 겹쳐 그려
Gazebo 스크린샷과 비교할 수 있도록 한다.

출력:
  - 회색 점: LiDAR 원본 포인트
  - 녹색 선: 검출된 모든 벽 (상위 K개)
  - 빨간 선: 최장 벽 (1위)
  - 파란 삼각형: 로봇 위치 (원점)
  - 텍스트: 길이(m), 각도(°)

실행 (단독):
  python3 src/Gazebo/scripts/wall_detection_visualizer.py
실행 (SIL 테스트 내부):
  visualize_detection(scan, walls, save_path)
"""
import math
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# Hough 파라미터 (find_walls_hough.py와 동일)
GRID_RANGE_M = 10.0     # 시각화는 10m로 축소 (가독성)
GRID_RES_M   = 0.02     # 0.02m/픽셀 → 1000×1000 이미지
HOUGH_THRESH = 30
MIN_LINE_LEN = 0.8
MAX_LINE_GAP = 0.15
TOP_K        = 5        # 상위 K 라인만 표시
INLIER_TOL_M = 0.1      # Inlier 판정 거리 (m)


# ── 탐지 로직 (find_walls_hough.py 동일) ──────────────────────────────────

def polar_to_xy(msg: LaserScan) -> np.ndarray:
    pts = []
    for i, r in enumerate(msg.ranges):
        if msg.range_min < r < msg.range_max:
            theta = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(theta), r * math.sin(theta)))
    return np.array(pts) if pts else np.empty((0, 2))


def normalize_angle(deg: float) -> float:
    deg = deg % 180.0
    return deg - 180.0 if deg >= 90.0 else deg


def detect_walls_with_inliers(pts: np.ndarray,
                               grid_range: float = 20.0,
                               res: float = 0.05) -> list[dict]:
    """
    Hough 탐지 + 각 선분에 대한 inlier_ratio 계산.
    """
    size   = int(2 * grid_range / res) + 1
    center = size // 2
    img    = np.zeros((size, size), dtype=np.uint8)
    for x, y in pts:
        px = int(center + x / res)
        py = int(center - y / res)
        if 0 <= px < size and 0 <= py < size:
            img[py, px] = 255
    img = cv2.dilate(img, np.ones((2, 2), np.uint8), iterations=1)

    lines = cv2.HoughLinesP(
        img, rho=1, theta=np.pi / 180,
        threshold=HOUGH_THRESH,
        minLineLength=int(MIN_LINE_LEN / res),
        maxLineGap   =int(MAX_LINE_GAP / res))
    if lines is None:
        return []

    walls = []
    for line in lines:
        x1p, y1p, x2p, y2p = line[0]
        x1 = (x1p - center) * res
        y1 = (center - y1p) * res
        x2 = (x2p - center) * res
        y2 = (center - y2p) * res
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue

        # Inlier ratio: 선분에 수직 거리 ≤ INLIER_TOL_M 인 포인트 비율
        nx, ny = -dy / length, dx / length           # 법선 벡터
        signed_dist = (pts[:, 0] - x1) * nx + (pts[:, 1] - y1) * ny
        # 선분 양 끝점 투영 범위 체크
        tx, ty = dx / length, dy / length            # 접선 단위 벡터
        t_coord = (pts[:, 0] - x1) * tx + (pts[:, 1] - y1) * ty
        in_range = (t_coord >= -0.2) & (t_coord <= length + 0.2)
        close = np.abs(signed_dist) < INLIER_TOL_M
        inliers = int((in_range & close).sum())
        # 이상적인 선분 위 점 수 (해상도 기반)
        expected = max(int(length / 0.02), 1)
        inlier_ratio = min(inliers / expected, 1.0)

        walls.append({
            "length":       length,
            "angle":        normalize_angle(math.degrees(math.atan2(dy, dx))),
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "inliers":      inliers,
            "inlier_ratio": inlier_ratio,
        })
    walls.sort(key=lambda w: w["length"], reverse=True)
    return walls


# ── 시각화 ────────────────────────────────────────────────────────────────

def visualize_detection(pts: np.ndarray, walls: list[dict],
                        save_path: str,
                        title: str = "Wall Detection") -> None:
    """
    LiDAR 포인트 + 검출 라인 오버레이 이미지 저장.
    """
    size   = int(2 * GRID_RANGE_M / GRID_RES_M) + 1
    center = size // 2
    img    = np.full((size, size, 3), 30, dtype=np.uint8)   # 어두운 회색 배경

    # 그리드 (1m 간격)
    for m in range(-int(GRID_RANGE_M), int(GRID_RANGE_M) + 1):
        px = int(center + m / GRID_RES_M)
        cv2.line(img, (px, 0), (px, size - 1), (60, 60, 60), 1)
        cv2.line(img, (0, px), (size - 1, px), (60, 60, 60), 1)
    # 축 강조
    cv2.line(img, (center, 0), (center, size - 1), (90, 90, 90), 1)
    cv2.line(img, (0, center), (size - 1, center), (90, 90, 90), 1)

    # LiDAR 포인트 (회색)
    for x, y in pts:
        px = int(center + x / GRID_RES_M)
        py = int(center - y / GRID_RES_M)
        if 0 <= px < size and 0 <= py < size:
            cv2.circle(img, (px, py), 2, (200, 200, 200), -1)

    # 검출 라인 (상위 K)
    for rank, w in enumerate(walls[:TOP_K]):
        color = (0, 0, 255) if rank == 0 else (0, 220, 0)   # 1위 빨강, 나머지 녹색
        thickness = 3 if rank == 0 else 2
        x1p = int(center + w["x1"] / GRID_RES_M)
        y1p = int(center - w["y1"] / GRID_RES_M)
        x2p = int(center + w["x2"] / GRID_RES_M)
        y2p = int(center - w["y2"] / GRID_RES_M)
        cv2.line(img, (x1p, y1p), (x2p, y2p), color, thickness)

        # 라벨
        mx = (x1p + x2p) // 2
        my = (y1p + y2p) // 2
        label = (f"#{rank+1} L={w['length']:.2f}m "
                 f"A={w['angle']:.1f} IR={w['inlier_ratio']:.2f}")
        cv2.putText(img, label, (mx + 5, my - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # 로봇 위치 (파란 삼각형, 전방 = +X)
    triangle = np.array([
        [center + int(0.3 / GRID_RES_M), center],
        [center - int(0.2 / GRID_RES_M), center - int(0.2 / GRID_RES_M)],
        [center - int(0.2 / GRID_RES_M), center + int(0.2 / GRID_RES_M)],
    ], dtype=np.int32)
    cv2.fillPoly(img, [triangle], (255, 100, 0))

    # 타이틀 + 범례
    cv2.putText(img, title, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, "Red: longest wall  Green: others  Blue: robot",
                (10, size - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, img)


# ── 단독 실행용 노드 ──────────────────────────────────────────────────────

class VisualizerNode(Node):
    def __init__(self, save_path: str):
        super().__init__('wall_detection_visualizer')
        self._save_path = save_path
        self._done = False
        self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self.get_logger().info(f"Waiting for /scan ... → {save_path}")

    def _cb(self, msg: LaserScan) -> None:
        if self._done:
            return
        self._done = True
        pts   = polar_to_xy(msg)
        walls = detect_walls_with_inliers(pts)
        visualize_detection(pts, walls, self._save_path,
                            title=f"Walls: {len(walls)} detected")

        # 검증 보고
        print(f"\n{'='*70}")
        print(f"  검출 검증  |  유효점: {len(pts)}  |  선분: {len(walls)}")
        print(f"{'='*70}")
        print(f"{'순위':>4}  {'길이(m)':>8}  {'각도(°)':>8}  "
              f"{'inliers':>7}  {'IR':>5}  판정")
        print(f"{'-'*70}")
        for rank, w in enumerate(walls[:10], 1):
            # IR (Inlier Ratio) 기반 검증
            ok = "✓" if w["inlier_ratio"] > 0.3 else "✗"
            print(f"  {rank:>2}  {w['length']:>8.3f}  {w['angle']:>8.1f}  "
                  f"{w['inliers']:>7}  {w['inlier_ratio']:>5.2f}  {ok}")
        print(f"{'='*70}")
        print(f"  저장: {self._save_path}\n")


def main() -> None:
    import sys
    save_path = sys.argv[1] if len(sys.argv) > 1 else \
                '/tmp/wall_detection_overlay.png'
    rclpy.init()
    node = VisualizerNode(save_path)
    rclpy.spin_once(node, timeout_sec=6.0)
    if not node._done:
        node.get_logger().error("/scan 수신 실패")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
