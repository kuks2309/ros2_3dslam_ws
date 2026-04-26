#!/usr/bin/env python3
"""
Hough 변환 기반 라이다 벽 탐지.

알고리즘:
  1. /scan (LaserScan) → 직교좌표 변환
  2. 2D 격자 이미지 래스터화
  3. cv2.HoughLinesP (확률적 Hough) 적용
  4. 검출된 선분을 (길이, 각도) 로 정리 후 출력

실행:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/Gazebo/scripts/find_walls_hough.py
"""
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── Hough 파라미터 ─────────────────────────────────────────────────────────
GRID_RANGE_M   = 20.0   # 격자 한 방향 크기 (m): -RANGE ~ +RANGE
GRID_RES_M     = 0.05   # 격자 해상도 (m/pixel)
HOUGH_RHO      = 1      # rho 해상도 (pixel)
HOUGH_THETA    = np.pi / 180   # theta 해상도 (1°)
HOUGH_THRESH   = 30     # 투표 임계값 (높을수록 엄격)
MIN_LINE_LEN   = 0.8    # 최소 선분 길이 (m)
MAX_LINE_GAP   = 0.15   # 선분 내 최대 갭 (m) — 작을수록 연속점만 연결


def polar_to_xy(msg: LaserScan) -> np.ndarray:
    pts = []
    for i, r in enumerate(msg.ranges):
        if msg.range_min < r < msg.range_max:
            theta = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(theta), r * math.sin(theta)))
    return np.array(pts) if pts else np.empty((0, 2))


def xy_to_image(pts: np.ndarray, grid_range: float,
                res: float) -> tuple[np.ndarray, int]:
    """포인트 → 이진 이미지. 중심 픽셀 인덱스도 반환."""
    size   = int(2 * grid_range / res) + 1
    center = size // 2
    img    = np.zeros((size, size), dtype=np.uint8)
    for x, y in pts:
        px = int(center + x / res)
        py = int(center - y / res)   # 이미지 y축은 아래 방향
        if 0 <= px < size and 0 <= py < size:
            img[py, px] = 255
    return img, center


def normalize_angle(deg: float) -> float:
    """[-90, 90) 정규화."""
    deg = deg % 180.0
    return deg - 180.0 if deg >= 90.0 else deg


def detect_walls(pts: np.ndarray, res: float = GRID_RES_M
                 ) -> list[dict]:
    """
    Hough 변환으로 선분 탐지.
    반환: [{"length": m, "angle": deg, "x1": ..., "y1": ..., "x2": ..., "y2": ...}, ...]
    """
    img, center = xy_to_image(pts, GRID_RANGE_M, res)

    # 팽창(dilate)으로 점 밀도 보완
    kernel = np.ones((2, 2), np.uint8)
    img    = cv2.dilate(img, kernel, iterations=1)

    min_len_px = int(MIN_LINE_LEN / res)
    max_gap_px = int(MAX_LINE_GAP / res)

    lines = cv2.HoughLinesP(
        img,
        rho         = HOUGH_RHO,
        theta       = HOUGH_THETA,
        threshold   = HOUGH_THRESH,
        minLineLength = min_len_px,
        maxLineGap    = max_gap_px,
    )
    if lines is None:
        return []

    walls = []
    for line in lines:
        x1p, y1p, x2p, y2p = line[0]
        # 픽셀 → 미터
        x1 = (x1p - center) * res
        y1 = (center - y1p) * res
        x2 = (x2p - center) * res
        y2 = (center - y2p) * res
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        angle  = normalize_angle(math.degrees(math.atan2(dy, dx)))
        walls.append({
            "length": length, "angle": angle,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
        })

    walls.sort(key=lambda w: w["length"], reverse=True)
    return walls


def merge_collinear(walls: list[dict],
                    angle_tol: float = 3.0,
                    rho_tol: float   = 0.3) -> list[dict]:
    """거의 같은 직선(각도·rho 유사)인 선분을 하나로 병합."""
    used   = [False] * len(walls)
    merged = []
    for i, w in enumerate(walls):
        if used[i]:
            continue
        group = [w]
        used[i] = True
        ri = w["x1"] * math.cos(math.radians(w["angle"])) + \
             w["y1"] * math.sin(math.radians(w["angle"]))
        for j, v in enumerate(walls[i + 1:], i + 1):
            if used[j]:
                continue
            if abs(normalize_angle(w["angle"] - v["angle"])) > angle_tol:
                continue
            rj = v["x1"] * math.cos(math.radians(v["angle"])) + \
                 v["y1"] * math.sin(math.radians(v["angle"]))
            if abs(ri - rj) < rho_tol:
                group.append(v)
                used[j] = True
        # 그룹의 끝점 중 가장 먼 두 점으로 길이 산출
        pts = [(g["x1"], g["y1"]) for g in group] + \
              [(g["x2"], g["y2"]) for g in group]
        pts_arr = np.array(pts)
        centroid = pts_arr.mean(axis=0)
        _, _, vh = np.linalg.svd(pts_arr - centroid)
        direction = vh[0]
        proj = (pts_arr - centroid) @ direction
        length = float(proj.max() - proj.min())
        angle  = normalize_angle(
            math.degrees(math.atan2(direction[1], direction[0])))
        merged.append({
            "length": length, "angle": angle,
            "x1": float(pts_arr[proj.argmin()][0]),
            "y1": float(pts_arr[proj.argmin()][1]),
            "x2": float(pts_arr[proj.argmax()][0]),
            "y2": float(pts_arr[proj.argmax()][1]),
        })
    return merged


# ── ROS2 노드 ──────────────────────────────────────────────────────────────

class WallHoughNode(Node):
    def __init__(self):
        super().__init__('wall_hough_finder')
        self._done = False
        self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self.get_logger().info("Waiting for /scan ...")

    def _cb(self, msg: LaserScan) -> None:
        if self._done:
            return
        self._done = True

        pts   = polar_to_xy(msg)
        walls = detect_walls(pts)

        if not walls:
            self.get_logger().error("벽 탐지 실패")
            return

        W = 70
        print(f"\n{'='*W}")
        print(f"  Hough 벽 탐지  |  유효점: {len(pts)}  |  선분: {len(walls)}")
        print(f"{'='*W}")
        print(f"{'순위':>4}  {'길이(m)':>8}  {'각도(°)':>8}  "
              f"{'시점X':>7}  {'시점Y':>7}  {'종점X':>7}  {'종점Y':>7}")
        print(f"{'-'*W}")
        for rank, w in enumerate(walls[:15], 1):
            marker = " ◀ 최장" if rank == 1 else ""
            print(f"{rank:>4}  {w['length']:>8.3f}  {w['angle']:>8.1f}°  "
                  f"{w['x1']:>7.2f}  {w['y1']:>7.2f}  "
                  f"{w['x2']:>7.2f}  {w['y2']:>7.2f}{marker}")
        print(f"{'='*W}")
        best = walls[0]
        print(f"\n  [최장 벽]  길이: {best['length']:.3f} m  "
              f"각도: {best['angle']:.1f}°\n")


def main() -> None:
    rclpy.init()
    node = WallHoughNode()
    rclpy.spin_once(node, timeout_sec=6.0)
    if not node._done:
        node.get_logger().error("/scan 수신 실패 (6 s 타임아웃)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
