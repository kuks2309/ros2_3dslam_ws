#!/usr/bin/env python3
"""
라이다 스캔에서 가장 긴 벽(직선 세그먼트)을 찾아 출력한다.

알고리즘:
  1. 극좌표 → 직교좌표 변환 (유효 범위 필터)
  2. 연속 점 간 거리 갭으로 세그먼트 분리
  3. SVD(주성분 분석)로 각 세그먼트의 직선 피팅 및 길이 산출
  4. 길이 기준 정렬 → 상위 출력

실행:
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/Gazebo/scripts/find_longest_wall.py
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def polar_to_cartesian(
        ranges, angle_min: float, angle_increment: float,
        range_min: float, range_max: float) -> np.ndarray:
    pts = []
    for i, r in enumerate(ranges):
        if range_min < r < range_max:
            theta = angle_min + i * angle_increment
            pts.append((r * math.cos(theta), r * math.sin(theta)))
    return np.array(pts) if pts else np.empty((0, 2))


def split_segments(pts: np.ndarray, gap: float = 0.25,
                   min_pts: int = 10) -> list[np.ndarray]:
    """연속 점을 gap(m) 기준으로 분리; min_pts 미만 세그먼트는 버림."""
    if len(pts) < min_pts:
        return []
    segments, seg = [], [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - seg[-1]) < gap:
            seg.append(p)
        else:
            if len(seg) >= min_pts:
                segments.append(np.array(seg))
            seg = [p]
    if len(seg) >= min_pts:
        segments.append(np.array(seg))
    return segments


def fit_segment(seg: np.ndarray) -> tuple[float, float, float, float, int]:
    """
    SVD로 직선 피팅 후 길이·각도·중심 반환.
    Returns: (length_m, angle_deg, cx, cy, n_points)
    """
    centroid = seg.mean(axis=0)
    _, _, vh = np.linalg.svd(seg - centroid)
    direction = vh[0]                      # 주성분 = 직선 방향 벡터
    proj = (seg - centroid) @ direction
    length = float(proj.max() - proj.min())
    angle = math.degrees(math.atan2(direction[1], direction[0]))
    return length, angle, float(centroid[0]), float(centroid[1]), len(seg)


class LongestWallFinder(Node):
    def __init__(self):
        super().__init__('longest_wall_finder')
        self._done = False
        self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self.get_logger().info("Waiting for /scan ...")

    def _cb(self, msg: LaserScan) -> None:
        if self._done:
            return
        self._done = True

        pts = polar_to_cartesian(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max)

        if len(pts) == 0:
            self.get_logger().error("유효 포인트 없음")
            return

        segs = split_segments(pts)
        if not segs:
            self.get_logger().error("세그먼트 검출 실패 (포인트 부족)")
            return

        results = sorted(
            [fit_segment(s) for s in segs],
            key=lambda x: x[0], reverse=True)

        W = 62
        print(f"\n{'='*W}")
        print(f"  라이다 벽 분석  |  유효점: {len(pts)}  |  세그먼트: {len(segs)}")
        print(f"{'='*W}")
        print(f"{'순위':>4}  {'길이(m)':>8}  {'각도(°)':>8}  {'중심X(m)':>9}  {'중심Y(m)':>9}  {'점수':>5}")
        print(f"{'-'*W}")
        for rank, (length, angle, cx, cy, npts) in enumerate(results[:10], 1):
            marker = " ◀ 최장" if rank == 1 else ""
            print(f"{rank:>4}  {length:>8.3f}  {angle:>8.1f}  {cx:>9.3f}  {cy:>9.3f}  {npts:>5}{marker}")
        print(f"{'='*W}")

        best = results[0]
        print(f"\n  [가장 긴 벽]")
        print(f"    길이  : {best[0]:.3f} m")
        print(f"    각도  : {best[1]:.1f}° (lidar_link 기준)")
        print(f"    중심  : ({best[2]:.3f}, {best[3]:.3f}) m")
        print(f"    점 수 : {best[4]}\n")


def main() -> None:
    rclpy.init()
    node = LongestWallFinder()
    rclpy.spin_once(node, timeout_sec=6.0)
    if not node._done:
        node.get_logger().error("/scan 수신 실패 (6 s 타임아웃)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
