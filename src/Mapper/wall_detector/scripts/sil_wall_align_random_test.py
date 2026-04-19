#!/usr/bin/env python3
"""
SIL 랜덤 자세 벽 정렬 테스트.

사전 조건 (별도 터미널):
  1. ros2 launch <gazebo_pkg> gazebo.launch.py
  2. ros2 launch <amr_motion_control_2wd> motion_control_gazebo.launch.py
  3. ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
  4. ros2 run wall_detector wall_detector_node --ros-args -p use_sim_time:=true
  5. ros2 run mapper wall_aligner_node --ros-args -p use_sim_time:=true

각 시나리오마다:
  1. 로봇을 랜덤 (x, y, yaw) 위치로 set_pose
  2. 2초 안정화 대기
  3. /wall_align Action 전송 (tolerance_deg=2.0)
  4. 결과 기록 (success/fail, 초기 오차, 최종 오차, 소요 시간)

실행:
  python3 src/Mapper/wall_detector/scripts/sil_wall_align_random_test.py
"""
import math
import random
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mapper_interfaces.action import WallAlign

# 설정
N_SCENARIOS       = 12
SETTLE_SEC        = 2.0
ACTION_TIMEOUT_S  = 120.0    # max_attempts=10 × 평균 6초 = 60초 + 마진
TOLERANCE_DEG     = 2.0
RANDOM_SEED       = 2026

# 창고 내부 안전 범위
X_RANGE = (-13.0, 4.0)
Y_RANGE = (-4.0,  5.0)
YAW_RANGE_DEG = (-180.0, 180.0)


def yaw_to_quat(yaw_deg: float):
    half = math.radians(yaw_deg) / 2
    return 0.0, 0.0, math.sin(half), math.cos(half)


def teleport(x: float, y: float, yaw_deg: float) -> bool:
    qx, qy, qz, qw = yaw_to_quat(yaw_deg)
    req = (f'name: "robot_scan", '
           f'position: {{x: {x}, y: {y}, z: 0.16}}, '
           f'orientation: {{x: {qx}, y: {qy}, z: {qz:.6f}, w: {qw:.6f}}}')
    r = subprocess.run([
        'ign', 'service',
        '-s', '/world/my_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', req,
    ], capture_output=True, text=True)
    return 'true' in r.stdout.lower()


class WallAlignTester(Node):
    def __init__(self):
        super().__init__('wall_align_tester')
        self.cli = ActionClient(self, WallAlign, '/wall_align')
        self._last_feedback = None

    def _feedback_cb(self, feedback_msg):
        self._last_feedback = feedback_msg.feedback

    def run_goal(self, tolerance_deg: float
                 ) -> tuple[str, float, float, float]:
        """
        WallAlign 호출. 반환: (status_str, initial_error, final_error, elapsed_s)
        """
        if not self.cli.wait_for_server(timeout_sec=5.0):
            return "NO_SERVER", 0.0, 0.0, 0.0

        goal = WallAlign.Goal()
        goal.tolerance_deg      = tolerance_deg
        goal.use_imu_correction = False

        self._last_feedback = None
        start_time = time.time()
        send_future = self.cli.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=5.0)
        if not send_future.done():
            return "SEND_TIMEOUT", 0.0, 0.0, 0.0
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return "REJECTED", 0.0, 0.0, 0.0

        # 첫 피드백이 도착할 때까지 잠시 대기해 초기 오차 캡처
        fb_deadline = time.time() + 3.0
        while time.time() < fb_deadline and self._last_feedback is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        initial_error = self._last_feedback.current_error_deg \
            if self._last_feedback else 0.0

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=ACTION_TIMEOUT_S)
        elapsed = time.time() - start_time

        if not result_future.done():
            # 클라이언트 timeout — 서버가 여전히 진행 중이므로 명시적 취소
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self, cancel_future, timeout_sec=5.0)
            # cancel 결과 처리까지 짧게 대기
            time.sleep(1.0)
            return "TIMEOUT", initial_error, 0.0, elapsed

        result = result_future.result()
        status_code = result.result.status
        final_error = result.result.current_error_deg

        status_str = {
            0:  "SUCCESS",
            -1: "ABORT",
            -2: "REJECT",
            -3: "CANCEL",
            -4: "NO_WALL_DATA",
        }.get(status_code, f"UNKNOWN({status_code})")
        return status_str, initial_error, final_error, elapsed


def main():
    rclpy.init()
    node = WallAlignTester()

    random.seed(RANDOM_SEED)
    scenarios = [("기본 자세 30°", -13.10, 2.64, 30.0)]
    for i in range(N_SCENARIOS):
        x   = round(random.uniform(*X_RANGE),       2)
        y   = round(random.uniform(*Y_RANGE),       2)
        yaw = round(random.uniform(*YAW_RANGE_DEG), 1)
        scenarios.append((f"랜덤 #{i+1:02d}", x, y, yaw))

    W = 106
    print(f"\n{'='*W}")
    print(f"  SIL 랜덤 자세 벽 정렬 테스트  |  {len(scenarios)} 시나리오  "
          f"|  tolerance={TOLERANCE_DEG}°")
    print(f"{'='*W}")
    print(f"{'시나리오':<14} {'X':>7} {'Y':>6} {'yaw':>7} "
          f"{'init_err':>10} {'final_err':>10} {'time(s)':>8}  결과")
    print(f"{'-'*W}")

    passed = failed = skipped = 0
    results = []

    for name, sx, sy, syaw in scenarios:
        if not teleport(sx, sy, syaw):
            print(f"  {name:<12} {sx:>7.2f} {sy:>6.2f} {syaw:>7.1f}°  SKIP (teleport 실패)")
            skipped += 1
            continue
        time.sleep(SETTLE_SEC)

        status, init_err, final_err, elapsed = node.run_goal(TOLERANCE_DEG)

        if status == "SUCCESS":
            passed += 1
            tag = "PASS"
        elif status == "NO_SERVER":
            skipped += 1
            tag = "SKIP"
        else:
            failed += 1
            tag = "FAIL"

        results.append({
            "name": name, "x": sx, "y": sy, "yaw": syaw,
            "init": init_err, "final": final_err,
            "time": elapsed, "status": status,
        })

        print(f"  {name:<12} {sx:>7.2f} {sy:>6.2f} {syaw:>7.1f}° "
              f"{init_err:>9.2f}° {final_err:>9.2f}° {elapsed:>7.2f}  "
              f"{tag} ({status})")

    print(f"{'='*W}")
    total = passed + failed
    rate  = passed / total * 100 if total > 0 else 0.0
    print(f"  결과: {passed} PASS / {failed} FAIL / {skipped} SKIP  "
          f"(정렬 통과율 {rate:.1f}%)\n")

    # CSV 저장
    import os
    csv_path = os.path.expanduser('~/Study/ros2_3dslam_ws/experiments/capture/sil_align_results.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w') as f:
        f.write("name,x,y,yaw_deg,init_err_deg,final_err_deg,time_s,status\n")
        for r in results:
            f.write(f"{r['name']},{r['x']},{r['y']},{r['yaw']},"
                    f"{r['init']:.3f},{r['final']:.3f},{r['time']:.3f},{r['status']}\n")
    print(f"  CSV: {csv_path}\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
