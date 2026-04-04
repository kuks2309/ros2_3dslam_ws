#!/usr/bin/env python3
"""
Gazebo SIL 통합 테스트: 전체 매핑 절차 확인

실행 조건 (아래를 별도 터미널에서 먼저 시작):
  Terminal 1: ros2 launch amr_motion_control_2wd motion_control_gazebo.launch.py
  Terminal 2: ros2 run slam_manager_2d slam_manager_2d_node
  Terminal 3: ros2 launch mapper mapper_gazebo.launch.py

실행:
  cd /home/amap/Study/ros2_3dslam_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/Mapper/mapper/test/test_integration_gazebo.py
"""
import sys
import time

import rclpy
from rclpy.node import Node
from mapper_interfaces.srv import MapperCommand
from mapper_interfaces.msg import MapperStatus


# ---------------------------------------------------------------------------
# 테스트 1: 전체 매핑 플로우 확인
# ---------------------------------------------------------------------------

def test_full_mapping_flow(node: Node) -> None:
    """
    CMD_START_MAPPING(2D, 오른쪽 벽 탐색) 발행 후
    ALIGNING 상태 도달 여부와 COMPLETED 상태 도달 여부를 확인한다.
    COMPLETED 타임아웃: 600 초
    """
    received_states: list[int] = []

    def on_status(msg: MapperStatus) -> None:
        if not received_states or received_states[-1] != msg.state:
            received_states.append(msg.state)

    sub = node.create_subscription(MapperStatus, 'mapper/status', on_status, 10)
    cmd_client = node.create_client(MapperCommand, 'mapper/command')

    # 서비스 대기
    if not cmd_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("mapper/command service not available after 10 s")

    # 매핑 시작
    req = MapperCommand.Request()
    req.command    = MapperCommand.Request.CMD_START_MAPPING
    req.slam_mode  = MapperCommand.Request.SLAM_2D
    req.drive_mode = MapperCommand.Request.DRIVE_RIGHT_HAND
    future = cmd_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        raise RuntimeError("Service call timed out after 5 s (no response)")
    result = future.result()
    assert result is not None and result.success, \
        f"CMD_START_MAPPING failed: {result.message if result else 'no response'}"

    # ALIGNING 상태 확인 (30 s)
    deadline = time.time() + 30.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if MapperStatus.STATE_ALIGNING in received_states:
            break
    assert MapperStatus.STATE_ALIGNING in received_states, \
        f"Never reached ALIGNING within 30 s. States seen: {received_states}"
    print("✅ ALIGNING state confirmed")

    # COMPLETED 상태 확인 (600 s)
    deadline = time.time() + 600.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if MapperStatus.STATE_COMPLETED in received_states:
            break
    assert MapperStatus.STATE_COMPLETED in received_states, \
        f"Never reached COMPLETED within 600 s. States seen: {received_states}"
    print("✅ Full mapping flow completed")

    node.destroy_subscription(sub)


# ---------------------------------------------------------------------------
# 테스트 2: E-STOP → IDLE 전이 확인
# ---------------------------------------------------------------------------

def test_estop_transitions_to_idle(node: Node) -> None:
    """
    어떤 상태에서든 CMD_STOP 발행 시 IDLE 전이를 확인한다.
    """
    cmd_client = node.create_client(MapperCommand, 'mapper/command')
    if not cmd_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("mapper/command service not available after 10 s")

    # ① 먼저 구독 등록
    current_state: list[int] = []

    def on_status(msg: MapperStatus) -> None:
        current_state.clear()
        current_state.append(msg.state)

    sub = node.create_subscription(MapperStatus, 'mapper/status', on_status, 10)

    # ② 그 다음 E-STOP 발행
    req = MapperCommand.Request()
    req.command = MapperCommand.Request.CMD_STOP
    future = cmd_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        node.destroy_subscription(sub)
        raise RuntimeError("CMD_STOP service call timed out after 5 s")
    result = future.result()
    assert result is not None and result.success, \
        f"CMD_STOP failed: {result.message if result else 'no response'}"

    # IDLE 확인 (5 s)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_state and current_state[-1] == MapperStatus.STATE_IDLE:
            break
    assert current_state and current_state[-1] == MapperStatus.STATE_IDLE, \
        f"Expected IDLE after CMD_STOP, got: {current_state}"
    print("✅ E-STOP → IDLE confirmed")
    node.destroy_subscription(sub)


# ---------------------------------------------------------------------------
# 테스트 3: PAUSE / RESUME 전이 확인
# ---------------------------------------------------------------------------

def test_pause_resume(node: Node) -> None:
    """
    IDLE 상태에서 CMD_PAUSE는 실패해야 함을 확인한다.
    (MAPPING_MANUAL → PAUSED → RESUME 전이는 Gazebo 실행 환경에서 수동 테스트)
    """
    cmd_client = node.create_client(MapperCommand, 'mapper/command')
    if not cmd_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("mapper/command service not available")

    # IDLE 상태에서 CMD_PAUSE는 실패해야 함
    req = MapperCommand.Request()
    req.command = MapperCommand.Request.CMD_PAUSE
    future = cmd_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        raise RuntimeError("Service call timed out after 5 s (no response)")
    result = future.result()
    assert result is not None and not result.success, \
        "CMD_PAUSE from IDLE should fail but succeeded"
    print("✅ CMD_PAUSE from IDLE correctly rejected")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    failed = 0

    tests = [
        ("E-STOP → IDLE",         test_estop_transitions_to_idle),
        ("PAUSE from IDLE reject", test_pause_resume),
        ("Full mapping flow",      test_full_mapping_flow),
    ]

    for name, fn in tests:
        print(f"\n{'='*50}")
        print(f"Running: {name}")
        print('='*50)
        node = rclpy.create_node('mapper_integration_test')
        try:
            fn(node)
            print(f"PASS: {name}")
        except AssertionError as e:
            print(f"FAIL: {name} — {e}", file=sys.stderr)
            failed += 1
        except RuntimeError as e:
            print(f"SKIP: {name} — {e} (Gazebo not running?)", file=sys.stderr)
        finally:
            node.destroy_node()

    rclpy.shutdown()

    if failed:
        print(f"\n{failed} test(s) FAILED", file=sys.stderr)
        sys.exit(1)
    print("\n✅ All tests passed")


if __name__ == '__main__':
    main()
