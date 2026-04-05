# YawControl 설계 문서

## 1. 개요

YawControl은 **목표 heading을 유지하면서 전진**하는 액션 서버이다.
목표점(end_x/y) 없이, 정지 명령이 올 때까지 계속 주행한다.

### 다른 액션과의 차이

| 액션 | 동작 | 종료 조건 |
|------|------|-----------|
| **Spin** | 제자리 회전 → 목표 각도 | 목표 각도 도달 |
| **Turn** | 아크 회전 (반경 지정) | 목표 각도 도달 |
| **Translate** | A→B 직선 이동 (heading + lateral 보정) | 목표점 도달 |
| **TranslateReverse** | A→B 후진 직선 이동 | 목표점 도달 |
| **YawControl** | **목표 heading 유지 + 전진** | **Stop service 또는 Cancel** |

## 2. 정지 방식 (3가지)

| 명령 | 방식 | 동작 | 구현 |
|------|------|------|------|
| **Stop (감속)** | Service | deceleration 프로파일로 부드러운 정지 | `ros2 service call /yaw_control_stop std_srvs/srv/Trigger` |
| **Cancel (즉시)** | Action Cancel | cmd_vel = 0 즉시 정지 | `ros2 action cancel /amr_motion_yaw_control` |
| **E-STOP (비상)** | Topic | 전체 비상 정지 (zero Twist publish) | GUI E-STOP 버튼 |

### Stop vs Cancel 시퀀스 다이어그램

```
[Stop 서비스]
  GUI → /yaw_control_stop → 서버 stop_requested_ = true
  제어 루프: phase=DECEL → 감속 → vx≈0 → succeed(result)

[Cancel]
  GUI → cancel_goal_async → 서버 is_canceling() = true
  제어 루프: 즉시 vx=0, omega=0 → canceled(result)
```

## 3. Action 인터페이스

```
# AMRMotionYawControl.action

# Goal
float64 target_heading       # 목표 heading (deg, map frame, -180~180)
float64 max_linear_speed     # m/s (> 0, Gazebo 제한: 0.4)
float64 acceleration         # m/s² (출발 가속)
float64 deceleration         # m/s² (Stop 시 감속률)
bool    hold_steer            # ACS 웨이포인트 시스템 필드
---
# Result
int8    status               # 0=stopped, -1=cancelled, -2=invalid_param, -3=timeout, -4=safety
float64 total_distance       # 총 이동 거리 (m)
float64 final_heading_error  # deg
float64 elapsed_time         # sec
---
# Feedback
float64 current_heading_error # deg
float64 current_vx           # m/s
float64 current_omega        # rad/s
uint8   phase                # 1=accel, 2=cruise, 3=decel
```

### Translate와 비교

| 필드 | Translate | YawControl |
|------|-----------|------------|
| start_x/y | O (시작점) | X (불필요) |
| end_x/y | O (목표점) | X (불필요) |
| target_heading | X (경로에서 자동 계산) | **O (사용자 지정)** |
| deceleration | X (프로파일 자동) | **O (Stop 시 감속률)** |
| exit_speed | O (세그먼트 연결) | X |
| has_next | O (세그먼트 연결) | X |

## 4. 제어 루프

```
20 Hz 루프:
  ┌─ a. Cancel 체크 → 즉시 정지 (vx=0, omega=0), status=-1
  ├─ b. Stop 체크 → decelerating=true, phase=DECEL
  ├─ c. Timeout 체크 (60초 기본)
  ├─ d. Localization watchdog 체크
  ├─ e. TF2로 로봇 pose 획득 (map→base_footprint)
  ├─ f. 거리 누적 (odometry 기반)
  ├─ g. 목표 vx 결정:
  │     ACCEL: 0 → max_linear_speed (acceleration 사용)
  │     CRUISE: max_linear_speed 유지
  │     DECEL: current_vx → 0 (deceleration 사용)
  ├─ h. vx ≈ 0이면 succeed (감속 완료)
  ├─ i. Heading error PD 제어:
  │     e_theta = normalize(rob_yaw - target_heading)
  │     omega = -Kp * e_theta - Kd * de_theta/dt
  ├─ j. Omega 클램핑:
  │     |omega| ≤ max_omega (1.0 rad/s)
  │     |omega| ≤ vx / min_turning_radius
  ├─ k. Omega rate limit (0.5 rad/s²)
  ├─ l. cmd_vel 발행
  └─ m. Feedback 발행
```

## 5. PD 제어 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `yaw_ctrl.control_rate_hz` | 20.0 | 제어 루프 주기 (Hz) |
| `yaw_ctrl.Kp_heading` | 1.0 | Heading 비례 게인 |
| `yaw_ctrl.Kd_heading` | 0.3 | Heading 미분 게인 |
| `yaw_ctrl.max_omega` | 1.0 | 최대 각속도 (rad/s) |
| `yaw_ctrl.min_turning_radius` | 0.7 | 최소 회전 반경 (m) |
| `yaw_ctrl.omega_rate_limit` | 0.5 | Omega 변화 제한 (rad/s²) |
| `yaw_ctrl.walk_accel_limit` | 0.5 | 선속도 가속 제한 (m/s²) |
| `yaw_ctrl.walk_decel_limit` | 1.0 | 선속도 감속 제한 (m/s²) |
| `yaw_ctrl.max_timeout_sec` | 60.0 | 최대 타임아웃 (초) |

## 6. 상태 코드

| 코드 | 의미 | 발생 조건 |
|------|------|-----------|
| 0 | 정상 정지 | Stop service → 감속 완료 |
| -1 | 취소 | Cancel → 즉시 정지 |
| -2 | 잘못된 파라미터 | speed/accel/decel ≤ 0 |
| -3 | 타임아웃 | max_timeout_sec 초과 |
| -4 | 안전 정지 | Localization watchdog 트리거 |

## 7. 실행 명령어

```bash
# 빌드
colcon build --symlink-install --packages-up-to amr_interfaces amr_motion_control_2wd amr_motion_test_ui

# CLI 테스트: heading 0° 유지하며 0.3 m/s 전진
ros2 action send_goal /amr_motion_yaw_control \
  amr_interfaces/action/AMRMotionYawControl \
  "{target_heading: 0.0, max_linear_speed: 0.3, acceleration: 0.2, deceleration: 0.3}" \
  --feedback

# 감속 정지
ros2 service call /yaw_control_stop std_srvs/srv/Trigger

# 즉시 정지
ros2 action cancel /amr_motion_yaw_control

# GUI 테스트
ros2 run amr_motion_test_ui gui_node
```

## 8. GUI (YawControl 탭)

```
┌─────────────────────────────────────┐
│ Target Heading (deg):  [ 0.00    ]  │
│ Max Speed (m/s):       [ 0.20    ]  │
│ Acceleration (m/s²):   [ 0.20    ]  │
│ Deceleration (m/s²):   [ 0.30    ]  │
│                                     │
│ [Use Current Heading]               │
│                                     │
│ [Send Goal]  [Stop 감속]  [Cancel]  │
│                                     │
│ Feedback: hdg_err=0.3° vx=0.200 ... │
│ Result: --                          │
└─────────────────────────────────────┘
```

- **Send Goal**: 목표 heading 유지하며 전진 시작
- **Stop (감속)**: `/yaw_control_stop` service → 부드러운 감속 정지
- **Cancel (즉시)**: action cancel → 즉시 정지
- **Use Current Heading**: 현재 로봇 heading을 target_heading에 자동 입력

## 9. 소스 파일 목록

| 파일 | 역할 |
|------|------|
| `amr_interfaces/action/AMRMotionYawControl.action` | 액션 인터페이스 정의 |
| `amr_motion_control_2wd/include/.../yaw_control_action_server.hpp` | C++ 헤더 |
| `amr_motion_control_2wd/src/yaw_control_action_server.cpp` | C++ 제어 루프 구현 |
| `amr_motion_test_ui/.../ros_bridge.py` | ROS2 브릿지 (service client 포함) |
| `amr_motion_test_ui/.../gui_node.py` | PyQt5 GUI (YawControl 탭) |
