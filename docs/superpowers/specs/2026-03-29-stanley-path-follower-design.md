# Stanley Path Following Action Server — Design Spec

**Date**: 2026-03-29
**Goal**: nav_msgs/Path 경로를 Stanley 제어법칙으로 추종하는 Action Server 추가
**Scope**: 기존 Pure Pursuit Action Server와 동일 수준의 독립 Action Server

---

## 1. 동기

기존 Stanley 구현은 `translate_action_server.cpp`에서 단일 직선 구간 (start→end)만 추종.
nav_msgs/Path로 주어지는 **다중 waypoint 경로**를 추종하는 Stanley 제어기가 필요.

Pure Pursuit은 **lookahead point** 기반 기하학적 제어 → 경로 곡률이 큰 구간에서 cutting 발생.
Stanley는 **현재 위치의 cross-track error** 직접 보정 → 경로 밀착 추종에 유리.

---

## 2. 아키텍처

```
기존 유지 (변경 없음):
  AMRMotionPurePursuit.action          (그대로)
  pure_pursuit_action_server.hpp/cpp   (그대로)
  UpdatePurePursuitPath.srv            (그대로)

신규 추가:
  AMRMotionStanley.action              (PP와 동일 구조, lookahead → K_stanley/K_soft)
  UpdateStanleyPath.srv                (PP UpdatePath와 동일 구조)
  stanley_action_server.hpp            (PP 헤더와 동일 구조, Stanley 고유 멤버)
  stanley_action_server.cpp            (PP .cpp를 기반으로 제어법칙만 교체)

수정:
  motion_common.hpp                    (ActiveAction에 STANLEY 추가)
  CMakeLists.txt (amr_interfaces)      (새 action/srv 추가)
  CMakeLists.txt (amr_motion_control_2wd) (새 .cpp 빌드 대상 추가)
  main.cpp                             (StanleyActionServer 인스턴스 생성)
  motion_params_gazebo.yaml            (stanley_* 파라미터 추가)
```

---

## 3. 인터페이스 정의

### 3.1 AMRMotionStanley.action

```
# Goal
nav_msgs/Path path               # waypoint path (map frame)
float64 max_linear_speed         # m/s
float64 acceleration             # m/s²
float64 K_stanley                # Stanley gain (default: 2.0)
float64 K_soft                   # softening gain (default: 0.8)
float64 goal_tolerance           # m
bool    align_final_heading      # align to last segment heading
---
# Result
int8    status                   # 0=success, -1=cancelled, -2=invalid, -3=timeout, -4=safety
float64 actual_distance
float64 final_cross_track_error
float64 elapsed_time
---
# Feedback
float64 current_distance
float64 remaining_distance
float64 cross_track_error
float64 heading_error
float64 current_speed
uint32  current_waypoint_index
uint32  total_waypoints
uint8   phase                    # 0=wait_pose, 1=initializing, 2=tracking, 3=final_align
```

PP와의 차이: Goal에서 `lookahead_distance` → `K_stanley` + `K_soft`

### 3.2 UpdateStanleyPath.srv

```
nav_msgs/Path new_path
---
bool success
string message
```

PP의 `UpdatePurePursuitPath.srv`와 동일 구조.

---

## 4. Stanley 제어법칙

PP의 `findClosestPoint()`, `buildPath()`, `cumulative_dist_` 로직을 **그대로 복사** 사용.
PP의 `findLookaheadPoint()`는 **사용하지 않음** (Stanley는 lookahead 불필요).

### 4.1 핵심 수식

```cpp
// 1. 경로 투영 (PP와 동일)
auto closest = findClosestPoint(rob_x, rob_y);
double e_lateral = closest.cross_track_err;  // signed CTE (+ = left of path)

// 2. 헤딩 에러
double seg_yaw = atan2(B.y - A.y, B.x - A.x);
double e_theta = normalizeAngle(seg_yaw - rob_yaw);

// 3. Stanley 보정
double stanley_correction = atan2(K_stanley * e_lateral, fabs(vx) + K_soft);

// 4. PD 헤딩 제어 + Stanley 융합
double de_theta = (e_theta - prev_e_heading_) / dt;
double omega = Kp_heading * (e_theta + stanley_correction) + Kd_heading * de_theta;
omega = clamp(omega, -max_omega, max_omega);
```

### 4.2 PP와의 차이점 (제어 루프 내)

| 단계 | Pure Pursuit | Stanley |
|------|-------------|---------|
| 경로 투영 | `findClosestPoint()` | `findClosestPoint()` (동일) |
| 타겟 포인트 | `findLookaheadPoint()` → alpha 계산 | 사용 안 함 |
| 조향 계산 | `omega = 2*v*sin(alpha)/L` | `omega = Kp*(e_theta + stanley) + Kd*de_theta` |
| 횡방향 보정 | 간접 (lookahead 기하학) | 직접 (`atan2(K*e_lat, |v|+K_soft)`) |

---

## 5. 재사용 코드 (PP에서 복사)

Stanley Action Server는 PP의 **~80% 코드를 그대로 사용**:

### 그대로 복사
- Constructor 패턴 (safeParam, TF2, publishers, subscribers, action server, path update service)
- `buildPath()` — nav_msgs/Path → waypoints_ + cumulative_dist_
- `findClosestPoint()` — 경로 투영 + signed CTE
- `handleGoal()`, `handleCancel()`, `handleAccepted()` — action 콜백
- `handlePathUpdate()` — 런타임 경로 교체
- execute() 내: mutual exclusion, timeout, safety, cancellation, localization watchdog, feedback
- `lookupRobotPose()`, `publishCmdVel()`, `publishPathMarker()`, `clearPathMarker()`, `normalizeAngle()`

### 제거 (Stanley 불필요)
- `findLookaheadPoint()` — lookahead 기반 기하학
- `lookahead_distance`, `min_lookahead_distance_`, `max_lookahead_distance_` 멤버/파라미터

### 변경 (제어법칙 교체)
- execute() 내 조향 계산 부분 (약 10줄): PP alpha → Stanley e_theta + stanley_correction
- Goal 파라미터: `lookahead_distance` → `K_stanley`, `K_soft`

---

## 6. 파일별 변경 목록

### 신규 생성 (4 파일)

| 파일 | 설명 | 추정 줄 수 |
|------|------|-----------|
| `amr_interfaces/action/AMRMotionStanley.action` | 액션 인터페이스 | 25줄 |
| `amr_interfaces/srv/UpdateStanleyPath.srv` | 경로 업데이트 서비스 | 5줄 |
| `amr_motion_control_2wd/include/.../stanley_action_server.hpp` | 헤더 (PP 기반) | ~150줄 |
| `amr_motion_control_2wd/src/stanley_action_server.cpp` | 구현 (PP 기반, 제어법칙 교체) | ~750줄 |

### 수정 (5 파일)

| 파일 | 변경 내용 | 변경량 |
|------|----------|--------|
| `amr_interfaces/CMakeLists.txt` | AMRMotionStanley.action, UpdateStanleyPath.srv 추가 | +2줄 |
| `amr_interfaces/package.xml` | nav_msgs 의존성 확인 (이미 PP에서 추가됨, 없으면 추가) | +0~1줄 |
| `amr_motion_control_2wd/include/.../motion_common.hpp` | `ActiveAction::STANLEY` 추가 + `to_string()` | +3줄 |
| `amr_motion_control_2wd/CMakeLists.txt` | `stanley_action_server.cpp` 빌드 대상 추가 | +1줄 |
| `amr_motion_control_2wd/src/main.cpp` | `StanleyActionServer` 인스턴스 생성 | +3줄 |
| `amr_motion_control_2wd/config/motion_params_gazebo.yaml` | `stanley_*` 파라미터 추가 | +15줄 |

---

## 7. 설정 파라미터 (motion_params_gazebo.yaml)

```yaml
# Stanley Path Following
stanley_ctrl_freq_hz: 50.0
stanley_default_K_stanley: 2.0
stanley_default_K_soft: 0.8
stanley_default_goal_tolerance: 0.10
stanley_max_omega: 1.0
stanley_max_timeout_sec: 120.0
stanley_Kp_heading: 1.5
stanley_Kd_heading: 0.3
stanley_lateral_abort_dist: 1.0
stanley_min_vx: 0.02
stanley_watchdog_timeout_sec: 2.0
stanley_watchdog_jump_threshold: 0.5
stanley_watchdog_velocity_margin: 1.3
stanley_pose_topic: "/rtabmap/odom"
stanley_enable_localization_watchdog: true
```

---

## 8. 검증 방법

```bash
# 1. 빌드
colcon build --packages-select amr_interfaces amr_motion_control_2wd

# 2. 인터페이스 확인
ros2 interface show amr_interfaces/action/AMRMotionStanley

# 3. 액션 서버 확인
ros2 action list | grep stanley

# 4. 직선 경로 테스트
ros2 action send_goal /amr_motion_stanley amr_interfaces/action/AMRMotionStanley \
  "{path: {header: {frame_id: 'map'}, poses: [
    {pose: {position: {x: 0.0, y: 0.0}}},
    {pose: {position: {x: 2.0, y: 0.0}}}
  ]}, max_linear_speed: 0.3, acceleration: 0.5, K_stanley: 2.0, K_soft: 0.8,
   goal_tolerance: 0.1, align_final_heading: false}" --feedback

# 5. L자 경로 테스트
ros2 action send_goal /amr_motion_stanley amr_interfaces/action/AMRMotionStanley \
  "{path: {header: {frame_id: 'map'}, poses: [
    {pose: {position: {x: 0.0, y: 0.0}}},
    {pose: {position: {x: 2.0, y: 0.0}}},
    {pose: {position: {x: 2.0, y: 2.0}}}
  ]}, max_linear_speed: 0.2, acceleration: 0.5, K_stanley: 2.0, K_soft: 0.8,
   goal_tolerance: 0.1, align_final_heading: true}" --feedback

# 6. 런타임 경로 업데이트 테스트
ros2 service call /update_stanley_path amr_interfaces/srv/UpdateStanleyPath \
  "{new_path: {header: {frame_id: 'map'}, poses: [...]}}"
```

---

## 9. 미포함 (Out of Scope)

- PP와 Stanley 공통 로직 추출 (base class) — 현 단계에서는 코드 복사로 충분
- Test Simulator GUI에 Stanley 탭 추가 — 별도 태스크
- ACS waypoint_manager에서 Stanley 선택 — 별도 태스크
- Stanley 고유 기능 (adaptive K_stanley, speed-dependent gain) — 추후 확장
