# Stanley Path Follower 설계 문서

## 1. 개요

Stanley Controller는 Stanford Racing Team이 DARPA Grand Challenge (2005)에서 개발한 경로 추종 알고리즘입니다.
Pure Pursuit과 달리 전방 예측점(lookahead) 대신 현재 위치의 **횡방향 오차(cross-track error)를 직접 보정**하는 방식입니다.

---

## 2. Stanley 제어법칙 수식

전체 조향각은 두 항의 합으로 구성됩니다:

```
δ(t) = ψ_e(t) + arctan( K_stanley * e(t) / (v(t) + K_soft) )
```

| 기호 | 설명 |
|------|------|
| `δ(t)` | 목표 조향각 (heading correction) |
| `ψ_e(t)` | 헤딩 오차 (현재 heading - 경로 tangent heading) |
| `e(t)` | 횡방향 오차 (cross-track error, 부호 있음) |
| `v(t)` | 현재 속도 (m/s) |
| `K_stanley` | Stanley gain (횡방향 오차 감도) |
| `K_soft` | Softening constant (저속 시 수치 안정성 보장) |

### 2.1 헤딩 오차 항 (`ψ_e`)

경로의 tangent 방향과 로봇의 현재 yaw 간의 각도 차이.
로봇이 경로 방향과 어긋나 있을 때 즉각 보정합니다.

### 2.2 횡방향 오차 항 (`arctan(K * e / v)`)

경로로부터 수직 거리(cross-track error)를 arctan으로 조향각으로 변환.
속도가 높을수록 조향 반응이 줄어들어 자연스러운 감속 보정이 이루어집니다.

---

## 3. Pure Pursuit과의 비교

| 항목 | Pure Pursuit | Stanley |
|------|-------------|---------|
| 제어 기준 | 전방 lookahead point | 현재 위치의 cross-track error |
| 헤딩 오차 처리 | 간접 (lookahead 방향에 포함) | 명시적 (`ψ_e` 항) |
| 저속 안정성 | lookahead 거리 조정 필요 | `K_soft` 항으로 자동 처리 |
| 고속 특성 | lookahead 길게 설정 시 안정 | arctan으로 자연 감쇠 |
| 구현 복잡도 | 낮음 | 중간 |
| 파라미터 수 | 1~2개 (lookahead, speed gain) | 4개 (K_stanley, K_soft, Kp, Kd) |

**요약**: Pure Pursuit은 "어디를 향해 가야 하나"를 lookahead로 결정하고,
Stanley는 "지금 얼마나 벗어났나"를 직접 측정하여 보정합니다.

---

## 4. 파라미터 설명

### 4.1 `K_stanley` (Stanley gain)
- **역할**: 횡방향 오차에 대한 조향 감도
- **기본값**: `2.5`
- **조정 방법**: 값이 클수록 오차에 빠르게 반응하지만 진동 가능성 증가
- **권장 범위**: `1.0 ~ 5.0`

### 4.2 `K_soft` (Softening constant)
- **역할**: 분모에 더해 저속(v≈0) 시 `arctan` 항의 발산 방지
- **기본값**: `1.0`
- **조정 방법**: 값이 작을수록 저속 반응이 강해지나 수치 불안정 위험 증가
- **권장 범위**: `0.5 ~ 2.0`

### 4.3 `Kp_heading` (Heading P gain)
- **역할**: 헤딩 오차(`ψ_e`) 보정에 대한 비례 제어 계수
- **기본값**: `1.2`
- **조정 방법**: 값이 크면 방향 오차에 빠르게 반응, 너무 크면 오버슈트

### 4.4 `Kd_heading` (Heading D gain)
- **역할**: 헤딩 오차 변화율에 대한 미분 제어 계수 (진동 억제)
- **기본값**: `0.1`
- **조정 방법**: 진동이 발생할 때 값을 높여 댐핑 증가

---

## 5. ROS2 인터페이스 요약

### 5.1 Action 인터페이스: `AMRMotionStanley.action`

```
# Goal
geometry_msgs/PoseStamped[] waypoints
float32 linear_speed

---
# Result
bool success
string message

---
# Feedback
geometry_msgs/Pose current_pose
int32 current_waypoint_index
float32 cross_track_error
float32 heading_error
```

### 5.2 Service 인터페이스: `UpdateStanleyPath.srv`

```
# Request
geometry_msgs/PoseStamped[] waypoints

---
# Response
bool success
string message
```

### 5.3 Action Server 노드

- **노드명**: `amr_motion_control_2wd`에 통합 (기존 PP와 동일 노드)
- **Action 이름**: `/stanley_path_follower`
- **서비스 이름**: `/update_stanley_path`
- **구독 토픽**: `/odometry/filtered` (nav_msgs/Odometry)
- **발행 토픽**: `/cmd_vel` (geometry_msgs/Twist)

### 5.4 파라미터 (motion_params_gazebo.yaml)

```yaml
stanley_path_follower:
  K_stanley: 2.5
  K_soft: 1.0
  Kp_heading: 1.2
  Kd_heading: 0.1
  linear_speed: 0.3
  goal_tolerance: 0.2
  max_steering_angle: 1.0  # rad
```

---

## 6. 구현 파일 목록

| 파일 | 설명 |
|------|------|
| `amr_interfaces/action/AMRMotionStanley.action` | Action 메시지 정의 |
| `amr_interfaces/srv/UpdateStanleyPath.srv` | 경로 업데이트 서비스 정의 |
| `amr_motion_control_2wd/include/.../stanley_action_server.hpp` | Action Server 헤더 |
| `amr_motion_control_2wd/src/stanley_action_server.cpp` | Stanley 제어 구현 |

---

## 7. 설계 결정 사항

- **독립 Action Server 채택**: PP와 인터페이스를 분리하여 독립적으로 테스트/교체 가능
- **PP 인프라 재사용**: `findClosestPoint()` 등 경로 처리 로직은 동일하게 활용
- **향후 개선**: 공통 로직을 `PathFollowerBase` 추상 클래스로 추출 가능
