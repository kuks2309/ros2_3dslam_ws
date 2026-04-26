# SLAM Mapper 설계 문서

**날짜:** 2026-04-04
**브랜치:** feature/acs-waypoint-system
**상태:** 승인됨 (Rev.3 — 5-에이전트 검토 반영)

---

## 1. 개요

`src/Mapper`에 구현할 자율 SLAM 매핑 절차 시스템.
2D/3D rtabmap 기반, `/scan` LaserScan 추상화, 90도 회전+직진 조합의 격자형 탐색.

### 매핑 절차 요약

1. 라이다로 가장 긴 벽 검출 → IMU로 0.2도 이내 정렬
2. 정렬 완료 후 SLAM 시작 (rtabmap 2D 또는 3D)
3. 생성된 맵에서 벽 수직/수평 정렬 검증 → 불일치 시 재정렬 후 재시작
4. 매핑 주행: IMU 헤딩 유지 직진, 90도 제자리 회전 후 직진 (곡선 없음)
5. 수동/자동 모드 주행 (우수법/좌수법)
6. 미탐색(회색) 영역 BFS/A* 경로 계획 → 탐색
7. 탐색 완료 후 시작점 복귀 → rtabmap 내장 루프 클로저 확정

---

## 2. 패키지 구성

```
src/Mapper/
├── mapper_interfaces/        # action/srv/msg 정의 (C++ ament_cmake)
├── mapper/                   # C++ 노드들 (ament_cmake)
└── mapper_ui/                # PyQt5 UI (Python ament_python)
```

> **구현 순서 제약**: `mapper_interfaces`를 Sprint 0에 먼저 확정/freeze해야 함.
> 모든 노드가 이 패키지에 의존하므로 인터페이스 변경은 PR 리뷰 게이트 필수.

---

## 3. 노드 그래프

```
[mapper_ui_node] ──service──▶ [mapper_orchestrator_node]
                                        │
                     ┌──────────────────┼──────────────────────┐
                     ▼ action           ▼ action               ▼ action
          [wall_aligner_node]  [map_alignment_checker_node]  [exploration_planner_node]
                                                                       │
                                                     [amr_motion ActionChainer]

토픽 구독:
  /scan (sensor_msgs/LaserScan)  ──▶ wall_aligner_node
  /map  (nav_msgs/OccupancyGrid) ──▶ map_alignment_checker_node, exploration_planner_node
  /rtabmap/info                  ──▶ mapper_orchestrator_node (루프 클로저 감지)

SLAM 제어:
  mapper_orchestrator_node ──service──▶ slam_manager_2d / slam_manager_3d
  (기존 slam_manager에 SlamControl.srv 추가 + headless 모드 리팩터링 필요)
```

### 노드별 역할

| 노드 | 역할 | 언어 |
|------|------|------|
| `mapper_orchestrator_node` | 전체 상태 머신 조율 (11개 상태, MultiThreadedExecutor) | C++ |
| `wall_aligner_node` | `/scan` RANSAC 직선 검출 → SpinAction으로 정렬 | C++ |
| `map_alignment_checker_node` | `/map` OGM HoughLinesP → 벽 각도 검증 | C++ |
| `exploration_planner_node` | `/map` BFS/A* → Segment 시퀀스 → ActionChainer | C++ |
| `slam_manager_2d` (기존, 리팩터링) | rtabmap 2D subprocess + SlamControl.srv + headless 모드 | Python |
| `slam_manager_3d` (기존, 리팩터링) | rtabmap 3D subprocess + SlamControl.srv (headless 이미 존재) | Python |
| `mapper_ui_node` | 상태 표시 + 수동/자동 제어 (PyQt5 + QTimer 100ms) | Python |

---

## 4. mapper_interfaces 정의

### 4.1 Actions

```
# WallAlign.action
float64 tolerance_deg        # 허용 오차 (기본 0.2도)
bool    use_imu_correction   # true: IMU 헤딩을 SpinAction 기준으로 사용
---
float64 aligned_heading      # 정렬된 절대 헤딩 (deg, map frame)
int8    status               # 0=success, -1=failed, -2=timeout, -3=cancelled
                             # SpinAction status 매핑: -1(cancelled)→-3, -4(tf_fail)→-1
---
float64 current_error_deg
float64 wall_angle_deg
```

```
# MapAlignmentCheck.action
float64 tolerance_deg
---
bool    is_aligned
float64 max_wall_error_deg
---
float64 progress             # 0.0~1.0 (HoughLinesP 처리 진행률)
float64 current_max_error_deg  # 실시간 최대 각도 편차 (UI 모니터링용)
```

```
# ExploreUnknown.action
uint8   MODE_RIGHT_HAND = 0
uint8   MODE_LEFT_HAND  = 1
uint8   MODE_AUTO       = 2
uint8   mode
float32 min_coverage_to_stop  # 탐색 종료 최소 커버리지 (기본 0.95 = 95%)
---
float32 coverage_percent
int8    status               # 0=success, -1=cancelled, -2=no_unknown_left
---
geometry_msgs/Point current_target
float32 coverage_percent
```

### 4.2 Services

```
# SlamControl.srv  (mapper_orchestrator → slam_manager_2d/3d)
uint8   CMD_START_2D = 0
uint8   CMD_START_3D = 1
uint8   CMD_STOP     = 2
uint8   CMD_SAVE_MAP = 3
uint8   command
string  map_name          # save_map 시 맵 이름 (기본: timestamp)
string  save_directory    # save_map 시 저장 경로 (기본: workspace/maps/)
---
bool    success
string  message
string  saved_path        # 실제 저장된 파일 경로
```

```
# MapperCommand.srv (UI → orchestrator)
uint8   CMD_START_MAPPING = 0
uint8   CMD_PAUSE         = 1
uint8   CMD_STOP          = 2
uint8   CMD_SAVE_MAP      = 3
uint8   CMD_RESUME        = 4
uint8   CMD_EXPLORE       = 5   # 수동 모드에서 MAPPING→EXPLORING 전환
uint8   command
uint8   SLAM_2D = 0
uint8   SLAM_3D = 1
uint8   slam_mode
uint8   DRIVE_MANUAL     = 0
uint8   DRIVE_RIGHT_HAND = 1
uint8   DRIVE_LEFT_HAND  = 2
uint8   drive_mode
string  map_name          # save_map 시 맵 이름
string  save_directory    # save_map 시 저장 경로
---
bool    success
string  message
```

### 4.3 Messages

```
# MapperStatus.msg (orchestrator → UI, 10Hz, RELIABLE + VOLATILE)
uint8   STATE_IDLE              = 0
uint8   STATE_ALIGNING          = 1
uint8   STATE_STARTING_SLAM     = 2
uint8   STATE_VERIFYING_MAP     = 3
uint8   STATE_MAPPING_MANUAL    = 4
uint8   STATE_MAPPING_AUTO      = 5
uint8   STATE_EXPLORING_UNKNOWN = 6
uint8   STATE_LOOP_CLOSING      = 7
uint8   STATE_COMPLETED         = 8
uint8   STATE_ERROR             = 9
uint8   STATE_PAUSED            = 10
uint8   state
uint8   previous_state          # PAUSED 복귀 시 이전 상태 저장
float32 coverage_percent        # 0.0~100.0 (EXPLORING 단계부터 유효)
float64 current_heading_error_deg
string  log_message             # 변경 시에만 전송 (rate-limit 정책)
```

---

## 5. 상태 머신 (mapper_orchestrator_node)

```
IDLE
 │ [CMD_START_MAPPING]
 ▼
ALIGNING_INITIAL
 │  WallAlignAction(tolerance_deg=0.2)
 │  [status=success]           [status=failed or retry>3]
 ▼                              ▼
STARTING_SLAM                  ERROR ──[CMD_STOP]──▶ IDLE
 │  SlamControl.srv(CMD_START_2D/3D) → slam_manager
 │  맵 안정화 대기: min 3초 + /map 첫 수신
 │ [slam running + map received]
 ▼
VERIFYING_MAP
 │  MapAlignmentCheckAction(tolerance_deg=0.5)
 │  HoughLinesP 파라미터 미검출 시 → 정렬 통과(fallback)로 처리
 │ [is_aligned=true]           [is_aligned=false, retry≤3]
 ▼                              ▼
MAPPING                        SlamControl.srv(CMD_STOP) → ALIGNING_INITIAL
 │
 ├── [DRIVE_MANUAL]
 │     YawControlAction (IMU 헤딩 유지 직진)
 │     SpinAction (UI 회전 명령)
 │     [CMD_PAUSE]  → previous_state=STATE_MAPPING_MANUAL → PAUSED
 │     [CMD_EXPLORE] → EXPLORING_UNKNOWN
 │
 └── [DRIVE_RIGHT_HAND / DRIVE_LEFT_HAND / DRIVE_AUTO]
       YawControlAction + SpinAction 반복 (우수법/좌수법 벽 추종)
       [벽 추종 1바퀴 완주: 시작점 재방문 + coverage ≥ min_coverage_to_stop]
       [CMD_PAUSE] → previous_state=STATE_MAPPING_AUTO → PAUSED
       → EXPLORING_UNKNOWN
 │
PAUSED
 │ [CMD_RESUME] → previous_state 복원
 ▼
(이전 상태로 복귀: previous_state_ 변수로 저장/복원)
 │
 ▼
EXPLORING_UNKNOWN
 │  ExploreUnknownAction(mode, min_coverage_to_stop=0.95)
 │  [CMD_PAUSE] → previous_state=STATE_EXPLORING_UNKNOWN → PAUSED
 │  장애물 감지 시: 정지 → 경로 재계획 (최대 3회)
 │ [coverage≥95% or no_unknown_left]
 ▼
LOOP_CLOSING
 │  BFS로 시작점 복귀 → 로봇 정지 대기 2초
 │  /rtabmap/info loop_closure_id > 0 대기
 │  타임아웃(기본 30초) 시 → 맵 저장 후 COMPLETED (루프 클로저 없이)
 ▼
COMPLETED
```

---

## 6. 핵심 알고리즘

### 6.1 벽 검출 및 정렬 (WallAlignerNode)

**RANSAC 직선 검출:**
- `/scan` LaserScan → Cartesian 포인트 변환
- RANSAC 반복 샘플링으로 인라이어 최다 직선 추출
- 각도를 **0~180도 범위로 정규화 (modulo 180)** — 180도 모호성 제거
- 후보 직선들 중 인라이어 수 기준 최장 직선 선택

**SpinAction 파라미터 매핑:**

| WallAligner 계산값 | SpinAction Goal 필드 | 비고 |
|---|---|---|
| `wall_angle_deg` (modulo 180, 맵 프레임) | `target_angle` | **단일 소스: `/tf` map→base_link 기준** |
| 파라미터 `spin_speed` | `max_angular_speed` | 기본 40 deg/s |
| 파라미터 `spin_accel` | `angular_acceleration` | 기본 30 deg/s² |
| `false` | `hold_steer` | 2WD diff drive — 무시됨 |
| `0.0` | `exit_steer_angle` | 2WD diff drive — 무시됨 |

> **주의**: `current_yaw`와 `wall_angle_deg` 모두 `/tf` map→base_link 에서 계산.
> IMU 소스와 혼용 금지 (프레임 불일치 시 정렬 발산).

**수렴 루프 의사코드:**
```
for attempt in range(max_attempts=5):
    scan = get_latest_scan()
    wall_angle = ransac_detect(scan) % 180.0  # 180도 정규화
    robot_yaw  = get_tf_yaw(map→base_link)    # 단일 소스
    error = normalize_angle(robot_yaw - wall_angle)
    if abs(error) <= tolerance_deg:
        return SUCCESS(aligned_heading=robot_yaw)
    spin_goal.target_angle = robot_yaw - error
    result = send_spin_goal_and_wait(spin_goal)
    if result.status == CANCELLED: return CANCELLED
    if result.status == TF_FAIL:   return FAILED
return FAILED
```

### 6.2 맵 정렬 검증 (MapAlignmentCheckerNode)

- `/map` OccupancyGrid → 점유 셀(value≥50) 추출
- OpenCV `HoughLinesP` 직선 검출
  - **fallback**: 직선 미검출 시 `is_aligned=true` 반환 (매핑 진행 허용)
  - HoughLinesP 파라미터 (ROS 파라미터로 노출): `rho`, `theta`, `threshold`, `min_line_length`, `max_line_gap`
- 각 직선 각도와 0°/90°/180° 편차 계산
- `max_wall_error_deg` > `tolerance_deg` → `is_aligned=false`

### 6.3 미탐색 영역 탐색 (ExplorationPlannerNode)

**BFS 탐색:**
- `/map` OGM에서 미탐색 셀(value==-1) 클러스터링
- BFS 계산 시작 시 맵 데이터 **깊은 복사** (실시간 업데이트와 분리)
- 접근 불가 클러스터 skip 후 다음 후보 선택 (isolated island 처리)
- BFS: 격자 기반 4방향 최단 경로 (대각선 없음)
- **연속 동일 방향 셀 병합 → 하나의 Segment로 압축** (100셀 → 수 개 Segment)

**ActionChainer Segment 매핑 (Segment.msg 필드 기준):**

| Segment 필드 | SPIN 세그먼트 | YAWCTRL 세그먼트 | 비고 |
|---|---|---|---|
| `action_type` | `SPIN` | `YAWCTRL` | |
| `segment_id` | BFS 인덱스 | BFS 인덱스 | |
| `waypoint_from` | 시퀀스 ID (정수) | 시퀀스 ID (정수) | 좌표 아님 |
| `waypoint_to` | 시퀀스 ID+1 | 시퀀스 ID+1 | 좌표 아님 |
| `start_x/y` | 회전 시작 위치 | 직진 시작 위치 | map frame (m) |
| `end_x/y` | 회전 시작 위치 (동일) | 직진 끝 위치 | map frame (m) |
| `spin_angle` | BFS 방향전환 각도 (deg) | 0.0 | |
| `max_linear_speed` | 0.0 | 파라미터 (기본 0.2 m/s) | |
| `acceleration` | 0.0 | 파라미터 (기본 0.3 m/s²) | |
| `stop_distance` | 0.0 | `end_x/y - start_x/y` 거리 | **무한직진 방지** |
| `exit_speed` | 0.0 | `has_next ? cruise_speed : 0.0` | |
| `has_next` | true/false | true/false | 마지막 Segment면 false |

**탐색 실패 시 재계획:**
- ActionChainer 세그먼트 실패 → 현재 위치에서 BFS 재계획 (최대 3회)
- 3회 실패 → ExploreUnknown action `status=-1(FAILED)` 반환

**장애물 회피:**
- YawControl 직진 중 `/scan` 기반 근접 장애물 감지 (파라미터: `obstacle_threshold_m`, 기본 0.3m)
- 감지 시: YawControlAction 취소 → 정지 → BFS 재계획

### 6.4 루프 클로저 (OrchestratorNode)

- STARTING_SLAM → 맵 안정화(3초 + /map 수신) 완료 후 시작 위치 기록 (`/tf` map→base_link)
- LOOP_CLOSING: BFS로 시작점 복귀 (기탐색 구간 우선) → 정지 → 2초 대기
- `/rtabmap/info`의 `loop_closure_id > 0` 수신 → COMPLETED
- **타임아웃 30초** 초과 시 → 경고 로그 후 맵 저장 + COMPLETED (루프 클로저 없이 완료)

### 6.5 Orchestrator Executor 모델 (데드락 방지)

**잘못된 패턴 (사용 금지):**
```cpp
// ❌ MutuallyExclusiveCallbackGroup에서 spin_until_future_complete → 데드락
auto future = action_client->async_send_goal(goal);
executor.spin_until_future_complete(future);  // 같은 그룹 콜백 영원히 dispatch 안됨
```

**올바른 패턴 (기존 ActionChainer와 동일):**
```cpp
// ✅ detached thread + atomic flag polling
std::atomic<bool> done{false};
action_client->async_send_goal(goal, [&done](auto result){ done.store(true); });
while (!done.load() && rclcpp::ok()) {
    std::this_thread::sleep_for(10ms);
}
```

**Executor 구성:**
- `MultiThreadedExecutor` 사용
- 상태 머신 루프: `MutuallyExclusiveCallbackGroup` (전용 스레드, detached polling)
- 서비스 서버(MapperCommand): `MutuallyExclusiveCallbackGroup` (별도 스레드)
- 액션 클라이언트 결과 콜백: `ReentrantCallbackGroup` (여러 콜백 동시 처리 허용)

---

## 7. slam_manager 리팩터링 요구사항

기존 `slam_manager_2d`는 UI 객체 직접 주입(`__init__(self, ui_window)`) 구조.
SlamControl.srv 통합을 위해 다음 리팩터링 필요:

### slam_manager_2d 변경 사항
1. **headless 모드 추가**: `ui_window=None` 허용, `self.ui.log()` 호출을 `if self.ui:` 가드로 감싸기
2. **SlamControl.srv 서비스 서버 추가**: `create_service(SlamControl, 'slam_control', callback)`
3. **비동기 시작 처리**: `start_launch_file()`의 `time.sleep(0.5)` → 별도 Thread로 분리, 서비스 응답은 PID 확인 즉시 반환
4. **기존 UI 기능 보존**: slam_manager_2d_ui.py의 버튼 동작 변경 없음

### slam_manager_3d
- headless 모드 이미 존재 → SlamControl.srv 서버 추가만 필요

---

## 8. mapper_ui 패키지 구조

```
mapper_ui/
├── package.xml
├── setup.py
├── ui/
│   └── mapper_main.ui          # Qt Designer XML
├── mapper_ui/
│   ├── __init__.py
│   ├── mapper_ui_node.py       # ROS2 노드, QTimer(100ms)+spin_once()
│   └── ros_bridge.py           # ROS2 콜백 → Qt Signal 변환
└── resource/
    └── mapper_ui
```

**UI 구성 요소 (mapper_main.ui):**
- **E-STOP 버튼** (모든 상태에서 항상 표시, 하단 고정, 빨간색)
- SLAM 모드 선택 (2D/3D ComboBox, IDLE 상태에서만 변경 가능)
- 탐색 모드 선택 (수동/우수법/좌수법/자동, IDLE 상태에서만 변경 가능)
- 상태 표시 레이블 (색상: IDLE=회색, 진행=파란색, ERROR=빨간색, COMPLETED=초록색, PAUSED=노란색)
- 헤딩 오차 표시 (`current_heading_error_deg`, ALIGNING 단계 모니터링)
- 커버리지 프로그레스바 (EXPLORING 단계부터 활성화, 초기값 0%)
- 매핑 시작 / 일시정지(토글→재개) / 중지 버튼
- 탐색 시작 버튼 (수동 모드에서만 표시)
- 수동 제어 버튼 (전진/좌회전/우회전, MAPPING_MANUAL 상태에서만 활성화)
- 로그 텍스트 영역 (변경 시만 append, rate-limit)

**상태별 버튼 활성화 매트릭스:**

| 버튼 | IDLE | ALIGNING | MAPPING_MANUAL | MAPPING_AUTO | EXPLORING | PAUSED | ERROR |
|------|------|----------|----------------|--------------|-----------|--------|-------|
| E-STOP | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| 매핑 시작 | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| 일시정지 | ❌ | ❌ | ✅ | ✅ | ✅ | ❌ | ❌ |
| 재개 | ❌ | ❌ | ❌ | ❌ | ❌ | ✅ | ❌ |
| 중지 | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| 탐색 시작 | ❌ | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ |
| 수동 제어 | ❌ | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ |

**UI 모드 ↔ 상태 머신 매핑:**

| UI 탐색 모드 | 상태 머신 상태 | ExploreUnknown mode |
|---|---|---|
| 수동 | MAPPING_MANUAL → (CMD_EXPLORE) → EXPLORING | — |
| 우수법 | MAPPING_AUTO → (자동) → EXPLORING | MODE_RIGHT_HAND |
| 좌수법 | MAPPING_AUTO → (자동) → EXPLORING | MODE_LEFT_HAND |
| 자동 | MAPPING_AUTO → (자동) → EXPLORING | MODE_AUTO |

---

## 9. QoS 프로파일

| 토픽 | 방향 | Reliability | Durability | Depth |
|------|------|-------------|------------|-------|
| `/scan` | 구독 | BEST_EFFORT | VOLATILE | 1 |
| `/map` | 구독 | RELIABLE | TRANSIENT_LOCAL | 3 |
| `/tf` | 구독 | RELIABLE | VOLATILE | 10 |
| `/rtabmap/info` | 구독 | RELIABLE | VOLATILE | 10 |
| `mapper/status` | 발행 | RELIABLE | VOLATILE | 10 |

> `/map` depth=3: BFS 계산 중 맵 갱신 시 정합성 보장

---

## 10. 파라미터 구조

### mapper_orchestrator_node 파라미터 (mapper_params.yaml)
```yaml
mapper_orchestrator:
  slam_mode: 0                    # 0=2D, 1=3D
  drive_mode: 0                   # 0=manual, 1=right_hand, 2=left_hand
  max_align_retries: 3
  map_stabilize_wait_sec: 3.0
  loop_closure_timeout_sec: 30.0
  min_coverage_to_stop: 0.95
  obstacle_threshold_m: 0.3
  exploration_max_replan: 3

wall_aligner:
  tolerance_deg: 0.2
  max_attempts: 5
  spin_speed_deg_s: 40.0
  spin_accel_deg_s2: 30.0

map_alignment_checker:
  tolerance_deg: 0.5
  hough_rho: 1.0
  hough_theta: 0.0174532925       # 1도 = pi/180
  hough_threshold: 50
  hough_min_line_length: 20.0
  hough_max_line_gap: 5.0

exploration_planner:
  max_linear_speed: 0.2
  acceleration: 0.3
  cruise_speed: 0.15
  coverage_cell_size_m: 0.05      # OGM 해상도
```

---

## 11. 테스트 전략

### 단위 테스트 (노드별 독립)

| 노드 | Mock/Stub | 테스트 방법 |
|------|-----------|------------|
| `wall_aligner_node` | Mock SpinAction 서버 + bag 파일 `/scan` | RANSAC 각도 정확도, 180도 모호성, 수렴 루프 |
| `map_alignment_checker_node` | bag 파일 `/map` | 정렬/불일치 OGM 입력, fallback 동작 |
| `exploration_planner_node` | Mock ActionChainer + bag 파일 `/map` | BFS 경로, Segment 병합, 접근불가 클러스터 skip |
| `mapper_orchestrator_node` | 전체 Mock 노드 | 상태 전이, PAUSED/resume, ERROR 복구 |

### 통합 테스트 (Gazebo SIL)
- rtabmap 2D + AMR 모델로 전체 매핑 절차 시뮬레이션
- 수용 기준: 커버리지 ≥ 95%, 정렬 오차 ≤ 0.2도, 루프 클로저 확정

### 커버리지 계산 방식
```
coverage = (탐색된 free 셀 수) / (전체 free + 미탐색 셀 수) × 100
탐색된 free 셀: value == 0 (흰색)
미탐색 셀: value == -1 (회색)
```

---

## 12. 기존 시스템 의존성

| 의존 대상 | 용도 | 변경 사항 |
|-----------|------|----------|
| `amr_interfaces` | SpinAction, YawControlAction, TranslateAction | 없음 |
| `waypoint_manager` ActionChainer | Segment 시퀀스 실행 | Segment 필드 매핑 규칙 추가 |
| `slam_manager_2d` (기존) | rtabmap 2D subprocess 관리 | headless 모드 + SlamControl.srv 추가 |
| `slam_manager_3d` (기존) | rtabmap 3D subprocess 관리 | SlamControl.srv 추가 |
| `/scan` (LaserScan) | 벽 검출 입력 | QoS: BEST_EFFORT + VOLATILE |
| `/map` (OccupancyGrid) | 탐색 계획 입력 | QoS: RELIABLE + TRANSIENT_LOCAL, depth=3 |
| `/tf` | 로봇 위치 (map→base_link) | 단일 소스 — IMU와 혼용 금지 |
| `/rtabmap/info` | 루프 클로저 감지 | 신규 구독 |

---

## 13. 구현 제약

- 이동: 90도 회전 + 직진만 허용, 곡선/대각선 없음
- 정렬 허용 오차: 0.2도 이내
- 맵 재정렬 시도: 최대 3회, 초과 시 ERROR
- SLAM 시작 후 맵 안정화: 최소 3초 + `/map` 첫 수신 후 시작 위치 기록
- SLAM 백엔드: rtabmap 2D 또는 3D (UI에서 선택, IDLE 상태에서만 변경)
- LiDAR 입력: `/scan` LaserScan (2D/3D 무관 추상화)
- 루프 클로저: rtabmap 내장 graph optimization 위임, 외부 ICP 없음
- 루프 클로저 타임아웃: 30초 후 COMPLETED (맵 저장 포함)
- Executor: MultiThreadedExecutor + detached thread polling (spin_until_future_complete 금지)
- 탐색 재계획: 최대 3회
- 장애물 감지 임계거리: 0.3m (파라미터)
