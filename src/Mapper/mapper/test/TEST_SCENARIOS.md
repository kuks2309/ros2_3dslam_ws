# Mapper Orchestrator 테스트 시나리오

## 계층 구조

| 계층 | 파일 | Gazebo 필요 |
|------|------|------------|
| Unit | `test_orchestrator_states.cpp` | ✗ |
| Integration | `test_integration_gazebo.py` | ✓ |

---

## 1. Unit Tests (GTest — Gazebo 불필요)

노드를 직접 인스턴스화하여 `handle_command_public()` 로 상태 머신 로직만 검증한다.

### TC-U01: 초기 상태는 IDLE
- **전제**: 노드 생성 직후
- **동작**: 없음
- **기대**: `get_state() == IDLE`

### TC-U02: CMD_START_MAPPING (IDLE → ALIGNING)
- **전제**: `state == IDLE`
- **동작**: `CMD_START_MAPPING {slam_mode=0, drive_mode=0}`
- **기대**:
  - `res.success == true`
  - `res.message == "Mapping started"`
  - (비동기이므로 즉시 ALIGNING 확인은 불필요 — worker thread 지연 허용)

### TC-U03: CMD_START_MAPPING 이미 진행 중 → 실패
- **전제**: 이미 ALIGNING 상태 (TC-U02 이후)
- **동작**: `CMD_START_MAPPING`
- **기대**:
  - `res.success == false`
  - `res.message == "Not in IDLE state"`

### TC-U04: CMD_STOP (어떤 상태 → IDLE)
- **전제**: `state == IDLE` (IDLE에서도 STOP 허용)
- **동작**: `CMD_STOP`
- **기대**:
  - `res.success == true`
  - `get_state() == IDLE`

### TC-U05: CMD_PAUSE from IDLE → 실패
- **전제**: `state == IDLE`
- **동작**: `CMD_PAUSE`
- **기대**:
  - `res.success == false`
  - `get_state() == IDLE` (상태 변화 없음)
  - (MAPPING_AUTO는 CMD_PAUSE 허용 상태에서 제외됨)

### TC-U06: CMD_PAUSE from MAPPING_MANUAL → PAUSED
- **전제**: 내부적으로 `state_.store(MAPPING_MANUAL)` 설정
- **동작**: `CMD_PAUSE`
- **기대**:
  - `res.success == true`
  - `get_state() == PAUSED`
  - `previous_state_ == MAPPING_MANUAL`

### TC-U07: CMD_PAUSE from MAPPING_AUTO → PAUSED
- **전제**: `state_.store(MAPPING_AUTO)` 설정
- **동작**: `CMD_PAUSE`
- **기대**:
  - `res.success == true`
  - `get_state() == PAUSED`
  - `previous_state_ == MAPPING_AUTO`

### TC-U08: CMD_PAUSE from EXPLORING_UNKNOWN → PAUSED
- **전제**: `state_.store(EXPLORING_UNKNOWN)` 설정
- **동작**: `CMD_PAUSE`
- **기대**:
  - `res.success == true`
  - `get_state() == PAUSED`

### TC-U09: CMD_RESUME from PAUSED (이전=MAPPING_MANUAL)
- **전제**: `state == PAUSED`, `previous_state == MAPPING_MANUAL`
- **동작**: `CMD_RESUME`
- **기대**:
  - `res.success == true`
  - `get_state() == MAPPING_MANUAL`

### TC-U10: CMD_RESUME from PAUSED (이전=MAPPING_AUTO)
- **전제**: `state == PAUSED`, `previous_state == MAPPING_AUTO`
- **동작**: `CMD_RESUME`
- **기대**:
  - `res.success == true`
  - `get_state() == MAPPING_AUTO`

### TC-U11: CMD_RESUME from non-PAUSED → 실패
- **전제**: `state == IDLE`
- **동작**: `CMD_RESUME`
- **기대**:
  - `res.success == false`
  - `res.message == "Not paused"`

### TC-U12: CMD_SAVE_MAP SLAM 서비스 없을 때 → 실패
- **전제**: SLAM 서비스 미가동
- **동작**: `CMD_SAVE_MAP`
- **기대**:
  - `res.success == false`
  - `res.message` 에 `"SLAM service not ready"` 포함

### TC-U13: CMD_EXPLORE from MAPPING_MANUAL → EXPLORING_UNKNOWN
- **전제**: `state_.store(MAPPING_MANUAL)` 설정
- **동작**: `CMD_EXPLORE`
- **기대**:
  - `res.success == true`
  - `get_state() == EXPLORING_UNKNOWN`

### TC-U14: CMD_EXPLORE from IDLE → 실패
- **전제**: `state == IDLE`
- **동작**: `CMD_EXPLORE`
- **기대**:
  - `res.success == false`
  - `res.message == "Must be in MAPPING_MANUAL state"`

---

## 2. Integration Tests (Python — Gazebo 필요)

실제 ROS2 서비스 콜로 라이브 노드와 통신한다.  
사전 조건: `ros2 launch mapper mapper_gazebo.launch.py` 기동 완료.

### TC-I01: 서비스 응답성 확인 (CMD_STOP)
- **목적**: cb_group 수정 후 서비스가 응답하는지 기본 확인
- **동작**: `CMD_STOP` 서비스 콜
- **기대**:
  - 5 s 이내 응답 수신
  - `res.success == true`
  - `/mapper/status.state == STATE_IDLE`

### TC-I02: CMD_START_MAPPING → ALIGNING 전이
- **전제**: `state == IDLE`
- **동작**: `CMD_START_MAPPING {slam_mode=SLAM_2D, drive_mode=DRIVE_MANUAL}`
- **기대**:
  - 서비스 응답 5 s 이내, `res.success == true`
  - 30 s 이내 `/mapper/status.state == STATE_ALIGNING` 수신

### TC-I03: E-STOP (ALIGNING 중 → IDLE)
- **전제**: TC-I02 직후 (ALIGNING 상태)
- **동작**: `CMD_STOP`
- **기대**:
  - 5 s 이내 응답
  - 5 s 이내 `/mapper/status.state == STATE_IDLE`

### TC-I04: CMD_PAUSE from IDLE → 실패 응답
- **전제**: `state == IDLE`
- **동작**: `CMD_PAUSE`
- **기대**:
  - `res.success == false`
  - `/mapper/status.state` 변화 없음 (IDLE 유지)

### TC-I05: CMD_SAVE_MAP SLAM 미기동 → 실패 응답
- **전제**: SLAM 서비스 미가동 (IDLE 상태)
- **동작**: `CMD_SAVE_MAP`
- **기대**:
  - `res.success == false`
  - `res.message` 에 `"SLAM service not ready"` 포함

### TC-I06: 전체 매핑 플로우 (수동 드라이브)
- **사전**: Gazebo + SLAM 서비스 기동
- **동작**: `CMD_START_MAPPING {slam_mode=SLAM_2D, drive_mode=DRIVE_MANUAL}`
- **기대 상태 순서**:
  ```
  IDLE → ALIGNING → STARTING_SLAM → VERIFYING_MAP → MAPPING_MANUAL
  ```
- **완료 조건**: MAPPING_MANUAL 도달 확인 (600 s 타임아웃)
- **참고**: LOOP_CLOSING → COMPLETED 는 coverage 95% 도달 후 자동 전이

### TC-I07: 전체 매핑 플로우 (자율 드라이브)
- **사전**: Gazebo + SLAM + ExplorationPlanner 기동
- **동작**: `CMD_START_MAPPING {slam_mode=SLAM_2D, drive_mode=DRIVE_RIGHT_HAND}`
- **기대 상태 순서**:
  ```
  IDLE → ALIGNING → STARTING_SLAM → VERIFYING_MAP
       → MAPPING_AUTO → EXPLORING_UNKNOWN → LOOP_CLOSING → COMPLETED
  ```
- **완료 조건**: COMPLETED 도달 (600 s 타임아웃)

### TC-I08: PAUSE / RESUME (MAPPING_MANUAL 중)
- **전제**: TC-I06 성공 후 MAPPING_MANUAL 상태
- **동작 순서**:
  1. `CMD_PAUSE` → `res.success == true`, state → PAUSED
  2. 3 s 대기 (state 변화 없음 확인)
  3. `CMD_RESUME` → `res.success == true`, state → MAPPING_MANUAL

---

## 3. 실행 방법

```bash
# Unit tests
cd /home/amap/Study/ros2_3dslam_ws
colcon test --packages-select mapper --ctest-args -R test_orchestrator

# Integration tests (Gazebo 기동 후)
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/Mapper/mapper/test/test_integration_gazebo.py
```

---

## 4. 커버리지 매트릭스

| TC | 상태 전이 | 서비스 응답 | 오류 경로 |
|----|----------|------------|---------|
| U01 | ✓ 초기 | - | - |
| U02 | ✓ IDLE→ALIGNING | - | - |
| U03 | - | - | ✓ 중복 시작 |
| U04 | ✓ ANY→IDLE | - | - |
| U05 | - | - | ✓ 잘못된 상태 |
| U06–U08 | ✓ →PAUSED | - | - |
| U09–U10 | ✓ PAUSED→이전 | - | - |
| U11 | - | - | ✓ 비-PAUSED에서 RESUME |
| U12 | - | - | ✓ SLAM 미가동 |
| U13 | ✓ MANUAL→EXPLORING | - | - |
| U14 | - | - | ✓ 잘못된 상태 |
| I01 | - | ✓ 응답성 | - |
| I02 | ✓ IDLE→ALIGNING | ✓ | - |
| I03 | ✓ ALIGNING→IDLE | ✓ | - |
| I04 | - | ✓ | ✓ |
| I05 | - | ✓ | ✓ SLAM 미가동 |
| I06 | ✓ 전체(수동) | ✓ | - |
| I07 | ✓ 전체(자율) | ✓ | - |
| I08 | ✓ PAUSE/RESUME | ✓ | - |
