# SIL Simulation Package Integration Plan

**Date**: 2026-03-29
**Source**: https://github.com/kuks2309/T-Robot_nav_ros2_ws/tree/main/src/SIL/amr_motion_control_simulation
**Target**: ros2_3dslam_ws (Pioneer 2DX, 2WD Differential Drive)
**Analysis**: 10-agent parallel analysis completed

---

## 1. Executive Summary

upstream `amr_motion_control_simulation`은 **4WS(4-Wheel-Steer) T-Robot**용 SIL(Software-In-the-Loop) 경로 예측기입니다.
우리 로봇은 **Pioneer 2DX 2WD 차동구동**이므로 직접 포팅은 불가하며, **적응 전략(adaptation strategy)**이 필요합니다.

### 핵심 결정사항

| 결정 | 선택 | 근거 |
|------|------|------|
| 적용 전략 | **Option B: 2WD 적응형 신규 패키지** | 4WS bicycle model은 2WD에 무의미 |
| 제어 모드 | **Mode A (direct vx/omega)** | 차동구동은 steering angle 없음 |
| 패키지 위치 | `src/Control/AMR-Motion-Control/amr_motion_control_simulation/` | 기존 AMR 패키지와 동일 디렉토리 |
| 빌드 타입 | `ament_cmake` + pybind11 | C++ 성능 + Python 접근성 |
| 핵심 의존성 | 자체 구현 (upstream `amr_motion_control` 미포팅) | 4WS 라이브러리 불필요 |

---

## 2. Upstream 패키지 분석 (10-Agent Findings)

### 2.1 아키텍처

```
amr_motion_control_simulation/
├── include/.../sil_predictor.hpp    # SilPredictor 클래스 (Config, predict())
├── include/.../sim_types.hpp        # SimState, SimTranslateGoal 구조체
├── src/sil_predictor.cpp            # 예측 엔진 (제어 루프 시뮬레이션)
├── src/bindings.cpp                 # pybind11 Python 바인딩
├── test/test_sil_predictor.cpp      # GTest 6개 테스트
├── CMakeLists.txt                   # ament_cmake + pybind11
└── package.xml
```

### 2.2 Upstream 의존성 (W1 분석)

upstream은 `amr_motion_control` 라이브러리의 4개 타겟에 의존:
- `amr_motion_control::inverse_kinematics` — DualSteerIK (4WS 역기구학)
- `amr_motion_control::bicycle_model` — BicycleModel (4WS 자전거 모델)
- `amr_motion_control::motion_profile` — TrapezoidalProfile (사다리꼴 속도 프로파일)
- `amr_motion_control::path_controller` — PathController (경로 추종 제어기)

**이 라이브러리는 로컬 워크스페이스에 존재하지 않음** (upstream T-Robot 레포에만 존재).

### 2.3 제어 모드 호환성 (W2 분석)

| Mode | Upstream | 2WD 적용 가능성 | 설명 |
|------|----------|-----------------|------|
| **A (0/1)** | Direct vy/omega | **적용 가능** | vy=0 고정, omega만 사용 |
| **B (2)** | BicycleModel | **불가** | 차동구동에 bicycle model 무의미 |
| **C (3)** | Sequential CTE | **부분 가능** | 단일 세그먼트 CTE는 기존 코드에 존재 |
| **D (4)** | Pure Stanley | **부분 가능** | Stanley 존재하나 PD+Stanley 융합 형태 |
| **E (5)** | Front Stanley | **불가** | 전방 축 개념 없음 |

**결론**: Mode A를 기본으로 사용, PathController의 핵심 알고리즘(Stanley+PD)을 차동구동용으로 적응.

### 2.4 기하학 파라미터 매핑 (W9 분석)

| Parameter | 4WS (upstream) | 2WD (Pioneer 2DX) | 출처 |
|-----------|---------------|-------------------|------|
| Wheel positions | (±0.330, ±0.135) | (0.10, ±0.17) | model.sdf |
| Wheel separation | 0.270 m | **0.34 m** | motion_params_gazebo.yaml |
| Wheel radius | 0.080 m | **0.11 m** | model.sdf |
| Max linear vel | N/A | **0.4 m/s** | DiffDrive plugin |
| Max angular vel | N/A | **1.0 rad/s** | DiffDrive plugin |
| Max linear accel | N/A | **1.5 m/s²** | DiffDrive plugin |
| Steering limit | π/4 (45°) | **N/A** (steering 없음) | — |
| gear_walk | 20.0 | **N/A** (steering gear 없음) | — |

### 2.5 호환성 요약 (W3-W6 분석)

| 컴포넌트 | 호환성 | 핵심 이슈 |
|----------|--------|----------|
| **TrapezoidalProfile** | **부분 호환** | `entry_speed` 파라미터 누락, include 경로 불일치 |
| **PathController** | **신규 구현 필요** | 독립 클래스 미존재 (inline 코드만), 2WD용 `(vx, omega)` 출력 필요 |
| **BicycleModel** | **불필요** | 차동구동 kinematic model로 대체 |
| **DualSteerIK** | **불필요** | SIL predict()에서 메서드 호출 없음 (링크 의존만) |

### 2.6 Pybind11 환경 (W7 분석)

- **pybind11-dev 2.9.1**: 설치 완료, `find_package(pybind11)` 사용 가능
- **Python 3.10.12**: ROS Humble 호환
- **GIL release**: `predict()` 메서드에서 안전하게 GIL 해제 가능 (순수 C++ 경로)
- **설치 경로**: `lib/python3.10/dist-packages/amr_motion_control_simulation/`

### 2.7 테스트 인프라 (W8 분석)

- **GTest**: `ament_cmake_gtest` 설치 확인됨
- Upstream 6개 테스트 모두 4WS 기하학 기반 → 2WD 파라미터로 재작성 필요
- `TrapezoidalProfile` 입력 검증 edge case 발견: negative distance → NaN, zero acceleration → div/0

---

## 3. 발견된 기존 버그 (Bonus Findings)

분석 과정에서 기존 `amr_motion_control_2wd` 코드의 버그 3건 발견:

### Bug 1: `wheel_base` 파라미터 불일치 (W9, CRITICAL)
- **위치**: `translate_action_server.cpp:37`, `translate_reverse_action_server.cpp:31`
- **문제**: `wheel_base` 기본값 0.54가 Pioneer 2DX의 어떤 치수와도 불일치 (실제 wheel_separation=0.34)
- **영향**: RPM 피드백 계산 오류

### Bug 2: Dead code (W2, LOW)
- **위치**: `translate_action_server.cpp:476`
- **문제**: `omega_pd + sign_v * (stanley_correction / dt) * 0.0` — `* 0.0` 곱셈으로 dead code

### Bug 3: ActionGuard 누락 (W2, MEDIUM)
- **위치**: `TranslateActionServer::execute()`
- **문제**: `TranslateReverseActionServer`에는 ActionGuard 있지만 forward Translate에는 없음
- **영향**: 동시 실행 시 cmd_vel 충돌 가능

---

## 4. 적응형 구현 계획

### Phase 0: 선행 버그 수정 (1일)

| # | 작업 | 파일 |
|---|------|------|
| 0.1 | `wheel_base` → `wheel_separation` 파라미터 수정 | translate_action_server.cpp, translate_reverse_action_server.cpp |
| 0.2 | Dead code 제거 (line 476 `* 0.0`) | translate_action_server.cpp |
| 0.3 | ActionGuard 추가 | translate_action_server.cpp |

### Phase 1: Core Library — DiffDrive SIL Predictor (2일)

upstream `sil_predictor.cpp`의 predict() 로직을 **차동구동 kinematic model**로 재작성.

#### 1.1 SimTypes 정의 (`sim_types.hpp`)

upstream `SimState`와 `SimTranslateGoal`을 거의 그대로 사용하되 조정:

```cpp
struct SimState {
  double t, x, y, yaw;      // 시간, 위치, 방향
  double vx;                 // 전진 속도 (m/s)
  double vy = 0.0;           // 항상 0 (차동구동)
  double omega;              // 각속도 (rad/s)
  double delta_f = 0.0;      // 사용 안 함 (호환성)
  double delta_r = 0.0;      // 사용 안 함 (호환성)
  double e_d;                // CTE (m)
  double e_theta;            // 방향 오차 (rad)
  double projection;         // 경로 진행 거리 (m)
  uint8_t phase;             // ACCEL/CRUISE/DECEL/DONE
};

struct SimTranslateGoal {
  double start_x, start_y, end_x, end_y;
  double max_linear_speed;
  double acceleration;
  double exit_speed = 0.0;
  double entry_speed = 0.0;   // 신규 추가 (upstream 호환)
  uint8_t control_mode = 0;   // 0=Direct (Mode A only for 2WD)
  double start_yaw = 0.0;
};
```

#### 1.2 DiffDrive SilPredictor (`sil_predictor.hpp/cpp`)

```cpp
struct Config {
  // 로봇 기하 (Pioneer 2DX)
  double wheel_separation = 0.34;
  double wheel_radius = 0.11;

  // 속도/가속도 제한
  double max_linear_vel = 0.4;
  double max_angular_vel = 1.0;
  double max_linear_accel = 1.5;

  // 제어 파라미터
  double control_rate_hz = 20.0;
  double K_stanley = 2.0;
  double K_soft = 0.8;
  double Kp_heading = 1.0;
  double Kd_heading = 0.3;
  double min_vx = 0.02;
  double behind_start_speed = 0.2;
  double goal_reach_threshold = 0.05;
  double max_timeout_sec = 60.0;
};
```

**핵심 변경 — Forward Model**:

upstream (4WS bicycle):
```
omega = vx * (tan(delta_f) - tan(delta_r)) / L
vy = vx * tan(delta_r)
```

**2WD (차동구동)**:
```
x += vx * cos(yaw) * dt      // vy = 0 항상
y += vx * sin(yaw) * dt
yaw += omega * dt
```

**제어기**: Stanley+PD 융합 (기존 translate_action_server.cpp 알고리즘 추출)
```
e_lateral = -(dx * path_uy - dy * path_ux)
e_theta_filt = movingAverage(e_theta, window=5)
de_theta = (e_theta_filt - prev_e_theta) / dt
stanley_correction = atan2(K_stanley * e_lateral, |vx| + K_soft)
omega = sign_v * (Kp * (e_theta_filt + stanley_correction) + Kd * de_theta)
```

- `e_theta_filt`: 헤딩 오차 이동평균 필터링 (window=5), raw `e_theta` 직접 사용 안 함
- `sign_v = (vx >= 0) ? 1.0 : -1.0`: 역주행 지원을 위한 부호 승수
- `de_theta`: 필터링된 헤딩 오차의 미분 (`e_theta_filt` 기반, raw `e_theta` 아님)

#### 1.3 TrapezoidalProfile `entry_speed` 확장

W5 발견: upstream은 5개 파라미터 생성자 사용, 우리는 4개만 지원.

```cpp
// 변경 전:
TrapezoidalProfile(double target_distance, double max_speed, double acceleration, double exit_speed = 0.0);

// 변경 후:
TrapezoidalProfile(double target_distance, double max_speed, double acceleration,
                   double exit_speed = 0.0, double entry_speed = 0.0);
```

`entry_speed` 추가 시 운동학 공식 변경:
- ACCEL: `v = sqrt(entry_speed² + 2 * a * pos)` (기존: `sqrt(2*a*pos)`)
- accel_dist 재계산: `(max_speed² - entry_speed²) / (2*a)`

### Phase 2: Pybind11 Python 바인딩 (1일)

upstream `bindings.cpp`를 거의 그대로 사용. 변경점:
- `DualBicycleCommand` 관련 바인딩 제거
- `DualSteerIK` 바인딩 제거
- Config struct 필드 변경 (wheel_separation 등)

```cmake
# CMakeLists.txt
find_package(pybind11 REQUIRED)
pybind11_add_module(_sil_predictor_cpp src/bindings.cpp)
target_link_libraries(_sil_predictor_cpp PRIVATE sil_predictor)
```

Python에서 사용:
```python
from amr_motion_control_simulation import SilPredictor, Config, SimTranslateGoal
config = Config()
config.wheel_separation = 0.34
predictor = SilPredictor(config)
trajectory = predictor.predict(goal)
```

### Phase 3: GTest 단위 테스트 (1일)

Upstream 6개 테스트를 2WD 파라미터로 재작성:

| # | 테스트 | 적응 사항 |
|---|--------|----------|
| 1 | StraightLineReachesGoal | control_mode=0, 2WD Config |
| 2 | CTEStaysNearZero | CTE < 10mm (diff-drive에서도 유효) |
| 3 | PhaseOrder | 3m 경로에서 ACCEL→CRUISE→DECEL 확인 |
| 4 | VelocityContinuity | entry_speed 지원 후 세그먼트 연속성 |
| 5 | NoPosJump | 위치 점프 없음 확인 |
| 6 | ZeroDistance | 빈 결과 반환 |

추가 테스트:
| 7 | PureRotation | vx=0, omega!=0 — 차동구동 고유 |
| 8 | ReverseTravel | max_linear_speed < 0 시 역주행 |
| 9 | EntrySpeedProfile | entry_speed > 0 시 프로파일 정합성 |
| 10 | VelocitySaturation | max_vel 초과 시 클램핑 확인 |

### Phase 4: 통합 및 검증 (1일)

#### 4.1 패키지 구조

```
src/Control/AMR-Motion-Control/amr_motion_control_simulation/
├── CMakeLists.txt
├── package.xml
├── include/amr_motion_control_simulation/
│   ├── sil_predictor.hpp
│   └── sim_types.hpp
├── src/
│   ├── sil_predictor.cpp
│   └── bindings.cpp
├── test/
│   └── test_sil_predictor.cpp
└── docs/
    └── run_commands.md
```

#### 4.2 빌드 순서

```
amr_interfaces (Level 0)
  └── amr_motion_control_2wd (Level 1) ← Phase 0 버그 수정
       └── amr_motion_control_simulation (Level 2) ← 자체 의존성만
```

**핵심**: `amr_motion_control_simulation`은 upstream `amr_motion_control`에 의존하지 않음.
TrapezoidalProfile과 제어 알고리즘을 **자체 구현**하여 독립 패키지로 구성.

#### 4.3 검증 절차

```bash
# 1. 빌드
colcon build --packages-select amr_motion_control_simulation

# 2. C++ 테스트
colcon test --packages-select amr_motion_control_simulation
colcon test-result --packages-select amr_motion_control_simulation --verbose

# 3. Python 바인딩 확인
source install/setup.bash
python3 -c "from amr_motion_control_simulation import SilPredictor, Config, SimTranslateGoal; print('OK')"

# 4. 기존 패키지 회귀 테스트
colcon build --packages-select amr_motion_control_2wd
colcon test --packages-select amr_motion_control_2wd
```

---

## 5. 파일별 작업 목록

### 신규 생성 (7 파일)

| 파일 | 설명 |
|------|------|
| `amr_motion_control_simulation/CMakeLists.txt` | ament_cmake + pybind11 빌드 |
| `amr_motion_control_simulation/package.xml` | 패키지 메타데이터 |
| `amr_motion_control_simulation/include/.../sil_predictor.hpp` | SilPredictor 클래스 정의 |
| `amr_motion_control_simulation/include/.../sim_types.hpp` | SimState, SimTranslateGoal |
| `amr_motion_control_simulation/src/sil_predictor.cpp` | 2WD 예측 엔진 |
| `amr_motion_control_simulation/src/bindings.cpp` | pybind11 바인딩 |
| `amr_motion_control_simulation/test/test_sil_predictor.cpp` | 10개 단위 테스트 |

### 기존 수정 (3 파일 — Phase 0 버그 수정)

| 파일 | 변경 |
|------|------|
| `amr_motion_control_2wd/src/translate_action_server.cpp` | wheel_base→wheel_separation, dead code 제거, ActionGuard 추가 |
| `amr_motion_control_2wd/src/translate_reverse_action_server.cpp` | wheel_base→wheel_separation |
| `amr_motion_control_2wd/config/motion_params_gazebo.yaml` | wheel_base 파라미터 추가/정리 (선택) |

### 포팅하지 않는 것

| Upstream 컴포넌트 | 이유 |
|-------------------|------|
| `amr_motion_control` 패키지 전체 | 4WS 전용, 2WD에 불필요 |
| `BicycleModel` | 차동구동에 적용 불가 |
| `DualSteerIK` | SIL에서 미사용 + 2WD에 불필요 |
| `PathController` (upstream 버전) | 4WS 모드 B-E 포함, 우리 코드에서 알고리즘 추출로 대체 |
| Control Modes B, C, D, E | 차동구동에 Mode A만 유효 |

---

## 6. 리스크 및 완화

| 리스크 | 심각도 | 완화 |
|--------|--------|------|
| TrapezoidalProfile entry_speed 추가 시 기존 코드 영향 | HIGH | 기본값 0.0으로 하위 호환 유지, SIL 내부 별도 구현 고려 |
| ACS 통합과 action interface 충돌 | MEDIUM | SIL은 action 정의에 직접 의존하지 않음 (SimTranslateGoal 자체 타입 사용) |
| 예측 정확도 vs Gazebo 실제 거동 차이 | MEDIUM | Phase 4에서 Gazebo 비교 검증, 허용 오차 정의 (위치 5cm, 방향 2°) |
| pybind11 바이너리 호환성 | LOW | Python 3.10 고정, rebuild on upgrade |

---

## 7. 일정 추정

| Phase | 작업 | 예상 |
|-------|------|------|
| **0** | 기존 버그 수정 3건 | 0.5일 |
| **1** | Core Library (SilPredictor + SimTypes) | 2일 |
| **2** | Pybind11 바인딩 | 0.5일 |
| **3** | GTest 10개 + 검증 | 1일 |
| **4** | 통합, 빌드, 회귀 테스트 | 0.5일 |
| | **합계** | **~4.5일** |

---

## 8. 향후 확장 (Out of Scope)

- [ ] GUI SIL 시뮬레이터 (`sil_path_viewer.py` — upstream docs 참조)
- [ ] `amr_motion_test_ui`와 Python 통합 (SIL 예측 오버레이)
- [ ] `slam_manager_3d` SIL 경로 시각화 탭
- [ ] 역주행 16-시나리오 배치 테스트 (`translate_reverse_sim.py`)
- [ ] Multi-segment sequential CTE (Mode C 적응)
- [ ] PurePursuit action server 구현 (`AMRMotionPurePursuit.action` 인터페이스만 존재)
