# amr_motion_control_simulation (2WD SIL Predictor) 실행 명령어

## 개요

2WD 차동구동(Pioneer 2DX) AMR용 Software-In-the-Loop 경로 예측 패키지.
T-Robot 4WS SIL 패키지를 기반으로 차동구동 kinematic model로 적응.

## 빌드

```bash
colcon build --packages-select amr_motion_control_simulation
```

## 단위 테스트

```bash
colcon test --packages-select amr_motion_control_simulation
colcon test-result --packages-select amr_motion_control_simulation --verbose
```

## Python 바인딩 확인

```bash
source install/setup.bash
python3 -c "from amr_motion_control_simulation import SilPredictor, Config, SimTranslateGoal, SimState; print('OK')"
```

## Python 사용 예제

```python
from amr_motion_control_simulation import SilPredictor, Config, SimTranslateGoal

# 기본 설정 (Pioneer 2DX)
config = Config()
config.wheel_separation = 0.34
config.wheel_radius = 0.11

predictor = SilPredictor(config)

# 2m 직선 주행 예측
goal = SimTranslateGoal()
goal.start_x = 0.0
goal.start_y = 0.0
goal.end_x = 2.0
goal.end_y = 0.0
goal.max_linear_speed = 0.3
goal.acceleration = 0.5

trajectory = predictor.predict(goal)
print(f"Trajectory has {len(trajectory)} states")
print(f"Final position: ({trajectory[-1].x:.3f}, {trajectory[-1].y:.3f})")
print(f"Final phase: {trajectory[-1].phase}")  # 4 = DONE
```

## Config 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| wheel_separation | 0.34 | 바퀴 간 거리 (m) |
| wheel_radius | 0.11 | 바퀴 반경 (m) |
| max_linear_vel | 0.4 | 최대 직선 속도 (m/s) |
| max_angular_vel | 1.0 | 최대 각속도 (rad/s) |
| control_rate_hz | 20.0 | 제어 루프 주파수 (Hz) |
| K_stanley | 2.0 | Stanley 횡방향 게인 |
| Kp_heading | 1.0 | PD 비례 게인 |
| Kd_heading | 0.3 | PD 미분 게인 |
| goal_reach_threshold | 0.05 | 목표 도달 판정 (m) |

## upstream 대비 변경사항

- BicycleModel → DiffDrive kinematic model (vy=0 항상)
- 4WS Mode B-E 제거 → Mode A (direct vx/omega) 전용
- DualSteerIK 제거 (미사용)
- TrapezoidalProfile: entry_speed 파라미터 추가
- PathController2WD: Stanley+PD 제어기 독립 클래스화
- 로봇 기하학: Pioneer 2DX (wheel_separation=0.34, radius=0.11)
