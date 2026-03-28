# AMR Motion Control 실행 명령어

## 1. 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --symlink-install --packages-up-to amr_interfaces amr_motion_control_2wd amr_motion_test_ui
source install/setup.bash
```

## 2. Gazebo 시뮬레이션 실행

### Step 1: Gazebo 실행 (odom TF 비활성화)

```bash
ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false
```

> `odom_tf:=false` — SLAM이 자체적으로 TF를 제공하므로 Gazebo odom TF 비활성화

### Step 2: 3D SLAM 실행 (새 터미널)

RTAB-Map 3D (3D LiDAR + RGB-D):
```bash
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_slam_gazebo.launch.py
```

RTAB-Map 3D (3D LiDAR Only):
```bash
ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_slam_gazebo.launch.py
```

LIO-SAM:
```bash
ros2 launch livox_lio_sam run_gazebo.launch.py
```

> SLAM 알고리즘에 따라 `motion_params_gazebo.yaml`의 `translate_pose_topic` 수정 필요:
> - RTAB-Map: `/rtabmap/odom` (기본값)
> - LIO-SAM: `/lio_sam/mapping/odometry`

### Step 3: Motion Control 실행 (새 터미널)

```bash
ros2 launch amr_motion_control_2wd motion_control_gazebo.launch.py
```

### Step 4: Test UI 실행 (새 터미널, 선택사항)

```bash
ros2 run amr_motion_test_ui test_ui_node
```

## 3. 액션 서버 확인

```bash
ros2 action list
```

예상 출력:
```
/amr_motion_translate
/amr_motion_yaw_control
/spin
/translate_reverse
/turn
```

## 4. CLI로 직접 테스트

### 4.1 Spin (제자리 회전)

```bash
# 90도 회전 (절대 각도, map 프레임)
ros2 action send_goal /spin amr_interfaces/action/AMRMotionSpin \
  "{target_angle: 90.0, max_angular_speed: 45.0, angular_acceleration: 90.0}" \
  --feedback

# 0도로 복귀
ros2 action send_goal /spin amr_interfaces/action/AMRMotionSpin \
  "{target_angle: 0.0, max_angular_speed: 45.0, angular_acceleration: 90.0}"

# 180도 회전
ros2 action send_goal /spin amr_interfaces/action/AMRMotionSpin \
  "{target_angle: 180.0, max_angular_speed: 45.0, angular_acceleration: 90.0}"
```

파라미터:
- `target_angle`: 목표 절대 각도 (deg, map 프레임, 최단 경로 자동 선택)
- `max_angular_speed`: 최대 회전 속도 (deg/s, Gazebo DiffDrive 제한: 57.3 deg/s = 1.0 rad/s)
- `angular_acceleration`: 회전 가속도 (deg/s²)

### 4.2 Turn (아크 턴)

```bash
# 90도 좌회전, 반경 0.5m
ros2 action send_goal /turn amr_interfaces/action/AMRMotionTurn \
  "{target_angle: 90.0, turn_radius: 0.5, max_linear_speed: 0.2, accel_angle: 10.0}" \
  --feedback

# 45도 우회전, 반경 0.3m
ros2 action send_goal /turn amr_interfaces/action/AMRMotionTurn \
  "{target_angle: -45.0, turn_radius: 0.3, max_linear_speed: 0.2, accel_angle: 10.0}"
```

파라미터:
- `target_angle`: 회전 각도 (deg, + CCW, - CW)
- `turn_radius`: 회전 반경 (m, > 0)
- `max_linear_speed`: 최대 선속도 (m/s, Gazebo 제한: 0.4 m/s)
- `accel_angle`: 가감속 구간 각도 (deg)

### 4.3 Translate (직선 이동)

```bash
# 현재 위치 확인
ros2 run tf2_ros tf2_echo map base_footprint

# (0,0)에서 (1,0)으로 1m 전진
ros2 action send_goal /amr_motion_translate amr_interfaces/action/AMRMotionTranslate \
  "{start_x: 0.0, start_y: 0.0, end_x: 1.0, end_y: 0.0, \
    max_linear_speed: 0.3, acceleration: 0.2, exit_speed: 0.0, has_next: false}" \
  --feedback

# 2m 전진 (시작점을 현재 위치에 맞춰서 입력)
ros2 action send_goal /amr_motion_translate amr_interfaces/action/AMRMotionTranslate \
  "{start_x: 1.0, start_y: 0.0, end_x: 3.0, end_y: 0.0, \
    max_linear_speed: 0.3, acceleration: 0.2, exit_speed: 0.0, has_next: false}"

# 대각선 이동 (1,0) → (2,1)
ros2 action send_goal /amr_motion_translate amr_interfaces/action/AMRMotionTranslate \
  "{start_x: 1.0, start_y: 0.0, end_x: 2.0, end_y: 1.0, \
    max_linear_speed: 0.3, acceleration: 0.2, exit_speed: 0.0, has_next: false}"
```

파라미터:
- `start_x`, `start_y`: 시작점 (m, map 프레임, 현재 로봇 위치와 가까워야 함)
- `end_x`, `end_y`: 목표점 (m, map 프레임)
- `max_linear_speed`: 최대 선속도 (m/s, Gazebo 제한: 0.4 m/s)
- `acceleration`: 가속도 (m/s²)
- `exit_speed`: 종료 속도 (0.0 = 완전 정지)
- `has_next`: 다음 세그먼트 존재 여부 (true 시 감속 생략)

### 4.4 TranslateReverse (후진 직선 이동)

```bash
# 1m 후진
ros2 action send_goal /translate_reverse amr_interfaces/action/AMRMotionTranslate \
  "{start_x: 1.0, start_y: 0.0, end_x: 0.0, end_y: 0.0, \
    max_linear_speed: -0.2, acceleration: 0.2, exit_speed: 0.0, has_next: false}"
```

> `max_linear_speed < 0`으로 후진 모드 활성화

### 4.5 YawControl (헤딩 전용 직선 이동)

```bash
# 1.5m 전진 (헤딩 PD 제어만, 횡방향 Stanley 보정 없음)
ros2 action send_goal /amr_motion_yaw_control amr_interfaces/action/AMRMotionYawControl \
  "{start_x: 0.0, start_y: 0.0, end_x: 1.5, end_y: 0.0, \
    max_linear_speed: 0.3, acceleration: 0.2}" \
  --feedback
```

파라미터:
- `start_x/y`, `end_x/y`: 시작/목표점 (m, map 프레임)
- `max_linear_speed`: 최대 선속도 (m/s)
- `acceleration`: 가속도 (m/s²)

## 5. 상태 코드

| 코드 | 의미 |
|------|------|
| 0 | 성공 |
| -1 | 취소됨 |
| -2 | 잘못된 파라미터 |
| -3 | 타임아웃 / 로컬라이제이션 실패 |
| -4 | TF 실패 / 안전 정지 |

## 6. Gazebo DiffDrive 속도 제한

Pioneer 2DX 모델 (model.sdf):
- 최대 선속도: **0.4 m/s**
- 최대 각속도: **1.0 rad/s** (57.3 deg/s)
- 최대 선가속도: 1.5 m/s²
- 최대 각가속도: 1.6 rad/s²

> 이 제한을 초과하는 속도를 요청하면 DiffDrive 플러그인이 내부적으로 클램핑하여 사다리꼴 프로파일 추종 오차가 발생할 수 있음

## 7. 설정 파일

Motion Control 파라미터:
```
install/amr_motion_control_2wd/share/amr_motion_control_2wd/config/motion_params_gazebo.yaml
```

주요 파라미터:
- `wheel_separation: 0.34` — Pioneer 2DX 휠 간격 (SDF 기준)
- `wheel_radius: 0.11` — Pioneer 2DX 휠 반경 (SDF 기준)
- `robot_base_frame: "base_footprint"` — TF 타겟 프레임 (LIO-SAM: `"base_link"`)
- `translate_pose_topic: "/rtabmap/odom"` — 로컬라이제이션 포즈 토픽
- `control_rate_hz: 50.0` — 제어 루프 주기

## 8. SLAM 알고리즘별 설정

| SLAM | robot_base_frame | translate_pose_topic |
|------|-----------------|---------------------|
| RTAB-Map 3D | `base_footprint` (기본값) | `/rtabmap/odom` (기본값) |
| LIO-SAM | `base_link` | `/lio_sam/mapping/odometry` |
| Cartographer 3D | `base_footprint` | TF 전용 (워치독 비활성화 권장) |

런타임 파라미터 변경:
```bash
ros2 param set /amr_motion_control_2wd robot_base_frame "base_link"
ros2 param set /amr_motion_control_2wd translate_pose_topic "/lio_sam/mapping/odometry"
```

## 9. 테스트 결과 (Gazebo, RTAB-Map 3D)

| 액션 | 테스트 | 결과 | 오차 | 시간 |
|------|--------|------|------|------|
| Spin 90° | target=90° | actual=90.27° | 0.27° | 2.6s |
| Spin 180° | target=180° | actual=178.77° | 1.23° | 5.0s |
| Translate 1m | (0,0)→(1,0) | actual=0.959m | 4.1cm, lat=6mm | 4.5s |
| Translate 2m | (0.98,0)→(2.98,0) | actual=1.955m | 4.5cm, lat=18mm | 7.9s |
| Turn 90° R=0.5m | CCW arc | actual=90.08° | 0.08° | 4.6s |
| Turn -45° R=0.3m | CW arc | actual=-45.11° | 0.11° | 1.9s |
| YawControl 1.5m | (3.49,0.74)→(4.99,0.74) | actual=1.455m | 4.5cm, hdg=0.03° | 5.9s |
