# Odometry 시각화 & rqt_robot_steering 종료 수정

**날짜**: 2026-04-19
**범위**: `src/Gazebo/` + `src/ACS/Waypoint-System/acs_waypoint_gui/`
**관련 이슈**: RViz 화면에서 odometry pose가 먼 위치에 부채꼴로 표시 · 런치 종료 시 rqt 창 잔존

---

## 1. 문제 현상

### 1.1 RViz Odometry display 이상 위치
- 로봇은 화면 중앙(0, 0)에 표시되는데 빨간 pose 화살표 + 노랑 커버리언스 cone 이 **화면 구석** 에 100개 정도 **부채꼴** 로 누적 표시
- 웨이포인트 마커와 로봇 위치가 서로 다른 기준처럼 보임

### 1.2 rqt_robot_steering 잔존
- ACS GUI 또는 launch 프로세스 종료 시 `rqt_robot_steering` 창과 프로세스가 계속 살아있음

---

## 2. 원인 분석

### 2.1 Odometry 좌표계 불일치

Gazebo `OdometryPublisher` 플러그인의 동작:
```xml
<plugin ... name="ignition::gazebo::systems::OdometryPublisher">
    <odom_topic>/odom</odom_topic>
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
</plugin>
```

→ 플러그인은 로봇의 **월드 좌표** 를 `/odom.pose.position` 으로 그대로 publish.
예: 스폰 위치 `(-13.10, 2.64)` → `/odom.pose` 도 `(-13.10, 2.64)`

ROS 관례: `odom` 프레임은 로봇 시작 pose 를 원점으로 해야 함 → 로봇이 (0, 0) 에서 출발해야 정상.

기존 `odom_to_tf.py` 는 **TF 에서만** 초기 위치를 빼서 broadcast (즉, `odom → base_footprint` TF 는 (0,0) 으로 보정) 하지만, **`/odom` 토픽 자체는 보정하지 않음**. 결과:

| 정보원 | 로봇 위치 (odom frame) |
|---|---|
| `/odom` 토픽 | **(-13.10, 2.64)** (미보정) |
| `odom → base_footprint` TF | **(0, 0)** (보정됨) |

RViz 에서 동시 표시 시:
- **RobotModel** (TF 기반) → 로봇 (0,0) 에 그려짐
- **Odometry display** (토픽 구독) → 빨간 화살표 (-13.10, 2.64) 에 그려짐

→ 13m 이상 떨어진 위치에 각각 표시됨.

추가로 `Keep: 100` + sim_time jump → 100개 pose 가 누적 → 부채꼴 모양 생성.

### 2.2 rqt_robot_steering orphan

기존 `gazebo.launch.py` 의 `Node(package='rqt_robot_steering', ...)` 로 기동:
- Qt 이벤트 루프가 `SIGINT` 를 block → ROS2 launch 의 shutdown cascade 시 응답 안 함
- 심지어 `ros2 run` 래퍼를 SIGTERM 으로 죽여도, 그 **자식 rqt 프로세스는 orphan** 으로 생존

---

## 3. 수정 내용

### 3.1 `src/Gazebo/scripts/odom_to_tf.py`
**`/odom_corrected` 퍼블리셔 추가** — 초기 위치 기준 상대 좌표로 재발행. TF 와 토픽이 **일관된** 좌표계를 갖게 됨.

```python
self.pub_corrected = self.create_publisher(Odometry, '/odom_corrected', 10)
...
corrected.pose.pose.position.x = msg.pose.pose.position.x - self.initial_x
# (동일 방식으로 y, z, 회전/공분산/twist 복사)
self.pub_corrected.publish(corrected)
```

추가로 `rclpy.ok()` 가드로 `shutdown already called` 트레이스백 제거.

### 3.2 `src/Gazebo/rviz2/gazebo.rviz`
Odometry display:
- `Keep: 100` → **`Keep: 1`** (과거 pose 누적 방지)
- `Topic.Value: /odom` → **`/odom_corrected`** (보정된 topic 구독)

### 3.3 `src/Gazebo/launch/gazebo.launch.py`
**rqt_robot_steering 안정 종료 보장**:

1. `Node(...)` → **`ExecuteProcess(cmd=['ros2','run','rqt_robot_steering','rqt_robot_steering'], sigterm_timeout='2', sigkill_timeout='3')`**
2. **OnShutdown 안전망** 추가:
   ```python
   def _kill_orphan_rqt(context):
       subprocess.run(['pkill', '-KILL', '-f', 'rqt_robot_steering'],
                      check=False, timeout=3)
       return None

   RegisterEventHandler(
       OnShutdown(on_shutdown=[OpaqueFunction(function=_kill_orphan_rqt)])
   )
   ```

→ launch 종료 시점에 orphan rqt 프로세스를 `pkill -KILL` 로 확실히 정리.

### 3.4 추가 — NVIDIA PRIME offload (Optimus 노트북)
`src/Gazebo/launch/gazebo.launch.py` 가 기동 시 `/proc/driver/nvidia/version` 및 `/usr/lib/.../libEGL_nvidia.so.0` 존재 여부로 **자동 감지**하여 `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json` 을 설정. NVIDIA 드라이버가 없는 환경에서는 자동 skip.

---

## 4. 검증 방법

### 4.1 실행
```bash
cd ~/Study/ros2_3dslam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch acs_waypoint_gui acs_gazebo_full.launch.py
```

### 4.2 확인 포인트

| 항목 | 확인 명령 | 기대 결과 |
|---|---|---|
| odom 보정 토픽 존재 | `ros2 topic info /odom_corrected` | Type: `nav_msgs/msg/Odometry`, Publisher=1 |
| 로봇이 odom 원점 | `ros2 topic echo /odom_corrected --once` | `position.x, y ≈ 0.0` |
| RViz 부채꼴 소멸 | RViz 화면 확인 | 빨간 pose 잔상 없음 |
| rqt 창 기동 | `pgrep -fa rqt_robot_steering` | 2개 (ros2 run 래퍼 + 자식) |
| 종료 시 정리 | `Ctrl+C` 후 `pgrep -fa rqt_robot_steering` | 결과 없음 |
| NVIDIA 활용 | `nvidia-smi dmon -s u -c 5` | `sm` 0% 이상 (노트북), `libEGL warning` 0개 |

---

## 5. 영향 범위 / 호환성

- `amr_motion_control_2wd` 는 여전히 `/odom` 을 subscribe 함 → 제어 로직은 기존 동작 유지 (raw world 좌표 사용)
- RViz 시각화만 `/odom_corrected` 로 전환
- `/odom_corrected` 는 신규 토픽이라 외부 노드 영향 없음
- NVIDIA env var 자동 감지 → NVIDIA 없는 시스템에서 무효 (배포 안전)

---

## 6. 알려진 잔존 이슈

- `acs_gui_node` 가 SIGTERM 에 10초 반응 없이 SIGKILL 로 escalate 되는 경우 있음
  - 원인: PyQt5 이벤트 루프 + `rclpy.spin_once()` timer race
  - 영향 없음 (launch 가 SIGKILL 로 자동 처리)
  - 추가 개선은 별도 작업
