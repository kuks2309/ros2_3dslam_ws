# 2026-04-26 — Nav2 GPU 풀스택 가동 + Wheel TF 수정

## 1. 작업 요약

순수 Nav2(`nav2_smac_hybrid`) standalone 풀스택을 GPU 가속 환경에서 가동하고, 가동 중 발견된 wheel TF 누락 문제를 SDF + bridge 기반으로 정식 수정.

**적용 범위:**
- mapper / waypoint_manager 연동 없음 (사용자 지시: 독립 트랙 유지)
- Planner: `SmacPlannerHybrid` (Dubin)
- Local Controller: `RegulatedPurePursuitController` (RPP)
- SLAM 백엔드: RTAB-Map 3D Lidar localization

---

## 2. 풀스택 가동 절차 (검증된 시퀀스)

### 사전 조건
- NVIDIA 드라이버 설치 (검증 환경: RTX 3080 Laptop, 565.57.01)
- Gazebo Fortress (Ignition)
- 313 MB 이상의 정상 매핑된 RTAB-Map DB

### 가동 순서

```bash
# T1: Gazebo (GPU + odom TF 비활성)
ros2 launch tm_gazebo gazebo.launch.py odom_tf:=false

# T2: RTAB-Map 3D 로컬라이제이션 (정상 DB 사용)
ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_localization_gazebo.launch.py \
  database_path:=$HOME/Study/ros2_3dslam_ws/maps/rtabmap_3d/rtabmap3d_3dlidar_20260419_145054.db

# T2-2: 초기 위치 publish (자동 로컬라이제이션 트리거)
ros2 topic pub --once /rtabmap/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.06,0,0, 0,0,0,0,0.06,0, 0,0,0,0,0,0.06]}}"

# T3: Nav2 풀스택
ros2 launch nav2_smac_hybrid smac_hybrid_planner.launch.py

# T4(선택): Nav2 전용 RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_smac_hybrid)/share/nav2_smac_hybrid/rviz/nav2_smac_hybrid.rviz
```

### 검증 지표
- RTAB-Map: `local map=232, WM=232` 로그 (DB 노드 수 일치)
- Nav2: `lifecycle_manager_navigation: Managed nodes are active`
- GPU: `nvidia-smi`에서 메모리 ~1.2 GB, 사용률 70% 이상

---

## 3. 발생한 문제와 해결

### 3.1 패키지명 오인 (해결)

| 잘못 쓴 이름 | 실제 패키지명 |
|--------------|---------------|
| `Gazebo` | `tm_gazebo` |
| `rtab_map_3d` | `rtab_map_3d_config` |

`ros2 pkg prefix <name>`로 확인 가능.

### 3.2 RTAB-Map DB 빈 파일 (해결)

가장 최근 파일이라 골랐던 `rtabmap3d_3dlidar_20260419_151228.db` (4.2 MB)는 `WM=1`로 매핑 거의 없는 빈 DB. **313 MB DB(`...145054.db`)로 교체 후 정상**(WM=232).

DB 정상 여부 1차 판단: 파일 크기. 정상 매핑된 DB는 보통 100 MB 이상.

**삭제한 작은 DB (사용자 지시):**
- `rtabmap_3dlidar_rgbd.db` (68 MB)
- `rtabmap_3dlidar_only.db` (9.3 MB) — `nav2_full_bringup.launch.py` 기본값으로 참조됨, 삭제 후 launch 사용 시 `database_path:=` 명시 필요
- `rtabmap3d_3dlidar_20260419_151228.db` (4.2 MB)

### 3.3 TF 트리 분리 — Gazebo `odom→base_footprint` ↔ RTAB-Map `odom_rtabmap→base_footprint` 충돌 (해결)

**현상:** Nav2 global_costmap이 `Could not find a connection between 'map' and 'base_link' because they are not part of the same tree`

**원인:** base_footprint가 두 부모(`odom`, `odom_rtabmap`) 가지면서 TF2 트리 분리

**해결:** `gazebo.launch.py odom_tf:=false`로 Gazebo의 odom→base_footprint TF 비활성화. RTAB-Map이 단일 odom 소스 담당.

**최종 TF 체인:**
```
map → odom_rtabmap → base_footprint → base_link → {lidar_link, camera_link, left_wheel, right_wheel}
```

### 3.4 GPU 미활성 (해결)

`gazebo_no_odom.launch.py`는 NVIDIA 환경변수 미설정. `gazebo.launch.py`에는 자동 GPU 감지 로직(`_detect_gpu_mode()` + `_setup_gpu_env()`)이 있어 PRIME offload 자동 적용.

→ `gazebo.launch.py odom_tf:=false`로 GPU + TF 충돌 둘 다 해결.

### 3.5 RViz "2D Goal Pose" 도구 누락 (해결)

RTAB-Map의 `rtabmap_localization.rviz`는 로컬라이제이션 검증 전용으로 만들어져 Tools에 `MoveCamera, Select`만 있음. Nav2용 도구 없음.

**해결:** Nav2 전용 RViz config 신규 작성:
- 파일: [src/Planner/Nav2/nav2_smac_hybrid/rviz/nav2_smac_hybrid.rviz](../../src/Planner/Nav2/nav2_smac_hybrid/rviz/nav2_smac_hybrid.rviz)
- 베이스: `nav2_bringup`의 `nav2_default_view.rviz`
- 수정: Map 토픽 `/map` → `/rtabmap/map`
- 포함: Costmap, Path, Footprint, RobotModel, 2D Pose Estimate, 2D Goal Pose, Nav2 Goal/Waypoint/Cancel 도구

### 3.6 Wheel link TF 누락 — RobotModel 에러 (해결, 정식 수정)

**현상:** RViz RobotModel에서 `left_wheel`, `right_wheel`만 "No transform" 에러. 다른 link(base_footprint, base_link, lidar_link, camera_link 등)는 정상.

**원인 분석:**
- URDF([pioneer2dx.urdf](../../src/Gazebo/urdf/pioneer2dx.urdf))에 `left_wheel_joint`, `right_wheel_joint` (`continuous`) 정의
- `robot_state_publisher`가 dynamic joint TF를 발행하려면 `/joint_states` 구독 필요
- `/joint_states` publisher 0개 → wheel TF 발행 불가

**기술 부채 회피를 위한 정식 수정 (옵션 C):**
1. SDF 측: `pioneer2dx/model.sdf`에 `ignition-gazebo-joint-state-publisher-system` 플러그인이 **이미 존재** (Gazebo가 `/world/my_world/model/robot_scan/joint_state` 발행)
2. Bridge 측: `ros_gz_bridge`에 해당 토픽을 추가하고 `/joint_states`로 리맵
3. URDF + robot_state_publisher가 자동으로 wheel link TF 생성 → Gazebo 실제 회전 반영

**적용 변경 ([gazebo.launch.py](../../src/Gazebo/launch/gazebo.launch.py)):**
```python
# Bridge args에 추가
'/world/my_world/model/robot_scan/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
# Node remappings 추가
remappings=[
    ('/world/my_world/model/robot_scan/joint_state', '/joint_states'),
],
```

**부수 정리: 중복 static TF 4개 노드 제거**
- 기존 launch가 `static_transform_publisher`로 발행하던 frame들(base_link, lidar_link, camera_link, camera_color_optical_frame)은 URDF + robot_state_publisher가 자동 발행 → 중복 제거로 `TF_REPEATED_DATA` 위험 차단

**검증 (실측):**
- `/joint_states` 1243 Hz ✓
- `/tf_static` 4 frames (URDF fixed joints) ✓
- `/tf`에 `left_wheel`, `right_wheel` 발행 확인 ✓

---

## 4. 핵심 결정 (ADR)

### ADR-1: Nav2는 mapper/waypoint_manager와 독립 트랙 유지
- **결정:** Nav2 보완계획·아키텍처에 mapper, ACS waypoint_manager 연동 설계 포함하지 않음
- **근거:** 사용자 지시 (2026-04-25)
- **영향:** worker5_nav2_planner.md의 옵션 A/B/C 통합안 모두 채택 안 함. Nav2는 자체 BT navigator로 standalone 운영

### ADR-2: 모델 인스턴스명은 SDF 정의명과 다를 수 있음
- **사실:** `pioneer2dx/model.sdf`의 `<model name="pioneer2dx">` 정의이지만 world에서 인스턴스명 `robot_scan`으로 spawn
- **확인 방법:** `ign topic -l | grep model`
- **시사점:** ros_gz_bridge 토픽 경로 작성 시 SDF 디렉토리/정의명이 아닌 **실제 world 인스턴스명** 사용 필수

### ADR-3: 로컬 컨트롤러는 RPP 유지
- **결정:** `RegulatedPurePursuitController` 유지 (MPPI 미도입)
- **근거:** Smac Hybrid (Dubin) + 차동구동 매칭 우수, CPU 부담 낮음, 순수 Nav2 standalone 단순성
- **재검토 시점:** 동적 장애물 회피가 핵심 요구사항이 될 때 MPPI 검토

---

## 5. 신규/수정 파일

| 변경 | 파일 | 내용 |
|------|------|------|
| MODIFY | `src/Gazebo/launch/gazebo.launch.py` | bridge에 joint_state 추가 + remap, 중복 static_tf 4개 제거 |
| CREATE | `src/Planner/Nav2/nav2_smac_hybrid/rviz/nav2_smac_hybrid.rviz` | Nav2 전용 RViz config (Costmap, Path, Footprint, Goal 도구 등) |
| DELETE | `maps/rtabmap_3d/rtabmap_3dlidar_rgbd.db` (68 MB) | 사용자 지시 |
| DELETE | `maps/rtabmap_3d/rtabmap_3dlidar_only.db` (9.3 MB) | 사용자 지시 (nav2_full_bringup.launch.py 기본값 깨짐 — 사용 시 database_path 명시 필요) |
| DELETE | `maps/rtabmap_3d/rtabmap3d_3dlidar_20260419_151228.db` (4.2 MB) | 빈 DB |

---

## 6. 후속 검토 항목 (이 세션에서 미해결)

- `odom_tf:=false`인데도 `/tf`에 `odom→base_footprint` 발행되는 출처 (Gazebo의 ign-gazebo-odometry-publisher-system 플러그인 가능성, SDF 확인 필요)
- `nav2_full_bringup.launch.py`의 `database_path` 기본값을 313 MB DB로 갱신 필요 (현재 삭제된 `rtabmap_3dlidar_only.db` 가리킴)
- `localization_params.yaml` ↔ `smac_hybrid_params.yaml`의 `robot_base_frame`/`robot_radius` 일관성 검토 (이전 보고서 §3 참조)
