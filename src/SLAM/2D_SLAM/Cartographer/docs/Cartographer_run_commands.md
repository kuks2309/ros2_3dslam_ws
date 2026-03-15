# Cartographer 2D SLAM 사용법

패키지명: `cartographer_slam`

## 개요

Google Cartographer는 서브맵(submap) 방식의 실시간 SLAM 라이브러리입니다.
로컬 서브맵을 생성한 후 글로벌 최적화(loop closure)로 정밀한 지도를 생성합니다.
지도는 `.pbstream` 형식으로 저장되며 위치추정 시 재사용합니다.

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select cartographer_slam
source install/setup.bash
```

## 실행 순서

### 1단계: Gazebo 시뮬레이터 기동

> Cartographer 2D는 외부 odom이 필요합니다. `gazebo.launch.py`는 `/odom` 토픽과 `odom → base_link` TF를 제공합니다.
> **`odom_tf:=false`를 사용하지 마세요** — 2D SLAM은 Gazebo의 odom TF가 필수입니다.

```bash
ros2 launch tm_gazebo gazebo.launch.py
```

### 2단계: Cartographer 기동

#### SLAM (매핑)

```bash
ros2 launch cartographer_slam cartographer_slam.launch.py
```

#### Localization (위치추정 - 기존 .pbstream 지도 사용)

```bash
ros2 launch cartographer_slam cartographer_localization.launch.py

# 커스텀 지도 파일 지정
ros2 launch cartographer_slam cartographer_localization.launch.py \
  map_file:=/path/to/map.pbstream
```

## 실행 옵션

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 |
| `scan_topic` | `/scan` | 레이저 스캔 토픽 |
| `map_file` | `maps/cartographer/map.pbstream` | pbstream 지도 경로 (localization 전용) |

## 지도 저장 (매핑 완료 후)

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$HOME/Study/ros2_3dslam_ws/maps/cartographer/map.pbstream'}"
```

## 기동되는 노드

| 노드 | 설명 |
|------|------|
| `cartographer_node` | SLAM 메인 노드 (서브맵 생성 + 루프 클로저) |
| `cartographer_occupancy_grid_node` | 점유 격자 지도 생성 (해상도 0.05m) |
| `rviz2` | `rviz2/cartographer.rviz` 시각화 |

## 설정 파일

| 파일 | 용도 |
|------|------|
| `config/cartographer.lua` | SLAM 매핑 파라미터 |
| `config/cartographer_localization.lua` | Pure localization 파라미터 |

## 필수 토픽

| 토픽 | 타입 |
|------|------|
| `/scan` | `sensor_msgs/LaserScan` |
| `/odom` | `nav_msgs/Odometry` |
