# Gazebo 실행 명령어

패키지명: `tm_gazebo`

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select tm_gazebo
source install/setup.bash
```

## 실행

### 기본 실행 (odom TF 포함)

Gazebo + ROS-Gazebo Bridge + odom→base_link TF + Static TF + RViz2 + rqt_robot_steering 전체 기동

```bash
ros2 launch tm_gazebo gazebo.launch.py
```

### odom 없이 실행

`/odom` 브리지를 제거한 버전 (외부 odometry 소스 사용 시)

```bash
ros2 launch tm_gazebo gazebo_no_odom.launch.py
```

## 실행 옵션

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `world` | `worlds/my_world.sdf` | 로드할 world 파일 경로 |
| `odom_tf` | `true` | odom → base_link TF 퍼블리시 활성화 여부 |

```bash
# odom TF 비활성화
ros2 launch tm_gazebo gazebo.launch.py odom_tf:=false

# 커스텀 world 파일 지정
ros2 launch tm_gazebo gazebo.launch.py world:=/path/to/your.sdf
```

## 기동되는 노드/프로세스

| 구성요소 | 설명 |
|----------|------|
| `ign gazebo` | Ignition Gazebo (Fortress) 시뮬레이터 |
| `ros_gz_bridge` | ROS2 ↔ Gazebo 토픽 브리지 |
| `odom_to_tf.py` | `/odom` → `odom→base_link` TF 변환 (`odom_tf:=true` 시) |
| `static_transform_publisher` (lidar) | `base_link → lidar_link` (z=0.09m) |
| `static_transform_publisher` (camera) | `base_link → camera_link` (x=0.25m, z=0.10m) |
| `static_transform_publisher` (optical) | `camera_link → camera_color_optical_frame` |
| `rviz2` | `rviz2/gazebo.rviz` 설정으로 시각화 |
| `rqt_robot_steering` | 수동 cmd_vel 제어 GUI |

## 브리지 토픽

| 토픽 | 타입 | 방향 |
|------|------|------|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS2 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS2 → Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS2 (`gazebo.launch.py`만) |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS2 |
| `/scan/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS2 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS2 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS2 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS2 |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS2 |

## 주의사항

launch 파일 내 `pkg_src` 경로가 하드코딩되어 있음. 워크스페이스 경로가 다를 경우 수정 필요.

```python
# launch/gazebo.launch.py, launch/gazebo_no_odom.launch.py 11번째 줄
pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'Gazebo')
```
