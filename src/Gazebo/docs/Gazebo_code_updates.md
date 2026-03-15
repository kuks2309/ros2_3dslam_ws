# Gazebo Code Updates

## 2026-03-13 / 수정 - launch/*.launch.py, rviz2/gazebo.rviz

- launch/gazebo.launch.py, launch/gazebo_no_odom.launch.py: `pkg_src` 경로 수정 (`T-Robot_nav_ros2_ws` → `Study/ros2_3dslam_ws`)
- rviz2/gazebo.rviz: `LidarPointCloud` (`/scan/points`) `Enabled: false` → `true` (3D 포인트 클라우드 기본 표시)

---

## 2026-03-09 / 07:30 - 2e86244 / 수정 - models/pioneer2dx/model.sdf

- front_lidar (gpu_lidar) horizontal samples: 720 → 1800 (0.5° → 0.2° 해상도, VLP-16 스펙)
- front_lidar (gpu_lidar) range max: 10m → 100m (VLP-16 최대 감지 거리)

