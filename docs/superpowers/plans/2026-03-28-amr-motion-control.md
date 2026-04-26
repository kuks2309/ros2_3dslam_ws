# AMR Motion Control for Gazebo Simulation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the T-AMR AMR-Motion-Control system (2WD differential drive) into this ROS2 3D SLAM workspace, adapted for Gazebo Ignition simulation with Pioneer 2DX robot and RTAB-Map/LIO-SAM localization.

**Architecture:** 6 new ROS2 packages under `src/Control/AMR-Motion-Control/`: custom interfaces (`amr_interfaces`), motor-level messages (`tc_msgs`), differential drive kinematics (`kinematics_2wd`), motion control action servers (`amr_motion_control_2wd`), Python test UI (`amr_motion_test_ui`), and Gazebo-specific launch integration. The kinematics node converts `cmd_vel` to Gazebo velocity commands and produces odometry from joint states. Five action servers (Spin, Turn, Translate, TranslateReverse, YawControl) provide precision motion primitives using TF2 + IMU feedback with trapezoidal velocity profiles.

**Tech Stack:** ROS2 Humble, C++17 (ament_cmake), Python 3.10 (ament_python), Ignition Gazebo Fortress, RTAB-Map, TF2, rclcpp_action

---

## Reference Architecture

```
src/Control/
├── AMR-Motion-Control/
│   ├── amr_interfaces/              # Custom action/service definitions
│   │   ├── action/
│   │   │   ├── AMRMotionSpin.action
│   │   │   ├── AMRMotionTurn.action
│   │   │   ├── AMRMotionTranslate.action
│   │   │   ├── AMRMotionYawControl.action
│   │   │   └── AMRMotionPurePursuit.action
│   │   ├── srv/
│   │   │   ├── AMRControlStop.srv
│   │   │   └── UpdateTranslateEndpoint.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── tc_msgs/                     # Motor-level message definitions
│   │   ├── msg/
│   │   │   ├── SafetyStatus.msg
│   │   │   ├── WheelMotor.msg
│   │   │   ├── WheelMotorState.msg
│   │   │   └── Odometry.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── kinematics_2wd/              # Diff drive kinematics (C++)
│   │   ├── include/kinematics_2wd/
│   │   │   ├── diff_drive_kinematics.hpp
│   │   │   └── kinematics_node.hpp
│   │   ├── src/
│   │   │   ├── diff_drive_kinematics.cpp
│   │   │   ├── kinematics_node.cpp
│   │   │   └── kinematics_main.cpp
│   │   ├── config/
│   │   │   ├── kinematics.yaml            # Physical robot params
│   │   │   └── kinematics_gazebo.yaml     # Pioneer 2DX Gazebo params
│   │   ├── launch/
│   │   │   ├── kinematics_launch.py
│   │   │   └── kinematics_gazebo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── amr_motion_control_2wd/      # Motion control action servers (C++)
│   │   ├── include/amr_motion_control_2wd/
│   │   │   ├── motion_common.hpp
│   │   │   ├── motion_profile.hpp
│   │   │   ├── recursive_moving_average.hpp
│   │   │   ├── localization_watchdog.hpp
│   │   │   ├── spin_action_server.hpp
│   │   │   ├── turn_action_server.hpp
│   │   │   ├── translate_action_server.hpp
│   │   │   ├── translate_reverse_action_server.hpp
│   │   │   └── yaw_control_action_server.hpp
│   │   ├── src/
│   │   │   ├── main.cpp
│   │   │   ├── motion_common.cpp
│   │   │   ├── motion_profile.cpp
│   │   │   ├── localization_watchdog.cpp
│   │   │   ├── spin_action_server.cpp
│   │   │   ├── turn_action_server.cpp
│   │   │   ├── translate_action_server.cpp
│   │   │   ├── translate_reverse_action_server.cpp
│   │   │   └── yaw_control_action_server.cpp
│   │   ├── config/
│   │   │   ├── motion_params.yaml
│   │   │   └── motion_params_gazebo.yaml
│   │   ├── launch/
│   │   │   ├── motion_control.launch.py
│   │   │   └── motion_control_gazebo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── amr_motion_test_ui/          # Test UI (Python)
│       ├── amr_motion_test_ui/
│       │   ├── __init__.py
│       │   └── test_ui_node.py
│       ├── config/
│       │   └── test_ui_config.yaml
│       ├── launch/
│       │   └── test_ui.launch.py
│       ├── resource/amr_motion_test_ui
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
```

## Gazebo Integration Context

**Current Gazebo Setup (gazebo_no_odom.launch.py):**
- Robot: Pioneer 2DX (2WD differential drive)
- Bridge topics: `/cmd_vel` (ROS2→Gazebo), `/scan`, `/camera/*`, `/imu/data` (Gazebo→ROS2)
- No `/odom` bridge — uses `odom_to_tf.py` script
- TF: `odom → base_footprint → base_link → lidar_link, camera_link`
- Static TF: base_footprint→base_link (z=0.16)

**Pioneer 2DX Parameters (estimated from SDF):**
- `wheel_radius`: ~0.08 m
- `wheel_separation`: ~0.33 m
- `gear_ratio`: 1.0 (direct drive in simulation)

**Key Adaptations from T-AMR:**
1. Kinematics output: `sim_velocity` mode → publishes `Float64MultiArray` to `velocity_controller/commands`
2. Kinematics input: `joint_state` mode → reads from Gazebo `drives/joint_states`
3. Add Gazebo velocity controller bridge to launch file
4. All nodes: `use_sim_time: true`
5. Localization: RTAB-Map `/rtabmap/localization_pose` topic
6. Pioneer 2DX wheel parameters instead of T-AMR parameters

---

## Task 1: Create `amr_interfaces` Package — Action & Service Definitions

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/package.xml`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/CMakeLists.txt`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionSpin.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionTurn.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionTranslate.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionYawControl.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/action/AMRMotionPurePursuit.action`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/srv/AMRControlStop.srv`
- Create: `src/Control/AMR-Motion-Control/amr_interfaces/srv/UpdateTranslateEndpoint.srv`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/amr_interfaces/{action,srv}
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_interfaces</name>
  <version>0.1.0</version>
  <description>Custom action and service interfaces for AMR motion control</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AMRMotionSpin.action"
  "action/AMRMotionTurn.action"
  "action/AMRMotionTranslate.action"
  "action/AMRMotionYawControl.action"
  "action/AMRMotionPurePursuit.action"
  "srv/AMRControlStop.srv"
  "srv/UpdateTranslateEndpoint.srv"
)

ament_package()
```

- [ ] **Step 4: Create all action definitions**

**AMRMotionSpin.action:**
```
# Goal
float64 target_angle          # deg (absolute, map frame; shortest path auto-selected)
float64 max_angular_speed     # deg/s (always > 0)
float64 angular_acceleration  # deg/s² (always > 0)
---
# Result
int8    status                # 0=success, -1=cancelled, -2=invalid_param, -3=timeout, -4=tf_fail
float64 actual_angle          # deg (absolute, map frame, final yaw)
float64 elapsed_time          # sec
---
# Feedback
float64 current_angle         # deg (absolute, map frame, current estimated yaw)
float64 current_speed         # deg/s (current angular speed)
uint8   phase                 # 1=accel, 2=cruise, 3=decel
```

**AMRMotionTurn.action:**
```
# Goal
float32 target_angle          # deg (+ CCW, - CW)
float32 turn_radius           # m (> 0, robot center to ICR)
float32 max_linear_speed      # m/s (always > 0)
float32 accel_angle           # deg (accel/decel angular distance, > 0)
---
# Result
int8    status                # 0=success, -1=cancelled, -2=invalid_param, -3=timeout
float32 actual_angle          # deg (achieved rotation)
float32 elapsed_time          # sec
---
# Feedback
float32 current_angle         # deg (accumulated so far)
float32 current_linear_speed  # m/s
float32 current_angular_speed # deg/s
float32 remaining_angle       # deg
uint8   phase                 # 1=accel, 2=cruise, 3=decel
float32 w1_drive_rpm          # left wheel RPM
float32 w2_drive_rpm          # right wheel RPM
```

**AMRMotionTranslate.action:**
```
# Goal
float64 start_x              # map frame start X (m)
float64 start_y              # map frame start Y (m)
float64 end_x                # map frame end X (m)
float64 end_y                # map frame end Y (m)
float64 max_linear_speed     # m/s (> 0 forward / < 0 reverse)
float64 acceleration         # m/s^2 (> 0)
float64 exit_speed           # m/s (0=full stop, >0=exit at this speed)
bool    has_next             # true=next segment exists, skip decel
---
# Result
int8    status               # 0=success, -1=cancelled, -2=invalid_param, -3=timeout, -4=safety_stop
float64 actual_distance      # m
float64 final_lateral_error  # m
float64 final_heading_error  # deg
float64 elapsed_time         # sec
---
# Feedback
float64 current_distance     # m (projection along path)
float64 current_lateral_error # m (cross-track error)
float64 current_heading_error # deg
float64 current_vx           # m/s
float64 current_vy           # m/s
float64 current_omega        # rad/s
uint8   phase                # 1=accel, 2=cruise, 3=decel
float64 w1_drive_rpm         # left wheel RPM
float64 w2_drive_rpm         # right wheel RPM
```

**AMRMotionYawControl.action:**
```
# Goal
float64 start_x              # map frame start X (m)
float64 start_y              # map frame start Y (m)
float64 end_x                # map frame end X (m)
float64 end_y                # map frame end Y (m)
float64 max_linear_speed     # m/s (> 0)
float64 acceleration         # m/s^2 (> 0)
---
# Result
int8    status               # 0=success, -1=cancelled, -2=invalid_param, -3=timeout, -4=safety_stop
float64 actual_distance      # m
float64 final_lateral_error  # m
float64 final_heading_error  # deg
float64 elapsed_time         # sec
---
# Feedback
float64 current_distance     # m
float64 current_lateral_error # m
float64 current_heading_error # deg
float64 current_vx           # m/s
float64 current_vy           # m/s
float64 current_omega        # rad/s
uint8   phase                # 1=accel, 2=cruise, 3=decel
float64 w1_drive_rpm         # left wheel RPM
float64 w2_drive_rpm         # right wheel RPM
```

**AMRMotionPurePursuit.action:**
```
# Goal
float64 max_linear_speed         # m/s
float64 acceleration             # m/s²
float64 lookahead_distance       # m
float64 goal_tolerance           # m
bool    align_final_heading      # align to last waypoint yaw
---
# Result
int8    status                   # 0=success, -1=cancelled, -2=invalid, -3=timeout, -4=safety
float64 actual_distance
float64 final_cross_track_error
float64 elapsed_time
---
# Feedback
float64 current_distance
float64 remaining_distance
float64 cross_track_error
float64 heading_error
float64 current_speed
uint32  current_waypoint_index
uint32  total_waypoints
uint8   phase                    # 0=wait_path, 2=tracking, 3=final_align
```

- [ ] **Step 5: Create service definitions**

**AMRControlStop.srv:**
```
uint8 SAFETY_STOP = 1
uint8 ESTOP = 2

uint8 stop_type
---
bool success
string message
```

**UpdateTranslateEndpoint.srv:**
```
float64 end_x
float64 end_y
bool    has_next
---
bool    success
string  message
```

- [ ] **Step 6: Build and verify**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --symlink-install --packages-select amr_interfaces
source install/setup.bash
ros2 interface list | grep amr_interfaces
```

Expected: 5 action types and 2 service types listed.

- [ ] **Step 7: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_interfaces/
git commit -m "feat(amr_interfaces): add custom action/service definitions for AMR motion control

Port AMRMotionSpin, AMRMotionTurn, AMRMotionTranslate,
AMRMotionYawControl, AMRMotionPurePursuit actions and
AMRControlStop, UpdateTranslateEndpoint services from T-AMR.

Simplified: removed hold_steer/exit_steer_angle fields
(not applicable to 2WD differential drive)."
```

---

## Task 2: Create `tc_msgs` Package — Motor-Level Messages

**Files:**
- Create: `src/Control/AMR-Motion-Control/tc_msgs/package.xml`
- Create: `src/Control/AMR-Motion-Control/tc_msgs/CMakeLists.txt`
- Create: `src/Control/AMR-Motion-Control/tc_msgs/msg/SafetyStatus.msg`
- Create: `src/Control/AMR-Motion-Control/tc_msgs/msg/WheelMotor.msg`
- Create: `src/Control/AMR-Motion-Control/tc_msgs/msg/WheelMotorState.msg`
- Create: `src/Control/AMR-Motion-Control/tc_msgs/msg/Odometry.msg`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/tc_msgs/msg
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tc_msgs</name>
  <version>0.1.0</version>
  <description>Motor-level messages for AMR control (SafetyStatus, WheelMotor, Odometry)</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(tc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  "msg/WheelMotor.msg"
  "msg/WheelMotorState.msg"
  "msg/Odometry.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

- [ ] **Step 4: Create message definitions**

**SafetyStatus.msg:**
```
uint8 STATUS_NORMAL=0
uint8 STATUS_WARNING=1
uint8 STATUS_DANGEROUS=2

uint8 status
uint8 safety_source    # 0=lidar, 1=bumper, 2=estop
```

**WheelMotor.msg:**
```
float64 velocity_left    # m/s
float64 velocity_right   # m/s
```

**WheelMotorState.msg:**
```
std_msgs/Header header
int32 encoder_left
int32 encoder_right
float64 velocity_left    # m/s
float64 velocity_right   # m/s
bool motor_ready
uint16 error_code
```

**Odometry.msg:**
```
std_msgs/Header header
float64 vx     # m/s
float64 vy     # m/s (always 0 for diff drive)
float64 vw     # rad/s
float64 x      # m
float64 y      # m
float64 theta  # rad
```

- [ ] **Step 5: Build and verify**

```bash
colcon build --symlink-install --packages-select tc_msgs
source install/setup.bash
ros2 interface show tc_msgs/msg/SafetyStatus
```

- [ ] **Step 6: Commit**

```bash
git add src/Control/AMR-Motion-Control/tc_msgs/
git commit -m "feat(tc_msgs): add motor-level message definitions

SafetyStatus, WheelMotor, WheelMotorState, Odometry messages
ported from T-AMR for compatibility with motion control system."
```

---

## Task 3: Create `kinematics_2wd` Package — Differential Drive Kinematics

This is a C++ package providing inverse/forward kinematics, odometry computation, and TF broadcasting for the 2WD differential drive robot.

**Files:**
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/package.xml`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/CMakeLists.txt`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/include/kinematics_2wd/diff_drive_kinematics.hpp`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/include/kinematics_2wd/kinematics_node.hpp`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/src/diff_drive_kinematics.cpp`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/src/kinematics_node.cpp`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/src/kinematics_main.cpp`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/config/kinematics_gazebo.yaml`
- Create: `src/Control/AMR-Motion-Control/kinematics_2wd/launch/kinematics_gazebo.launch.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/kinematics_2wd/{include/kinematics_2wd,src,config,launch}
```

- [ ] **Step 2: Create package.xml and CMakeLists.txt**

Copy directly from the T-AMR reference. The package.xml depends on: `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `trajectory_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tc_msgs`.

CMakeLists.txt builds:
- `kinematics_2wd_node` executable from all 3 source files
- Links against all dependencies
- Installs `include/`, `config/`, `launch/`

- [ ] **Step 3: Port header files from T-AMR**

Port `diff_drive_kinematics.hpp` (DiffDriveParams struct, DiffDriveKinematics class with IK/FK/ICR methods) and `kinematics_node.hpp` (KinematicsNode class with OutputMode::SIM_VELOCITY support) exactly from T-AMR source.

- [ ] **Step 4: Port source files from T-AMR**

Port all 3 source files:
- `diff_drive_kinematics.cpp` — IK/FK math, odometry integration, velocity clamping
- `kinematics_node.cpp` — ROS2 node with cmd_vel→wheel commands (IK) and joint_state→odom (FK)
- `kinematics_main.cpp` — main() entry point

No modifications needed — the code already supports `sim_velocity` output mode for Gazebo.

- [ ] **Step 5: Create Gazebo-specific config**

**config/kinematics_gazebo.yaml:**
```yaml
kinematics_2wd_node:
  ros__parameters:
    # Pioneer 2DX parameters for Gazebo
    wheel_radius: 0.08
    wheel_separation: 0.33
    gear_ratio: 1.0
    max_linear_vel: 1.0
    max_angular_vel: 3.14
    max_motor_rpm: 300.0

    # Gazebo simulation modes
    output_mode: "sim_velocity"    # Float64MultiArray to velocity_controller/commands
    input_mode: "joint_state"      # Read from drives/joint_states

    # TF and frames
    send_transform: true
    odom_frame: "odom"
    base_frame: "base_footprint"
    cmd_vel_timeout: 0.5
```

- [ ] **Step 6: Create Gazebo launch file**

**launch/kinematics_gazebo.launch.py:**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kinematics_2wd')
    config_file = os.path.join(pkg_share, 'config', 'kinematics_gazebo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),

        Node(
            package='kinematics_2wd',
            executable='kinematics_2wd_node',
            name='kinematics_2wd_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
```

- [ ] **Step 7: Build and verify**

```bash
colcon build --symlink-install --packages-up-to kinematics_2wd
source install/setup.bash
ros2 run kinematics_2wd kinematics_2wd_node --ros-args --params-file \
  install/kinematics_2wd/share/kinematics_2wd/config/kinematics_gazebo.yaml
```

Verify node starts without errors and logs "KinematicsNode started".

- [ ] **Step 8: Commit**

```bash
git add src/Control/AMR-Motion-Control/kinematics_2wd/
git commit -m "feat(kinematics_2wd): add 2WD differential drive kinematics package

Ported from T-AMR with Gazebo simulation support:
- IK: cmd_vel → sim_velocity (Float64MultiArray) for Gazebo
- FK: joint_states → odometry + TF (odom→base_footprint)
- Pioneer 2DX wheel parameters (r=0.08m, L=0.33m)
- Configurable output/input modes"
```

---

## Task 4: Create `amr_motion_control_2wd` Package — Motion Control Action Servers

This is the core package with 5 action servers for precision motion control.

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/package.xml`
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/CMakeLists.txt`
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/include/amr_motion_control_2wd/` (9 headers)
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/src/` (9 source files)
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/config/motion_params_gazebo.yaml`
- Create: `src/Control/AMR-Motion-Control/amr_motion_control_2wd/launch/motion_control_gazebo.launch.py`

### Sub-task 4a: Headers and Common Utilities

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/amr_motion_control_2wd/{include/amr_motion_control_2wd,src,config,launch}
```

- [ ] **Step 2: Create package.xml**

Dependencies: `rclcpp`, `rclcpp_action`, `amr_interfaces`, `tc_msgs`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`.

- [ ] **Step 3: Create CMakeLists.txt**

Builds `motion_profile` static library and `amr_motion_control_2wd_node` executable from all source files. Links motion_profile into the main executable.

- [ ] **Step 4: Port all 9 header files from T-AMR**

Port exactly from T-AMR source (already fetched):
- `motion_common.hpp` — ActiveAction enum, ActionGuard RAII, safeParam, publishCmdVel, normalizeAngle, lookupRobotPose, lookupTfYaw, readImuDelta utilities
- `motion_profile.hpp` — TrapezoidalProfile class (ProfilePhase, ProfileOutput)
- `recursive_moving_average.hpp` — O(1) recursive moving average filter
- `localization_watchdog.hpp` — Timeout + position jump detection
- `spin_action_server.hpp` — SpinActionServer class
- `turn_action_server.hpp` — TurnActionServer class
- `translate_action_server.hpp` — TranslateActionServer class with Stanley lateral control
- `translate_reverse_action_server.hpp` — TranslateReverseActionServer class
- `yaw_control_action_server.hpp` — YawControlActionServer class with PD heading

### Sub-task 4b: Source Files

- [ ] **Step 5: Port all 9 source files from T-AMR**

Port exactly from T-AMR (already fetched):
- `motion_common.cpp` — g_active_action atomic definition
- `motion_profile.cpp` — Trapezoidal/triangular profile with exit_speed support
- `localization_watchdog.cpp` — Pose timeout + velocity-aware jump detection
- `main.cpp` — Creates node + 5 action servers
- `spin_action_server.cpp` — IMU-based absolute angle spin with fine correction
- `turn_action_server.cpp` — Arc turn with trapezoidal angular profile
- `translate_action_server.cpp` — Linear path following with Stanley + PD heading, localization watchdog, safety speed limit, endpoint update service
- `translate_reverse_action_server.cpp` — Reverse translation with inverted heading
- `yaw_control_action_server.cpp` — PD heading-only control with min turning radius

### Sub-task 4c: Gazebo Configuration

- [ ] **Step 6: Create Gazebo-specific config**

**config/motion_params_gazebo.yaml:**
```yaml
amr_motion_control_2wd:
  ros__parameters:
    # Pioneer 2DX geometry
    wheel_separation: 0.33
    wheel_radius: 0.08

    # Control rate
    control_rate_hz: 50.0

    # Translate arrival
    translate_arrival_tolerance: 0.05
    translate_reverse_arrival_tolerance: 0.05

    # Spin precision (relaxed for Gazebo)
    imu_deadband_deg: 0.1
    min_speed_dps: 3.0
    fine_correction_threshold_deg: 0.5
    fine_correction_speed_dps: 5.0
    fine_correction_timeout_sec: 5.0
    settling_delay_ms: 300

    # Translate parameters
    translate_Kp_heading: 1.0
    translate_Kd_heading: 0.3
    translate_K_stanley: 2.0
    translate_K_soft: 1.0
    translate_max_omega: 1.0
    translate_alpha_max: 0.5
    translate_heading_threshold_deg: 45.0
    translate_max_lateral_offset: 1.0
    translate_heading_filter_window: 5
    translate_goal_reach_threshold: 0.05
    translate_arrival_tolerance: 0.05
    translate_min_vx: 0.02
    translate_behind_start_speed: 0.2
    translate_max_timeout_sec: 60.0
    translate_localization_timeout_sec: 2.0
    translate_position_jump_threshold: 0.3
    translate_enable_localization_watchdog: true
    translate_walk_accel_limit: 0.5
    translate_walk_decel_limit: 1.0

    # Localization pose topic (RTAB-Map)
    translate_pose_topic: "/rtabmap/localization_pose"
    translate_pose_qos: 2    # SensorDataQoS

    # YawControl parameters
    yaw_control_Kp_heading: 1.0
    yaw_control_Kd_heading: 0.3
    yaw_control_max_omega: 1.0
    yaw_control_heading_threshold_deg: 180.0
    yaw_control_max_lateral_offset: 5.0
    yaw_control_alpha_max: 0.5
    yaw_control_min_turning_radius: 0.5
    yaw_control_heading_filter_window: 5
    yaw_control_goal_reach_threshold: 0.05
    yaw_control_min_vx: 0.02
    yaw_control_max_timeout_sec: 60.0
    yaw_control_localization_timeout_sec: 2.0
    yaw_control_position_jump_threshold: 0.3
    yaw_control_enable_localization_watchdog: true
    yaw_control_walk_accel_limit: 0.5
    yaw_control_walk_decel_limit: 1.0
    yaw_control_pose_topic: "/rtabmap/localization_pose"
    yaw_control_pose_qos: 2
    transient_omega_rate_limit: 0.5
```

- [ ] **Step 7: Create Gazebo launch file**

**launch/motion_control_gazebo.launch.py:**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_motion_control_2wd')
    config_file = os.path.join(pkg_share, 'config', 'motion_params_gazebo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),

        Node(
            package='amr_motion_control_2wd',
            executable='amr_motion_control_2wd_node',
            name='amr_motion_control_2wd',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
```

- [ ] **Step 8: Build and verify**

```bash
colcon build --symlink-install --packages-up-to amr_motion_control_2wd
source install/setup.bash
```

Verify: Build succeeds with no errors. The node depends on IMU/TF at runtime, so standalone execution will log warnings but should not crash.

- [ ] **Step 9: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_control_2wd/
git commit -m "feat(amr_motion_control_2wd): add motion control action servers

5 action servers ported from T-AMR for 2WD differential drive:
- SpinActionServer: absolute angle spin (TF + IMU)
- TurnActionServer: arc turn with trapezoidal profile
- TranslateActionServer: linear path with Stanley + PD heading
- TranslateReverseActionServer: reverse translation
- YawControlActionServer: PD heading-only straight line

Gazebo config uses RTAB-Map localization_pose, relaxed
precision thresholds for simulation."
```

---

## Task 5: Create `amr_motion_test_ui` Package — Python Test UI

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/package.xml`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/setup.py`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/setup.cfg`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/resource/amr_motion_test_ui`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/__init__.py`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/test_ui_node.py`
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/launch/test_ui.launch.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/Control/AMR-Motion-Control/amr_motion_test_ui/{amr_motion_test_ui,launch,resource}
touch src/Control/AMR-Motion-Control/amr_motion_test_ui/resource/amr_motion_test_ui
touch src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/__init__.py
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>amr_motion_test_ui</name>
  <version>0.1.0</version>
  <description>CLI test UI for AMR motion control action servers</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>amr_interfaces</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create setup.py and setup.cfg**

**setup.py:**
```python
from setuptools import setup

package_name = 'amr_motion_test_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_ui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'test_ui_node = amr_motion_test_ui.test_ui_node:main',
        ],
    },
)
```

**setup.cfg:**
```ini
[develop]
script_dir=$base/lib/amr_motion_test_ui
[install]
install_scripts=$base/lib/amr_motion_test_ui
```

- [ ] **Step 4: Create test_ui_node.py**

Interactive CLI node that sends goals to the 5 action servers. Menu-driven interface:

```python
#!/usr/bin/env python3
"""
AMR Motion Control Test UI
Interactive CLI for testing motion control action servers in Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from amr_interfaces.action import (
    AMRMotionSpin, AMRMotionTurn, AMRMotionTranslate, AMRMotionYawControl
)
import sys
import threading


class MotionTestUI(Node):
    def __init__(self):
        super().__init__(
            'amr_motion_test_ui',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.spin_client = ActionClient(self, AMRMotionSpin, 'amr_spin_action')
        self.turn_client = ActionClient(self, AMRMotionTurn, 'amr_turn_action')
        self.translate_client = ActionClient(self, AMRMotionTranslate, 'amr_translate_action')
        self.yaw_control_client = ActionClient(self, AMRMotionYawControl, 'amr_yaw_control_action')
        self.get_logger().info('Motion Test UI started')

    def send_spin_goal(self, target_angle, max_speed=90.0, accel=180.0):
        goal = AMRMotionSpin.Goal()
        goal.target_angle = target_angle
        goal.max_angular_speed = max_speed
        goal.angular_acceleration = accel
        self.get_logger().info(f'Sending Spin goal: target={target_angle} deg')
        self.spin_client.wait_for_server()
        future = self.spin_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def send_turn_goal(self, target_angle, radius=0.5, max_speed=0.3, accel_angle=10.0):
        goal = AMRMotionTurn.Goal()
        goal.target_angle = float(target_angle)
        goal.turn_radius = float(radius)
        goal.max_linear_speed = float(max_speed)
        goal.accel_angle = float(accel_angle)
        self.get_logger().info(f'Sending Turn goal: angle={target_angle} deg, R={radius} m')
        self.turn_client.wait_for_server()
        future = self.turn_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def send_translate_goal(self, sx, sy, ex, ey, max_speed=0.3, accel=0.2):
        goal = AMRMotionTranslate.Goal()
        goal.start_x = float(sx)
        goal.start_y = float(sy)
        goal.end_x = float(ex)
        goal.end_y = float(ey)
        goal.max_linear_speed = float(max_speed)
        goal.acceleration = float(accel)
        goal.exit_speed = 0.0
        goal.has_next = False
        self.get_logger().info(f'Sending Translate: ({sx},{sy})->({ex},{ey})')
        self.translate_client.wait_for_server()
        future = self.translate_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def send_yaw_control_goal(self, sx, sy, ex, ey, max_speed=0.3, accel=0.2):
        goal = AMRMotionYawControl.Goal()
        goal.start_x = float(sx)
        goal.start_y = float(sy)
        goal.end_x = float(ex)
        goal.end_y = float(ey)
        goal.max_linear_speed = float(max_speed)
        goal.acceleration = float(accel)
        self.get_logger().info(f'Sending YawControl: ({sx},{sy})->({ex},{ey})')
        self.yaw_control_client.wait_for_server()
        future = self.yaw_control_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, 'current_angle'):
            self.get_logger().info(f'  [FB] angle={fb.current_angle:.1f} speed={fb.current_speed:.1f} phase={fb.phase}')
        elif hasattr(fb, 'current_distance'):
            self.get_logger().info(f'  [FB] dist={fb.current_distance:.3f} lat={fb.current_lateral_error:.3f} vx={fb.current_vx:.3f}')

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: status={result.status}, time={result.elapsed_time:.1f}s')


def run_menu(node):
    """Interactive CLI menu."""
    while rclpy.ok():
        print('\n=== AMR Motion Control Test UI ===')
        print('1. Spin (absolute angle)')
        print('2. Turn (arc with radius)')
        print('3. Translate (point-to-point)')
        print('4. YawControl (heading-only straight line)')
        print('q. Quit')
        choice = input('Select: ').strip()

        if choice == '1':
            angle = float(input('  Target angle (deg, absolute): '))
            speed = float(input('  Max angular speed (deg/s) [90]: ') or '90')
            node.send_spin_goal(angle, speed)
        elif choice == '2':
            angle = float(input('  Target angle (deg, +CCW/-CW): '))
            radius = float(input('  Turn radius (m) [0.5]: ') or '0.5')
            speed = float(input('  Max linear speed (m/s) [0.3]: ') or '0.3')
            node.send_turn_goal(angle, radius, speed)
        elif choice == '3':
            sx = float(input('  Start X (m): '))
            sy = float(input('  Start Y (m): '))
            ex = float(input('  End X (m): '))
            ey = float(input('  End Y (m): '))
            speed = float(input('  Max speed (m/s) [0.3]: ') or '0.3')
            node.send_translate_goal(sx, sy, ex, ey, speed)
        elif choice == '4':
            sx = float(input('  Start X (m): '))
            sy = float(input('  Start Y (m): '))
            ex = float(input('  End X (m): '))
            ey = float(input('  End Y (m): '))
            speed = float(input('  Max speed (m/s) [0.3]: ') or '0.3')
            node.send_yaw_control_goal(sx, sy, ex, ey, speed)
        elif choice == 'q':
            break


def main():
    rclpy.init()
    node = MotionTestUI()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_menu(node)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 5: Create launch file**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='amr_motion_test_ui',
            executable='test_ui_node',
            name='amr_motion_test_ui',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            prefix='xterm -e',
        ),
    ])
```

- [ ] **Step 6: Build and verify**

```bash
colcon build --symlink-install --packages-select amr_motion_test_ui
source install/setup.bash
ros2 run amr_motion_test_ui test_ui_node
```

Verify: Menu displays correctly, `q` exits cleanly.

- [ ] **Step 7: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/
git commit -m "feat(amr_motion_test_ui): add interactive CLI test UI

Menu-driven test interface for all motion control action servers:
Spin, Turn, Translate, YawControl.
Displays real-time feedback and results."
```

---

## Task 6: Gazebo Bridge Integration — Add Velocity Controller and Joint State Bridges

The Gazebo `gazebo_no_odom.launch.py` needs additional bridges for the kinematics node to work:
- Output: `velocity_controller/commands` (ROS2→Gazebo) for driving wheels
- Input: `drives/joint_states` (Gazebo→ROS2) for odometry feedback

**Files:**
- Modify: `src/Gazebo/launch/gazebo_no_odom.launch.py`

- [ ] **Step 1: Add velocity controller and joint state bridges**

Add to the bridge arguments in `gazebo_no_odom.launch.py`:
```python
'/velocity_controller/commands@std_msgs/msg/Float64MultiArray]ignition.msgs.Double_V',
'/drives/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
```

Note: The exact Ignition message types depend on the Pioneer 2DX SDF model's plugin configuration. If the model uses `ignition::gazebo::systems::DiffDrive`, it already accepts `cmd_vel` Twist and may not need a separate velocity controller. In that case, the kinematics node should use `output_mode: "joint_trajectory"` or be configured to just pass through `cmd_vel`.

**Alternative (simpler):** If Pioneer 2DX uses the DiffDrive plugin, the kinematics node can be configured to NOT publish IK commands (since Gazebo handles cmd_vel→wheel conversion internally) and only compute FK for odometry from the `/odom` topic or joint states.

- [ ] **Step 2: Verify Pioneer 2DX SDF model plugins**

```bash
grep -i "plugin\|diff_drive\|joint_state" src/Gazebo/models/pioneer2dx/model.sdf
```

Adapt the bridge configuration based on what plugins are available.

- [ ] **Step 3: Commit**

```bash
git add src/Gazebo/launch/gazebo_no_odom.launch.py
git commit -m "feat(gazebo): add velocity controller and joint state bridges

Enable kinematics_2wd node to drive wheels and read joint states
from Gazebo simulation for odometry computation."
```

---

## Task 7: SLAM Manager Integration

Update the existing SLAM Manager 3D to include motion control launch capability.

**Files:**
- Modify: `src/SLAM/SLAM_Manager/3D/slam_manager_3d/slam_manager_3d/slam_manager_3d_node.py`
- Modify: `src/SLAM/SLAM_Manager/3D/slam_manager_3d/slam_manager_3d/slam_manager_3d_ui.py`

- [ ] **Step 1: Add motion control process tracking to slam_manager_3d_node.py**

Add to the node:
- `motion_control_process` variable for tracking the motion control subprocess
- `start_motion_control()` method that launches `motion_control_gazebo.launch.py` + `kinematics_gazebo.launch.py`
- `stop_motion_control()` method
- Integration into the existing process cleanup workflow

- [ ] **Step 2: Add motion control UI buttons to slam_manager_3d_ui.py**

Add "Start Motion Control" / "Stop Motion Control" buttons similar to the existing Gazebo control buttons.

- [ ] **Step 3: Build and verify**

```bash
colcon build --symlink-install --packages-select slam_manager_3d
```

- [ ] **Step 4: Commit**

```bash
git add src/SLAM/SLAM_Manager/3D/
git commit -m "feat(slam_manager_3d): integrate motion control into SLAM Manager

Add start/stop controls for kinematics_2wd and
amr_motion_control_2wd Gazebo launch files."
```

---

## Task 8: Full Integration Launch File

Create a single launch file that starts everything needed for motion control testing in Gazebo.

**Files:**
- Create: `src/Control/AMR-Motion-Control/launch/motion_control_full_gazebo.launch.py`

- [ ] **Step 1: Create integrated launch file**

```python
"""
Full motion control stack for Gazebo simulation.
Launches: kinematics_2wd + amr_motion_control_2wd
(Assumes Gazebo + SLAM are already running)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Kinematics node (IK + FK + odom + TF)
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('kinematics_2wd'), 'launch', 'kinematics_gazebo.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Motion control action servers
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('amr_motion_control_2wd'), 'launch', 'motion_control_gazebo.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
```

- [ ] **Step 2: Commit**

```bash
git add src/Control/AMR-Motion-Control/launch/
git commit -m "feat(motion_control): add integrated Gazebo launch file

Launches kinematics_2wd + amr_motion_control_2wd together.
Requires Gazebo + SLAM to be running separately."
```

---

## Task 9: End-to-End Verification in Gazebo

- [ ] **Step 1: Full workspace build**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --symlink-install
source install/setup.bash
```

- [ ] **Step 2: Launch Gazebo simulation**

```bash
# Terminal 1: Gazebo
ros2 launch gazebo gazebo_no_odom.launch.py
```

- [ ] **Step 3: Launch SLAM (localization mode)**

```bash
# Terminal 2: RTAB-Map localization
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_localization_gazebo.launch.py
```

- [ ] **Step 4: Launch motion control stack**

```bash
# Terminal 3: Motion control
ros2 launch amr_motion_control_2wd motion_control_gazebo.launch.py
```

Or with kinematics:
```bash
ros2 launch kinematics_2wd kinematics_gazebo.launch.py &
ros2 launch amr_motion_control_2wd motion_control_gazebo.launch.py
```

- [ ] **Step 5: Verify topics and action servers**

```bash
ros2 topic list | grep -E "cmd_vel|odom|imu"
ros2 action list
# Expected: /amr_spin_action, /amr_turn_action, /amr_translate_action, etc.
```

- [ ] **Step 6: Test with Test UI**

```bash
# Terminal 4: Test UI
ros2 run amr_motion_test_ui test_ui_node
```

Test sequence:
1. Spin: target_angle=90, speed=45 → robot should rotate 90 degrees
2. Turn: angle=90, radius=0.5, speed=0.2 → arc turn
3. Translate: start=(current_x, current_y), end=(current_x+1, current_y) → 1m forward

- [ ] **Step 7: Commit verification results**

```bash
git add .
git commit -m "feat: complete AMR Motion Control integration for Gazebo

Full motion control stack operational:
- amr_interfaces: 5 actions, 2 services
- tc_msgs: motor-level messages
- kinematics_2wd: diff drive IK/FK for Pioneer 2DX
- amr_motion_control_2wd: 5 precision action servers
- amr_motion_test_ui: interactive CLI test interface
- Gazebo bridge integration
- SLAM Manager integration"
```

---

## Dependencies and Build Order

```
1. amr_interfaces  (no deps)
2. tc_msgs          (no deps, parallel with 1)
3. kinematics_2wd   (depends on: tc_msgs)
4. amr_motion_control_2wd  (depends on: amr_interfaces, tc_msgs)
5. amr_motion_test_ui      (depends on: amr_interfaces)
6-8. Integration (depends on: all above)
9. Verification
```

Tasks 1-2 can run in parallel. Tasks 3-5 can run in parallel after 1-2 complete.
