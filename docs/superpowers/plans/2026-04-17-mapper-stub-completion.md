# Mapper Stub Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 코드 리뷰에서 발견된 CRITICAL 2개 + HIGH 5개 미구현 항목을 완성하여 Mapper 패키지를 프로덕션 수준으로 끌어올린다.

**Architecture:** mapper_orchestrator_node (C++)에 CMD_SAVE_MAP, TF 기반 시작위치 기록, 커버리지 루프, MAPPING_AUTO 전환, 스레드 안전 종료를 추가한다. mapper_ui (Python)에 /cmd_vel 기반 수동 드라이브 버튼 핸들러를 추가한다.

**Tech Stack:** ROS2 Humble, C++17, rclcpp/rclcpp_action, tf2_ros, nav_msgs, geometry_msgs, PyQt5

---

## 파일 구조 (수정 대상)

| 파일 | 변경 내용 |
|------|-----------|
| `mapper/include/mapper/mapper_orchestrator_node.hpp` | tf2_ros 멤버, shutdown flag, map 구독자, run_mapping_auto 선언 |
| `mapper/src/mapper_orchestrator_node.cpp` | 6개 항목 구현 |
| `mapper_ui/mapper_ui/ros_bridge.py` | cmd_vel 퍼블리셔 + publish_cmd_vel() |
| `mapper_ui/mapper_ui/mapper_ui_node.py` | btn_forward/left/right 핸들러 |
| `mapper/config/mapper_params.yaml` | manual_drive_linear_speed, manual_drive_angular_speed 추가 |

---

## Task 1: Thread Lifetime 안전화 (CRITICAL-1)

**Files:**
- Modify: `mapper/include/mapper/mapper_orchestrator_node.hpp`
- Modify: `mapper/src/mapper_orchestrator_node.cpp`

- [ ] **Step 1: 헤더에 shutdown flag + active worker counter 추가**

`mapper/include/mapper/mapper_orchestrator_node.hpp` 의 private 섹션에 추가:

```cpp
// thread lifetime (LOCK ORDERING: state_mutex_ -> log_mutex_)
std::atomic<bool> shutdown_requested_{false};
std::atomic<int>  active_workers_{0};

void launch_worker(std::function<void()> fn);
```

- [ ] **Step 2: 소스에 launch_worker 헬퍼 + 소멸자 구현**

`mapper/src/mapper_orchestrator_node.cpp` 의 namespace 안, `MapperOrchestratorNode::transition_to` 앞에 추가:

```cpp
MapperOrchestratorNode::~MapperOrchestratorNode() {
    shutdown_requested_.store(true);
    transition_to(MapperState::IDLE);   // detached thread 루프 탈출 유도
    // 최대 3초 대기
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (active_workers_.load() > 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MapperOrchestratorNode::launch_worker(std::function<void()> fn) {
    active_workers_.fetch_add(1);
    std::thread([this, fn = std::move(fn)] {
        fn();
        active_workers_.fetch_sub(1);
    }).detach();
}
```

- [ ] **Step 3: 헤더에 소멸자 선언 추가**

`mapper_orchestrator_node.hpp` public 섹션에:

```cpp
~MapperOrchestratorNode();
```

- [ ] **Step 4: 모든 `std::thread([self]{ ... }).detach()` 를 `launch_worker` 로 교체**

`mapper_orchestrator_node.cpp` 에서 4곳 수정:

```cpp
// CMD_START_MAPPING (line ~100)
// 기존: std::thread([self]{ self->run_aligning(); }).detach();
launch_worker([self = shared_from_this()] {
    std::static_pointer_cast<MapperOrchestratorNode>(self)->run_aligning();
});

// CMD_RESUME -> MAPPING_MANUAL (line ~126)
// 기존: std::thread([self]{ self->run_mapping_manual(); }).detach();
launch_worker([self = shared_from_this()] {
    std::static_pointer_cast<MapperOrchestratorNode>(self)->run_mapping_manual();
});

// CMD_RESUME -> EXPLORING_UNKNOWN (line ~128)
// 기존: std::thread([self]{ self->run_exploring(); }).detach();
launch_worker([self = shared_from_this()] {
    std::static_pointer_cast<MapperOrchestratorNode>(self)->run_exploring();
});

// CMD_EXPLORE (line ~149)
// 기존: std::thread([self]{ self->run_exploring(); }).detach();
launch_worker([self = shared_from_this()] {
    std::static_pointer_cast<MapperOrchestratorNode>(self)->run_exploring();
});
```

- [ ] **Step 5: 각 worker 함수 첫 번째 IDLE 체크에 shutdown_requested_ 추가**

`run_aligning()`, `run_starting_slam()`, `run_verifying_map()`, `run_exploring()`, `run_loop_closing()` 의 첫 줄 조건:

```cpp
// 기존
if (state_.load() == MapperState::IDLE) return;
// 변경
if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;
```

- [ ] **Step 6: 빌드 확인**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select mapper --symlink-install 2>&1 | tail -20
```

Expected: `Finished <<< mapper` (warning 없음)

- [ ] **Step 7: 커밋**

```bash
cd ~/Study/ros2_3dslam_ws
git add src/Mapper/mapper/include/mapper/mapper_orchestrator_node.hpp \
        src/Mapper/mapper/src/mapper_orchestrator_node.cpp
git commit -m "fix(mapper): add thread lifetime management with shutdown flag and active_workers counter"
```

---

## Task 2: CMD_SAVE_MAP 구현 (HIGH-1)

**Files:**
- Modify: `mapper/src/mapper_orchestrator_node.cpp:158-161`

- [ ] **Step 1: handle_command 의 CMD_SAVE_MAP 케이스 구현**

`mapper_orchestrator_node.cpp` 의 `case Cmd::CMD_SAVE_MAP:` 블록을 교체:

```cpp
case Cmd::CMD_SAVE_MAP: {
    auto& slam_client = (slam_mode_.load() == 0) ?
        slam_ctrl_client_2d_ : slam_ctrl_client_3d_;
    if (!slam_client->service_is_ready()) {
        res->success = false;
        res->message = "SLAM service not ready";
        break;
    }
    using SlamCtrl = mapper_interfaces::srv::SlamControl;
    auto slam_req = std::make_shared<SlamCtrl::Request>();
    slam_req->command        = SlamCtrl::Request::CMD_SAVE_MAP;
    slam_req->map_name       = req->map_name;
    slam_req->save_directory = req->save_directory;

    auto done     = std::make_shared<std::atomic<bool>>(false);
    auto suc      = std::make_shared<std::atomic<bool>>(false);
    auto saved_path = std::make_shared<std::string>();
    std::mutex path_mutex;

    slam_client->async_send_request(slam_req,
        [done, suc, saved_path, &path_mutex]
        (rclcpp::Client<SlamCtrl>::SharedFuture fut) {
            auto r = fut.get();
            suc->store(r->success);
            std::lock_guard<std::mutex> lk(path_mutex);
            *saved_path = r->saved_path;
            done->store(true);
        });

    // 서비스 응답 대기 (최대 10초, handle_command 는 service callback 이므로 blocking 허용)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!done->load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!done->load()) {
        res->success = false;
        res->message = "Save map timeout";
        break;
    }
    res->success = suc->load();
    std::lock_guard<std::mutex> lk(path_mutex);
    res->message = res->success ?
        ("Map saved to: " + *saved_path) : "Map save failed";
    if (res->success) log("Map saved: " + *saved_path);
    break;
}
```

- [ ] **Step 2: 빌드 확인**

```bash
colcon build --packages-select mapper --symlink-install 2>&1 | tail -10
```

Expected: `Finished <<< mapper`

- [ ] **Step 3: 커밋**

```bash
git add src/Mapper/mapper/src/mapper_orchestrator_node.cpp
git commit -m "feat(mapper): implement CMD_SAVE_MAP by forwarding to SLAM control service"
```

---

## Task 3: TF 기반 시작 위치 기록 (HIGH-4)

**Files:**
- Modify: `mapper/include/mapper/mapper_orchestrator_node.hpp`
- Modify: `mapper/src/mapper_orchestrator_node.cpp`

- [ ] **Step 1: 헤더에 tf2_ros 멤버 추가**

`mapper_orchestrator_node.hpp` include 섹션에 추가:

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
```

private 멤버 섹션에 추가:

```cpp
std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
```

- [ ] **Step 2: 생성자에서 tf2 초기화**

`mapper_orchestrator_node.cpp` 생성자 마지막 줄 (`status_timer_` 생성 후) 에 추가:

```cpp
tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

- [ ] **Step 3: run_starting_slam() 에서 시작 위치 기록**

`run_starting_slam()` 의 `transition_to(MapperState::VERIFYING_MAP);` 바로 앞에 추가:

```cpp
// 로봇 시작 위치 기록 (루프 클로저 복귀용)
try {
    auto tf = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero,
        tf2::durationFromSec(1.0));
    start_x_ = tf.transform.translation.x;
    start_y_ = tf.transform.translation.y;
    RCLCPP_INFO(get_logger(), "Start position recorded: (%.2f, %.2f)",
        start_x_, start_y_);
} catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not record start position: %s", ex.what());
}
```

- [ ] **Step 4: 빌드 확인**

```bash
colcon build --packages-select mapper --symlink-install 2>&1 | tail -10
```

Expected: `Finished <<< mapper`

- [ ] **Step 5: 커밋**

```bash
git add src/Mapper/mapper/include/mapper/mapper_orchestrator_node.hpp \
        src/Mapper/mapper/src/mapper_orchestrator_node.cpp
git commit -m "feat(mapper): record robot start position via TF in run_starting_slam for loop closure return"
```

---

## Task 4: run_mapping_manual() 커버리지 모니터링 (HIGH-2)

**Files:**
- Modify: `mapper/include/mapper/mapper_orchestrator_node.hpp`
- Modify: `mapper/src/mapper_orchestrator_node.cpp`

- [ ] **Step 1: 헤더에 map 구독자 멤버 추가**

`mapper_orchestrator_node.hpp` include 섹션에 추가:

```cpp
#include <nav_msgs/msg/occupancy_grid.hpp>
```

private 멤버에 추가:

```cpp
rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
std::atomic<float> map_coverage_{0.0f};
```

- [ ] **Step 2: 생성자에 map 구독 추가**

`mapper_orchestrator_node.cpp` 생성자에 tf_buffer_ 초기화 다음에 추가:

```cpp
map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg->data.empty()) return;
        int known = 0;
        for (auto cell : msg->data) { if (cell != -1) ++known; }
        float cov = static_cast<float>(known) / static_cast<float>(msg->data.size()) * 100.0f;
        map_coverage_.store(cov);
        coverage_percent_.store(cov);
    });
```

- [ ] **Step 3: run_mapping_manual() 구현**

`mapper_orchestrator_node.cpp` 의 `run_mapping_manual()` 을 교체:

```cpp
void MapperOrchestratorNode::run_mapping_manual() {
    log("Manual mapping mode -- waiting for commands or coverage target");
    while (rclcpp::ok()) {
        if (state_.load() == MapperState::IDLE ||
            shutdown_requested_.load()) return;
        if (state_.load() == MapperState::PAUSED) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        float cov = map_coverage_.load();
        if (cov >= min_coverage_to_stop_.load() * 100.0f) {
            log("Coverage target reached in manual mode -- starting loop closure");
            transition_to(MapperState::LOOP_CLOSING);
            run_loop_closing();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
```

- [ ] **Step 4: 빌드 확인**

```bash
colcon build --packages-select mapper --symlink-install 2>&1 | tail -10
```

Expected: `Finished <<< mapper`

- [ ] **Step 5: 커밋**

```bash
git add src/Mapper/mapper/include/mapper/mapper_orchestrator_node.hpp \
        src/Mapper/mapper/src/mapper_orchestrator_node.cpp
git commit -m "feat(mapper): implement run_mapping_manual with map coverage monitoring and auto loop-closure transition"
```

---

## Task 5: MAPPING_AUTO 상태 활성화 (HIGH-3)

**Files:**
- Modify: `mapper/include/mapper/mapper_orchestrator_node.hpp`
- Modify: `mapper/src/mapper_orchestrator_node.cpp`

- [ ] **Step 1: 헤더에 run_mapping_auto 선언 추가**

`mapper_orchestrator_node.hpp` private 메서드 목록에 추가:

```cpp
void run_mapping_auto();
```

- [ ] **Step 2: run_verifying_map() 성공 분기에서 drive_mode 기반 전환**

`mapper_orchestrator_node.cpp` `run_verifying_map()` 의:

```cpp
// 기존 (line ~323-325)
if (is_aligned->load()) {
    log("Map alignment verified");
    transition_to(MapperState::MAPPING_MANUAL);
}
```

를 교체:

```cpp
if (is_aligned->load()) {
    log("Map alignment verified");
    if (drive_mode_.load() == 0) {
        // drive_mode 0 = MANUAL
        transition_to(MapperState::MAPPING_MANUAL);
        launch_worker([self = shared_from_this()] {
            std::static_pointer_cast<MapperOrchestratorNode>(self)->run_mapping_manual();
        });
    } else {
        // drive_mode 1+ = AUTO (탐색 자동 시작)
        transition_to(MapperState::MAPPING_AUTO);
        launch_worker([self = shared_from_this()] {
            std::static_pointer_cast<MapperOrchestratorNode>(self)->run_mapping_auto();
        });
    }
}
```

- [ ] **Step 3: run_mapping_auto() 구현**

`mapper_orchestrator_node.cpp` `run_loop_closing()` 앞에 추가:

```cpp
void MapperOrchestratorNode::run_mapping_auto() {
    log("Auto mapping mode -- starting exploration");
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;
    transition_to(MapperState::EXPLORING_UNKNOWN);
    run_exploring();
}
```

- [ ] **Step 4: run_mapping_manual() 의 launch_worker 호출 중복 제거**

현재 `run_verifying_map` 성공 시 `run_mapping_manual()` 을 직접 호출하지 않고 있음 — Task 3에서 이미 `launch_worker` 로 감쌌는지 확인. `run_mapping_manual()` 이 내부에서 `run_loop_closing()` 을 직접 호출하므로 중복 worker 생성이 없는지 검토.

`run_mapping_manual()` 의 `run_loop_closing()` 직접 호출은 같은 thread 내에서 이루어지므로 추가 launch_worker 불필요. ✅

- [ ] **Step 5: PAUSE/RESUME 에서 MAPPING_AUTO 처리 추가**

`handle_command` 의 `CMD_PAUSE` 조건에 `MAPPING_AUTO` 추가:

```cpp
// 기존
if (state == MapperState::MAPPING_MANUAL ||
    state == MapperState::EXPLORING_UNKNOWN) {
// 변경
if (state == MapperState::MAPPING_MANUAL ||
    state == MapperState::MAPPING_AUTO ||
    state == MapperState::EXPLORING_UNKNOWN) {
```

`CMD_RESUME` 에 MAPPING_AUTO 복귀 추가:

```cpp
// 기존 resume 분기에 추가
else if (prev == MapperState::MAPPING_AUTO)
    launch_worker([self = shared_from_this()] {
        std::static_pointer_cast<MapperOrchestratorNode>(self)->run_mapping_auto();
    });
```

- [ ] **Step 6: 빌드 확인**

```bash
colcon build --packages-select mapper --symlink-install 2>&1 | tail -10
```

Expected: `Finished <<< mapper`

- [ ] **Step 7: 커밋**

```bash
git add src/Mapper/mapper/include/mapper/mapper_orchestrator_node.hpp \
        src/Mapper/mapper/src/mapper_orchestrator_node.cpp
git commit -m "feat(mapper): activate MAPPING_AUTO state with drive_mode-based branching and run_mapping_auto implementation"
```

---

## Task 6: UI 수동 드라이브 버튼 (/cmd_vel) (HIGH-5)

**Files:**
- Modify: `mapper_ui/mapper_ui/ros_bridge.py`
- Modify: `mapper_ui/mapper_ui/mapper_ui_node.py`
- Modify: `mapper/config/mapper_params.yaml`

- [ ] **Step 1: ros_bridge.py 에 cmd_vel 퍼블리셔 추가**

`ros_bridge.py` import 섹션에 추가:

```python
from geometry_msgs.msg import Twist
```

`RosBridge.__init__()` 의 `self._cmd_client` 생성 다음에 추가:

```python
self._cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
```

`RosBridge` 클래스에 메서드 추가:

```python
def publish_cmd_vel(self, linear_x: float = 0.0, angular_z: float = 0.0):
    msg = Twist()
    msg.linear.x  = linear_x
    msg.angular.z = angular_z
    self._cmd_vel_pub.publish(msg)
```

- [ ] **Step 2: mapper_ui_node.py 에 드라이브 버튼 핸들러 추가**

`MapperMainWindow.__init__()` 의 버튼 연결 섹션에 추가:

```python
self.btn_forward.pressed.connect(self._on_forward_pressed)
self.btn_forward.released.connect(self._on_drive_released)
self.btn_left.pressed.connect(self._on_left_pressed)
self.btn_left.released.connect(self._on_drive_released)
self.btn_right.pressed.connect(self._on_right_pressed)
self.btn_right.released.connect(self._on_drive_released)
```

`MapperMainWindow` 클래스에 핸들러 추가 (`_on_explore` 다음):

```python
DRIVE_LINEAR  = 0.2   # m/s
DRIVE_ANGULAR = 0.5   # rad/s

def _on_forward_pressed(self):
    self.bridge.publish_cmd_vel(linear_x=self.DRIVE_LINEAR)

def _on_left_pressed(self):
    self.bridge.publish_cmd_vel(angular_z=self.DRIVE_ANGULAR)

def _on_right_pressed(self):
    self.bridge.publish_cmd_vel(angular_z=-self.DRIVE_ANGULAR)

def _on_drive_released(self):
    self.bridge.publish_cmd_vel()   # 0, 0 → 정지
```

- [ ] **Step 3: mapper_ui package.xml 에 geometry_msgs 의존성 확인**

```bash
grep geometry_msgs ~/Study/ros2_3dslam_ws/src/Mapper/mapper_ui/package.xml
```

없으면 `package.xml` 의 `<depend>rclpy</depend>` 다음에 추가:

```xml
<depend>geometry_msgs</depend>
```

- [ ] **Step 4: 빌드 확인**

```bash
colcon build --packages-select mapper_ui --symlink-install 2>&1 | tail -10
```

Expected: `Finished <<< mapper_ui`

- [ ] **Step 5: 커밋**

```bash
git add src/Mapper/mapper_ui/mapper_ui/ros_bridge.py \
        src/Mapper/mapper_ui/mapper_ui/mapper_ui_node.py \
        src/Mapper/mapper_ui/package.xml
git commit -m "feat(mapper_ui): connect forward/left/right buttons to /cmd_vel publisher"
```

---

## Task 7: 전체 빌드 + 통합 검증

- [ ] **Step 1: 전체 Mapper 패키지 빌드**

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select mapper mapper_interfaces mapper_ui --symlink-install 2>&1 | tail -20
```

Expected: 3개 패키지 모두 `Finished <<<`

- [ ] **Step 2: 오케스트레이터 단위 테스트 실행**

```bash
colcon test --packages-select mapper --event-handlers console_direct+ 2>&1 | tail -30
```

Expected: 모든 테스트 PASS

- [ ] **Step 3: 상태 전이 수동 검증 (별도 터미널)**

```bash
# Terminal 1
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 launch mapper mapper_gazebo.launch.py

# Terminal 2 - IDLE → ALIGNING (drive_mode=1 AUTO)
ros2 service call /mapper/command mapper_interfaces/srv/MapperCommand \
  '{command: 0, slam_mode: 0, drive_mode: 1}'

# Terminal 2 - MAPPING_MANUAL 진입 후 SAVE_MAP
ros2 service call /mapper/command mapper_interfaces/srv/MapperCommand \
  '{command: 3, map_name: "test_map", save_directory: "/tmp"}'

# Terminal 2 - STOP
ros2 service call /mapper/command mapper_interfaces/srv/MapperCommand '{command: 2}'
```

Expected output 순서: `IDLE→ALIGNING→STARTING_SLAM→VERIFYING_MAP→MAPPING_AUTO→EXPLORING_UNKNOWN→LOOP_CLOSING→COMPLETED`

- [ ] **Step 4: 최종 커밋**

```bash
git add -A
git commit -m "test(mapper): verify full pipeline IDLE→COMPLETED with MAPPING_AUTO and CMD_SAVE_MAP"
```

---

## 구현 우선순위 요약

| Task | 항목 | 예상 시간 |
|------|------|-----------|
| 1 | Thread lifetime 안전화 | 30분 |
| 2 | CMD_SAVE_MAP | 20분 |
| 3 | TF 시작 위치 기록 | 20분 |
| 4 | run_mapping_manual() 커버리지 루프 | 20분 |
| 5 | MAPPING_AUTO 활성화 | 30분 |
| 6 | UI 드라이브 버튼 | 20분 |
| 7 | 통합 검증 | 30분 |

**총 예상 시간: 약 2.5시간**
