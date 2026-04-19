#include "mapper/mapper_orchestrator_node.hpp"
#include <chrono>

namespace mapper {

MapperOrchestratorNode::MapperOrchestratorNode(
    const rclcpp::NodeOptions & options)
: Node("mapper_orchestrator_node", options)
{
    using namespace std::placeholders;

    // C1: 파라미터 선언 및 로드
    max_align_retries_        = static_cast<int>(declare_parameter("max_align_retries", 3));
    map_stabilize_wait_sec_   = declare_parameter("map_stabilize_wait_sec", 3.0);
    loop_closure_timeout_sec_ = declare_parameter("loop_closure_timeout_sec", 30.0);
    min_coverage_to_stop_.store(static_cast<float>(declare_parameter("min_coverage_to_stop", 0.95)));

    status_pub_ = create_publisher<mapper_interfaces::msg::MapperStatus>(
        "mapper/status", rclcpp::QoS(10).reliable());

    cmd_service_ = create_service<mapper_interfaces::srv::MapperCommand>(
        "mapper/command",
        std::bind(&MapperOrchestratorNode::handle_command, this, _1, _2));

    wall_align_client_ = rclcpp_action::create_client<
        mapper_interfaces::action::WallAlign>(this, "wall_align");
    map_check_client_  = rclcpp_action::create_client<
        mapper_interfaces::action::MapAlignmentCheck>(this, "map_alignment_check");
    explore_client_    = rclcpp_action::create_client<
        mapper_interfaces::action::ExploreUnknown>(this, "explore_unknown");

    // C2: 2D/3D SLAM 서비스 클라이언트 분리 (서비스 이름 파라미터화)
    auto svc_2d = declare_parameter("slam_control_service_2d",
        std::string("slam_manager_2d/slam_control"));
    auto svc_3d = declare_parameter("slam_control_service_3d",
        std::string("slam_manager_3d/slam_control"));
    slam_ctrl_client_2d_ = create_client<mapper_interfaces::srv::SlamControl>(svc_2d);
    slam_ctrl_client_3d_ = create_client<mapper_interfaces::srv::SlamControl>(svc_3d);

    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapperOrchestratorNode::publish_status, this));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (msg->data.empty()) return;
            int known = 0;
            for (auto cell : msg->data) { if (cell != -1) ++known; }
            float cov = static_cast<float>(known) /
                        static_cast<float>(msg->data.size()) * 100.0f;
            map_coverage_.store(cov);
            coverage_percent_.store(cov);
        });
}

MapperOrchestratorNode::~MapperOrchestratorNode() {
    shutdown_requested_.store(true);
    transition_to(MapperState::IDLE);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (active_workers_.load() > 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MapperOrchestratorNode::launch_worker(std::function<void()> fn) {
    active_workers_.fetch_add(1);
    auto self = std::static_pointer_cast<MapperOrchestratorNode>(shared_from_this());
    std::thread([self, fn = std::move(fn)] {
        fn();
        self->active_workers_.fetch_sub(1);
    }).detach();
}

// H4: 내부 버전 (이미 mutex를 잡은 경우)
void MapperOrchestratorNode::transition_to_unlocked(MapperState new_state) {
    auto old_state = state_.load();
    previous_state_.store(old_state);  // H2: previous_state_ 갱신
    state_.store(new_state);
    RCLCPP_INFO(get_logger(), "State: %d -> %d",
        static_cast<int>(old_state), static_cast<int>(new_state));
}

// H4: 외부 버전 (mutex 자동 획득)
void MapperOrchestratorNode::transition_to(MapperState new_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_.load() == MapperState::IDLE && new_state != MapperState::IDLE) return;
    transition_to_unlocked(new_state);
}

void MapperOrchestratorNode::log(const std::string & msg) {
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    std::lock_guard<std::mutex> lk(log_mutex_);
    log_message_ = msg;
}

void MapperOrchestratorNode::publish_status() {
    mapper_interfaces::msg::MapperStatus status;
    status.state                   = static_cast<uint8_t>(state_.load());
    status.previous_state          = static_cast<uint8_t>(previous_state_.load());
    status.coverage_percent        = coverage_percent_.load();   // H3: atomic load
    status.current_heading_error_deg = heading_error_deg_.load(); // H3: atomic load
    {
        std::lock_guard<std::mutex> lk(log_mutex_);
        status.log_message = log_message_;
    }
    status_pub_->publish(status);
}

void MapperOrchestratorNode::handle_command(
    const mapper_interfaces::srv::MapperCommand::Request::SharedPtr req,
    mapper_interfaces::srv::MapperCommand::Response::SharedPtr res)
{
    using Cmd = mapper_interfaces::srv::MapperCommand::Request;
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto state = state_.load();
    auto self  = std::static_pointer_cast<MapperOrchestratorNode>(shared_from_this());

    switch (req->command) {
    case Cmd::CMD_START_MAPPING:
        if (state == MapperState::IDLE) {
            slam_mode_.store(req->slam_mode);
            drive_mode_.store(req->drive_mode);
            align_retry_count_.store(0);
            transition_to_unlocked(MapperState::ALIGNING);  // H4: 이미 mutex 보유
            launch_worker([self]{ self->run_aligning(); });
            res->success = true;
            res->message = "Mapping started";
        } else {
            res->success = false;
            res->message = "Not in IDLE state";
        }
        break;

    case Cmd::CMD_PAUSE:
        if (state == MapperState::MAPPING_MANUAL ||
            state == MapperState::MAPPING_AUTO ||
            state == MapperState::EXPLORING_UNKNOWN) {
            transition_to_unlocked(MapperState::PAUSED);  // H2+H4: previous_state_ 자동 갱신
            res->success = true;
            res->message = "Paused";
        } else {
            res->success = false;
            res->message = "Cannot pause in current state";
        }
        break;

    case Cmd::CMD_RESUME:
        if (state == MapperState::PAUSED) {
            auto prev = previous_state_.load();
            transition_to_unlocked(prev);  // H4
            if (prev == MapperState::MAPPING_MANUAL)
                launch_worker([self]{ self->run_mapping_manual(); });
            else if (prev == MapperState::MAPPING_AUTO)
                launch_worker([self]{ self->run_mapping_auto(); });
            else if (prev == MapperState::EXPLORING_UNKNOWN)
                launch_worker([self]{ self->run_exploring(); });
            res->success = true;
            res->message = "Resumed";
        } else {
            res->success = false;
            res->message = "Not paused";
        }
        break;

    case Cmd::CMD_STOP:
        transition_to_unlocked(MapperState::IDLE);  // H4
        if (wall_align_client_) wall_align_client_->async_cancel_all_goals();
        if (map_check_client_)  map_check_client_->async_cancel_all_goals();
        if (explore_client_)    explore_client_->async_cancel_all_goals();
        res->success = true;
        res->message = "Stopped";
        break;

    case Cmd::CMD_EXPLORE:
        if (state == MapperState::MAPPING_MANUAL) {
            transition_to_unlocked(MapperState::EXPLORING_UNKNOWN);  // H4
            launch_worker([self]{ self->run_exploring(); });
            res->success = true;
            res->message = "Exploration started";
        } else {
            res->success = false;
            res->message = "Must be in MAPPING_MANUAL state";
        }
        break;

    case Cmd::CMD_SAVE_MAP: {
        auto & slam_client = (slam_mode_.load() == 0) ?
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

        auto done      = std::make_shared<std::atomic<bool>>(false);
        auto suc       = std::make_shared<std::atomic<bool>>(false);
        auto saved     = std::make_shared<std::string>();
        auto saved_mtx = std::make_shared<std::mutex>();

        slam_client->async_send_request(slam_req,
            [done, suc, saved, saved_mtx]
            (rclcpp::Client<SlamCtrl>::SharedFuture fut) {
                try {
                    auto r = fut.get();
                    suc->store(r->success);
                    std::lock_guard<std::mutex> lk(*saved_mtx);
                    *saved = r->saved_path;
                } catch (const std::exception & e) {
                    suc->store(false);
                    std::lock_guard<std::mutex> lk(*saved_mtx);
                    *saved = std::string("Error: ") + e.what();
                }
                done->store(true);
            });

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
        {
            std::lock_guard<std::mutex> lk(*saved_mtx);
            res->message = res->success ?
                ("Map saved to: " + *saved) : "Map save failed";
            if (res->success) log("Map saved: " + *saved);
        }
        break;
    }

    default:
        res->success = false;
        res->message = "Unknown command";
    }
}

void MapperOrchestratorNode::run_aligning() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;

    // Poll for action server with early exit on IDLE/stop
    bool server_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (wall_align_client_->action_server_is_ready()) {
            server_ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!server_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("WallAligner not available");
        transition_to(MapperState::ERROR);
        return;
    }

    mapper_interfaces::action::WallAlign::Goal goal{};
    goal.tolerance_deg      = 0.2;
    goal.use_imu_correction = false;

    auto done = std::make_shared<std::atomic<bool>>(false);
    auto result_status = std::make_shared<std::atomic<int8_t>>(-1);

    auto opts = rclcpp_action::Client<
        mapper_interfaces::action::WallAlign>::SendGoalOptions{};
    opts.result_callback = [done, result_status](const auto & r) {
        result_status->store(r.result->status);
        done->store(true);
    };

    wall_align_client_->async_send_goal(goal, opts);
    while (!done->load() && rclcpp::ok() &&
           state_.load() != MapperState::IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (state_.load() == MapperState::IDLE) return;

    int8_t status = result_status->load();
    if (status == 0) {
        align_retry_count_.store(0);  // H3: atomic store
        transition_to(MapperState::STARTING_SLAM);
        run_starting_slam();
    } else if (status == -4) {
        // NO_WALL_DATA — wall_detector 미동작/탐지 실패. 재시도 무의미.
        log("Wall detector unavailable (status=-4) — going to ERROR");
        transition_to(MapperState::ERROR);
    } else {
        // -1 (ABORT), -3 (CANCEL) 등 일반 실패 — 재시도
        align_retry_count_.fetch_add(1);  // H3: atomic
        if (align_retry_count_.load() >= max_align_retries_) {
            log("Alignment failed after max retries (last_status=" +
                std::to_string(status) + ")");
            transition_to(MapperState::ERROR);
        } else {
            run_aligning();
        }
    }
}

void MapperOrchestratorNode::run_starting_slam() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;

    // C2: slam_mode_에 따라 적절한 클라이언트 선택
    auto& slam_client = (slam_mode_.load() == 0) ? slam_ctrl_client_2d_ : slam_ctrl_client_3d_;

    bool svc_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (slam_client->service_is_ready()) { svc_ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!svc_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("SlamControl service not available");
        transition_to(MapperState::ERROR);
        return;
    }

    using SlamCtrl = mapper_interfaces::srv::SlamControl;
    auto req = std::make_shared<SlamCtrl::Request>();
    req->command = (slam_mode_.load() == 0) ?
        SlamCtrl::Request::CMD_START_2D :
        SlamCtrl::Request::CMD_START_3D;

    auto done = std::make_shared<std::atomic<bool>>(false);
    auto success = std::make_shared<std::atomic<bool>>(false);
    slam_client->async_send_request(req,
        [done, success](rclcpp::Client<SlamCtrl>::SharedFuture fut) {
            success->store(fut.get()->success);
            done->store(true);
        });

    // M3: IDLE 체크 추가
    while (!done->load() && rclcpp::ok() &&
           state_.load() != MapperState::IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (state_.load() == MapperState::IDLE) return;

    if (!success->load()) {
        log("SLAM start failed");
        transition_to(MapperState::ERROR);
        return;
    }

    {
        auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(map_stabilize_wait_sec_);
        while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
            if (state_.load() == MapperState::IDLE) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (state_.load() == MapperState::IDLE) return;
    }

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

    transition_to(MapperState::VERIFYING_MAP);
    run_verifying_map();
}

void MapperOrchestratorNode::run_verifying_map() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;

    bool server_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (map_check_client_->action_server_is_ready()) { server_ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!server_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("MapAlignmentChecker not available -- skipping");
        transition_to(MapperState::MAPPING_MANUAL);
        return;
    }

    mapper_interfaces::action::MapAlignmentCheck::Goal goal{};
    goal.tolerance_deg = 0.5;

    auto done = std::make_shared<std::atomic<bool>>(false);
    auto is_aligned = std::make_shared<std::atomic<bool>>(true);

    auto opts = rclcpp_action::Client<
        mapper_interfaces::action::MapAlignmentCheck>::SendGoalOptions{};
    opts.result_callback = [done, is_aligned](const auto & r) {
        is_aligned->store(r.result->is_aligned);
        done->store(true);
    };

    map_check_client_->async_send_goal(goal, opts);
    while (!done->load() && rclcpp::ok() &&
           state_.load() != MapperState::IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (state_.load() == MapperState::IDLE) return;

    if (is_aligned->load()) {
        log("Map alignment verified");
        if (drive_mode_.load() == 0) {
            transition_to(MapperState::MAPPING_MANUAL);
            launch_worker([self = std::static_pointer_cast<MapperOrchestratorNode>(
                               shared_from_this())]{ self->run_mapping_manual(); });
        } else {
            transition_to(MapperState::MAPPING_AUTO);
            launch_worker([self = std::static_pointer_cast<MapperOrchestratorNode>(
                               shared_from_this())]{ self->run_mapping_auto(); });
        }
    } else {
        log("Map misaligned -- re-aligning");
        total_realign_count_.fetch_add(1);
        if (total_realign_count_.load() >= max_total_realigns_) {
            log("Max re-alignment cycles exceeded");
            transition_to(MapperState::ERROR);
            return;
        }
        align_retry_count_.store(0);  // H3: atomic store
        transition_to(MapperState::ALIGNING);
        run_aligning();
    }
}

void MapperOrchestratorNode::run_mapping_manual() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;
    log("Manual mapping mode -- waiting for commands or coverage target");
    while (rclcpp::ok()) {
        auto cur = state_.load();
        if (cur == MapperState::IDLE || shutdown_requested_.load()) return;
        if (cur == MapperState::PAUSED) {
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

void MapperOrchestratorNode::run_exploring() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;

    bool server_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (explore_client_->action_server_is_ready()) { server_ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!server_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("ExplorationPlanner not available");
        transition_to(MapperState::ERROR);
        return;
    }

    mapper_interfaces::action::ExploreUnknown::Goal goal{};
    goal.mode = drive_mode_.load();
    goal.min_coverage_to_stop = min_coverage_to_stop_.load();

    auto done = std::make_shared<std::atomic<bool>>(false);
    auto result_status = std::make_shared<std::atomic<int8_t>>(-1);
    auto coverage = std::make_shared<std::atomic<float>>(0.0f);

    auto opts = rclcpp_action::Client<
        mapper_interfaces::action::ExploreUnknown>::SendGoalOptions{};
    opts.feedback_callback = [coverage](auto, const auto & fb) {
        coverage->store(fb->coverage_percent);
    };
    opts.result_callback = [done, result_status](const auto & r) {
        result_status->store(r.result->status);
        done->store(true);
    };

    explore_client_->async_send_goal(goal, opts);
    while (!done->load() && rclcpp::ok()) {
        auto cur = state_.load();
        if (cur == MapperState::IDLE || cur == MapperState::PAUSED) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    coverage_percent_.store(coverage->load());  // H3: atomic store

    auto final_state = state_.load();
    if (final_state == MapperState::IDLE || final_state == MapperState::PAUSED) return;

    if (result_status->load() == 0) {
        transition_to(MapperState::LOOP_CLOSING);
        run_loop_closing();
    } else {
        log("Exploration failed");
        transition_to(MapperState::MAPPING_MANUAL);
    }
}

void MapperOrchestratorNode::run_mapping_auto() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;
    log("Auto mapping mode -- starting exploration");
    transition_to(MapperState::EXPLORING_UNKNOWN);
    run_exploring();
}

void MapperOrchestratorNode::run_loop_closing() {
    if (state_.load() == MapperState::IDLE || shutdown_requested_.load()) return;
    log("Waiting for loop closure...");
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && state_.load() != MapperState::IDLE) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= static_cast<long>(loop_closure_timeout_sec_)) {
            log("Loop closure timeout -- completing");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (state_.load() == MapperState::IDLE) return;
    transition_to(MapperState::COMPLETED);
    log("Mapping completed");
}

}  // namespace mapper
