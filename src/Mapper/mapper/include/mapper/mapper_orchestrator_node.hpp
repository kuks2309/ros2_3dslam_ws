#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mapper_interfaces/msg/mapper_status.hpp>
#include <mapper_interfaces/srv/mapper_command.hpp>
#include <mapper_interfaces/srv/slam_control.hpp>
#include <mapper_interfaces/action/wall_align.hpp>
#include <mapper_interfaces/action/map_alignment_check.hpp>
#include <mapper_interfaces/action/explore_unknown.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace mapper {

enum class MapperState : uint8_t {
    IDLE              = 0,
    ALIGNING          = 1,
    STARTING_SLAM     = 2,
    VERIFYING_MAP     = 3,
    MAPPING_MANUAL    = 4,
    MAPPING_AUTO      = 5,
    EXPLORING_UNKNOWN = 6,
    LOOP_CLOSING      = 7,
    COMPLETED         = 8,
    ERROR             = 9,
    PAUSED            = 10
};

class MapperOrchestratorNode : public rclcpp::Node {
public:
    explicit MapperOrchestratorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    MapperState get_state() const { return state_.load(); }

    // public for testing
    void handle_command_public(
        const mapper_interfaces::srv::MapperCommand::Request::SharedPtr req,
        mapper_interfaces::srv::MapperCommand::Response::SharedPtr res) {
        handle_command(req, res);
    }

private:
    std::atomic<MapperState> state_{MapperState::IDLE};
    std::atomic<MapperState> previous_state_{MapperState::IDLE};
    std::mutex  state_mutex_;
    std::atomic<int>    align_retry_count_{0};
    std::atomic<int>    total_realign_count_{0};
    const int           max_total_realigns_{10};
    std::atomic<float>  coverage_percent_{0.0f};
    std::atomic<double> heading_error_deg_{0.0};

    // set once in constructor, treated as const
    int    max_align_retries_{3};
    double map_stabilize_wait_sec_{3.0};
    double loop_closure_timeout_sec_{30.0};
    std::atomic<float>   min_coverage_to_stop_{0.95f};
    std::atomic<uint8_t> slam_mode_{0};
    std::atomic<uint8_t> drive_mode_{0};
    double start_x_{0.0}, start_y_{0.0};

    rclcpp::Service<mapper_interfaces::srv::MapperCommand>::SharedPtr cmd_service_;
    rclcpp::Publisher<mapper_interfaces::msg::MapperStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    rclcpp_action::Client<mapper_interfaces::action::WallAlign>::SharedPtr wall_align_client_;
    rclcpp_action::Client<mapper_interfaces::action::MapAlignmentCheck>::SharedPtr map_check_client_;
    rclcpp_action::Client<mapper_interfaces::action::ExploreUnknown>::SharedPtr explore_client_;
    rclcpp::Client<mapper_interfaces::srv::SlamControl>::SharedPtr slam_ctrl_client_2d_;
    rclcpp::Client<mapper_interfaces::srv::SlamControl>::SharedPtr slam_ctrl_client_3d_;

    void transition_to(MapperState new_state);
    void transition_to_unlocked(MapperState new_state);
    void run_aligning();
    void run_starting_slam();
    void run_verifying_map();
    void run_mapping_manual();
    void run_loop_closing();
    void run_exploring();

    void handle_command(
        const mapper_interfaces::srv::MapperCommand::Request::SharedPtr req,
        mapper_interfaces::srv::MapperCommand::Response::SharedPtr res);

    void publish_status();
    void log(const std::string & msg);

    // LOCK ORDERING: state_mutex_ -> log_mutex_ (never reverse)
    mutable std::mutex log_mutex_;
    std::string log_message_;
};

}  // namespace mapper
