#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mapper_interfaces/action/wall_align.hpp>
#include <amr_interfaces/action/amr_motion_spin.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <mutex>
#include <vector>

namespace mapper {

struct Line2D {
    double angle_deg;  // 0~180도 정규화
    int    inlier_count;
};

class WallAlignerNode : public rclcpp::Node {
public:
    using WallAlignAction = mapper_interfaces::action::WallAlign;
    using SpinAction      = amr_interfaces::action::AMRMotionSpin;
    using GoalHandleWallAlign = rclcpp_action::ServerGoalHandle<WallAlignAction>;

    explicit WallAlignerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // 테스트용 public
    Line2D detect_longest_wall(const sensor_msgs::msg::LaserScan & scan);

private:
    rclcpp_action::Server<WallAlignAction>::SharedPtr wall_align_server_;
    rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::mutex scan_mutex_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double tolerance_deg_{0.2};
    int    max_iterations_{100};
    double inlier_dist_m_{0.05};
    int    min_inliers_{10};
    double spin_speed_deg_s_{40.0};
    double spin_accel_deg_s2_{30.0};
    std::string spin_server_name_{"spin"};

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const WallAlignAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWallAlign> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleWallAlign> goal_handle);
    void execute(const std::shared_ptr<GoalHandleWallAlign> goal_handle);

    double get_robot_yaw_deg();
    bool send_spin_and_wait(double target_angle_deg, std::atomic<bool> & cancelled);
};

}  // namespace mapper
