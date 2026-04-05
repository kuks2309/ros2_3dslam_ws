#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mapper_interfaces/action/map_alignment_check.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace mapper {

class MapAlignmentCheckerNode : public rclcpp::Node {
public:
    using CheckAction = mapper_interfaces::action::MapAlignmentCheck;
    using GoalHandle  = rclcpp_action::ServerGoalHandle<CheckAction>;

    explicit MapAlignmentCheckerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // 테스트용 public 메서드
    double check_alignment(const nav_msgs::msg::OccupancyGrid & map,
                           double tolerance_deg) const;

private:
    rclcpp_action::Server<CheckAction>::SharedPtr server_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::mutex map_mutex_;

    double hough_rho_{1.0};
    double hough_theta_{0.01745};  // ~1도
    int    hough_threshold_{50};
    double hough_min_line_length_{20.0};
    double hough_max_line_gap_{5.0};

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CheckAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
    void execute(std::shared_ptr<GoalHandle> goal_handle);
    cv::Mat occupancy_grid_to_mat(const nav_msgs::msg::OccupancyGrid & map) const;
};

}  // namespace mapper
