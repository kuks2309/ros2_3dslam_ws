#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mapper_interfaces/action/explore_unknown.hpp>
#include <waypoint_interfaces/msg/segment.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <queue>
#include <mutex>

namespace mapper {

struct GridCell { int x, y; };
struct GridPath { std::vector<GridCell> cells; };

class ExplorationPlannerNode : public rclcpp::Node {
public:
    using ExploreAction = mapper_interfaces::action::ExploreUnknown;
    using Segment       = waypoint_interfaces::msg::Segment;
    using GoalHandle    = rclcpp_action::ServerGoalHandle<ExploreAction>;

    explicit ExplorationPlannerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // 테스트용 public 메서드
    std::vector<Segment> plan_segments(
        const nav_msgs::msg::OccupancyGrid & map,
        double robot_x, double robot_y, uint8_t mode);

    static GridPath bfs(
        const nav_msgs::msg::OccupancyGrid & map,
        GridCell start, GridCell goal);

    static std::vector<GridCell> find_nearest_unknown_cluster(
        const nav_msgs::msg::OccupancyGrid & map,
        GridCell robot_cell);

private:
    rclcpp_action::Server<ExploreAction>::SharedPtr server_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::mutex map_mutex_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double max_linear_speed_{0.2};
    double acceleration_{0.3};

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExploreAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
    void execute(std::shared_ptr<GoalHandle> goal_handle);

    std::vector<Segment> path_to_segments(
        const GridPath & path,
        const nav_msgs::msg::OccupancyGrid & map,
        double robot_yaw_deg);
};

}  // namespace mapper
