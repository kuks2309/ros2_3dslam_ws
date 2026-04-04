#include "mapper/map_alignment_checker_node.hpp"
#include <thread>
#include <cmath>

namespace mapper {

MapAlignmentCheckerNode::MapAlignmentCheckerNode(
    const rclcpp::NodeOptions & options)
: Node("map_alignment_checker_node", options)
{
    using namespace std::placeholders;

    // /map QoS: RELIABLE + TRANSIENT_LOCAL (OGM 표준)
    auto map_qos = rclcpp::QoS(3)
        .reliable()
        .transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            latest_map_ = msg;
        });

    auto cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    server_ = rclcpp_action::create_server<CheckAction>(
        this, "map_alignment_check",
        std::bind(&MapAlignmentCheckerNode::handle_goal, this, _1, _2),
        std::bind(&MapAlignmentCheckerNode::handle_cancel, this, _1),
        std::bind(&MapAlignmentCheckerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), cb_group);
}

double MapAlignmentCheckerNode::check_alignment(
    const nav_msgs::msg::OccupancyGrid & map,
    double /*tolerance_deg*/)
{
    cv::Mat img = occupancy_grid_to_mat(map);
    if (img.empty()) return 0.0;

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img, lines,
        hough_rho_, hough_theta_, hough_threshold_,
        hough_min_line_length_, hough_max_line_gap_);

    if (lines.empty()) return 0.0;

    double max_error = 0.0;
    for (auto & l : lines) {
        double dx = l[2] - l[0];
        double dy = l[3] - l[1];
        double angle = std::atan2(dy, dx) * 180.0 / M_PI;
        angle = std::fmod(angle + 180.0, 180.0);
        double err0  = std::min(angle, 180.0 - angle);
        double err90 = std::abs(angle - 90.0);
        double err   = std::min(err0, err90);
        max_error = std::max(max_error, err);
    }
    return max_error;
}

cv::Mat MapAlignmentCheckerNode::occupancy_grid_to_mat(
    const nav_msgs::msg::OccupancyGrid & map)
{
    if (map.info.width == 0 || map.info.height == 0) return {};
    cv::Mat img(map.info.height, map.info.width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < (int)map.info.height; ++y) {
        for (int x = 0; x < (int)map.info.width; ++x) {
            int8_t val = map.data[y * map.info.width + x];
            if (val >= 50) img.at<uint8_t>(y, x) = 255;
        }
    }
    return img;
}

rclcpp_action::GoalResponse MapAlignmentCheckerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CheckAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MapAlignmentCheckerNode::handle_cancel(
    std::shared_ptr<GoalHandle>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MapAlignmentCheckerNode::handle_accepted(
    std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread([this, goal_handle]() {
        execute(goal_handle);
    }).detach();
}

void MapAlignmentCheckerNode::execute(
    std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal     = goal_handle->get_goal();
    auto feedback = std::make_shared<CheckAction::Feedback>();
    auto result   = std::make_shared<CheckAction::Result>();

    // 최신 맵 대기 (최대 5초)
    nav_msgs::msg::OccupancyGrid map_copy;
    for (int i = 0; i < 50; ++i) {
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (latest_map_) { map_copy = *latest_map_; break; }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    double max_error = check_alignment(map_copy, goal->tolerance_deg);

    feedback->progress = 1.0;
    feedback->current_max_error_deg = max_error;
    goal_handle->publish_feedback(feedback);

    result->is_aligned      = (max_error <= goal->tolerance_deg);
    result->max_wall_error_deg = max_error;
    goal_handle->succeed(result);
}

}  // namespace mapper
