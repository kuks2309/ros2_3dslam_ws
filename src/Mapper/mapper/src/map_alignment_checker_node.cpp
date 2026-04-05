#include "mapper/map_alignment_checker_node.hpp"
#include <thread>
#include <cmath>
#include <algorithm>
#include <vector>

namespace mapper {

MapAlignmentCheckerNode::MapAlignmentCheckerNode(
    const rclcpp::NodeOptions & options)
: Node("map_alignment_checker_node", options)
{
    using namespace std::placeholders;

    // C1: 파라미터 선언 및 로드
    hough_rho_             = declare_parameter("hough_rho", 1.0);
    hough_theta_           = declare_parameter("hough_theta", 0.017453292);
    hough_threshold_       = static_cast<int>(declare_parameter("hough_threshold", 50));
    hough_min_line_length_ = declare_parameter("hough_min_line_length", 20.0);
    hough_max_line_gap_    = declare_parameter("hough_max_line_gap", 5.0);

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
    double /*tolerance_deg*/) const
{
    cv::Mat img = occupancy_grid_to_mat(map);
    if (img.empty()) return 0.0;

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img, lines,
        hough_rho_, hough_theta_, hough_threshold_,
        hough_min_line_length_, hough_max_line_gap_);

    if (lines.empty()) return 0.0;

    std::vector<double> errors;
    for (auto & l : lines) {
        double angle_rad = std::atan2(l[3] - l[1], l[2] - l[0]);
        double angle_deg = angle_rad * 180.0 / M_PI;
        double normalized = std::fmod(angle_deg + 360.0, 180.0);
        double err0  = std::min(normalized, 180.0 - normalized);
        double err90 = std::abs(normalized - 90.0);
        double err   = std::min(err0, err90);
        errors.push_back(err);
    }
    if (errors.empty()) return 0.0;
    std::sort(errors.begin(), errors.end());
    size_t p90_idx = static_cast<size_t>(errors.size() * 0.9);
    if (p90_idx >= errors.size()) p90_idx = errors.size() - 1;
    return errors[p90_idx];
}

cv::Mat MapAlignmentCheckerNode::occupancy_grid_to_mat(
    const nav_msgs::msg::OccupancyGrid & map) const
{
    if (map.info.width == 0 || map.info.height == 0) return {};
    if (map.data.size() < static_cast<size_t>(map.info.width * map.info.height)) {
        return cv::Mat();
    }
    cv::Mat img(map.info.height, map.info.width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < (int)map.info.height; ++y) {
        for (int x = 0; x < (int)map.info.width; ++x) {
            int8_t val = map.data[y * map.info.width + x];
            if (val > 50) img.at<uint8_t>(y, x) = 255;
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
    auto self = std::static_pointer_cast<MapAlignmentCheckerNode>(shared_from_this());
    std::thread([self, goal_handle]() {
        self->execute(goal_handle);
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
        if (goal_handle->is_canceling()) {
            result->is_aligned = false;
            result->max_wall_error_deg = -1.0;
            goal_handle->canceled(result);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (latest_map_) { map_copy = *latest_map_; break; }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (map_copy.info.width == 0 || map_copy.info.height == 0) {
        result->is_aligned = false;
        result->max_wall_error_deg = -1.0;
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "No map received within timeout");
        return;
    }

    if (goal_handle->is_canceling()) {
        result->is_aligned = false;
        result->max_wall_error_deg = -1.0;
        goal_handle->canceled(result);
        return;
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
