#include "mapper/wall_aligner_node.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <random>
#include <thread>
#include <chrono>

namespace mapper {

WallAlignerNode::WallAlignerNode(const rclcpp::NodeOptions & options)
: Node("wall_aligner_node", options)
{
    using namespace std::placeholders;

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // C1: 파라미터 선언 및 로드
    tolerance_deg_    = declare_parameter("tolerance_deg", 0.2);
    ransac_iterations_ = static_cast<int>(declare_parameter("ransac_iterations", 200));
    max_attempts_      = static_cast<int>(declare_parameter("max_attempts", 5));
    inlier_dist_m_    = declare_parameter("inlier_dist_m", 0.05);
    min_inliers_      = static_cast<int>(declare_parameter("min_inliers", 10));
    spin_speed_deg_s_ = declare_parameter("spin_speed_deg_s", 40.0);
    spin_accel_deg_s2_ = declare_parameter("spin_accel_deg_s2", 30.0);
    spin_server_name_  = declare_parameter("spin_server", std::string("spin"));

    // /scan QoS: BEST_EFFORT + VOLATILE (LiDAR 센서 표준)
    auto scan_qos = rclcpp::QoS(10)
        .best_effort()
        .durability_volatile();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", scan_qos,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            latest_scan_ = msg;
        });

    spin_client_ = rclcpp_action::create_client<SpinAction>(this, spin_server_name_);

    auto cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    wall_align_server_ = rclcpp_action::create_server<WallAlignAction>(
        this, "wall_align",
        std::bind(&WallAlignerNode::handle_goal, this, _1, _2),
        std::bind(&WallAlignerNode::handle_cancel, this, _1),
        std::bind(&WallAlignerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), cb_group);
}

Line2D WallAlignerNode::detect_longest_wall(
    const sensor_msgs::msg::LaserScan & scan)
{
    std::vector<std::pair<double,double>> pts;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (r < scan.range_min || r > scan.range_max || !std::isfinite(r)) continue;
        double angle = scan.angle_min + i * scan.angle_increment;
        pts.push_back({r * std::cos(angle), r * std::sin(angle)});
    }

    if (pts.size() < 2) return {0.0, 0};

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, (int)pts.size() - 1);

    Line2D best{0.0, 0};
    for (int iter = 0; iter < ransac_iterations_; ++iter) {
        int i1 = dist(rng), i2 = dist(rng);
        if (i1 == i2) continue;
        double dx = pts[i2].first  - pts[i1].first;
        double dy = pts[i2].second - pts[i1].second;
        double len = std::hypot(dx, dy);
        if (len < 0.1) continue;

        double a = dy / len, b = -dx / len;
        double c = -(a * pts[i1].first + b * pts[i1].second);

        int inliers = 0;
        for (auto & p : pts) {
            if (std::abs(a * p.first + b * p.second + c) < inlier_dist_m_) {
                ++inliers;
            }
        }
        if (inliers > best.inlier_count) {
            double raw_angle = std::atan2(dy, dx) * 180.0 / M_PI;
            double normalized = std::fmod(raw_angle + 180.0, 180.0);
            best = {normalized, inliers};
        }
    }
    if (best.inlier_count < min_inliers_) return {0.0, 0};
    return best;
}

double WallAlignerNode::get_robot_yaw_deg()
{
    try {
        auto tf = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero);
        const auto & q = tf.transform.rotation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF lookup failed: %s", ex.what());
        return 0.0;
    }
}

rclcpp_action::GoalResponse WallAlignerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WallAlignAction::Goal> goal)
{
    if (goal->tolerance_deg <= 0.0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WallAlignerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWallAlign>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WallAlignerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWallAlign> goal_handle)
{
    auto self = std::static_pointer_cast<WallAlignerNode>(shared_from_this());
    std::thread([self, goal_handle]() {
        self->execute(goal_handle);
    }).detach();
}

void WallAlignerNode::execute(
    const std::shared_ptr<GoalHandleWallAlign> goal_handle)
{
    auto goal     = goal_handle->get_goal();
    auto feedback = std::make_shared<WallAlignAction::Feedback>();
    auto result   = std::make_shared<WallAlignAction::Result>();

    std::atomic<bool> cancelled{false};

    for (int attempt = 0; attempt < max_attempts_; ++attempt) {
        if (goal_handle->is_canceling()) {
            result->status = -3;
            goal_handle->canceled(result);
            return;
        }

        sensor_msgs::msg::LaserScan scan_copy;
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (!latest_scan_) {
                RCLCPP_WARN(get_logger(), "No scan received yet, waiting...");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            scan_copy = *latest_scan_;
        }

        Line2D wall = detect_longest_wall(scan_copy);
        double robot_yaw = get_robot_yaw_deg();
        double error = robot_yaw - wall.angle_deg;
        while (error >  90.0) error -= 180.0;
        while (error < -90.0) error += 180.0;

        feedback->current_error_deg = error;
        feedback->wall_angle_deg    = wall.angle_deg;
        goal_handle->publish_feedback(feedback);

        if (std::abs(error) <= goal->tolerance_deg) {
            result->aligned_heading      = robot_yaw;
            result->status               = 0;
            result->current_error_deg    = error;
            result->wall_angle_deg       = wall.angle_deg;
            goal_handle->succeed(result);
            return;
        }

        double target = robot_yaw - error;
        if (!send_spin_and_wait(target, cancelled)) {
            if (goal_handle->is_canceling()) {
                result->status = -3;
                goal_handle->canceled(result);
            } else {
                result->status = -1;
                goal_handle->abort(result);
            }
            return;
        }
    }

    result->status = -1;
    goal_handle->abort(result);
}

bool WallAlignerNode::send_spin_and_wait(
    double target_angle_deg,
    std::atomic<bool> & cancelled)
{
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(get_logger(), "SpinAction server not available");
        return false;
    }

    auto goal = SpinAction::Goal{};
    goal.target_angle         = target_angle_deg;
    goal.max_angular_speed    = spin_speed_deg_s_;
    goal.angular_acceleration = spin_accel_deg_s2_;
    goal.hold_steer           = false;
    goal.exit_steer_angle     = 0.0;

    auto done    = std::make_shared<std::atomic<bool>>(false);
    auto success = std::make_shared<std::atomic<bool>>(false);

    auto opts = rclcpp_action::Client<SpinAction>::SendGoalOptions{};
    opts.result_callback =
        [done, success, &cancelled](const auto & wrapped) {
            // SpinAction: status 0=success, -1=cancelled
            success->store(wrapped.result->status == 0);
            if (wrapped.result->status == -1) cancelled.store(true);
            done->store(true);
        };

    auto goal_handle_future = spin_client_->async_send_goal(goal, opts);
    // wait up to 5s for goal acceptance
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Spin goal send timed out");
        return false;
    }
    if (!goal_handle_future.get()) {
        RCLCPP_ERROR(get_logger(), "Spin goal was rejected");
        return false;
    }
    while (!done->load() && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return success->load();
}

}  // namespace mapper
