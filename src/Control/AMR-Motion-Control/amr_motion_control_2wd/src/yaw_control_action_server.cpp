#include "amr_motion_control_2wd/yaw_control_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/localization_watchdog.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace amr_motion_control_2wd
{

YawControlActionServer::YawControlActionServer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  control_rate_hz_          = safeParam(node_, "yaw_ctrl.control_rate_hz",    20.0);
  max_timeout_sec_          = safeParam(node_, "yaw_ctrl.max_timeout_sec",     60.0);
  Kp_heading_               = safeParam(node_, "yaw_ctrl.Kp_heading",           1.0);
  Kd_heading_               = safeParam(node_, "yaw_ctrl.Kd_heading",           0.3);
  max_omega_                = safeParam(node_, "yaw_ctrl.max_omega",             1.0);
  min_turning_radius_       = safeParam(node_, "yaw_ctrl.min_turning_radius",    0.7);
  omega_rate_limit_         = safeParam(node_, "yaw_ctrl.omega_rate_limit",      0.5);
  walk_accel_limit_         = safeParam(node_, "yaw_ctrl.walk_accel_limit",      0.5);
  walk_decel_limit_         = safeParam(node_, "yaw_ctrl.walk_decel_limit",      1.0);
  robot_base_frame_         = safeParam(node_, "robot_base_frame", std::string("base_footprint"));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::imuCallback, this, _1));

  std::string pose_topic = safeParam(node_, "yaw_control_pose_topic", std::string("/rtabmap/odom"));
  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    pose_topic, rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::poseCallback, this, _1));

  // Stop service: graceful deceleration
  stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "yaw_control_stop",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      stop_requested_.store(true);
      res->success = true;
      res->message = "Deceleration stop requested";
      RCLCPP_INFO(node_->get_logger(), "YawControlActionServer: stop service called — decelerating");
    });

  action_server_ = rclcpp_action::create_server<YawControl>(
    node_, "amr_motion_yaw_control",
    std::bind(&YawControlActionServer::handle_goal,     this, _1, _2),
    std::bind(&YawControlActionServer::handle_cancel,   this, _1),
    std::bind(&YawControlActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(),
    "YawControlActionServer ready (stop service: /yaw_control_stop)");
}

void YawControlActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

void YawControlActionServer::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  if (watchdog_) { watchdog_->updatePose(x, y, yaw); }
}

bool YawControlActionServer::lookupRobotPose(double & x, double & y, double & yaw) const
{
  try {
    auto tf = tf_buffer_->lookupTransform("map", robot_base_frame_,
                                          tf2::TimePointZero,
                                          tf2::durationFromSec(0.1));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "YawControlActionServer: TF lookup failed: %s", ex.what());
    return false;
  }
}

void YawControlActionServer::publishCmdVel(double vx, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = vx;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

rclcpp_action::GoalResponse YawControlActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const YawControl::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YawControlActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleYaw> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void YawControlActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleYaw> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void YawControlActionServer::execute(const std::shared_ptr<GoalHandleYaw> goal_handle)
{
  // ── Mutual exclusion: only one action at a time ───────────────────────────
  ActiveAction expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::YAW_CONTROL)) {
    RCLCPP_WARN(node_->get_logger(),
      "YawControlActionServer: another action is active (%s), aborting new goal",
      to_string(g_active_action.load()));
    auto result = std::make_shared<YawControl::Result>();
    result->status = -2;
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControlActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }
  ActionGuard action_guard;

  const auto & goal = goal_handle->get_goal();
  const auto start_time = node_->now();

  // Reset stop flag
  stop_requested_.store(false);

  // ── 1. Validate parameters ────────────────────────────────────────────────
  if (goal->max_linear_speed <= 0.0 || goal->acceleration <= 0.0 ||
      goal->deceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "YawControlActionServer: invalid params (speed=%.3f accel=%.3f decel=%.3f)",
      goal->max_linear_speed, goal->acceleration, goal->deceleration);
    auto result = std::make_shared<YawControl::Result>();
    result->status = -2;
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControlActionServer: abort failed: %s", e.what());
    }
    return;
  }

  const double target_heading_rad = goal->target_heading * M_PI / 180.0;

  RCLCPP_INFO(node_->get_logger(),
    "YawControlActionServer: heading=%.1f deg, v_max=%.3f, accel=%.3f, decel=%.3f",
    goal->target_heading, goal->max_linear_speed,
    goal->acceleration, goal->deceleration);

  // ── 2. Localization watchdog ──────────────────────────────────────────────
  LocalizationWatchdog::Config wd_cfg;
  wd_cfg.timeout_sec          = 2.0;
  wd_cfg.fixed_jump_threshold = 0.5;
  wd_cfg.velocity_margin      = 1.3;
  watchdog_.emplace(wd_cfg, node_->get_logger());

  // ── 3. Wait for pose (5 s timeout) ───────────────────────────────────────
  {
    rclcpp::Rate wait_rate(20);
    int wait_count = 0;
    while (rclcpp::ok() && !watchdog_->poseReceived()) {
      wait_rate.sleep();
      if (++wait_count > 100) {
        RCLCPP_ERROR(node_->get_logger(),
          "YawControlActionServer: no pose received after 5 s, aborting");
        publishCmdVel(0.0, 0.0);
        auto result = std::make_shared<YawControl::Result>();
        result->status       = -3;
        result->elapsed_time = (node_->now() - start_time).seconds();
        try { goal_handle->abort(result); } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(),
            "YawControlActionServer: abort failed: %s", e.what());
        }
        return;
      }
    }
  }

  // ── 4. Get initial pose for distance tracking ─────────────────────────────
  double rob_x = 0.0, rob_y = 0.0, rob_yaw = 0.0;
  if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
    rob_x   = watchdog_->x();
    rob_y   = watchdog_->y();
    rob_yaw = watchdog_->yaw();
  }
  double prev_x = rob_x;
  double prev_y = rob_y;
  double total_distance = 0.0;

  RCLCPP_INFO(node_->get_logger(),
    "YawControlActionServer: initial pose=(%.3f, %.3f, %.1f deg)",
    rob_x, rob_y, rob_yaw * 180.0 / M_PI);

  // ── 5. Control loop ───────────────────────────────────────────────────────
  rclcpp::Rate rate(control_rate_hz_);
  const double dt = 1.0 / control_rate_hz_;

  double prev_e_theta = 0.0;
  double prev_omega   = 0.0;
  double prev_vx      = 0.0;
  bool   decelerating = false;

  while (rclcpp::ok()) {
    // ── a. Cancel check → immediate stop ──────────────────────────────────
    if (goal_handle->is_canceling()) {
      publishCmdVel(0.0, 0.0);
      auto result = std::make_shared<YawControl::Result>();
      result->status          = -1;  // cancelled
      result->total_distance  = total_distance;
      result->final_heading_error =
        normalizeAngle(rob_yaw - target_heading_rad) * 180.0 / M_PI;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->canceled(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: canceled failed: %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(),
        "YawControlActionServer: cancelled (dist=%.3f m)", total_distance);
      return;
    }

    // ── b. Stop service check → start deceleration ────────────────────────
    if (stop_requested_.load() && !decelerating) {
      decelerating = true;
      RCLCPP_INFO(node_->get_logger(),
        "YawControlActionServer: deceleration started (decel=%.3f m/s^2)",
        goal->deceleration);
    }

    // ── c. Timeout check ──────────────────────────────────────────────────
    const double elapsed = (node_->now() - start_time).seconds();
    if (elapsed > max_timeout_sec_) {
      publishCmdVel(0.0, 0.0);
      RCLCPP_ERROR(node_->get_logger(),
        "YawControlActionServer: timeout after %.1f s", elapsed);
      auto result = std::make_shared<YawControl::Result>();
      result->status          = -3;
      result->total_distance  = total_distance;
      result->elapsed_time    = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: abort failed: %s", e.what());
      }
      return;
    }

    // ── d. Localization watchdog ───────────────────────────────────────────
    if (enable_localization_watchdog_ && !watchdog_->checkHealth()) {
      publishCmdVel(0.0, 0.0);
      RCLCPP_ERROR(node_->get_logger(),
        "YawControlActionServer: localization watchdog triggered");
      auto result = std::make_shared<YawControl::Result>();
      result->status          = -4;
      result->total_distance  = total_distance;
      result->elapsed_time    = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: abort failed: %s", e.what());
      }
      return;
    }

    // ── e. Get robot pose ─────────────────────────────────────────────────
    if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
      rob_x   = watchdog_->x();
      rob_y   = watchdog_->y();
      rob_yaw = watchdog_->yaw();
    }

    // ── f. Accumulate distance ────────────────────────────────────────────
    total_distance += std::hypot(rob_x - prev_x, rob_y - prev_y);
    prev_x = rob_x;
    prev_y = rob_y;

    // ── f2. Auto-stop when stop_distance reached ──────────────────────────
    if (goal->stop_distance > 0.0 &&
        total_distance >= goal->stop_distance &&
        !stop_requested_.load())
    {
      stop_requested_.store(true);
      RCLCPP_INFO(node_->get_logger(),
        "YawControlActionServer: stop_distance=%.3f m reached (traveled=%.3f m) — decelerating",
        goal->stop_distance, total_distance);
    }

    // ── g. Compute target vx ──────────────────────────────────────────────
    double vx_target;
    uint8_t phase;

    if (decelerating) {
      // Decelerate to zero
      vx_target = 0.0;
      phase = 3;  // DECEL
    } else if (prev_vx < goal->max_linear_speed) {
      // Accelerating
      vx_target = goal->max_linear_speed;
      phase = 1;  // ACCEL
    } else {
      // Cruising
      vx_target = goal->max_linear_speed;
      phase = 2;  // CRUISE
    }

    // ── h. Walk velocity smoother ─────────────────────────────────────────
    double vx = vx_target;
    if (decelerating) {
      // Use goal deceleration rate
      const double decel_step = goal->deceleration * dt;
      vx = std::max(0.0, prev_vx - decel_step);
    } else if (vx > prev_vx) {
      const double accel_step = goal->acceleration * dt;
      vx = std::min(vx, prev_vx + accel_step);
    }
    prev_vx = vx;

    // ── i. Deceleration complete → succeed ────────────────────────────────
    if (decelerating && vx < 1e-3) {
      publishCmdVel(0.0, 0.0);
      auto result = std::make_shared<YawControl::Result>();
      result->status          = 0;  // stopped (success)
      result->total_distance  = total_distance;
      result->final_heading_error =
        normalizeAngle(rob_yaw - target_heading_rad) * 180.0 / M_PI;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->succeed(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: succeed failed: %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(),
        "YawControlActionServer: stopped (dist=%.3f m, hdg_err=%.2f deg, t=%.2f s)",
        total_distance, result->final_heading_error, result->elapsed_time);
      return;
    }

    // ── j. Heading error PD control ───────────────────────────────────────
    const double e_theta = normalizeAngle(rob_yaw - target_heading_rad);
    const double de_theta = (e_theta - prev_e_theta) / dt;
    double omega = -Kp_heading_ * e_theta - Kd_heading_ * de_theta;
    prev_e_theta = e_theta;

    // ── k. Clamp omega ────────────────────────────────────────────────────
    omega = std::clamp(omega, -max_omega_, max_omega_);
    if (vx > 1e-6 && min_turning_radius_ > 1e-6) {
      const double omega_radius_limit = vx / min_turning_radius_;
      omega = std::clamp(omega, -omega_radius_limit, omega_radius_limit);
    }

    // ── l. Omega rate limit ───────────────────────────────────────────────
    const double omega_delta_max = omega_rate_limit_ * dt;
    omega = std::clamp(omega,
                       prev_omega - omega_delta_max,
                       prev_omega + omega_delta_max);
    prev_omega = omega;

    // ── m. Publish velocity ───────────────────────────────────────────────
    watchdog_->setCurrentSpeed(std::abs(vx));
    publishCmdVel(vx, omega);

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "YawCtrl: pos=(%.2f,%.2f) hdg=%.1f° err=%.1f° vx=%.3f ω=%.3f dist=%.2f phase=%u",
      rob_x, rob_y, rob_yaw * 180.0 / M_PI,
      e_theta * 180.0 / M_PI, vx, omega, total_distance, phase);

    // ── n. Publish feedback ───────────────────────────────────────────────
    {
      auto feedback = std::make_shared<YawControl::Feedback>();
      feedback->current_heading_error = e_theta * 180.0 / M_PI;
      feedback->current_vx            = vx;
      feedback->current_omega         = omega;
      feedback->phase                 = phase;
      goal_handle->publish_feedback(feedback);
    }

    rate.sleep();
  }

  // rclcpp::ok() went false — shutdown
  publishCmdVel(0.0, 0.0);
}

}  // namespace amr_motion_control_2wd
