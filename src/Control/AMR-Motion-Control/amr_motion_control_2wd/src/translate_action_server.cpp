#include "amr_motion_control_2wd/translate_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace amr_motion_control_2wd
{

// ─── Helper: safe parameter read ──────────────────────────────────────────────
template <typename T>
static T safeParam(rclcpp::Node::SharedPtr node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

// ─── Constructor ──────────────────────────────────────────────────────────────
TranslateActionServer::TranslateActionServer(rclcpp::Node::SharedPtr node)
: node_(node),
  e_theta_filter_(5)
{
  // ── Control parameters
  ctrl_freq_hz_           = safeParam(node_, "ctrl_freq_hz",            50.0);
  wheel_base_             = safeParam(node_, "wheel_base",               0.54);
  wheel_radius_           = safeParam(node_, "wheel_radius",             0.1);
  max_wheel_rpm_          = safeParam(node_, "max_wheel_rpm",          100.0);
  stanley_k_              = safeParam(node_, "stanley_k",               0.8);
  stanley_softening_      = safeParam(node_, "stanley_softening",       0.3);
  pd_kp_                  = safeParam(node_, "pd_heading_kp",           2.0);
  pd_kd_                  = safeParam(node_, "pd_heading_kd",           0.1);
  omega_smoother_alpha_   = safeParam(node_, "omega_smoother_alpha",    0.4);
  max_omega_              = safeParam(node_, "max_omega_rad_s",         1.5);
  arrive_dist_            = safeParam(node_, "translate_goal_reach_threshold", 0.05);
  lateral_recover_dist_   = safeParam(node_, "lateral_recover_dist_m",  0.8);
  watchdog_timeout_sec_   = safeParam(node_, "watchdog_timeout_sec",    2.0);
  watchdog_jump_threshold_= safeParam(node_, "watchdog_jump_threshold", 0.5);
  watchdog_velocity_margin_= safeParam(node_, "watchdog_velocity_margin",1.3);
  robot_base_frame_       = safeParam(node_, "robot_base_frame",  std::string("base_footprint"));

  // ── TF2
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Publishers
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "translate_path", rclcpp::QoS(10).transient_local());

  // ── Subscribers
  auto imu_qos  = rclcpp::SensorDataQoS();
  auto pose_qos = rclcpp::SensorDataQoS();

  std::string imu_topic  = safeParam(node_, "imu_topic",          std::string("/imu/data"));
  std::string pose_topic = safeParam(node_, "translate_pose_topic", std::string("/rtabmap/odom"));

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, imu_qos,
    std::bind(&TranslateActionServer::imuCallback, this, _1));

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    pose_topic, pose_qos,
    std::bind(&TranslateActionServer::odomPoseCallback, this, _1));

  safety_speed_limit_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "safety_speed_limit", 10,
    std::bind(&TranslateActionServer::safetySpeedLimitCallback, this, _1));

  safety_status_sub_ = node_->create_subscription<amr_interfaces::msg::SafetyStatus>(
    "safety_status", 10,
    std::bind(&TranslateActionServer::safetyStatusCallback, this, _1));

  // ── Dynamic endpoint update service
  update_endpoint_srv_ = node_->create_service<UpdateEndpoint>(
    "update_translate_endpoint",
    [this](
      const std::shared_ptr<UpdateEndpoint::Request>  req,
      const std::shared_ptr<UpdateEndpoint::Response> res)
    {
      new_end_x_.store(req->end_x);
      new_end_y_.store(req->end_y);
      new_has_next_.store(req->has_next);
      endpoint_update_pending_.store(true);
      res->success = true;
      res->message = "Endpoint update queued";
      RCLCPP_INFO(node_->get_logger(),
        "TranslateActionServer: endpoint update queued -> (%.3f, %.3f)",
        req->end_x, req->end_y);
    });

  // ── Action server
  action_server_ = rclcpp_action::create_server<Translate>(
    node_,
    "amr_motion_translate",
    std::bind(&TranslateActionServer::handleGoal,     this, _1, _2),
    std::bind(&TranslateActionServer::handleCancel,   this, _1),
    std::bind(&TranslateActionServer::handleAccepted, this, _1));

  RCLCPP_INFO(node_->get_logger(),
    "TranslateActionServer ready (base_frame=%s, freq=%.0f Hz)",
    robot_base_frame_.c_str(), ctrl_freq_hz_);
}

// ─── Sensor callbacks ─────────────────────────────────────────────────────────
void TranslateActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->orientation.x, msg->orientation.y,
    msg->orientation.z, msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

void TranslateActionServer::odomPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  if (watchdog_) {
    watchdog_->updatePose(x, y, yaw);
  }
}

void TranslateActionServer::safetySpeedLimitCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  safety_speed_limit_.store(msg->data);
}

void TranslateActionServer::safetyStatusCallback(
  const amr_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  safety_state_.store(msg->status);
}

// ─── Action callbacks ─────────────────────────────────────────────────────────
rclcpp_action::GoalResponse TranslateActionServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Translate::Goal> goal)
{
  if (std::abs(goal->max_linear_speed) < 1e-6 || goal->acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "TranslateActionServer: invalid params (speed=%.3f accel=%.3f)",
      goal->max_linear_speed, goal->acceleration);
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(node_->get_logger(),
    "TranslateActionServer: goal received (%.3f,%.3f)->(%.3f,%.3f) v=%.3f",
    goal->start_x, goal->start_y, goal->end_x, goal->end_y,
    goal->max_linear_speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranslateActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "TranslateActionServer: cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranslateActionServer::handleAccepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&TranslateActionServer::execute, this, goal_handle)}.detach();
}

// ─── Main execution loop ──────────────────────────────────────────────────────
void TranslateActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto & goal = goal_handle->get_goal();
  auto start_time   = node_->now();

  // ── Validate path
  double dx = goal->end_x - goal->start_x;
  double dy = goal->end_y - goal->start_y;
  double path_len = std::hypot(dx, dy);

  if (path_len < 1e-4) {
    RCLCPP_WARN(node_->get_logger(),
      "TranslateActionServer: path length near zero (%.4f m)", path_len);
    auto result = std::make_shared<Translate::Result>();
    result->status           = -2;  // invalid_param
    result->actual_distance  = 0.0;
    result->elapsed_time     = 0.0;
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }

  // ── Init path geometry
  path_start_x_    = goal->start_x;
  path_start_y_    = goal->start_y;
  path_end_x_      = goal->end_x;
  path_end_y_      = goal->end_y;
  theta_path_      = std::atan2(dy, dx);
  target_distance_ = path_len;
  path_ux_         = dx / path_len;
  path_uy_         = dy / path_len;
  prev_e_theta_    = 0.0;
  prev_omega_      = 0.0;
  e_theta_filter_.reset();

  bool has_next   = goal->has_next;
  double end_x    = goal->end_x;
  double end_y    = goal->end_y;
  double end_dist = target_distance_;

  // ── Trapezoidal speed profile
  double sign_v = (goal->max_linear_speed >= 0.0) ? 1.0 : -1.0;
  double exit_v = (has_next) ? std::abs(goal->exit_speed) : 0.0;
  amr_motion_control::TrapezoidalProfile profile(
    target_distance_,
    std::abs(goal->max_linear_speed),
    goal->acceleration,
    exit_v);

  // ── Watchdog
  LocalizationWatchdog::Config wd_cfg;
  wd_cfg.timeout_sec          = watchdog_timeout_sec_;
  wd_cfg.fixed_jump_threshold = watchdog_jump_threshold_;
  wd_cfg.velocity_margin      = watchdog_velocity_margin_;
  watchdog_.emplace(wd_cfg, node_->get_logger());

  // Wait for first pose
  {
    rclcpp::Rate wait_rate(20);
    int wait_count = 0;
    while (rclcpp::ok() && !watchdog_->poseReceived()) {
      if (!rclcpp::ok()) break;
      wait_rate.sleep();
      if (++wait_count > 100) {  // 5 s
        RCLCPP_ERROR(node_->get_logger(),
          "TranslateActionServer: no pose received after 5 s, aborting");
        publishCmdVel(0.0, 0.0);
        auto result = std::make_shared<Translate::Result>();
        result->status      = -3;  // timeout
        result->elapsed_time =
          (node_->now() - start_time).seconds();
        try { goal_handle->abort(result); } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: abort failed (stale handle): %s", e.what());
        }
        return;
      }
    }
  }

  // ── Visualise path
  publishPathMarker(goal_handle);

  // ── Control loop
  rclcpp::Rate rate(ctrl_freq_hz_);
  const double dt = 1.0 / ctrl_freq_hz_;

  double traveled_dist = 0.0;
  double rob_x = 0.0, rob_y = 0.0, rob_yaw = 0.0;

  // Snapshot initial robot position from TF2 (map frame)
  if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
    rob_x   = watchdog_->x();
    rob_y   = watchdog_->y();
    rob_yaw = watchdog_->yaw();
  }
  RCLCPP_INFO(node_->get_logger(),
    "TranslateActionServer: initial pose=(%.3f, %.3f, %.3f rad) "
    "path=(%.3f,%.3f)->(%.3f,%.3f) len=%.3f m arrive_dist=%.3f m",
    rob_x, rob_y, rob_yaw,
    path_start_x_, path_start_y_, path_end_x_, path_end_y_,
    target_distance_, arrive_dist_);

  // Reference origin for distance projection
  double ref_x = path_start_x_;
  double ref_y = path_start_y_;

  while (rclcpp::ok()) {
    if (!rclcpp::ok()) break;

    // ── Cancellation check
    if (goal_handle->is_canceling()) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<Translate::Result>();
      result->status          = -1;  // cancelled
      result->actual_distance = traveled_dist;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->canceled(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: canceled failed (stale handle): %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(), "TranslateActionServer: cancelled");
      return;
    }

    // ── Dynamic endpoint update
    if (endpoint_update_pending_.exchange(false)) {
      handleEndpointUpdate(end_x, end_y, has_next);
      // Recompute path geometry for new endpoint
      double new_dx   = end_x - path_start_x_;
      double new_dy   = end_y - path_start_y_;
      double new_len  = std::hypot(new_dx, new_dy);
      if (new_len > 1e-4) {
        end_dist     = new_len;
        path_end_x_  = end_x;
        path_end_y_  = end_y;
        theta_path_  = std::atan2(new_dy, new_dx);
        path_ux_     = new_dx / new_len;
        path_uy_     = new_dy / new_len;
        // Rebuild profile from current position
        double remaining = std::max(0.0, end_dist - traveled_dist);
        exit_v = has_next ? std::abs(goal->exit_speed) : 0.0;
        profile = amr_motion_control::TrapezoidalProfile(
          remaining,
          std::abs(goal->max_linear_speed),
          goal->acceleration,
          exit_v);
        ref_x = rob_x;
        ref_y = rob_y;
        RCLCPP_INFO(node_->get_logger(),
          "TranslateActionServer: endpoint updated -> (%.3f, %.3f)", end_x, end_y);
      }
    }

    // ── Localization watchdog
    if (!watchdog_->checkHealth()) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<Translate::Result>();
      result->status          = -3;  // timeout
      result->actual_distance = traveled_dist;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: abort failed (stale handle): %s", e.what());
      }
      RCLCPP_ERROR(node_->get_logger(),
        "TranslateActionServer: localization watchdog triggered, aborting");
      return;
    }

    // ── Safety stop
    if (safety_state_.load() == amr_interfaces::msg::SafetyStatus::STATUS_DANGEROUS) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<Translate::Result>();
      result->status          = -4;  // safety_stop
      result->actual_distance = traveled_dist;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: abort failed (stale handle): %s", e.what());
      }
      RCLCPP_WARN(node_->get_logger(),
        "TranslateActionServer: safety stop triggered");
      return;
    }

    // ── Get current robot pose (TF2: map → base_footprint)
    if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
      // Use watchdog fallback if TF lookup fails
      rob_x   = watchdog_->x();
      rob_y   = watchdog_->y();
      rob_yaw = watchdog_->yaw();
    }

    // ── Project onto path: distance along path from ref
    double dx_r = rob_x - ref_x;
    double dy_r = rob_y - ref_y;
    double proj  = dx_r * path_ux_ + dy_r * path_uy_;
    traveled_dist = std::max(0.0, proj);

    // ── Cross-track error (positive = robot left of path)
    double e_lateral = -(dx_r * path_uy_ - dy_r * path_ux_);

    // ── Remaining distance to new endpoint
    double remaining = std::max(0.0, end_dist - traveled_dist);

    // ── Arrival check
    if (remaining <= arrive_dist_) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      double e_head_deg = normalizeAngle(rob_yaw - theta_path_) * 180.0 / M_PI;
      auto result = std::make_shared<Translate::Result>();
      result->status               = 0;
      result->actual_distance      = traveled_dist;
      result->final_lateral_error  = e_lateral;
      result->final_heading_error  = e_head_deg;
      result->elapsed_time         = (node_->now() - start_time).seconds();
      try { goal_handle->succeed(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: succeed failed (stale handle): %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(),
        "TranslateActionServer: arrived (dist=%.3f m, lat=%.3f m, t=%.2f s)",
        traveled_dist, e_lateral, result->elapsed_time);
      return;
    }

    // ── Lateral error too large
    if (std::abs(e_lateral) > lateral_recover_dist_) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<Translate::Result>();
      result->status          = -3;  // treat as timeout/fault
      result->actual_distance = traveled_dist;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "TranslateActionServer: abort failed (stale handle): %s", e.what());
      }
      RCLCPP_ERROR(node_->get_logger(),
        "TranslateActionServer: lateral error %.3f m > %.3f m limit, aborting",
        e_lateral, lateral_recover_dist_);
      return;
    }

    // ── Speed profile
    double profile_pos = std::min(traveled_dist, end_dist);
    auto   profile_out = profile.getSpeed(profile_pos);
    double vx_des      = sign_v * profile_out.speed;

    // Minimum start speed to overcome static friction / zero-speed deadlock
    constexpr double kMinVx = 0.02;  // m/s
    if (profile_out.phase != amr_motion_control::ProfilePhase::DONE &&
        std::abs(vx_des) < kMinVx && remaining > arrive_dist_) {
      vx_des = sign_v * kMinVx;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "TranslateActionServer: rob=(%.3f,%.3f,%.3fdeg) traveled=%.3f remaining=%.3f vx_des=%.3f e_lat=%.3f",
      rob_x, rob_y, rob_yaw * 180.0 / M_PI, traveled_dist, remaining, vx_des, e_lateral);

    // Apply safety speed limit
    double spd_limit = safety_speed_limit_.load();
    if (std::abs(vx_des) > spd_limit) {
      vx_des = sign_v * spd_limit;
    }

    // ── Heading error
    double e_theta = normalizeAngle(theta_path_ - rob_yaw);
    // for reverse travel, flip heading reference
    if (sign_v < 0.0) {
      e_theta = normalizeAngle(theta_path_ + M_PI - rob_yaw);
    }
    double e_theta_filt = e_theta_filter_.update(e_theta);

    // ── Stanley lateral correction
    double stanley_correction = 0.0;
    double speed_abs = std::abs(vx_des);
    if (speed_abs > 1e-3) {
      stanley_correction = std::atan2(
        stanley_k_ * e_lateral,
        speed_abs + stanley_softening_);
    }

    // ── PD heading control
    double de_theta = (e_theta_filt - prev_e_theta_) / dt;
    double omega_pd = pd_kp_ * e_theta_filt + pd_kd_ * de_theta;
    prev_e_theta_   = e_theta_filt;

    // Combine Stanley + PD
    double omega_des = omega_pd + sign_v * (stanley_correction / dt) * 0.0;
    // Use Stanley angle as heading correction input
    omega_des = sign_v * (pd_kp_ * (e_theta_filt + stanley_correction) +
                          pd_kd_ * de_theta);

    // Low-pass smoother
    double omega_smooth = omega_smoother_alpha_ * omega_des +
                          (1.0 - omega_smoother_alpha_) * prev_omega_;
    prev_omega_ = omega_smooth;

    // Clamp
    omega_smooth = std::clamp(omega_smooth, -max_omega_, max_omega_);

    // ── Publish
    watchdog_->setCurrentSpeed(std::abs(vx_des));
    publishCmdVel(vx_des, omega_smooth);

    // ── Feedback
    auto feedback = std::make_shared<Translate::Feedback>();
    feedback->current_distance      = traveled_dist;
    feedback->current_lateral_error = e_lateral;
    feedback->current_heading_error = e_theta * 180.0 / M_PI;
    feedback->current_vx            = vx_des;
    feedback->current_vy            = 0.0;
    feedback->current_omega         = omega_smooth;
    feedback->phase                 = static_cast<uint8_t>(profile_out.phase);
    // Wheel RPM (informational)
    double omega_wheel_l = (vx_des - omega_smooth * wheel_base_ / 2.0) / wheel_radius_;
    double omega_wheel_r = (vx_des + omega_smooth * wheel_base_ / 2.0) / wheel_radius_;
    feedback->w1_drive_rpm = omega_wheel_l * 60.0 / (2.0 * M_PI);
    feedback->w2_drive_rpm = omega_wheel_r * 60.0 / (2.0 * M_PI);
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // rclcpp::ok() went false — shutdown
  publishCmdVel(0.0, 0.0);
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
bool TranslateActionServer::lookupRobotPose(double & x, double & y, double & yaw)
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      "map", robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "TranslateActionServer: TF lookup failed: %s", ex.what());
    return false;
  }
}

void TranslateActionServer::publishCmdVel(double vx, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = vx;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

void TranslateActionServer::publishPathMarker(
  const std::shared_ptr<GoalHandle> & /*goal_handle*/)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = node_->now();
  geometry_msgs::msg::PoseStamped ps0, ps1;
  ps0.pose.position.x = path_start_x_;
  ps0.pose.position.y = path_start_y_;
  ps0.pose.orientation.w = 1.0;
  ps1.pose.position.x = path_end_x_;
  ps1.pose.position.y = path_end_y_;
  ps1.pose.orientation.w = 1.0;
  path_msg.poses = {ps0, ps1};
  path_viz_pub_->publish(path_msg);
}

void TranslateActionServer::clearPathMarker()
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  empty.header.stamp = node_->now();
  path_viz_pub_->publish(empty);
}

double TranslateActionServer::normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void TranslateActionServer::handleEndpointUpdate(
  double & end_x, double & end_y, bool & has_next)
{
  end_x    = new_end_x_.load();
  end_y    = new_end_y_.load();
  has_next = new_has_next_.load();
}

}  // namespace amr_motion_control_2wd
