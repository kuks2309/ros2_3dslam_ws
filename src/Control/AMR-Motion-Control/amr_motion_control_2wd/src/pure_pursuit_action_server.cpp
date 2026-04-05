#include "amr_motion_control_2wd/pure_pursuit_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_motion_control_2wd/localization_watchdog.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace amr_motion_control_2wd
{

// ─── Constructor ──────────────────────────────────────────────────────────────
PurePursuitActionServer::PurePursuitActionServer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // ── Control parameters
  ctrl_freq_hz_               = safeParam(node_, "pure_pursuit_ctrl_freq_hz",              50.0);
  default_lookahead_distance_ = safeParam(node_, "pure_pursuit_default_lookahead_distance", 0.5);
  min_lookahead_distance_     = safeParam(node_, "pure_pursuit_min_lookahead_distance",     0.2);
  max_lookahead_distance_     = safeParam(node_, "pure_pursuit_max_lookahead_distance",     2.0);
  default_goal_tolerance_     = safeParam(node_, "pure_pursuit_default_goal_tolerance",     0.1);
  max_omega_                  = safeParam(node_, "pure_pursuit_max_omega",                  1.5);
  max_timeout_sec_            = safeParam(node_, "pure_pursuit_max_timeout_sec",           120.0);
  Kp_heading_                 = safeParam(node_, "pure_pursuit_Kp_heading",                 1.0);
  Kd_heading_                 = safeParam(node_, "pure_pursuit_Kd_heading",                 0.2);
  lateral_abort_dist_         = safeParam(node_, "pure_pursuit_lateral_abort_dist",         1.5);
  min_vx_                     = safeParam(node_, "pure_pursuit_min_vx",                     0.02);
  watchdog_timeout_sec_       = safeParam(node_, "pure_pursuit_watchdog_timeout_sec",        2.0);
  watchdog_jump_threshold_    = safeParam(node_, "pure_pursuit_watchdog_jump_threshold",     0.5);
  watchdog_velocity_margin_   = safeParam(node_, "pure_pursuit_watchdog_velocity_margin",    1.3);
  robot_base_frame_           = safeParam(node_, "robot_base_frame",
                                          std::string("base_footprint"));

  // ── TF2
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Publishers
  cmd_vel_pub_  = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "pure_pursuit_path", rclcpp::QoS(10).transient_local());

  // ── Subscribers
  auto imu_qos  = rclcpp::SensorDataQoS();
  auto pose_qos = rclcpp::SensorDataQoS();

  std::string imu_topic  = safeParam(node_, "imu_topic",
                                     std::string("/imu/data"));
  std::string pose_topic = safeParam(node_, "pure_pursuit_pose_topic",
                                     std::string("/rtabmap/odom"));

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, imu_qos,
    std::bind(&PurePursuitActionServer::imuCallback, this, _1));

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    pose_topic, pose_qos,
    std::bind(&PurePursuitActionServer::odomPoseCallback, this, _1));

  safety_speed_limit_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "safety_speed_limit", 10,
    std::bind(&PurePursuitActionServer::safetySpeedLimitCallback, this, _1));

  safety_status_sub_ = node_->create_subscription<amr_interfaces::msg::SafetyStatus>(
    "safety_status", 10,
    std::bind(&PurePursuitActionServer::safetyStatusCallback, this, _1));

  // ── Path update service
  update_path_srv_ = node_->create_service<UpdatePath>(
    "update_pure_pursuit_path",
    [this](
      const std::shared_ptr<UpdatePath::Request>  req,
      const std::shared_ptr<UpdatePath::Response> res)
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      pending_path_ = req->new_path;
      path_update_pending_.store(true);
      res->success = true;
      res->message = "Path update queued";
      RCLCPP_INFO(node_->get_logger(),
        "PurePursuitActionServer: path update queued (%zu poses)",
        req->new_path.poses.size());
    });

  // ── Action server
  action_server_ = rclcpp_action::create_server<PurePursuit>(
    node_,
    "amr_motion_pure_pursuit",
    std::bind(&PurePursuitActionServer::handleGoal,     this, _1, _2),
    std::bind(&PurePursuitActionServer::handleCancel,   this, _1),
    std::bind(&PurePursuitActionServer::handleAccepted, this, _1));

  RCLCPP_INFO(node_->get_logger(),
    "PurePursuitActionServer ready (base_frame=%s, freq=%.0f Hz)",
    robot_base_frame_.c_str(), ctrl_freq_hz_);
}

// ─── Sensor callbacks ─────────────────────────────────────────────────────────
void PurePursuitActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->orientation.x, msg->orientation.y,
    msg->orientation.z, msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

void PurePursuitActionServer::odomPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void PurePursuitActionServer::safetySpeedLimitCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  safety_speed_limit_.store(msg->data);
}

void PurePursuitActionServer::safetyStatusCallback(
  const amr_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  safety_state_.store(msg->status);
}

// ─── Action callbacks ─────────────────────────────────────────────────────────
rclcpp_action::GoalResponse PurePursuitActionServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PurePursuit::Goal> goal)
{
  if (goal->max_linear_speed <= 0.0 || goal->acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: invalid params (speed=%.3f accel=%.3f)",
      goal->max_linear_speed, goal->acceleration);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->path.poses.size() < 2) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: path has fewer than 2 poses (%zu), rejecting",
      goal->path.poses.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(node_->get_logger(),
    "PurePursuitActionServer: goal received (path=%zu pts, v=%.3f, a=%.3f, "
    "ld=%.3f, tol=%.3f)",
    goal->path.poses.size(), goal->max_linear_speed, goal->acceleration,
    goal->lookahead_distance, goal->goal_tolerance);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PurePursuitActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "PurePursuitActionServer: cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PurePursuitActionServer::handleAccepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&PurePursuitActionServer::execute, this, goal_handle)}.detach();
}

// ─── buildPath ────────────────────────────────────────────────────────────────
bool PurePursuitActionServer::buildPath(const nav_msgs::msg::Path & path_msg)
{
  waypoints_.clear();
  cumulative_dist_.clear();
  total_path_length_ = 0.0;

  if (path_msg.poses.size() < 2) {
    return false;
  }

  waypoints_.reserve(path_msg.poses.size());
  for (const auto & ps : path_msg.poses) {
    waypoints_.push_back({ps.pose.position.x, ps.pose.position.y});
  }

  cumulative_dist_.resize(waypoints_.size(), 0.0);
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    double dx = waypoints_[i].x - waypoints_[i - 1].x;
    double dy = waypoints_[i].y - waypoints_[i - 1].y;
    cumulative_dist_[i] = cumulative_dist_[i - 1] + std::hypot(dx, dy);
  }

  total_path_length_ = cumulative_dist_.back();
  if (total_path_length_ < 1e-4) {
    return false;
  }

  return true;
}

// ─── findClosestPoint ────────────────────────────────────────────────────────
PurePursuitActionServer::ClosestPointResult
PurePursuitActionServer::findClosestPoint(double robot_x, double robot_y) const
{
  ClosestPointResult best{};
  best.segment_idx    = 0;
  best.t              = 0.0;
  best.x              = waypoints_[0].x;
  best.y              = waypoints_[0].y;
  best.cross_track_err = 0.0;
  best.arc_length     = 0.0;

  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
    const Waypoint & A = waypoints_[i];
    const Waypoint & B = waypoints_[i + 1];

    double seg_dx = B.x - A.x;
    double seg_dy = B.y - A.y;
    double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

    double t = 0.0;
    if (seg_len_sq > 1e-12) {
      t = ((robot_x - A.x) * seg_dx + (robot_y - A.y) * seg_dy) / seg_len_sq;
      t = std::clamp(t, 0.0, 1.0);
    }

    double cx = A.x + t * seg_dx;
    double cy = A.y + t * seg_dy;
    double dist_sq = (robot_x - cx) * (robot_x - cx) +
                     (robot_y - cy) * (robot_y - cy);

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best.segment_idx = i;
      best.t           = t;
      best.x           = cx;
      best.y           = cy;

      // Signed CTE: cross product of path direction × offset vector
      // positive = robot is to the left of the path direction
      double seg_len = std::sqrt(seg_len_sq);
      double ux = (seg_len > 1e-12) ? seg_dx / seg_len : 1.0;
      double uy = (seg_len > 1e-12) ? seg_dy / seg_len : 0.0;
      double off_x = robot_x - cx;
      double off_y = robot_y - cy;
      // cross product (path_dir x offset) z-component: ux*off_y - uy*off_x
      best.cross_track_err = ux * off_y - uy * off_x;

      double seg_t_dist = t * std::sqrt(seg_len_sq);
      best.arc_length = cumulative_dist_[i] + seg_t_dist;
    }
  }

  return best;
}

// ─── findLookaheadPoint ──────────────────────────────────────────────────────
PurePursuitActionServer::LookaheadResult
PurePursuitActionServer::findLookaheadPoint(
  const ClosestPointResult & closest,
  double lookahead_distance) const
{
  double target_arc = closest.arc_length + lookahead_distance;

  // Past end of path: return last waypoint
  if (target_arc >= total_path_length_) {
    const Waypoint & last = waypoints_.back();
    double dx = last.x - closest.x;
    double dy = last.y - closest.y;
    return {last.x, last.y, std::hypot(dx, dy), true};
  }

  // Find the segment that contains target_arc
  // cumulative_dist_[i] is the arc-length to waypoint i
  size_t seg = closest.segment_idx;
  for (size_t i = seg; i + 1 < waypoints_.size(); ++i) {
    if (cumulative_dist_[i] <= target_arc && target_arc <= cumulative_dist_[i + 1]) {
      seg = i;
      break;
    }
    // If we overshoot (shouldn't happen given target_arc < total), catch here
    if (i + 2 == waypoints_.size()) {
      seg = i;
    }
  }

  double seg_len = cumulative_dist_[seg + 1] - cumulative_dist_[seg];
  double t = 0.0;
  if (seg_len > 1e-12) {
    t = (target_arc - cumulative_dist_[seg]) / seg_len;
    t = std::clamp(t, 0.0, 1.0);
  }

  const Waypoint & A = waypoints_[seg];
  const Waypoint & B = waypoints_[seg + 1];
  double lx = A.x + t * (B.x - A.x);
  double ly = A.y + t * (B.y - A.y);

  // Distance from robot's closest point to lookahead (informational)
  double dx = lx - closest.x;
  double dy = ly - closest.y;

  return {lx, ly, std::hypot(dx, dy), true};
}

// ─── handlePathUpdate ────────────────────────────────────────────────────────
void PurePursuitActionServer::handlePathUpdate()
{
  nav_msgs::msg::Path new_path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    new_path = pending_path_;
  }
  if (buildPath(new_path)) {
    RCLCPP_INFO(node_->get_logger(),
      "PurePursuitActionServer: path updated (%zu wpts, %.3f m)",
      waypoints_.size(), total_path_length_);
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: path update failed (invalid path), keeping old path");
  }
}

// ─── Main execution loop ──────────────────────────────────────────────────────
void PurePursuitActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  // ── 1. Mutual exclusion: CAS NONE → PURE_PURSUIT ─────────────────────────
  ActiveAction expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::PURE_PURSUIT)) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: another action is active (%s), aborting new goal",
      to_string(g_active_action.load()));
    auto result = std::make_shared<PurePursuit::Result>();
    result->status = -2;  // invalid (busy)
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }
  ActionGuard action_guard;  // clears g_active_action on scope exit

  const auto & goal      = goal_handle->get_goal();
  const auto   start_time = node_->now();

  // ── 2. Build initial path ─────────────────────────────────────────────────
  if (!buildPath(goal->path)) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: invalid path (fewer than 2 pts or zero length)");
    auto result = std::make_shared<PurePursuit::Result>();
    result->status       = -2;  // invalid
    result->elapsed_time = 0.0;
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }

  // ── 3. Effective lookahead and goal tolerance ─────────────────────────────
  double lookahead_dist = (goal->lookahead_distance > 1e-6)
    ? goal->lookahead_distance
    : default_lookahead_distance_;
  lookahead_dist = std::clamp(lookahead_dist,
                              min_lookahead_distance_,
                              max_lookahead_distance_);

  double goal_tolerance = (goal->goal_tolerance > 1e-6)
    ? goal->goal_tolerance
    : default_goal_tolerance_;

  // ── 4. Trapezoidal speed profile ──────────────────────────────────────────
  amr_motion_control::TrapezoidalProfile profile(
    total_path_length_,
    goal->max_linear_speed,
    goal->acceleration,
    0.0);

  // ── 5. Localization watchdog ──────────────────────────────────────────────
  LocalizationWatchdog::Config wd_cfg;
  wd_cfg.timeout_sec          = watchdog_timeout_sec_;
  wd_cfg.fixed_jump_threshold = watchdog_jump_threshold_;
  wd_cfg.velocity_margin      = watchdog_velocity_margin_;
  watchdog_.emplace(wd_cfg, node_->get_logger());

  // ── 6. Wait for first pose (5 s timeout) ─────────────────────────────────
  {
    rclcpp::Rate wait_rate(20);
    int wait_count = 0;
    while (rclcpp::ok() && !watchdog_->poseReceived()) {
      if (!rclcpp::ok()) { break; }
      wait_rate.sleep();
      if (++wait_count > 100) {
        RCLCPP_ERROR(node_->get_logger(),
          "PurePursuitActionServer: no pose received after 5 s, aborting");
        publishCmdVel(0.0, 0.0);
        auto result = std::make_shared<PurePursuit::Result>();
        result->status       = -3;  // timeout
        result->elapsed_time = (node_->now() - start_time).seconds();
        try { goal_handle->abort(result); } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(),
            "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
        }
        return;
      }
    }
  }

  // ── 7. Publish path visualization ────────────────────────────────────────
  publishPathMarker(goal_handle);

  // ── 8. Initialise control loop state ─────────────────────────────────────
  rclcpp::Rate rate(ctrl_freq_hz_);
  prev_e_heading_ = 0.0;
  prev_omega_     = 0.0;

  double rob_x = 0.0, rob_y = 0.0, rob_yaw = 0.0;
  if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
    rob_x   = watchdog_->x();
    rob_y   = watchdog_->y();
    rob_yaw = watchdog_->yaw();
  }

  RCLCPP_INFO(node_->get_logger(),
    "PurePursuitActionServer: initial pose=(%.3f, %.3f, %.3f rad) "
    "path_len=%.3f m wpts=%zu ld=%.3f m tol=%.3f m",
    rob_x, rob_y, rob_yaw, total_path_length_,
    waypoints_.size(), lookahead_dist, goal_tolerance);

  // ── 9. Control loop ───────────────────────────────────────────────────────
  double arc_length_traveled = 0.0;

  while (rclcpp::ok()) {
    if (!rclcpp::ok()) { break; }

    // ── a. Cancellation check ─────────────────────────────────────────────
    if (goal_handle->is_canceling()) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status                 = -1;  // cancelled
      result->actual_distance        = arc_length_traveled;
      result->final_cross_track_error = 0.0;
      result->elapsed_time           = (node_->now() - start_time).seconds();
      try { goal_handle->canceled(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: canceled failed (stale handle): %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(), "PurePursuitActionServer: cancelled");
      return;
    }

    // ── b. Timeout check ─────────────────────────────────────────────────
    const double elapsed = (node_->now() - start_time).seconds();
    if (elapsed > max_timeout_sec_) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      RCLCPP_ERROR(node_->get_logger(),
        "PurePursuitActionServer: timeout after %.1f s, aborting", elapsed);
      auto result = std::make_shared<PurePursuit::Result>();
      result->status       = -3;  // timeout
      result->elapsed_time = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
      }
      return;
    }

    // ── c. Safety stop ────────────────────────────────────────────────────
    if (safety_state_.load() == amr_interfaces::msg::SafetyStatus::STATUS_DANGEROUS) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status          = -4;  // safety_stop
      result->actual_distance = arc_length_traveled;
      result->elapsed_time    = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
      }
      RCLCPP_WARN(node_->get_logger(),
        "PurePursuitActionServer: safety stop triggered");
      return;
    }

    // ── d. Path update check ──────────────────────────────────────────────
    if (path_update_pending_.exchange(false)) {
      handlePathUpdate();
      // Rebuild profile with remaining distance from current arc position
      double remaining_len = std::max(0.0, total_path_length_ - arc_length_traveled);
      profile = amr_motion_control::TrapezoidalProfile(
        remaining_len,
        goal->max_linear_speed,
        goal->acceleration,
        0.0);
      publishPathMarker(goal_handle);
      RCLCPP_INFO(node_->get_logger(),
        "PurePursuitActionServer: profile rebuilt (remaining=%.3f m)", remaining_len);
    }

    // ── e. Localization watchdog ──────────────────────────────────────────
    if (!watchdog_->checkHealth()) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status          = -3;  // timeout (localization lost)
      result->actual_distance = arc_length_traveled;
      result->elapsed_time    = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
      }
      RCLCPP_ERROR(node_->get_logger(),
        "PurePursuitActionServer: localization watchdog triggered, aborting");
      return;
    }

    // ── f. Get current robot pose (TF2: map → base_footprint) ────────────
    if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
      rob_x   = watchdog_->x();
      rob_y   = watchdog_->y();
      rob_yaw = watchdog_->yaw();
    }

    // ── g. Find closest point on path ────────────────────────────────────
    const auto closest = findClosestPoint(rob_x, rob_y);
    arc_length_traveled = closest.arc_length;
    const double cte     = closest.cross_track_err;

    // ── h. Lateral abort ──────────────────────────────────────────────────
    if (std::abs(cte) > lateral_abort_dist_) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status                 = -3;  // treat as fault/timeout
      result->actual_distance        = arc_length_traveled;
      result->final_cross_track_error = cte;
      result->elapsed_time           = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: abort failed (stale handle): %s", e.what());
      }
      RCLCPP_ERROR(node_->get_logger(),
        "PurePursuitActionServer: lateral error %.3f m > %.3f m limit, aborting",
        cte, lateral_abort_dist_);
      return;
    }

    // ── i. Goal arrival check (dist from robot to last waypoint) ─────────
    const Waypoint & last_wp = waypoints_.back();
    const double dist_to_goal = std::hypot(last_wp.x - rob_x, last_wp.y - rob_y);

    if (dist_to_goal < goal_tolerance) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();

      // Optional final heading alignment via PD rotation
      if (goal->align_final_heading && waypoints_.size() >= 2) {
        const Waypoint & prev_wp = waypoints_[waypoints_.size() - 2];
        double final_yaw = std::atan2(
          last_wp.y - prev_wp.y,
          last_wp.x - prev_wp.x);

        RCLCPP_INFO(node_->get_logger(),
          "PurePursuitActionServer: aligning final heading to %.3f rad", final_yaw);

        rclcpp::Rate align_rate(ctrl_freq_hz_);
        const double dt = 1.0 / ctrl_freq_hz_;
        double prev_e   = 0.0;
        auto   align_start = node_->now();
        const double align_timeout = 5.0;

        while (rclcpp::ok()) {
          if (goal_handle->is_canceling()) { break; }
          if ((node_->now() - align_start).seconds() > align_timeout) { break; }

          if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
            rob_x   = watchdog_->x();
            rob_y   = watchdog_->y();
            rob_yaw = watchdog_->yaw();
          }
          double e = normalizeAngle(final_yaw - rob_yaw);
          if (std::abs(e) < 0.03) { break; }  // ~1.7 deg tolerance

          double de     = (e - prev_e) / dt;
          prev_e        = e;
          double omega  = std::clamp(Kp_heading_ * e + Kd_heading_ * de,
                                     -max_omega_, max_omega_);
          publishCmdVel(0.0, omega);
          align_rate.sleep();
        }
        publishCmdVel(0.0, 0.0);
      }

      const double final_cte = findClosestPoint(rob_x, rob_y).cross_track_err;
      auto result = std::make_shared<PurePursuit::Result>();
      result->status                 = 0;  // success
      result->actual_distance        = arc_length_traveled;
      result->final_cross_track_error = final_cte;
      result->elapsed_time           = (node_->now() - start_time).seconds();
      try { goal_handle->succeed(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: succeed failed (stale handle): %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(),
        "PurePursuitActionServer: arrived (dist=%.3f m, cte=%.3f m, t=%.2f s)",
        arc_length_traveled, final_cte, result->elapsed_time);
      return;
    }

    // ── j. Speed from trapezoidal profile ─────────────────────────────────
    const auto profile_out = profile.getSpeed(
      std::min(arc_length_traveled, total_path_length_));
    double vx = profile_out.speed;

    // Minimum start speed to overcome static friction
    if (profile_out.phase != amr_motion_control::ProfilePhase::DONE &&
        vx < min_vx_ && dist_to_goal > goal_tolerance) {
      vx = min_vx_;
    }

    // ── k. Apply safety speed limit ────────────────────────────────────────
    double spd_limit = safety_speed_limit_.load();
    if (vx > spd_limit) {
      vx = spd_limit;
    }

    // ── l. Find lookahead point ────────────────────────────────────────────
    const auto lookahead = findLookaheadPoint(closest, lookahead_dist);

    // ── m. Pure pursuit steering: alpha = heading to lookahead − rob_yaw ──
    double alpha = std::atan2(lookahead.y - rob_y, lookahead.x - rob_x) - rob_yaw;
    alpha = normalizeAngle(alpha);

    // Wheelbase: use wheel_separation as effective L (default 0.34 m)
    // The parameter is shared with the other controllers
    double L_actual = safeParam(node_, "wheel_separation", 0.34);
    L_actual = std::max(L_actual, 0.10);  // safety floor

    double omega = (std::abs(L_actual) > 1e-6)
      ? 2.0 * vx * std::sin(alpha) / L_actual
      : 0.0;
    omega = std::clamp(omega, -max_omega_, max_omega_);

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "PurePursuitActionServer: rob=(%.3f,%.3f,%.3f deg) "
      "arc=%.3f/%.3f cte=%.3f vx=%.3f omega=%.3f alpha=%.3f deg",
      rob_x, rob_y, rob_yaw * 180.0 / M_PI,
      arc_length_traveled, total_path_length_, cte,
      vx, omega, alpha * 180.0 / M_PI);

    // ── n. Publish velocity ────────────────────────────────────────────────
    watchdog_->setCurrentSpeed(std::abs(vx));
    publishCmdVel(vx, omega);

    // ── o. Publish feedback ───────────────────────────────────────────────
    {
      const double remaining_distance = std::max(0.0,
        total_path_length_ - arc_length_traveled);

      // Heading error: angle between path direction at closest point and robot yaw
      const Waypoint & A = waypoints_[closest.segment_idx];
      const Waypoint & B = waypoints_[closest.segment_idx + 1];
      double seg_yaw = std::atan2(B.y - A.y, B.x - A.x);
      double e_heading = normalizeAngle(seg_yaw - rob_yaw);

      // Find closest waypoint index for feedback
      uint32_t wp_idx = static_cast<uint32_t>(closest.segment_idx);

      auto feedback = std::make_shared<PurePursuit::Feedback>();
      feedback->current_distance       = arc_length_traveled;
      feedback->remaining_distance     = remaining_distance;
      feedback->cross_track_error      = cte;
      feedback->heading_error          = e_heading * 180.0 / M_PI;
      feedback->current_speed          = vx;
      feedback->current_waypoint_index = wp_idx;
      feedback->total_waypoints        = static_cast<uint32_t>(waypoints_.size());
      feedback->phase                  = 2;  // tracking
      try { goal_handle->publish_feedback(feedback); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "PurePursuitActionServer: publish_feedback failed: %s", e.what());
      }
    }

    rate.sleep();
  }

  // rclcpp::ok() went false — shutdown
  publishCmdVel(0.0, 0.0);
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
bool PurePursuitActionServer::lookupRobotPose(double & x, double & y, double & yaw)
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
      "PurePursuitActionServer: TF lookup failed: %s", ex.what());
    return false;
  }
}

void PurePursuitActionServer::publishCmdVel(double vx, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = vx;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

void PurePursuitActionServer::publishPathMarker(
  const std::shared_ptr<GoalHandle> & /*goal_handle*/)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp    = node_->now();
  path_msg.poses.reserve(waypoints_.size());
  for (const auto & wp : waypoints_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x  = wp.x;
    ps.pose.position.y  = wp.y;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }
  path_viz_pub_->publish(path_msg);
}

void PurePursuitActionServer::clearPathMarker()
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  empty.header.stamp    = node_->now();
  path_viz_pub_->publish(empty);
}

double PurePursuitActionServer::normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

}  // namespace amr_motion_control_2wd
