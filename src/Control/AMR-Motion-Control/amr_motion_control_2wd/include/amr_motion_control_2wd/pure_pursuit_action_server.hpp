#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "amr_interfaces/action/amr_motion_pure_pursuit.hpp"
#include "amr_interfaces/srv/update_pure_pursuit_path.hpp"
#include "amr_interfaces/msg/safety_status.hpp"

#include "amr_motion_control_2wd/localization_watchdog.hpp"

namespace amr_motion_control_2wd
{

class PurePursuitActionServer
{
public:
  using PurePursuit   = amr_interfaces::action::AMRMotionPurePursuit;
  using GoalHandle    = rclcpp_action::ServerGoalHandle<PurePursuit>;
  using UpdatePath    = amr_interfaces::srv::UpdatePurePursuitPath;

  // ── Nested geometry types ─────────────────────────────────────────────────
  struct Waypoint
  {
    double x;
    double y;
  };

  struct ClosestPointResult
  {
    size_t segment_idx;     // index of the segment containing the closest point
    double t;               // [0,1] parametric position on segment
    double x;               // closest point x in map frame
    double y;               // closest point y in map frame
    double cross_track_err; // signed cross-track error (positive = left of path)
    double arc_length;      // cumulative distance along path to closest point
  };

  struct LookaheadResult
  {
    double x;               // lookahead point x in map frame
    double y;               // lookahead point y in map frame
    double distance;        // actual distance from robot to lookahead point
    bool   valid;           // false when lookahead could not be resolved
  };

  explicit PurePursuitActionServer(rclcpp::Node::SharedPtr node);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void safetySpeedLimitCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void safetyStatusCallback(const amr_interfaces::msg::SafetyStatus::SharedPtr msg);

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Server<PurePursuit>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr         path_viz_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr             imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr            safety_speed_limit_sub_;
  rclcpp::Subscription<amr_interfaces::msg::SafetyStatus>::SharedPtr safety_status_sub_;

  rclcpp::Service<UpdatePath>::SharedPtr update_path_srv_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Sensor state ─────────────────────────────────────────────────────────
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool>   imu_received_{false};

  std::optional<LocalizationWatchdog> watchdog_;

  // ── Safety ───────────────────────────────────────────────────────────────
  std::atomic<double>  safety_speed_limit_{1.0};   // m/s; default=unlimited-ish
  std::atomic<uint8_t> safety_state_{
    amr_interfaces::msg::SafetyStatus::STATUS_NORMAL};

  // ── Path update (replaces endpoint update) ───────────────────────────────
  std::atomic<bool>     path_update_pending_{false};
  std::mutex            path_mutex_;
  nav_msgs::msg::Path   pending_path_;

  // ── Inline path controller state ─────────────────────────────────────────
  std::vector<Waypoint> waypoints_;
  std::vector<double>   cumulative_dist_;   // cumulative arc-length per waypoint
  double                total_path_length_{0.0};

  double prev_e_heading_{0.0};  // rad — previous heading error for PD
  double prev_omega_{0.0};      // rad/s — previous omega for rate limiting

  // ── Control parameters (loaded via safeParam in constructor) ─────────────
  double ctrl_freq_hz_;
  double default_lookahead_distance_;
  double min_lookahead_distance_;
  double max_lookahead_distance_;
  double default_goal_tolerance_;
  double max_omega_;
  double max_timeout_sec_;
  double Kp_heading_;
  double Kd_heading_;
  double lateral_abort_dist_;
  double min_vx_;
  double watchdog_timeout_sec_;
  double watchdog_jump_threshold_;
  double watchdog_velocity_margin_;
  std::string robot_base_frame_;

  // ── Action callbacks ─────────────────────────────────────────────────────
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PurePursuit::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // ── Geometry helpers ─────────────────────────────────────────────────────
  ClosestPointResult findClosestPoint(double robot_x, double robot_y) const;

  LookaheadResult findLookaheadPoint(
    const ClosestPointResult & closest,
    double lookahead_distance) const;

  bool buildPath(const nav_msgs::msg::Path & path_msg);

  // ── Path update handler ───────────────────────────────────────────────────
  void handlePathUpdate();

  // ── Helpers ──────────────────────────────────────────────────────────────
  bool lookupRobotPose(double & x, double & y, double & yaw);
  void publishCmdVel(double vx, double omega);
  void publishPathMarker(const std::shared_ptr<GoalHandle> & goal_handle);
  void clearPathMarker();
  double normalizeAngle(double angle);
};

}  // namespace amr_motion_control_2wd
