#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_interfaces/action/amr_motion_yaw_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "amr_motion_control_2wd/localization_watchdog.hpp"

namespace amr_motion_control_2wd
{

class YawControlActionServer
{
public:
  using YawControl     = amr_interfaces::action::AMRMotionYawControl;
  using GoalHandleYaw  = rclcpp_action::ServerGoalHandle<YawControl>;

  explicit YawControlActionServer(rclcpp::Node::SharedPtr node);

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<YawControl>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  pose_sub_;

  // ── Stop service ────────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  std::atomic<bool> stop_requested_{false};

  // ── TF2 ──────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── IMU state (callback thread ↔ execute thread) ─────────────────────────
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool>   imu_received_{false};

  // ── Localization watchdog ─────────────────────────────────────────────────
  std::optional<LocalizationWatchdog> watchdog_;

  // ── Control parameters (declared from YAML in constructor) ───────────────
  double control_rate_hz_{20.0};
  double max_timeout_sec_{60.0};
  bool   enable_localization_watchdog_{true};

  // Heading PD controller
  double Kp_heading_{1.0};
  double Kd_heading_{0.3};
  double max_omega_{1.0};
  double min_turning_radius_{0.7};

  // Omega rate limiter
  double omega_rate_limit_{0.5};   // rad/s per second

  // Walk velocity smoother
  double walk_accel_limit_{0.5};   // m/s^2
  double walk_decel_limit_{1.0};   // m/s^2

  std::string robot_base_frame_{"base_footprint"};

  // ── Action server callbacks ───────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const YawControl::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleYaw> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleYaw> goal_handle);

  // ── Execution thread ──────────────────────────────────────────────────────
  void execute(const std::shared_ptr<GoalHandleYaw> goal_handle);

  // ── Subscription callbacks ────────────────────────────────────────────────
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ── Helpers ──────────────────────────────────────────────────────────────
  bool lookupRobotPose(double & x, double & y, double & yaw) const;
  void publishCmdVel(double vx, double omega);
};

}  // namespace amr_motion_control_2wd
