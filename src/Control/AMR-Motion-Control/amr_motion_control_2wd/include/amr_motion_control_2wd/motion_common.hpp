#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace amr_motion_control_2wd
{

enum class ActiveAction : uint8_t {
  NONE = 0, SPIN, TURN, TRANSLATE, TRANSLATE_REVERSE, YAW_CONTROL, PURE_PURSUIT, STANLEY
};

extern std::atomic<ActiveAction> g_active_action;

inline const char* to_string(ActiveAction a)
{
  switch (a) {
    case ActiveAction::NONE:              return "NONE";
    case ActiveAction::SPIN:              return "SPIN";
    case ActiveAction::TURN:              return "TURN";
    case ActiveAction::TRANSLATE:         return "TRANSLATE";
    case ActiveAction::TRANSLATE_REVERSE: return "TRANSLATE_REVERSE";
    case ActiveAction::YAW_CONTROL:       return "YAW_CONTROL";
    case ActiveAction::PURE_PURSUIT:     return "PURE_PURSUIT";
    case ActiveAction::STANLEY:          return "STANLEY";
    default:                              return "UNKNOWN";
  }
}

class ActionGuard
{
public:
  ActionGuard() = default;
  ~ActionGuard() { g_active_action.store(ActiveAction::NONE); }
  ActionGuard(const ActionGuard &) = delete;
  ActionGuard & operator=(const ActionGuard &) = delete;
};

template <typename T>
inline T safeParam(rclcpp::Node::SharedPtr node,
                   const std::string & name, T default_val)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_val);
  }
  return node->get_parameter(name).get_value<T>();
}

inline void publishCmdVel(
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
    double linear_x, double angular_z)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  pub->publish(msg);
}

inline double normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

inline bool lookupRobotPose(
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    double & x, double & y, double & yaw,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const std::string & target_frame = "base_footprint")
{
  try {
    auto tf_stamped = tf_buffer->lookupTransform(
      "map", target_frame, tf2::TimePointZero);
    x = tf_stamped.transform.translation.x;
    y = tf_stamped.transform.translation.y;
    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);
    double r, p;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
      "TF2 map->%s lookup failed: %s", target_frame.c_str(), ex.what());
    return false;
  }
}

inline bool lookupTfYaw(
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    double & yaw_deg,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const std::string & target_frame = "base_footprint")
{
  double x, y, yaw_rad;
  if (!lookupRobotPose(tf_buffer, x, y, yaw_rad, logger, clock, target_frame)) {
    return false;
  }
  yaw_deg = yaw_rad * 180.0 / M_PI;
  return true;
}

inline double readImuDelta(
    double current_yaw_rad, double & prev_yaw_rad,
    double deadband_rad)
{
  double delta = current_yaw_rad - prev_yaw_rad;
  if (delta >  M_PI) { delta -= 2.0 * M_PI; }
  if (delta < -M_PI) { delta += 2.0 * M_PI; }
  if (std::abs(delta) < deadband_rad) { return 0.0; }
  prev_yaw_rad = current_yaw_rad;
  return delta * 180.0 / M_PI;
}

}  // namespace amr_motion_control_2wd
