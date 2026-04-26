#include <cmath>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("odd_global_planner"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<double>("sample_spacing", 0.1);
    declare_parameter<double>("collision_step", 0.05);
    declare_parameter<int>("occupancy_threshold", 50);
    declare_parameter<bool>("allow_unknown", false);
    declare_parameter<std::string>("robot_frame", "base_footprint");
    declare_parameter<std::string>("map_frame", "map");

    sample_spacing_ = get_parameter("sample_spacing").as_double();
    collision_step_ = get_parameter("collision_step").as_double();
    occupancy_threshold_ = get_parameter("occupancy_threshold").as_int();
    allow_unknown_ = get_parameter("allow_unknown").as_bool();
    robot_frame_ = get_parameter("robot_frame").as_string();
    map_frame_ = get_parameter("map_frame").as_string();

    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/rtabmap/map", map_qos,
      std::bind(&GlobalPlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(1).reliable().transient_local());

    collision_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/global_planner/collision_marker",
      rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(get_logger(),
      "odd_global_planner started. dS=%.2fm, collision_step=%.3fm, thresh=%d, allow_unknown=%s",
      sample_spacing_, collision_step_, occupancy_threshold_,
      allow_unknown_ ? "true" : "false");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
    RCLCPP_INFO_ONCE(get_logger(),
      "Map received: %ux%u @ %.3f m/cell, origin=(%.2f, %.2f), frame=%s",
      msg->info.width, msg->info.height, msg->info.resolution,
      msg->info.origin.position.x, msg->info.origin.position.y,
      msg->header.frame_id.c_str());
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = map_;
    }
    if (!map) {
      RCLCPP_WARN(get_logger(), "Goal received but no map yet — ignored.");
      return;
    }

    auto start = lookupRobotPose();
    if (!start) {
      RCLCPP_WARN(get_logger(),
        "Cannot plan: TF lookup %s -> %s failed.",
        map_frame_.c_str(), robot_frame_.c_str());
      return;
    }

    const double sx = start->pose.position.x;
    const double sy = start->pose.position.y;
    const double gx = goal->pose.position.x;
    const double gy = goal->pose.position.y;
    const double dist = std::hypot(gx - sx, gy - sy);

    auto collision = checkStraightCollision(*map, sx, sy, gx, gy);
    if (collision) {
      RCLCPP_WARN(get_logger(),
        "Plan REJECTED: straight line start=(%.2f,%.2f) -> goal=(%.2f,%.2f) "
        "collides at (%.2f,%.2f), cost=%d",
        sx, sy, gx, gy, collision->x, collision->y, collision->cost);
      nav_msgs::msg::Path empty;
      empty.header.frame_id = map_frame_;
      empty.header.stamp = now();
      path_pub_->publish(empty);
      publishCollisionMarker(collision->x, collision->y, true);
      return;
    }

    publishCollisionMarker(0.0, 0.0, false);
    auto path = buildStraightLinePath(*start, *goal);
    path_pub_->publish(path);

    RCLCPP_INFO(get_logger(),
      "Plan OK: start=(%.2f,%.2f) -> goal=(%.2f,%.2f), distance=%.2fm, poses=%zu",
      sx, sy, gx, gy, dist, path.poses.size());
  }

  struct CollisionPoint
  {
    double x;
    double y;
    int cost;
  };

  std::optional<CollisionPoint> checkStraightCollision(
    const nav_msgs::msg::OccupancyGrid & map,
    double x0, double y0, double x1, double y1) const
  {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::hypot(dx, dy);
    const double step = std::max(collision_step_, map.info.resolution * 0.5);
    const size_t n = std::max<size_t>(2,
      static_cast<size_t>(std::ceil(dist / step)) + 1);

    for (size_t i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n - 1);
      const double wx = x0 + t * dx;
      const double wy = y0 + t * dy;

      const int mx = static_cast<int>(
        std::floor((wx - map.info.origin.position.x) / map.info.resolution));
      const int my = static_cast<int>(
        std::floor((wy - map.info.origin.position.y) / map.info.resolution));

      if (mx < 0 || my < 0 ||
          mx >= static_cast<int>(map.info.width) ||
          my >= static_cast<int>(map.info.height))
      {
        return CollisionPoint{wx, wy, -2};  // out of map
      }

      const size_t idx = static_cast<size_t>(my) * map.info.width +
                         static_cast<size_t>(mx);
      const int cost = map.data[idx];

      if (cost == -1 && !allow_unknown_) {
        return CollisionPoint{wx, wy, -1};  // unknown
      }
      if (cost >= occupancy_threshold_) {
        return CollisionPoint{wx, wy, cost};
      }
    }
    return std::nullopt;
  }

  void publishCollisionMarker(double x, double y, bool show)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = now();
    m.ns = "collision";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = show ? visualization_msgs::msg::Marker::ADD
                    : visualization_msgs::msg::Marker::DELETE;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.35;
    m.scale.y = 0.35;
    m.scale.z = 0.35;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 0.9f;
    collision_marker_pub_->publish(m);
  }

  std::optional<geometry_msgs::msg::PoseStamped> lookupRobotPose()
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        map_frame_, robot_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.2));

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = tf.header.stamp;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & e) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", e.what());
      return std::nullopt;
    }
  }

  nav_msgs::msg::Path buildStraightLinePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();

    const double dx = goal.pose.position.x - start.pose.position.x;
    const double dy = goal.pose.position.y - start.pose.position.y;
    const double dist = std::hypot(dx, dy);
    const double yaw = std::atan2(dy, dx);

    const size_t n = std::max<size_t>(2,
      static_cast<size_t>(std::ceil(dist / sample_spacing_)) + 1);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    for (size_t i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n - 1);
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = start.pose.position.x + t * dx;
      p.pose.position.y = start.pose.position.y + t * dy;
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      path.poses.push_back(p);
    }

    if (!path.poses.empty()) {
      path.poses.back().pose.orientation = goal.pose.orientation;
    }
    return path;
  }

  double sample_spacing_;
  double collision_step_;
  int occupancy_threshold_;
  bool allow_unknown_;
  std::string robot_frame_;
  std::string map_frame_;

  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
