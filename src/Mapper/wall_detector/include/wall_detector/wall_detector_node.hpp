#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mapper_interfaces/msg/longest_wall.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace wall_detector {

struct WallSegment {
    double length_m;
    double angle_rad;       // [-π/2, π/2)
    double distance_m;      // 로봇으로부터 벽까지 수직거리
    double x1, y1, x2, y2;  // robot frame
    int    inlier_count;
    double inlier_ratio;
    double confidence;
};

class WallDetectorNode : public rclcpp::Node {
public:
    explicit WallDetectorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // 테스트용 public — 순수 함수
    std::vector<WallSegment> detect_walls(
        const sensor_msgs::msg::LaserScan & scan) const;

private:
    // Subscription / Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<mapper_interfaces::msg::LongestWall>::SharedPtr wall_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::mutex scan_mutex_;

    // 파라미터
    double grid_range_m_{20.0};
    double grid_res_m_{0.05};
    int    hough_thresh_{30};
    double min_line_len_m_{0.8};
    double max_line_gap_m_{0.15};
    double inlier_tol_m_{0.10};
    double publish_rate_hz_{10.0};
    std::string scan_topic_{"scan"};
    std::string output_topic_{"wall_detector/longest_wall"};

    // 콜백
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publish_longest_wall();

    // 내부 유틸
    std::vector<cv::Point2f> polar_to_xy(
        const sensor_msgs::msg::LaserScan & scan) const;
    cv::Mat xy_to_image(
        const std::vector<cv::Point2f> & pts, int & center_out) const;
    double normalize_angle_rad(double rad) const;
};

}  // namespace wall_detector
