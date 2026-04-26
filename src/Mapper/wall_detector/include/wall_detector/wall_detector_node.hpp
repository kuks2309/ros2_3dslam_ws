#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mapper_interfaces/msg/longest_wall.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
    // 후보 선별 필터 (lidar 원점 ↔ 직선 수직거리 기준)
    double min_distance_m_{0.0};   // 근거리 배제 (선반 제거용)
    double max_distance_m_{1e6};   // 원거리 노이즈 배제
    int    publish_top_n_{4};      // 시각화용 상위 N개 벽 (composite score 내림차순)
    double marker_line_width_m_{0.10};
    // 중복 선분 merge 임계 (Codex 권고 — 3°, 0.2m)
    double merge_angle_tol_deg_{3.0};
    double merge_distance_tol_m_{0.2};
    std::string scan_topic_{"scan"};
    std::string output_topic_{"wall_detector/longest_wall"};
    std::string marker_topic_{"wall_detector/wall_markers"};

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
