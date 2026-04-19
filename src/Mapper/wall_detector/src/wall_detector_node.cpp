#include "wall_detector/wall_detector_node.hpp"
#include <cmath>
#include <algorithm>

namespace wall_detector {

WallDetectorNode::WallDetectorNode(const rclcpp::NodeOptions & options)
: Node("wall_detector_node", options)
{
    // 파라미터 선언
    grid_range_m_     = declare_parameter("grid_range_m",     20.0);
    grid_res_m_       = declare_parameter("grid_res_m",       0.05);
    hough_thresh_     = static_cast<int>(declare_parameter("hough_thresh", 30));
    min_line_len_m_   = declare_parameter("min_line_len_m",   0.8);
    max_line_gap_m_   = declare_parameter("max_line_gap_m",   0.15);
    inlier_tol_m_     = declare_parameter("inlier_tol_m",     0.10);
    publish_rate_hz_  = declare_parameter("publish_rate_hz",  10.0);
    scan_topic_       = declare_parameter("scan_topic",
                                           std::string("scan"));
    output_topic_     = declare_parameter("output_topic",
                                           std::string("wall_detector/longest_wall"));

    // /scan 구독 — 프로젝트 관례 (BEST_EFFORT, VOLATILE)
    auto scan_qos = rclcpp::QoS(5).best_effort().durability_volatile();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, scan_qos,
        std::bind(&WallDetectorNode::scan_callback, this, std::placeholders::_1));

    // 결과 퍼블리시 — RELIABLE, VOLATILE
    auto wall_qos = rclcpp::QoS(10).reliable().durability_volatile();
    wall_pub_ = create_publisher<mapper_interfaces::msg::LongestWall>(
        output_topic_, wall_qos);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    pub_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&WallDetectorNode::publish_longest_wall, this));

    RCLCPP_INFO(get_logger(),
        "wall_detector started | scan=%s output=%s rate=%.1fHz "
        "hough_thresh=%d min_len=%.2fm",
        scan_topic_.c_str(), output_topic_.c_str(),
        publish_rate_hz_, hough_thresh_, min_line_len_m_);
}

double WallDetectorNode::normalize_angle_rad(double rad) const
{
    // [-π, π] → [-π/2, π/2) 직선은 방향 부호 무관
    double d = std::fmod(rad, M_PI);
    if (d >=  M_PI / 2.0) d -= M_PI;
    if (d <  -M_PI / 2.0) d += M_PI;
    return d;
}

std::vector<cv::Point2f> WallDetectorNode::polar_to_xy(
    const sensor_msgs::msg::LaserScan & scan) const
{
    std::vector<cv::Point2f> pts;
    pts.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
        double a = scan.angle_min + i * scan.angle_increment;
        pts.emplace_back(
            static_cast<float>(r * std::cos(a)),
            static_cast<float>(r * std::sin(a)));
    }
    return pts;
}

cv::Mat WallDetectorNode::xy_to_image(
    const std::vector<cv::Point2f> & pts, int & center_out) const
{
    int size = static_cast<int>(2.0 * grid_range_m_ / grid_res_m_) + 1;
    int center = size / 2;
    center_out = center;
    cv::Mat img(size, size, CV_8UC1, cv::Scalar(0));
    for (const auto & p : pts) {
        int px = static_cast<int>(center + p.x / grid_res_m_);
        int py = static_cast<int>(center - p.y / grid_res_m_);
        if (px >= 0 && px < size && py >= 0 && py < size) {
            img.at<uint8_t>(py, px) = 255;
        }
    }
    cv::dilate(img, img, cv::Mat::ones(2, 2, CV_8U));
    return img;
}

std::vector<WallSegment> WallDetectorNode::detect_walls(
    const sensor_msgs::msg::LaserScan & scan) const
{
    auto pts = polar_to_xy(scan);
    std::vector<WallSegment> result;
    if (pts.empty()) return result;

    int center = 0;
    cv::Mat img = xy_to_image(pts, center);

    // [Fix] 음수/0 가드 — OpenCV가 0을 받으면 fragment 모두 반환
    int min_len_px = std::max(1,
        static_cast<int>(min_line_len_m_ / grid_res_m_));
    int max_gap_px = std::max(1,
        static_cast<int>(max_line_gap_m_ / grid_res_m_));

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img, lines,
        1.0, CV_PI / 180.0,
        hough_thresh_, min_len_px, max_gap_px);

    // 포인트 배열을 cv::Mat로 변환 (inlier 계산용)
    cv::Mat pts_mat(pts.size(), 2, CV_32F);
    for (size_t i = 0; i < pts.size(); ++i) {
        pts_mat.at<float>(i, 0) = pts[i].x;
        pts_mat.at<float>(i, 1) = pts[i].y;
    }

    int max_inliers_seen = 1;
    for (const auto & ln : lines) {
        double x1 = (ln[0] - center) * grid_res_m_;
        double y1 = (center - ln[1]) * grid_res_m_;
        double x2 = (ln[2] - center) * grid_res_m_;
        double y2 = (center - ln[3]) * grid_res_m_;
        double dx = x2 - x1, dy = y2 - y1;
        double length = std::hypot(dx, dy);
        if (length < 1e-6) continue;

        // 법선·접선 단위 벡터
        double nx = -dy / length, ny = dx / length;
        double tx =  dx / length, ty = dy / length;

        int inliers = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            double px = pts[i].x - x1, py = pts[i].y - y1;
            double d_perp = std::abs(px * nx + py * ny);
            double t      = px * tx + py * ty;
            if (d_perp < inlier_tol_m_ && t >= -0.2 && t <= length + 0.2) {
                ++inliers;
            }
        }

        double angle = normalize_angle_rad(std::atan2(dy, dx));
        // 로봇으로부터 벽까지의 수직 거리 (원점 → 선분까지의 거리)
        double distance = std::abs(x1 * nx + y1 * ny);

        WallSegment ws;
        ws.length_m      = length;
        ws.angle_rad     = angle;
        ws.distance_m    = distance;
        ws.x1 = x1; ws.y1 = y1; ws.x2 = x2; ws.y2 = y2;
        ws.inlier_count  = inliers;
        // [Fix] grid 해상도 기반 — 이상적인 점 개수 = length / grid_res_m_
        // 다른 grid_res 설정에서도 일관된 의미 유지
        ws.inlier_ratio  = std::min(
            inliers / std::max(length / grid_res_m_, 1.0), 1.0);
        result.push_back(ws);
        max_inliers_seen = std::max(max_inliers_seen, inliers);
    }

    // confidence: 동일 frame 내 상대 순위 (1.0 = 최다 inlier 벽)
    // 주의 — 절대 신뢰도 아님. 시계열 비교 무의미.
    for (auto & w : result) {
        w.confidence = static_cast<double>(w.inlier_count) / max_inliers_seen;
    }

    std::sort(result.begin(), result.end(),
        [](const WallSegment & a, const WallSegment & b) {
            return a.length_m > b.length_m;
        });
    return result;
}

void WallDetectorNode::scan_callback(
    sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
}

void WallDetectorNode::publish_longest_wall()
{
    sensor_msgs::msg::LaserScan scan_copy;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (!latest_scan_) return;
        scan_copy = *latest_scan_;
    }

    auto walls = detect_walls(scan_copy);
    if (walls.empty()) return;

    const auto & best = walls.front();

    mapper_interfaces::msg::LongestWall msg;
    msg.header.stamp    = scan_copy.header.stamp;
    msg.header.frame_id = scan_copy.header.frame_id.empty()
                              ? "lidar_link" : scan_copy.header.frame_id;
    msg.angle_rad        = best.angle_rad;
    msg.length_m         = best.length_m;
    msg.distance_m       = best.distance_m;
    msg.inlier_count     = best.inlier_count;
    msg.inlier_ratio     = best.inlier_ratio;
    msg.confidence       = best.confidence;
    msg.detection_method = mapper_interfaces::msg::LongestWall::METHOD_HOUGH;
    msg.start.x = best.x1;  msg.start.y = best.y1;  msg.start.z = 0.0;
    msg.end.x   = best.x2;  msg.end.y   = best.y2;  msg.end.z   = 0.0;

    wall_pub_->publish(msg);
}

}  // namespace wall_detector
