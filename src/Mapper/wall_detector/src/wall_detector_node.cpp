#include "wall_detector/wall_detector_node.hpp"
#include <cmath>
#include <algorithm>
#include <array>
#include <cstdio>
#include <geometry_msgs/msg/point.hpp>

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
    min_distance_m_      = declare_parameter("min_distance_m",      0.0);
    max_distance_m_      = declare_parameter("max_distance_m",      1e6);
    publish_top_n_        = static_cast<int>(declare_parameter("publish_top_n", 4));
    marker_line_width_m_  = declare_parameter("marker_line_width_m",  0.10);
    merge_angle_tol_deg_  = declare_parameter("merge_angle_tol_deg",  3.0);
    merge_distance_tol_m_ = declare_parameter("merge_distance_tol_m", 0.2);
    scan_topic_          = declare_parameter("scan_topic",
                                              std::string("scan"));
    output_topic_        = declare_parameter("output_topic",
                                              std::string("wall_detector/longest_wall"));
    marker_topic_        = declare_parameter("marker_topic",
                                              std::string("wall_detector/wall_markers"));

    // /scan 구독 — 프로젝트 관례 (BEST_EFFORT, VOLATILE)
    auto scan_qos = rclcpp::QoS(5).best_effort().durability_volatile();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, scan_qos,
        std::bind(&WallDetectorNode::scan_callback, this, std::placeholders::_1));

    // 결과 퍼블리시 — RELIABLE, VOLATILE
    auto wall_qos = rclcpp::QoS(10).reliable().durability_volatile();
    wall_pub_ = create_publisher<mapper_interfaces::msg::LongestWall>(
        output_topic_, wall_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, rclcpp::QoS(1).reliable().durability_volatile());

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    pub_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&WallDetectorNode::publish_longest_wall, this));

    RCLCPP_INFO(get_logger(),
        "wall_detector started | scan=%s longest=%s markers=%s "
        "top_n=%d rate=%.1fHz min_len=%.2fm dist=[%.2f,%.2f]m "
        "merge=(±%.1f°, ±%.2fm) score=length×inlier_ratio",
        scan_topic_.c_str(), output_topic_.c_str(), marker_topic_.c_str(),
        publish_top_n_, publish_rate_hz_, min_line_len_m_,
        min_distance_m_, max_distance_m_,
        merge_angle_tol_deg_, merge_distance_tol_m_);
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

    // ── Stage 1: 중복 Hough 선분 merge ─────────────────────────────
    // 같은 물리적 벽이 (angle ±3°, distance ±0.2m) 안에서 여러 번 검출되는 것을
    // 하나의 대표 선분(클러스터 내 최장)으로 통합 → 프레임간 순위 안정화.
    const double angle_tol_rad = merge_angle_tol_deg_ * M_PI / 180.0;
    std::vector<WallSegment> merged;
    merged.reserve(result.size());
    for (const auto & w : result) {
        bool absorbed = false;
        for (auto & m : merged) {
            // normalize_180(각도차): 선분은 방향 부호 무관이므로 [-π/2, π/2) 정규화 후 비교
            double da_rad = w.angle_rad - m.angle_rad;
            da_rad = normalize_angle_rad(da_rad);
            double da = std::abs(da_rad);
            double dd = std::abs(w.distance_m - m.distance_m);
            if (da < angle_tol_rad && dd < merge_distance_tol_m_) {
                // 같은 벽 → 대표는 length 더 긴 것으로 교체
                if (w.length_m > m.length_m) m = w;
                absorbed = true;
                break;
            }
        }
        if (!absorbed) merged.push_back(w);
    }

    // ── Stage 2: composite score ranking (Codex 권고) ──────────────
    // length × inlier_ratio : 짧지만 밀도 높은 가짜, 길지만 지지점 빈약한 선분 둘 다 억제
    std::sort(merged.begin(), merged.end(),
        [](const WallSegment & a, const WallSegment & b) {
            return (a.length_m * a.inlier_ratio) > (b.length_m * b.inlier_ratio);
        });
    return merged;
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

    // distance 필터만 적용 (detect_walls에서 이미 composite score로 정렬됨)
    std::vector<WallSegment> filtered;
    filtered.reserve(walls.size());
    for (const auto & w : walls) {
        if (w.distance_m < min_distance_m_ || w.distance_m > max_distance_m_) continue;
        filtered.push_back(w);
    }
    if (filtered.empty()) return;

    const std::string frame_id = scan_copy.header.frame_id.empty()
        ? "lidar_link" : scan_copy.header.frame_id;

    // (1) 최장 벽을 LongestWall 단일 메시지로 발행 (wall_aligner용)
    const auto & best = filtered.front();
    mapper_interfaces::msg::LongestWall msg;
    msg.header.stamp     = scan_copy.header.stamp;
    msg.header.frame_id  = frame_id;
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

    // (2) 상위 N개 벽을 색상 차별화 MarkerArray로 발행 (RViz 시각화)
    // 색상표 — 라이다 초록(green) / 기존 빨강(red) 회피
    //   rank 0 (최장) = Yellow
    //   rank 1        = Cyan
    //   rank 2        = Magenta
    //   rank 3        = Orange
    static const std::array<std::array<float,3>, 4> kColors = {{
        {1.0f, 1.0f, 0.0f},   // yellow  — #1
        {0.0f, 1.0f, 1.0f},   // cyan    — #2
        {1.0f, 0.0f, 1.0f},   // magenta — #3
        {1.0f, 0.5f, 0.0f},   // orange  — #4
    }};

    visualization_msgs::msg::MarkerArray arr;

    // 이전 마커 삭제 (DELETEALL)
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp    = scan_copy.header.stamp;
    del.action          = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int n = std::min(publish_top_n_, static_cast<int>(filtered.size()));
    for (int i = 0; i < n; ++i) {
        const auto & w = filtered[i];
        const auto & c = kColors[std::min<size_t>(i, kColors.size() - 1)];

        // 선분 (LINE_STRIP)
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp    = scan_copy.header.stamp;
        m.ns     = "wall_lines";
        m.id     = i;
        m.type   = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = marker_line_width_m_;
        m.color.r = c[0]; m.color.g = c[1]; m.color.b = c[2]; m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;
        geometry_msgs::msg::Point p1, p2;
        p1.x = w.x1; p1.y = w.y1; p1.z = 0.05;
        p2.x = w.x2; p2.y = w.y2; p2.z = 0.05;
        m.points.push_back(p1);
        m.points.push_back(p2);
        arr.markers.push_back(m);

        // 텍스트 라벨 ("#N  L=... D=...")
        visualization_msgs::msg::Marker t;
        t.header = m.header;
        t.ns     = "wall_labels";
        t.id     = i;
        t.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.pose.position.x = (w.x1 + w.x2) / 2.0;
        t.pose.position.y = (w.y1 + w.y2) / 2.0;
        t.pose.position.z = 0.5;
        t.pose.orientation.w = 1.0;
        t.scale.z = 0.3;
        t.color.r = c[0]; t.color.g = c[1]; t.color.b = c[2]; t.color.a = 1.0f;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "#%d  L=%.1fm D=%.1fm",
                      i + 1, w.length_m, w.distance_m);
        t.text = buf;
        arr.markers.push_back(t);
    }
    marker_pub_->publish(arr);
}

}  // namespace wall_detector
