#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "wall_detector/wall_detector_node.hpp"

using sensor_msgs::msg::LaserScan;

class WallDetectorTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()   { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// 합성 스캔 생성: y=-5m 위치의 가로 벽 (길이 10m)
static LaserScan make_horizontal_wall_scan()
{
    LaserScan s;
    s.angle_min       = -M_PI;
    s.angle_max       =  M_PI;
    s.angle_increment = M_PI / 180.0;  // 1도 간격
    s.range_min       = 0.05;
    s.range_max       = 50.0;
    const double wall_y = -5.0;
    const double wall_half_len = 5.0;

    int n = static_cast<int>((s.angle_max - s.angle_min) / s.angle_increment) + 1;
    s.ranges.resize(n);
    for (int i = 0; i < n; ++i) {
        double a = s.angle_min + i * s.angle_increment;
        // y = wall_y 와 광선 (cos, sin) 교점
        double sa = std::sin(a);
        if (sa > -0.01) {
            s.ranges[i] = s.range_max;  // 벽 방향 아님
            continue;
        }
        double r = wall_y / sa;
        double x = r * std::cos(a);
        if (std::abs(x) > wall_half_len) {
            s.ranges[i] = s.range_max;
        } else {
            s.ranges[i] = static_cast<float>(r);
        }
    }
    return s;
}

TEST_F(WallDetectorTest, DetectsHorizontalWallAtZeroDegrees) {
    auto node = std::make_shared<wall_detector::WallDetectorNode>();
    auto scan = make_horizontal_wall_scan();

    auto walls = node->detect_walls(scan);
    ASSERT_FALSE(walls.empty()) << "탐지된 벽 없음";

    const auto & best = walls.front();
    EXPECT_NEAR(best.angle_rad, 0.0, 5.0 * M_PI / 180.0)
        << "수평 벽 각도 0 rad 기대, 실제 " << best.angle_rad;
    EXPECT_GT(best.length_m, 5.0) << "최장 벽 길이 5m 이상 기대";
    EXPECT_NEAR(best.distance_m, 5.0, 0.3) << "벽까지 거리 5m ± 0.3";
}

TEST_F(WallDetectorTest, EmptyScanProducesNoWalls) {
    auto node = std::make_shared<wall_detector::WallDetectorNode>();
    LaserScan empty;
    empty.angle_min       = -M_PI;
    empty.angle_max       =  M_PI;
    empty.angle_increment = M_PI / 180.0;
    empty.range_min       = 0.05;
    empty.range_max       = 50.0;
    empty.ranges.assign(361, static_cast<float>(empty.range_max));

    auto walls = node->detect_walls(empty);
    EXPECT_TRUE(walls.empty());
}
