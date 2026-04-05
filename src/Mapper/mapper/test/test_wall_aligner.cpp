#include <gtest/gtest.h>
#include "mapper/wall_aligner_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

TEST(WallAlignerTest, DetectsHorizontalWall) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::WallAlignerNode>();

    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = -M_PI / 2;
    scan.angle_max =  M_PI / 2;
    scan.angle_increment = 0.01f;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;

    int n = (int)((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(n);
    for (int i = 0; i < n; ++i) {
        double a = scan.angle_min + i * scan.angle_increment;
        if (std::abs(std::sin(a)) > 0.05) {
            scan.ranges[i] = (float)std::abs(1.0 / std::sin(a));
        } else {
            scan.ranges[i] = scan.range_max;
        }
    }

    auto result = node->detect_longest_wall(scan);
    EXPECT_GT(result.inlier_count, 10);
    EXPECT_GE(result.angle_deg, 0.0);
    EXPECT_LT(result.angle_deg, 180.0);
    rclcpp::shutdown();
}

TEST(WallAlignerTest, AngleNormalized0To180) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::WallAlignerNode>();

    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = -M_PI / 2;
    scan.angle_max =  M_PI / 2;
    scan.angle_increment = 0.01f;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    int n = (int)((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(n, 2.0f);

    auto result = node->detect_longest_wall(scan);
    EXPECT_GE(result.angle_deg, 0.0);
    EXPECT_LT(result.angle_deg, 180.0);
    rclcpp::shutdown();
}
