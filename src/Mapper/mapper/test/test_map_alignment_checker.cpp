#include <gtest/gtest.h>
#include "mapper/map_alignment_checker_node.hpp"

// 수직/수평 벽 OGM → max_error 낮음 (정렬됨)
TEST(MapAlignmentCheckerTest, AlignedMapPassesCheck) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::MapAlignmentCheckerNode>();

    nav_msgs::msg::OccupancyGrid map;
    map.info.width  = 100;
    map.info.height = 100;
    map.info.resolution = 0.05f;
    map.data.resize(100 * 100, 0);
    // 수평 벽 (y=10)
    for (int x = 0; x < 100; ++x) map.data[10 * 100 + x] = 100;
    // 수직 벽 (x=10)
    for (int y = 0; y < 100; ++y) map.data[y * 100 + 10] = 100;

    double error = node->check_alignment(map, 2.0);
    EXPECT_LT(error, 2.0);

    rclcpp::shutdown();
}

// 45도 기울어진 벽 → max_error 높음 (미정렬)
TEST(MapAlignmentCheckerTest, MisalignedMapFailsCheck) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::MapAlignmentCheckerNode>();

    nav_msgs::msg::OccupancyGrid map;
    map.info.width  = 100;
    map.info.height = 100;
    map.data.resize(100 * 100, 0);
    // 45도 대각선
    for (int i = 0; i < 100; ++i) map.data[i * 100 + i] = 100;

    double error = node->check_alignment(map, 2.0);
    EXPECT_GT(error, 2.0);

    rclcpp::shutdown();
}

// 직선 없는 맵 → fallback (error=0, 통과)
TEST(MapAlignmentCheckerTest, EmptyMapFallbackPasses) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::MapAlignmentCheckerNode>();

    nav_msgs::msg::OccupancyGrid map;
    map.info.width  = 100;
    map.info.height = 100;
    map.data.resize(100 * 100, 0);

    double error = node->check_alignment(map, 2.0);
    EXPECT_EQ(error, 0.0);

    rclcpp::shutdown();
}
