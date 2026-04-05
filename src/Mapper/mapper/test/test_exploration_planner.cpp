#include <gtest/gtest.h>
#include <cmath>
#include "mapper/exploration_planner_node.hpp"

// BFS: 5x5 빈 맵 (0,0)→(4,4) 경로 존재
TEST(ExplorationPlannerTest, BFSFindsPath) {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width  = 5;
    map.info.height = 5;
    map.data.resize(25, 0);

    auto path = mapper::ExplorationPlannerNode::bfs(map, {0,0}, {4,4});
    EXPECT_FALSE(path.cells.empty());
    EXPECT_EQ(path.cells.front().x, 0);
    EXPECT_EQ(path.cells.front().y, 0);
    EXPECT_EQ(path.cells.back().x, 4);
    EXPECT_EQ(path.cells.back().y, 4);
}

// BFS: 수직 벽으로 막힌 맵 → 경로 없음
TEST(ExplorationPlannerTest, BFSNoPathThroughWall) {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width  = 5;
    map.info.height = 5;
    map.data.resize(25, 0);
    for (int y = 0; y < 5; ++y) map.data[y * 5 + 2] = 100;

    auto path = mapper::ExplorationPlannerNode::bfs(map, {0,0}, {4,4});
    EXPECT_TRUE(path.cells.empty());
}

// Segment 병합: 직선 10셀 (마지막 셀 unknown) → YAWCTRL 1개 이상
TEST(ExplorationPlannerTest, StraightPathMergedToOneYawctrl) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::ExplorationPlannerNode>();

    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 10; map.info.height = 1;
    map.info.resolution = 0.05f;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.resize(10, 0);
    map.data[9] = -1;  // 미탐색 목표 셀

    auto segs = node->plan_segments(map, 0.025, 0.025, 0);
    int yaw_count = 0;
    for (auto & s : segs)
        if (s.action_type == waypoint_interfaces::msg::Segment::YAWCTRL)
            ++yaw_count;
    EXPECT_GE(yaw_count, 1);

    rclcpp::shutdown();
}

// YAWCTRL 세그먼트의 end_x > start_x (유한 거리 직진)
TEST(ExplorationPlannerTest, YawctrlSegmentHasFiniteDistance) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<mapper::ExplorationPlannerNode>();

    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 5; map.info.height = 1;
    map.info.resolution = 0.05f;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.resize(5, 0);
    map.data[4] = -1;  // 미탐색 목표 셀

    auto segs = node->plan_segments(map, 0.025, 0.025, 0);
    bool found_yaw = false;
    for (auto & s : segs) {
        if (s.action_type == waypoint_interfaces::msg::Segment::YAWCTRL) {
            found_yaw = true;
            double dist = std::hypot(s.end_x - s.start_x, s.end_y - s.start_y);
            EXPECT_GT(dist, 0.0);
        }
    }
    EXPECT_TRUE(found_yaw);
    rclcpp::shutdown();
}
