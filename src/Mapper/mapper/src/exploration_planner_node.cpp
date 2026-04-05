#include "mapper/exploration_planner_node.hpp"
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace mapper {

ExplorationPlannerNode::ExplorationPlannerNode(
    const rclcpp::NodeOptions & options)
: Node("exploration_planner_node", options)
{
    using namespace std::placeholders;

    // C1: 파라미터 선언 및 로드
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.2);
    acceleration_     = declare_parameter("acceleration", 0.3);

    auto map_qos = rclcpp::QoS(3).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            latest_map_ = msg;
        });

    auto cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    server_ = rclcpp_action::create_server<ExploreAction>(
        this, "explore_unknown",
        std::bind(&ExplorationPlannerNode::handle_goal, this, _1, _2),
        std::bind(&ExplorationPlannerNode::handle_cancel, this, _1),
        std::bind(&ExplorationPlannerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), cb_group);
}

GridPath ExplorationPlannerNode::bfs(
    const nav_msgs::msg::OccupancyGrid & map,
    GridCell start, GridCell goal)
{
    int W = static_cast<int>(map.info.width);
    int H = static_cast<int>(map.info.height);
    if (W == 0 || H == 0) return {};

    std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));
    std::vector<std::vector<GridCell>> parent(H,
        std::vector<GridCell>(W, {-1, -1}));

    std::queue<GridCell> q;
    q.push(start);
    visited[start.y][start.x] = true;

    const int dx[] = {1, -1, 0,  0};
    const int dy[] = {0,  0, 1, -1};

    while (!q.empty()) {
        auto cur = q.front(); q.pop();
        if (cur.x == goal.x && cur.y == goal.y) {
            GridPath path;
            GridCell c = goal;
            while (c.x != -1) {
                path.cells.push_back(c);
                GridCell p = parent[c.y][c.x];
                c = p;
            }
            std::reverse(path.cells.begin(), path.cells.end());
            return path;
        }
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (visited[ny][nx]) continue;
            int8_t cell_val = map.data[ny * W + nx];
            if (cell_val > 50) continue;  // 장애물
            visited[ny][nx] = true;
            parent[ny][nx] = cur;
            q.push({nx, ny});
        }
    }
    return {};
}

std::vector<GridCell> ExplorationPlannerNode::find_nearest_unknown_cluster(
    const nav_msgs::msg::OccupancyGrid & map,
    GridCell robot_cell)
{
    int W = static_cast<int>(map.info.width);
    int H = static_cast<int>(map.info.height);
    std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));
    std::queue<GridCell> q;
    q.push(robot_cell);
    visited[robot_cell.y][robot_cell.x] = true;

    const int dx[] = {1, -1, 0,  0};
    const int dy[] = {0,  0, 1, -1};

    while (!q.empty()) {
        auto cur = q.front(); q.pop();
        int idx = cur.y * W + cur.x;
        if (idx >= 0 && idx < static_cast<int>(map.data.size()) &&
            map.data[idx] == -1) {
            return {cur};  // 미탐색 셀 발견
        }
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (visited[ny][nx]) continue;
            if (map.data[ny * W + nx] > 50) continue;  // 장애물 통과 불가
            visited[ny][nx] = true;
            q.push({nx, ny});
        }
    }
    return {};
}

std::vector<ExplorationPlannerNode::Segment>
ExplorationPlannerNode::plan_segments(
    const nav_msgs::msg::OccupancyGrid & map,
    double robot_x, double robot_y, uint8_t /*mode*/)
{
    if (map.info.width == 0) return {};

    double res = map.info.resolution > 0 ? map.info.resolution : 0.05;
    double ox = map.info.origin.position.x;
    double oy = map.info.origin.position.y;

    GridCell robot_cell{
        static_cast<int>((robot_x - ox) / res),
        static_cast<int>((robot_y - oy) / res)
    };

    // 미탐색 목표 셀 탐색
    auto unknowns = find_nearest_unknown_cluster(map, robot_cell);
    if (unknowns.empty()) return {};

    GridCell goal = unknowns[0];
    GridPath path = bfs(map, robot_cell, goal);
    if (path.cells.empty()) return {};

    return path_to_segments(path, map, 0.0);
}

std::vector<ExplorationPlannerNode::Segment>
ExplorationPlannerNode::path_to_segments(
    const GridPath & path,
    const nav_msgs::msg::OccupancyGrid & map,
    double /*robot_yaw_deg*/)
{
    std::vector<Segment> segments;
    if (path.cells.size() < 2) return segments;

    double res = map.info.resolution > 0 ? map.info.resolution : 0.05;
    double ox  = map.info.origin.position.x;
    double oy  = map.info.origin.position.y;

    auto cell_to_world = [&](GridCell c) -> std::pair<double, double> {
        return {ox + (c.x + 0.5) * res, oy + (c.y + 0.5) * res};
    };

    uint32_t seg_id = 0;

    // 방향 계산 헬퍼
    auto get_dir = [](const GridCell & a, const GridCell & b) -> int {
        int ddx = b.x - a.x, ddy = b.y - a.y;
        if (ddx == 1)  return 0;  // 동
        if (ddx == -1) return 1;  // 서
        if (ddy == 1)  return 2;  // 북
        return 3;                 // 남
    };

    // 연속 방향 병합
    size_t seg_start = 0;
    int prev_dir = (path.cells.size() >= 2) ?
        get_dir(path.cells[0], path.cells[1]) : -1;

    for (size_t i = 2; i <= path.cells.size(); ++i) {
        int cur_dir = (i < path.cells.size()) ?
            get_dir(path.cells[i-1], path.cells[i]) : -1;

        if (cur_dir != prev_dir) {
            // 직진 세그먼트 확정
            auto [sx, sy] = cell_to_world(path.cells[seg_start]);
            auto [ex, ey] = cell_to_world(path.cells[i-1]);
            double dist   = std::hypot(ex - sx, ey - sy);
            bool has_next = (i < path.cells.size());

            // 방향에 따른 헤딩
            double heading = (prev_dir == 0) ? 0.0 :
                             (prev_dir == 1) ? 180.0 :
                             (prev_dir == 2) ? 90.0 : 270.0;

            // SPIN 세그먼트 (방향 전환, 첫 번째 제외)
            if (seg_start > 0) {
                Segment spin_seg;
                spin_seg.action_type   = Segment::SPIN;
                spin_seg.segment_id    = seg_id;
                spin_seg.waypoint_from = seg_id;
                spin_seg.waypoint_to   = seg_id + 1;
                spin_seg.spin_angle    = heading;
                spin_seg.has_next      = true;
                spin_seg.hold_steer    = true;
                ++seg_id;
                segments.push_back(spin_seg);
            }

            // YAWCTRL 세그먼트 (직진)
            Segment yaw_seg;
            yaw_seg.action_type      = Segment::YAWCTRL;
            yaw_seg.segment_id       = seg_id;
            yaw_seg.waypoint_from    = seg_id;
            yaw_seg.waypoint_to      = seg_id + 1;
            yaw_seg.start_x          = sx;
            yaw_seg.start_y          = sy;
            yaw_seg.end_x            = ex;
            yaw_seg.end_y            = ey;
            yaw_seg.max_linear_speed = max_linear_speed_;
            yaw_seg.acceleration     = acceleration_;
            yaw_seg.exit_speed       = has_next ? 0.05 : 0.0;
            yaw_seg.has_next         = has_next;
            yaw_seg.hold_steer       = false;
            ++seg_id;
            segments.push_back(yaw_seg);

            seg_start = i - 1;
            prev_dir = cur_dir;
        }
    }
    return segments;
}

rclcpp_action::GoalResponse ExplorationPlannerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExploreAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExplorationPlannerNode::handle_cancel(
    std::shared_ptr<GoalHandle>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ExplorationPlannerNode::handle_accepted(
    std::shared_ptr<GoalHandle> goal_handle)
{
    auto self = std::static_pointer_cast<ExplorationPlannerNode>(shared_from_this());
    std::thread([self, goal_handle]() {
        self->execute(goal_handle);
    }).detach();
}

void ExplorationPlannerNode::execute(
    std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal     = goal_handle->get_goal();
    auto result   = std::make_shared<ExploreAction::Result>();

    nav_msgs::msg::OccupancyGrid map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (latest_map_) map_copy = *latest_map_;
    }

    auto segs = plan_segments(map_copy, 0.0, 0.0, goal->mode);

    result->coverage_percent = 0.0f;
    result->status = segs.empty() ? -1 : 0;
    goal_handle->succeed(result);
}

}  // namespace mapper
