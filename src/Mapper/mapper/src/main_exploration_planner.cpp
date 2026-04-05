#include <rclcpp/rclcpp.hpp>
#include "mapper/exploration_planner_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<mapper::ExplorationPlannerNode>();
    exec->add_node(node);
    exec->spin();
    rclcpp::shutdown();
    return 0;
}
