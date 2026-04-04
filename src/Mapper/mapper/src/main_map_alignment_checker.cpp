#include <rclcpp/rclcpp.hpp>
#include "mapper/map_alignment_checker_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<mapper::MapAlignmentCheckerNode>();
    exec->add_node(node);
    exec->spin();
    rclcpp::shutdown();
    return 0;
}
