#include <rclcpp/rclcpp.hpp>
#include "mapper/mapper_orchestrator_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<mapper::MapperOrchestratorNode>();
    exec->add_node(node);
    exec->spin();
    rclcpp::shutdown();
    return 0;
}
