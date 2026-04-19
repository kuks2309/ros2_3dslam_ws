#include <rclcpp/rclcpp.hpp>
#include "wall_detector/wall_detector_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wall_detector::WallDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
