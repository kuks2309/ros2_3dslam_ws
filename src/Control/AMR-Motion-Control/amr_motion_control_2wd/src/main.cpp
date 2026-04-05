#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "amr_motion_control_2wd/spin_action_server.hpp"
#include "amr_motion_control_2wd/turn_action_server.hpp"
#include "amr_motion_control_2wd/yaw_control_action_server.hpp"
#include "amr_motion_control_2wd/translate_action_server.hpp"
#include "amr_motion_control_2wd/translate_reverse_action_server.hpp"
#include "amr_motion_control_2wd/pure_pursuit_action_server.hpp"
#include "amr_motion_control_2wd/stanley_action_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("amr_motion_control_2wd");

  auto spin_server = std::make_shared<amr_motion_control_2wd::SpinActionServer>(node);
  auto turn_server = std::make_shared<amr_motion_control_2wd::TurnActionServer>(node);
  auto yaw_control_server =
    std::make_shared<amr_motion_control_2wd::YawControlActionServer>(node);
  auto translate_server =
    std::make_shared<amr_motion_control_2wd::TranslateActionServer>(node);
  auto translate_reverse_server =
    std::make_shared<amr_motion_control_2wd::TranslateReverseActionServer>(node);
  auto pure_pursuit_server =
    std::make_shared<amr_motion_control_2wd::PurePursuitActionServer>(node);
  auto stanley_server = std::make_shared<amr_motion_control_2wd::StanleyActionServer>(node);

  RCLCPP_INFO(node->get_logger(), "amr_motion_control_2wd node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
