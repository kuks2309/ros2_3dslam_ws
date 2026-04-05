#include <gtest/gtest.h>
#include "mapper/mapper_orchestrator_node.hpp"

class OrchestratorTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(OrchestratorTest, InitialStateIsIdle) {
    auto node = std::make_shared<mapper::MapperOrchestratorNode>();
    EXPECT_EQ(node->get_state(), mapper::MapperState::IDLE);
}

TEST_F(OrchestratorTest, StartMappingFromIdle) {
    auto node = std::make_shared<mapper::MapperOrchestratorNode>();

    auto req = std::make_shared<mapper_interfaces::srv::MapperCommand::Request>();
    req->command = mapper_interfaces::srv::MapperCommand::Request::CMD_START_MAPPING;
    auto res = std::make_shared<mapper_interfaces::srv::MapperCommand::Response>();
    node->handle_command_public(req, res);

    // CMD_START_MAPPING transitions to ALIGNING asynchronously
    EXPECT_TRUE(res->success);
    EXPECT_EQ(res->message, "Mapping started");

    // Stop before node destruction so detached thread exits its poll loop
    auto stop_req = std::make_shared<mapper_interfaces::srv::MapperCommand::Request>();
    stop_req->command = mapper_interfaces::srv::MapperCommand::Request::CMD_STOP;
    auto stop_res = std::make_shared<mapper_interfaces::srv::MapperCommand::Response>();
    node->handle_command_public(stop_req, stop_res);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

TEST_F(OrchestratorTest, StopFromAnyStateGoesToIdle) {
    auto node = std::make_shared<mapper::MapperOrchestratorNode>();

    auto req = std::make_shared<mapper_interfaces::srv::MapperCommand::Request>();
    req->command = mapper_interfaces::srv::MapperCommand::Request::CMD_STOP;
    auto res = std::make_shared<mapper_interfaces::srv::MapperCommand::Response>();
    node->handle_command_public(req, res);

    EXPECT_EQ(node->get_state(), mapper::MapperState::IDLE);
    EXPECT_TRUE(res->success);
}

TEST_F(OrchestratorTest, PauseRejectsWhenIdle) {
    auto node = std::make_shared<mapper::MapperOrchestratorNode>();

    auto req = std::make_shared<mapper_interfaces::srv::MapperCommand::Request>();
    req->command = mapper_interfaces::srv::MapperCommand::Request::CMD_PAUSE;
    auto res = std::make_shared<mapper_interfaces::srv::MapperCommand::Response>();
    node->handle_command_public(req, res);

    EXPECT_FALSE(res->success);
    EXPECT_EQ(node->get_state(), mapper::MapperState::IDLE);
    // Note: MAPPING_AUTO is intentionally not in the CMD_PAUSE allowed states.
    // Autonomous mapping cannot be paused mid-run; use CMD_STOP instead.
}

TEST_F(OrchestratorTest, SaveMapReturnsNotImplemented) {
    auto node = std::make_shared<mapper::MapperOrchestratorNode>();

    auto req = std::make_shared<mapper_interfaces::srv::MapperCommand::Request>();
    auto res = std::make_shared<mapper_interfaces::srv::MapperCommand::Response>();
    req->command = mapper_interfaces::srv::MapperCommand::Request::CMD_SAVE_MAP;
    node->handle_command_public(req, res);

    EXPECT_FALSE(res->success);
    EXPECT_NE(res->message.find("not yet implemented"), std::string::npos);
}
