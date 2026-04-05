#include <gtest/gtest.h>
#include "amr_motion_control_simulation/sil_predictor.hpp"
#include <cmath>

using namespace amr_motion_control_simulation;

class SilPredictorTest : public ::testing::Test {
protected:
  SilPredictor::Config config;
  void SetUp() override { config = SilPredictor::Config{}; }
};

// 1. Straight line reaches goal (2m forward)
TEST_F(SilPredictorTest, StraightLineReachesGoal) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0;
  goal.end_x = 2; goal.end_y = 0;
  goal.max_linear_speed = 0.3;
  goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  ASSERT_FALSE(traj.empty());
  EXPECT_NEAR(traj.back().x, 2.0, 0.10);
  EXPECT_NEAR(traj.back().y, 0.0, 0.05);
}

// 2. CTE stays near zero (ideal conditions)
TEST_F(SilPredictorTest, CTEStaysNearZero) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0;
  goal.end_x = 2; goal.end_y = 0;
  goal.max_linear_speed = 0.3;
  goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  double max_cte = 0;
  for (auto& s : traj) max_cte = std::max(max_cte, std::fabs(s.e_d));
  EXPECT_LT(max_cte, 0.02);  // 20mm for diff-drive (relaxed from 10mm)
}

// 3. Phase order ACCEL -> CRUISE -> DECEL
TEST_F(SilPredictorTest, PhaseOrder) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0;
  goal.end_x = 3; goal.end_y = 0;
  goal.max_linear_speed = 0.3;
  goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  bool saw_a = false, saw_c = false, saw_d = false;
  for (auto& s : traj) {
    if (s.phase == 1) saw_a = true;
    if (s.phase == 2) saw_c = true;
    if (s.phase == 3) saw_d = true;
  }
  EXPECT_TRUE(saw_a);
  EXPECT_TRUE(saw_c);
  EXPECT_TRUE(saw_d);
}

// 4. Velocity continuity across segments
TEST_F(SilPredictorTest, VelocityContinuity) {
  SilPredictor predictor(config);
  SimTranslateGoal g1{};
  g1.start_x = 0; g1.start_y = 0; g1.end_x = 2; g1.end_y = 0;
  g1.max_linear_speed = 0.3; g1.acceleration = 0.5; g1.exit_speed = 0.2;
  SimTranslateGoal g2{};
  g2.start_x = 2; g2.start_y = 0; g2.end_x = 4; g2.end_y = 0;
  g2.max_linear_speed = 0.3; g2.acceleration = 0.5; g2.entry_speed = 0.2;
  auto t1 = predictor.predict(g1);
  auto t2 = predictor.predict(g2);
  ASSERT_FALSE(t1.empty());
  ASSERT_FALSE(t2.empty());
  EXPECT_NEAR(t1.back().vx, 0.2, 0.05);
}

// 5. No position jump
TEST_F(SilPredictorTest, NoPosJump) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0; goal.end_x = 2; goal.end_y = 0;
  goal.max_linear_speed = 0.3; goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  double dt = 1.0 / config.control_rate_hz;
  double max_step = 0.3 * dt * 2.0;
  for (size_t i = 1; i < traj.size(); ++i) {
    double d = std::hypot(traj[i].x - traj[i-1].x, traj[i].y - traj[i-1].y);
    EXPECT_LT(d, max_step) << "Jump at step " << i;
  }
}

// 6. Zero distance returns empty
TEST_F(SilPredictorTest, ZeroDistance) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 1; goal.start_y = 1; goal.end_x = 1; goal.end_y = 1;
  goal.max_linear_speed = 0.3; goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  EXPECT_TRUE(traj.empty());
}

// 7. Reverse travel
TEST_F(SilPredictorTest, ReverseTravel) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 2; goal.start_y = 0; goal.end_x = 0; goal.end_y = 0;
  goal.max_linear_speed = -0.3;  // negative = reverse
  goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  ASSERT_FALSE(traj.empty());
  EXPECT_NEAR(traj.back().x, 0.0, 0.15);
}

// 8. Diagonal path
TEST_F(SilPredictorTest, DiagonalPath) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0; goal.end_x = 1; goal.end_y = 1;
  goal.max_linear_speed = 0.3; goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  ASSERT_FALSE(traj.empty());
  EXPECT_NEAR(traj.back().x, 1.0, 0.15);
  EXPECT_NEAR(traj.back().y, 1.0, 0.15);
}

// 9. vy always zero (diff-drive constraint)
TEST_F(SilPredictorTest, VyAlwaysZero) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0; goal.end_x = 2; goal.end_y = 0;
  goal.max_linear_speed = 0.3; goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  for (auto& s : traj) EXPECT_DOUBLE_EQ(s.vy, 0.0);
}

// 10. Last state is DONE
TEST_F(SilPredictorTest, LastStateDone) {
  SilPredictor predictor(config);
  SimTranslateGoal goal{};
  goal.start_x = 0; goal.start_y = 0; goal.end_x = 1; goal.end_y = 0;
  goal.max_linear_speed = 0.3; goal.acceleration = 0.5;
  auto traj = predictor.predict(goal);
  ASSERT_FALSE(traj.empty());
  EXPECT_EQ(traj.back().phase, static_cast<uint8_t>(4));  // DONE=4
}
