#include <gtest/gtest.h>
#include "amr_motion_control_simulation/path_controller_2wd.hpp"
#include <cmath>

using namespace amr_motion_control_simulation;

class PathController2WDTest : public ::testing::Test {
protected:
  PathController2WD::Params params;
  void SetUp() override { params = PathController2WD::Params{}; }
};

// 1. On-path robot produces near-zero corrections
TEST_F(PathController2WDTest, OnPathZeroCorrection) {
  PathController2WD ctrl(params);
  ctrl.setPath(0, 0, 2, 0);  // straight line along X
  auto out = ctrl.update(0.5, 0.0, 0.0, 0.3, 0.05);  // on path, facing right
  EXPECT_NEAR(out.e_d, 0.0, 0.01);
  EXPECT_NEAR(out.e_theta, 0.0, 0.01);
  EXPECT_NEAR(out.omega, 0.0, 0.1);
}

// 2. Lateral offset produces CTE
TEST_F(PathController2WDTest, LateralOffsetCTE) {
  PathController2WD ctrl(params);
  ctrl.setPath(0, 0, 2, 0);
  auto out = ctrl.update(0.5, 0.1, 0.0, 0.3, 0.05);  // 0.1m off to left
  EXPECT_NEAR(out.e_d, 0.1, 0.02);
  EXPECT_NE(out.omega, 0.0);  // should correct
}

// 3. Heading error produces correction
TEST_F(PathController2WDTest, HeadingErrorCorrection) {
  PathController2WD ctrl(params);
  ctrl.setPath(0, 0, 2, 0);
  auto out = ctrl.update(0.5, 0.0, 0.1, 0.3, 0.05);  // 0.1 rad heading error
  EXPECT_GT(std::fabs(out.e_theta), 0.05);
  EXPECT_NE(out.omega, 0.0);
}

// 4. Reset clears state
TEST_F(PathController2WDTest, ResetClearsState) {
  PathController2WD ctrl(params);
  ctrl.setPath(0, 0, 2, 0);
  ctrl.update(0.5, 0.1, 0.1, 0.3, 0.05);
  ctrl.reset();
  ctrl.setPath(0, 0, 0, 2);  // new path
  auto out = ctrl.update(0.0, 0.5, M_PI/2, 0.3, 0.05);
  EXPECT_NEAR(out.e_d, 0.0, 0.02);
}

// 5. Diagonal path works
TEST_F(PathController2WDTest, DiagonalPath) {
  PathController2WD ctrl(params);
  ctrl.setPath(0, 0, 1, 1);  // 45 degree path
  auto out = ctrl.update(0.5, 0.5, M_PI/4, 0.3, 0.05);  // on path
  EXPECT_NEAR(out.e_d, 0.0, 0.02);
  EXPECT_NEAR(out.e_theta, 0.0, 0.02);
}

// 6. Low speed disables Stanley
TEST_F(PathController2WDTest, LowSpeedNoStanley) {
  PathController2WD ctrl(params);
  ctrl.setPath(0, 0, 2, 0);
  auto out = ctrl.update(0.5, 0.1, 0.0, 0.0001, 0.05);  // near-zero speed
  // Should still produce some output (from PD), but Stanley term should be ~0
  // Just verify no crash/NaN
  EXPECT_FALSE(std::isnan(out.omega));
}
