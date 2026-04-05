#include <gtest/gtest.h>
#include "amr_motion_control_simulation/trapezoidal_profile.hpp"
#include <cmath>

using namespace amr_motion_control_simulation;

// 1. Basic trapezoidal profile reaches target
TEST(TrapezoidalProfile, ReachesTarget) {
  TrapezoidalProfile p(2.0, 0.3, 0.5);
  auto out = p.getSpeed(2.0);
  EXPECT_EQ(out.phase, ProfilePhase::DONE);
}

// 2. Phase sequence ACCEL -> CRUISE -> DECEL for long path
// accel_dist = 0.3²/(2×0.5) = 0.09 m, decel_start = 3.0 - 0.09 = 2.91 m
TEST(TrapezoidalProfile, PhaseSequence) {
  TrapezoidalProfile p(3.0, 0.3, 0.5);
  auto a = p.getSpeed(0.01);
  EXPECT_EQ(a.phase, ProfilePhase::ACCEL);
  auto c = p.getSpeed(0.5);
  EXPECT_EQ(c.phase, ProfilePhase::CRUISE);
  auto d = p.getSpeed(2.95);
  EXPECT_EQ(d.phase, ProfilePhase::DECEL);
}

// 3. Exit speed continuity
TEST(TrapezoidalProfile, ExitSpeed) {
  TrapezoidalProfile p(2.0, 0.3, 0.5, 0.2);
  auto out = p.getSpeed(2.0);
  EXPECT_EQ(out.phase, ProfilePhase::DONE);
  EXPECT_NEAR(out.speed, 0.2, 0.01);
}

// 4. Entry speed - speed starts at entry_speed, not 0
TEST(TrapezoidalProfile, EntrySpeed) {
  TrapezoidalProfile p(2.0, 0.3, 0.5, 0.0, 0.2);
  auto out = p.getSpeed(0.0);
  EXPECT_NEAR(out.speed, 0.2, 0.03);
  EXPECT_EQ(out.phase, ProfilePhase::ACCEL);
}

// 5. Entry + exit speed continuity across segments
TEST(TrapezoidalProfile, SegmentContinuity) {
  TrapezoidalProfile p1(2.0, 0.3, 0.5, 0.2, 0.0);  // exit=0.2
  TrapezoidalProfile p2(2.0, 0.3, 0.5, 0.0, 0.2);   // entry=0.2
  auto end1 = p1.getSpeed(2.0);
  auto start2 = p2.getSpeed(0.0);
  EXPECT_NEAR(end1.speed, start2.speed, 0.03);
}

// 6. Zero distance returns DONE
TEST(TrapezoidalProfile, ZeroDistance) {
  TrapezoidalProfile p(0.0, 0.3, 0.5);
  auto out = p.getSpeed(0.0);
  EXPECT_EQ(out.phase, ProfilePhase::DONE);
}

// 7. Speed never exceeds max
TEST(TrapezoidalProfile, SpeedCap) {
  TrapezoidalProfile p(3.0, 0.3, 0.5);
  for (double pos = 0; pos < 3.0; pos += 0.01) {
    auto out = p.getSpeed(pos);
    EXPECT_LE(out.speed, 0.3 + 1e-6);
  }
}

// 8. No speed discontinuity between consecutive 1mm steps
// Max delta per step: sqrt(2 * 0.5 * 0.001) ≈ 0.032 m/s < 0.05 threshold
TEST(TrapezoidalProfile, NoPosJump) {
  TrapezoidalProfile p(2.0, 0.3, 0.5);
  double prev_speed = p.getSpeed(0.0).speed;
  for (double pos = 0.001; pos < 2.0; pos += 0.001) {
    auto out = p.getSpeed(pos);
    double diff = std::fabs(out.speed - prev_speed);
    EXPECT_LT(diff, 0.05) << "Speed jump at pos=" << pos;
    prev_speed = out.speed;
  }
}

// 9. Negative distance handled
TEST(TrapezoidalProfile, NegativeDistance) {
  TrapezoidalProfile p(-1.0, 0.3, 0.5);
  auto out = p.getSpeed(0.0);
  EXPECT_EQ(out.phase, ProfilePhase::DONE);
}

// 10. Zero acceleration handled
TEST(TrapezoidalProfile, ZeroAcceleration) {
  TrapezoidalProfile p(2.0, 0.3, 0.0);
  auto out = p.getSpeed(0.0);
  EXPECT_EQ(out.phase, ProfilePhase::DONE);
}
