#include "amr_motion_control_simulation/trapezoidal_profile.hpp"

#include <algorithm>
#include <cmath>

namespace amr_motion_control_simulation
{

TrapezoidalProfile::TrapezoidalProfile(
  double target_distance,
  double max_speed,
  double acceleration,
  double exit_speed,
  double entry_speed)
: target_distance_(target_distance),
  max_speed_(max_speed),
  acceleration_(acceleration),
  exit_speed_(exit_speed),
  entry_speed_(entry_speed),
  peak_speed_(0.0),
  accel_dist_(0.0),
  decel_start_(0.0),
  is_triangular_(false)
{
  // Guard: degenerate profiles complete immediately
  if (target_distance_ <= 0.0 || acceleration_ <= 0.0) {
    peak_speed_ = exit_speed_;
    accel_dist_ = 0.0;
    decel_start_ = 0.0;
    return;
  }

  // Distance required to accelerate from entry_speed to max_speed
  double d_accel_full = (max_speed_ * max_speed_ - entry_speed_ * entry_speed_) /
                        (2.0 * acceleration_);

  // Distance required to decelerate from max_speed to exit_speed
  double d_decel_full = (max_speed_ * max_speed_ - exit_speed_ * exit_speed_) /
                        (2.0 * acceleration_);

  if (d_accel_full + d_decel_full <= target_distance_) {
    // Standard trapezoidal profile: full cruise phase at max_speed
    is_triangular_ = false;
    peak_speed_   = max_speed_;
    accel_dist_   = d_accel_full;
    decel_start_  = target_distance_ - d_decel_full;
  } else {
    // Triangular profile: no cruise phase — compute achievable peak speed
    // Derived from: d_accel + d_decel == target_distance
    //   (v² - v_entry²)/(2a) + (v² - v_exit²)/(2a) = D
    //   2v² = 2aD + v_entry² + v_exit²
    //   v = sqrt((2aD + v_entry² + v_exit²) / 2)
    is_triangular_ = true;
    peak_speed_ = std::sqrt(
      (2.0 * acceleration_ * target_distance_ +
       entry_speed_ * entry_speed_ +
       exit_speed_ * exit_speed_) / 2.0);

    // Peak must be at least as fast as either boundary speed
    peak_speed_ = std::max(peak_speed_, std::max(entry_speed_, exit_speed_));

    // Clamp to max_speed in case of numerical overshoot
    peak_speed_ = std::min(peak_speed_, max_speed_);

    accel_dist_  = (peak_speed_ * peak_speed_ - entry_speed_ * entry_speed_) /
                   (2.0 * acceleration_);
    decel_start_ = accel_dist_;
  }
}

ProfileOutput TrapezoidalProfile::getSpeed(double current_position) const
{
  ProfileOutput out{};
  double pos = std::max(0.0, std::min(current_position, target_distance_));

  // Guard: degenerate profile (target_distance <= 0 or acceleration <= 0)
  if (target_distance_ <= 0.0 || acceleration_ <= 0.0) {
    out.speed = exit_speed_;
    out.phase = ProfilePhase::DONE;
    return out;
  }

  if (pos >= target_distance_) {
    out.speed = exit_speed_;
    out.phase = ProfilePhase::DONE;
    return out;
  }

  if (pos < accel_dist_) {
    // Acceleration phase: v = sqrt(v_entry² + 2*a*pos)
    out.speed = std::sqrt(entry_speed_ * entry_speed_ + 2.0 * acceleration_ * pos);
    out.speed = std::min(out.speed, peak_speed_);
    out.phase = ProfilePhase::ACCEL;
  } else if (pos < decel_start_) {
    // Cruise phase: constant speed
    out.speed = peak_speed_;
    out.phase = ProfilePhase::CRUISE;
  } else {
    // Deceleration phase: v = sqrt(v_exit² + 2*a*(D - pos))
    double remaining = target_distance_ - pos;
    out.speed = std::sqrt(exit_speed_ * exit_speed_ + 2.0 * acceleration_ * remaining);
    out.speed = std::min(out.speed, peak_speed_);
    out.phase = ProfilePhase::DECEL;
  }

  // Clamp to valid speed range
  out.speed = std::max(0.0, std::min(out.speed, max_speed_));

  return out;
}

}  // namespace amr_motion_control_simulation
