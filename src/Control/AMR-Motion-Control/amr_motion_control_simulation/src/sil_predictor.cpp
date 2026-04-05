#include "amr_motion_control_simulation/sil_predictor.hpp"

#include "amr_motion_control_simulation/path_controller_2wd.hpp"
#include "amr_motion_control_simulation/trapezoidal_profile.hpp"

#include <algorithm>
#include <cmath>

namespace amr_motion_control_simulation
{

namespace
{

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

}  // namespace

SilPredictor::SilPredictor()
: config_()
{}

SilPredictor::SilPredictor(const Config & config)
: config_(config)
{}

std::vector<SimState> SilPredictor::predict(const SimTranslateGoal & goal) const
{
  const double dt = 1.0 / config_.control_rate_hz;
  const int max_steps = static_cast<int>(config_.max_timeout_sec * config_.control_rate_hz);

  // Path geometry
  const double dx = goal.end_x - goal.start_x;
  const double dy = goal.end_y - goal.start_y;
  const double path_angle = std::atan2(dy, dx);
  const double target_distance = std::hypot(dx, dy);

  if (target_distance < 1e-6) {
    return {};
  }

  // Reverse detection: negative max_linear_speed means reverse motion
  const bool is_reverse = (goal.max_linear_speed < 0.0);
  const double abs_max_speed = std::fabs(goal.max_linear_speed);

  // PathController2WD setup
  PathController2WD::Params pc_params;
  pc_params.K_stanley = is_reverse ? config_.K_stanley_reverse : config_.K_stanley;
  pc_params.K_soft = config_.K_soft;
  pc_params.Kp_heading = config_.Kp_heading;
  pc_params.Kd_heading = config_.Kd_heading;
  pc_params.heading_filter_window = config_.heading_filter_window;

  PathController2WD path_ctrl(pc_params);
  path_ctrl.setPath(goal.start_x, goal.start_y, goal.end_x, goal.end_y);

  // TrapezoidalProfile: always operates on positive distances and speeds
  TrapezoidalProfile profile(
    target_distance,
    abs_max_speed,
    goal.acceleration,
    std::fabs(goal.exit_speed),
    std::fabs(goal.entry_speed));

  // Initial state
  double x = goal.start_x;
  double y = goal.start_y;
  // Use provided start_yaw if non-zero, otherwise align with path direction
  double yaw = (std::fabs(goal.start_yaw) > 1e-9) ? goal.start_yaw : path_angle;
  // Reverse: robot faces opposite to travel direction
  if (is_reverse) {
    yaw = normalizeAngle(yaw + M_PI);
  }
  double t = 0.0;

  std::vector<SimState> trajectory;
  trajectory.reserve(max_steps);

  for (int step = 0; step < max_steps; ++step) {
    // Projection of current position onto path axis
    const double rx = x - goal.start_x;
    const double ry = y - goal.start_y;
    const double projection = rx * std::cos(path_angle) + ry * std::sin(path_angle);

    // Query trapezoidal profile at clamped projection
    const double clamped = std::max(0.0, projection);
    const ProfileOutput prof = profile.getSpeed(clamped);
    double vx_profile = prof.speed;

    // Corrections for positions behind start or at low speed during accel
    if (projection < 0.0) {
      vx_profile = config_.behind_start_speed;
    } else if (prof.phase == ProfilePhase::ACCEL && vx_profile < config_.behind_start_speed) {
      vx_profile = config_.behind_start_speed;
    }

    // Enforce minimum forward speed while not done
    if (prof.phase != ProfilePhase::DONE && vx_profile < config_.min_vx) {
      vx_profile = config_.min_vx;
    }

    // Clamp to robot's maximum linear velocity
    vx_profile = std::min(vx_profile, config_.max_linear_vel);

    // Arrival check: profile finished or robot passed the goal
    if (prof.phase == ProfilePhase::DONE || projection >= target_distance) {
      SimState s{};
      s.t = t;
      s.x = x;
      s.y = y;
      s.yaw = yaw;
      s.vx = std::fabs(goal.exit_speed);
      s.vy = 0.0;
      s.omega = 0.0;
      s.e_d = 0.0;
      s.e_theta = 0.0;
      s.projection = projection;
      s.phase = static_cast<uint8_t>(ProfilePhase::DONE);
      trajectory.push_back(s);
      break;
    }

    // Path controller heading reference: for reverse, rotate yaw by PI to get
    // the "forward-equivalent" heading the path controller expects
    const double yaw_ctrl = is_reverse ? normalizeAngle(yaw + M_PI) : yaw;
    const PathControlOutput pc = path_ctrl.update(x, y, yaw_ctrl, vx_profile, dt);

    // Clamp angular velocity to robot limits
    const double omega = std::clamp(pc.omega, -config_.max_angular_vel, config_.max_angular_vel);

    // Differential-drive forward integration (vy always zero)
    // For reverse, invert the linear component in the world frame
    const double int_vx = is_reverse ? -vx_profile : vx_profile;
    x += int_vx * std::cos(yaw) * dt;
    y += int_vx * std::sin(yaw) * dt;
    yaw = normalizeAngle(yaw + omega * dt);
    t += dt;

    // Record state at end of this timestep
    SimState s{};
    s.t = t;
    s.x = x;
    s.y = y;
    s.yaw = yaw;
    s.vx = vx_profile;
    s.vy = 0.0;
    s.omega = omega;
    s.delta_f = 0.0;
    s.delta_r = 0.0;
    s.e_d = pc.e_d;
    s.e_theta = pc.e_theta;
    s.projection = projection;
    s.phase = static_cast<uint8_t>(prof.phase);
    trajectory.push_back(s);
  }

  return trajectory;
}

}  // namespace amr_motion_control_simulation
