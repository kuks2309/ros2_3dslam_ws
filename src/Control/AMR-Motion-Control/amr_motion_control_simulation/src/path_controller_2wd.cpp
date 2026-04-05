#include "amr_motion_control_simulation/path_controller_2wd.hpp"

#include <cmath>

namespace amr_motion_control_simulation
{

// ─── Helper ──────────────────────────────────────────────────────────────────
static double normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

// ─── Constructors ─────────────────────────────────────────────────────────────
PathController2WD::PathController2WD()
: params_{}
{
  reset();
}

PathController2WD::PathController2WD(const Params & params)
: params_(params)
{
  reset();
}

// ─── setPath ─────────────────────────────────────────────────────────────────
void PathController2WD::setPath(
  double start_x, double start_y,
  double end_x,   double end_y)
{
  path_sx_ = start_x;
  path_sy_ = start_y;
  path_ex_ = end_x;
  path_ey_ = end_y;

  double dx = end_x - start_x;
  double dy = end_y - start_y;
  path_length_ = std::hypot(dx, dy);

  if (path_length_ > 1e-9) {
    path_ux_    = dx / path_length_;
    path_uy_    = dy / path_length_;
    path_angle_ = std::atan2(dy, dx);
  } else {
    path_ux_    = 1.0;
    path_uy_    = 0.0;
    path_angle_ = 0.0;
  }

  path_set_ = true;
  reset();
}

// ─── reset ───────────────────────────────────────────────────────────────────
void PathController2WD::reset()
{
  prev_e_theta_ = 0.0;
  first_update_ = true;
  e_theta_sum_  = 0.0;
  filter_count_ = 0;
  filter_idx_   = 0;
  for (int i = 0; i < MAX_FILTER; i++) { filter_buf_[i] = 0.0; }
}

// ─── update ──────────────────────────────────────────────────────────────────
PathControlOutput PathController2WD::update(
  double x, double y, double yaw, double vx, double dt)
{
  // 1. Cross-track error (positive = robot left of path)
  double dx_r      = x - path_sx_;
  double dy_r      = y - path_sy_;
  double e_lateral = -(dx_r * path_uy_ - dy_r * path_ux_);

  // 2. Heading error
  double e_theta = normalizeAngle(path_angle_ - yaw);

  // 3. Moving-average filter for heading error
  int window = params_.heading_filter_window;
  if (window < 1)       { window = 1; }
  if (window > MAX_FILTER) { window = MAX_FILTER; }

  double e_theta_filt;
  if (filter_count_ < window) {
    // Warm-up: recompute sum from filled slots
    filter_buf_[filter_idx_] = e_theta;
    filter_idx_ = (filter_idx_ + 1) % window;
    filter_count_++;
    double sum = 0.0;
    for (int i = 0; i < filter_count_; i++) { sum += filter_buf_[i]; }
    e_theta_sum_ = sum;
    e_theta_filt = sum / filter_count_;
  } else {
    // Steady state: subtract oldest, add newest
    double old_val        = filter_buf_[filter_idx_];
    filter_buf_[filter_idx_] = e_theta;
    filter_idx_ = (filter_idx_ + 1) % window;
    e_theta_sum_ = e_theta_sum_ + (e_theta - old_val);
    e_theta_filt = e_theta_sum_ / window;
  }

  // 4. Derivative of filtered heading error
  double de_theta = first_update_ ? 0.0 : (e_theta_filt - prev_e_theta_) / dt;
  prev_e_theta_   = e_theta_filt;
  first_update_   = false;

  // 5. Stanley lateral correction (skip if speed too low)
  double stanley_correction = 0.0;
  double speed_abs = std::fabs(vx);
  if (speed_abs > 1e-3) {
    stanley_correction = std::atan2(
      params_.K_stanley * e_lateral,
      speed_abs + params_.K_soft);
  }

  // 6. Final omega = sign_v * (Kp*(e_theta_filt + stanley) + Kd*de_theta)
  double sign_v = (vx >= 0.0) ? 1.0 : -1.0;
  double omega  = sign_v * (
    params_.Kp_heading * (e_theta_filt + stanley_correction) +
    params_.Kd_heading * de_theta);

  return PathControlOutput{omega, e_lateral, e_theta};
}

}  // namespace amr_motion_control_simulation
