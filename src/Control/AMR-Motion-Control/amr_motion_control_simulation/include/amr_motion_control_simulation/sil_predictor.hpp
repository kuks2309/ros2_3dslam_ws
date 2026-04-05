#ifndef AMR_MOTION_CONTROL_SIMULATION__SIL_PREDICTOR_HPP_
#define AMR_MOTION_CONTROL_SIMULATION__SIL_PREDICTOR_HPP_

#include "amr_motion_control_simulation/sim_types.hpp"
#include <vector>

namespace amr_motion_control_simulation
{

class SilPredictor
{
public:
  struct Config
  {
    // Robot geometry (Pioneer 2DX defaults)
    double wheel_separation = 0.34;
    double wheel_radius = 0.11;

    // Velocity / acceleration limits
    double max_linear_vel = 0.4;
    double max_angular_vel = 1.0;
    double max_linear_accel = 1.5;

    // Control loop
    double control_rate_hz = 20.0;
    double max_timeout_sec = 60.0;

    // Path controller params
    double K_stanley = 2.0;
    double K_stanley_reverse = 2.0;
    double K_soft = 0.8;
    double Kp_heading = 1.0;
    double Kd_heading = 0.3;
    int heading_filter_window = 5;

    // Speed thresholds
    double min_vx = 0.02;
    double behind_start_speed = 0.2;
    double goal_reach_threshold = 0.05;
  };

  SilPredictor();
  explicit SilPredictor(const Config & config);

  std::vector<SimState> predict(const SimTranslateGoal & goal) const;

private:
  Config config_;
};

}  // namespace amr_motion_control_simulation

#endif
