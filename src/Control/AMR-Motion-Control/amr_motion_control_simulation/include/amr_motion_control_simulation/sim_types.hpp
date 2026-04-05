#ifndef AMR_MOTION_CONTROL_SIMULATION__SIM_TYPES_HPP_
#define AMR_MOTION_CONTROL_SIMULATION__SIM_TYPES_HPP_

#include <cstdint>
#include <vector>

namespace amr_motion_control_simulation
{

struct SimState
{
  double t;             // sec
  double x;             // m, map frame
  double y;             // m, map frame
  double yaw;           // rad
  double vx;            // m/s, body forward
  double vy = 0.0;      // m/s, always 0 for diff-drive
  double omega;         // rad/s
  double delta_f = 0.0; // unused for diff-drive (upstream compat)
  double delta_r = 0.0; // unused for diff-drive (upstream compat)
  double e_d;           // m, lateral error (CTE)
  double e_theta;       // rad, heading error
  double projection;    // m, along-path distance
  uint8_t phase;        // 1=ACCEL, 2=CRUISE, 3=DECEL, 4=DONE
};

struct SimTranslateGoal
{
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double max_linear_speed;
  double acceleration;
  double exit_speed = 0.0;
  double entry_speed = 0.0;
  uint8_t control_mode = 0;  // 0=direct vx/omega (only mode for diff-drive)
  double start_yaw = 0.0;    // rad, 0.0 = auto (path angle)
};

}  // namespace amr_motion_control_simulation

#endif
