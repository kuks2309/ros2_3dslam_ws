#ifndef AMR_MOTION_CONTROL_SIMULATION__PATH_CONTROLLER_2WD_HPP_
#define AMR_MOTION_CONTROL_SIMULATION__PATH_CONTROLLER_2WD_HPP_

namespace amr_motion_control_simulation
{

struct PathControlOutput
{
  double omega;     // rad/s, commanded angular velocity
  double e_d;       // m, cross-track error (CTE)
  double e_theta;   // rad, heading error
};

class PathController2WD
{
public:
  struct Params
  {
    double K_stanley = 2.0;
    double K_soft = 0.8;
    double Kp_heading = 1.0;
    double Kd_heading = 0.3;
    int heading_filter_window = 5;
  };

  PathController2WD();
  explicit PathController2WD(const Params & params);

  void setPath(double start_x, double start_y, double end_x, double end_y);
  PathControlOutput update(double x, double y, double yaw, double vx, double dt);
  void reset();

private:
  Params params_;
  // Path definition
  double path_sx_, path_sy_, path_ex_, path_ey_;
  double path_ux_, path_uy_;  // unit direction
  double path_angle_;
  double path_length_;
  bool path_set_ = false;
  // State for derivative
  double prev_e_theta_ = 0.0;
  bool first_update_ = true;
  // Simple moving average for heading error
  double e_theta_sum_ = 0.0;
  int filter_count_ = 0;
  static constexpr int MAX_FILTER = 20;
  double filter_buf_[20] = {};
  int filter_idx_ = 0;
};

}  // namespace amr_motion_control_simulation

#endif
