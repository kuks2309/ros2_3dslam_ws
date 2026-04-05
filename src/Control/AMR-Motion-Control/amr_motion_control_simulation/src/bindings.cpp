#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "amr_motion_control_simulation/sim_types.hpp"
#include "amr_motion_control_simulation/sil_predictor.hpp"

namespace py = pybind11;
using namespace amr_motion_control_simulation;

PYBIND11_MODULE(_sil_predictor_cpp, m) {
    m.doc() = "SIL Predictor for 2WD differential drive AMR";

    // SimState
    py::class_<SimState>(m, "SimState")
        .def(py::init<>())
        .def_readwrite("t", &SimState::t)
        .def_readwrite("x", &SimState::x)
        .def_readwrite("y", &SimState::y)
        .def_readwrite("yaw", &SimState::yaw)
        .def_readwrite("vx", &SimState::vx)
        .def_readwrite("vy", &SimState::vy)
        .def_readwrite("omega", &SimState::omega)
        .def_readwrite("delta_f", &SimState::delta_f)
        .def_readwrite("delta_r", &SimState::delta_r)
        .def_readwrite("e_d", &SimState::e_d)
        .def_readwrite("e_theta", &SimState::e_theta)
        .def_readwrite("projection", &SimState::projection)
        .def_readwrite("phase", &SimState::phase)
        .def("__repr__", [](const SimState & s) {
            return "<SimState t=" + std::to_string(s.t) +
                   " x=" + std::to_string(s.x) +
                   " y=" + std::to_string(s.y) + ">";
        });

    // SimTranslateGoal
    py::class_<SimTranslateGoal>(m, "SimTranslateGoal")
        .def(py::init<>())
        .def_readwrite("start_x", &SimTranslateGoal::start_x)
        .def_readwrite("start_y", &SimTranslateGoal::start_y)
        .def_readwrite("end_x", &SimTranslateGoal::end_x)
        .def_readwrite("end_y", &SimTranslateGoal::end_y)
        .def_readwrite("max_linear_speed", &SimTranslateGoal::max_linear_speed)
        .def_readwrite("acceleration", &SimTranslateGoal::acceleration)
        .def_readwrite("exit_speed", &SimTranslateGoal::exit_speed)
        .def_readwrite("entry_speed", &SimTranslateGoal::entry_speed)
        .def_readwrite("control_mode", &SimTranslateGoal::control_mode)
        .def_readwrite("start_yaw", &SimTranslateGoal::start_yaw);

    // Config
    py::class_<SilPredictor::Config>(m, "Config")
        .def(py::init<>())
        .def_readwrite("wheel_separation", &SilPredictor::Config::wheel_separation)
        .def_readwrite("wheel_radius", &SilPredictor::Config::wheel_radius)
        .def_readwrite("max_linear_vel", &SilPredictor::Config::max_linear_vel)
        .def_readwrite("max_angular_vel", &SilPredictor::Config::max_angular_vel)
        .def_readwrite("max_linear_accel", &SilPredictor::Config::max_linear_accel)
        .def_readwrite("control_rate_hz", &SilPredictor::Config::control_rate_hz)
        .def_readwrite("max_timeout_sec", &SilPredictor::Config::max_timeout_sec)
        .def_readwrite("K_stanley", &SilPredictor::Config::K_stanley)
        .def_readwrite("K_stanley_reverse", &SilPredictor::Config::K_stanley_reverse)
        .def_readwrite("K_soft", &SilPredictor::Config::K_soft)
        .def_readwrite("Kp_heading", &SilPredictor::Config::Kp_heading)
        .def_readwrite("Kd_heading", &SilPredictor::Config::Kd_heading)
        .def_readwrite("heading_filter_window", &SilPredictor::Config::heading_filter_window)
        .def_readwrite("min_vx", &SilPredictor::Config::min_vx)
        .def_readwrite("behind_start_speed", &SilPredictor::Config::behind_start_speed)
        .def_readwrite("goal_reach_threshold", &SilPredictor::Config::goal_reach_threshold);

    // SilPredictor
    py::class_<SilPredictor>(m, "SilPredictor")
        .def(py::init<>())
        .def(py::init<const SilPredictor::Config &>())
        .def("predict", &SilPredictor::predict,
             py::arg("goal"),
             py::call_guard<py::gil_scoped_release>(),
             "Predict trajectory for a translate goal");
}
