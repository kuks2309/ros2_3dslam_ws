#!/bin/bash
# ACS Waypoint Integration Test Runner
# Runs each scenario sequentially with a clean Gazebo stack per test.
# Usage: ./run_integration_tests.sh [scenario_name]
#   If scenario_name given, runs only that scenario.

set -eo pipefail

WS="$HOME/Study/ros2_3dslam_ws"
SRC="$WS/src"
GAZEBO_SRC="$SRC/Gazebo"
LOG_BASE="$WS/logs/integration_tests/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_BASE"

# ROS2 setup files use unbound variables — disable -u around sourcing
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

SHARE="$WS/waypoints"
WP_SHARE="$(ros2 pkg prefix waypoint_manager)/share/waypoint_manager/config"
MC_SHARE="$(ros2 pkg prefix amr_motion_control_2wd)/share/amr_motion_control_2wd/config"

# -----------------------------------------------------------------------
# Scenario definitions: name | max_speed | timeout_sec | expected_min_wps
# -----------------------------------------------------------------------
declare -A SCENARIO_SPEED=( [gazebo]=0.3 [straight]=0.4 [large_loop]=0.5
  [multistop]=0.3 [drive_modes]=0.3 [yawctrl]=0.3 [precision]=0.15 )
declare -A SCENARIO_TIMEOUT=( [gazebo]=120 [straight]=90 [large_loop]=240
  [multistop]=180 [drive_modes]=180 [yawctrl]=240 [precision]=360 )
declare -A SCENARIO_MIN_WPS=( [gazebo]=4 [straight]=2 [large_loop]=5
  [multistop]=7 [drive_modes]=5 [yawctrl]=6 [precision]=7 )

SCENARIO_ORDER=(gazebo straight large_loop multistop drive_modes yawctrl precision)

FILTER="${1:-}"

PASS=0
FAIL=0
declare -a RESULTS

# -----------------------------------------------------------------------
kill_stack() {
  echo "[kill_stack] Stopping all ROS2/Gazebo processes..."
  pkill -f "ign gazebo"            2>/dev/null || true
  pkill -f "parameter_bridge"      2>/dev/null || true
  pkill -f "odom_to_tf"            2>/dev/null || true
  pkill -f "static_transform_pub"  2>/dev/null || true
  pkill -f "amr_motion_control"    2>/dev/null || true
  pkill -f "waypoint_manager"      2>/dev/null || true
  pkill -f "acs_test_node"         2>/dev/null || true
  sleep 3
}

# -----------------------------------------------------------------------
wait_for_action_server() {
  local server="/waypoint_mission"
  local max_wait=30
  local elapsed=0
  echo "[wait] Waiting for action server $server ..."
  while [ $elapsed -lt $max_wait ]; do
    if ros2 action list 2>/dev/null | grep -q "waypoint_mission"; then
      echo "[wait] Action server ready (${elapsed}s)"
      return 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
  done
  echo "[wait] ERROR: Action server not available after ${max_wait}s"
  return 1
}

# -----------------------------------------------------------------------
start_stack() {
  local log_dir="$1"
  echo "[start] Launching Gazebo (server-only) ..."
  IGN_GAZEBO_RESOURCE_PATH="$GAZEBO_SRC/models" \
    ign gazebo -s -r "$GAZEBO_SRC/worlds/my_world.sdf" \
    >"$log_dir/gazebo.log" 2>&1 &
  sleep 6

  echo "[start] Launching ROS-Gazebo bridge ..."
  ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
    /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
    /imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU \
    >"$log_dir/bridge.log" 2>&1 &
  sleep 2

  echo "[start] Launching odom_to_tf + static TFs ..."
  python3 "$GAZEBO_SRC/scripts/odom_to_tf.py" \
    >"$log_dir/odom_to_tf.log" 2>&1 &

  ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map odom \
    >"$log_dir/tf_map_odom.log" 2>&1 &

  ros2 run tf2_ros static_transform_publisher \
    0 0 0.16 0 0 0 base_footprint base_link \
    >"$log_dir/tf_footprint_base.log" 2>&1 &
  sleep 2

  echo "[start] Launching amr_motion_control_2wd ..."
  ros2 run amr_motion_control_2wd amr_motion_control_2wd_node \
    --ros-args \
    --params-file "$MC_SHARE/motion_params_gazebo_no_slam.yaml" \
    -p use_sim_time:=true \
    >"$log_dir/motion_control.log" 2>&1 &
  sleep 3

  echo "[start] Launching waypoint_manager ..."
  ros2 run waypoint_manager waypoint_manager_node \
    --ros-args \
    --params-file "$WP_SHARE/waypoint_params_gazebo.yaml" \
    -p use_sim_time:=true \
    >"$log_dir/waypoint_manager.log" 2>&1 &
  sleep 2
}

# -----------------------------------------------------------------------
run_scenario() {
  local name="$1"
  local job_file="$SHARE/job_test_${name}.txt"
  local speed="${SCENARIO_SPEED[$name]}"
  local timeout_sec="${SCENARIO_TIMEOUT[$name]}"
  local min_wps="${SCENARIO_MIN_WPS[$name]}"
  local log_dir="$LOG_BASE/$name"
  local test_log="$log_dir/test.log"

  mkdir -p "$log_dir"
  echo ""
  echo "================================================================"
  echo "SCENARIO: job_test_${name}"
  echo "job_file: $job_file"
  echo "speed: $speed  timeout: ${timeout_sec}s  expected_min_wps: $min_wps"
  echo "================================================================"

  if [ ! -f "$job_file" ]; then
    echo "ERROR: job file not found: $job_file"
    FAIL=$((FAIL + 1))
    RESULTS+=("FAIL [$name]: job file missing")
    return 1
  fi

  # Clean start
  kill_stack
  start_stack "$log_dir"

  # Wait for action server
  if ! wait_for_action_server; then
    echo "ERROR: action server not ready for $name"
    FAIL=$((FAIL + 1))
    RESULTS+=("FAIL [$name]: action server not available")
    kill_stack
    return 1
  fi

  # Run the test node
  echo "[run] Starting acs_test_node for $name ..."
  timeout "$timeout_sec" ros2 run acs_waypoint_gui acs_test_node \
    --ros-args \
    -p use_sim_time:=true \
    -p job_file:="$job_file" \
    -p default_max_speed:="$speed" \
    -p default_acceleration:=0.3 \
    -p log_dir:="$log_dir/mission_logs" \
    2>&1 | tee "$test_log"
  local exit_code=${PIPESTATUS[0]}

  # Parse result
  local accepted=false
  local success=false
  local completed_wps=0
  local elapsed_time="?"
  local summary=""

  if grep -q "Mission ACCEPTED" "$test_log" 2>/dev/null; then
    accepted=true
  fi
  if grep -q "Mission SUCCESS" "$test_log" 2>/dev/null; then
    success=true
    summary=$(grep "Mission SUCCESS" "$test_log" | tail -1)
    # Extract completed_wp: "Mission SUCCESS: X/Y WPs, ..."
    completed_wps=$(echo "$summary" | grep -oP '\d+(?=/\d+ WPs)' | head -1)
    elapsed_time=$(echo "$summary" | grep -oP '[\d.]+(?=s$)' | head -1)
  fi

  echo ""
  echo "--- RESULT for $name ---"
  echo "Accepted:       $accepted"
  echo "Success:        $success"
  echo "Completed WPs:  $completed_wps (min expected: $min_wps)"
  echo "Elapsed:        ${elapsed_time}s"
  echo "Summary:        $summary"
  echo "Exit code:      $exit_code"

  # Validate
  local pass=true
  local fail_reason=""

  if [ "$accepted" != "true" ]; then
    pass=false; fail_reason="Mission not ACCEPTED"
  elif [ "$success" != "true" ]; then
    pass=false; fail_reason="Mission did not succeed (no 'Mission SUCCESS' in log)"
  elif [ -n "$completed_wps" ] && [ "$completed_wps" -lt "$min_wps" ]; then
    pass=false; fail_reason="completed_wps=$completed_wps < min=$min_wps"
  fi

  if [ "$pass" = "true" ]; then
    PASS=$((PASS + 1))
    RESULTS+=("PASS [$name]: $summary")
    echo "RESULT: PASS"
  else
    FAIL=$((FAIL + 1))
    RESULTS+=("FAIL [$name]: $fail_reason")
    echo "RESULT: FAIL — $fail_reason"
  fi

  kill_stack
}

# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
echo "ACS Waypoint Integration Tests"
echo "Log dir: $LOG_BASE"
echo "Scenarios: ${SCENARIO_ORDER[*]}"
echo ""

for scenario in "${SCENARIO_ORDER[@]}"; do
  if [ -n "$FILTER" ] && [ "$FILTER" != "$scenario" ]; then
    continue
  fi
  run_scenario "$scenario"
done

# Final summary
echo ""
echo "================================================================"
echo "FINAL RESULTS"
echo "================================================================"
for r in "${RESULTS[@]}"; do
  echo "  $r"
done
echo ""
echo "PASSED: $PASS / $((PASS + FAIL))"
if [ $FAIL -gt 0 ]; then
  echo "FAILED: $FAIL scenarios"
  exit 1
else
  echo "All scenarios PASSED"
  exit 0
fi
