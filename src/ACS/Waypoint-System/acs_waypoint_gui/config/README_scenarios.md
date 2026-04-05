# ACS Waypoint System - Test Scenarios

## Prerequisites

Before running any scenario:

1. **Gazebo simulation running** with `my_world` (flat 100x100 m environment), robot spawned at origin (0, 0).
2. **Workspace built and sourced**:
   ```bash
   cd ~/Study/ros2_3dslam_ws
   colcon build --packages-select acs_waypoint_gui amr_motion_control_2wd amr_interfaces
   source install/setup.bash
   ```
3. **Motion control nodes active** — the AMR motion control action servers (`yaw_control`, `pure_pursuit`, `stanley`) must be running before launching a scenario.
4. **TF frames available** — `map -> odom -> base_link` transform chain must be publishing.

---

## Job File Format

Each non-comment line defines one waypoint:

| Column | Field | Unit | Description |
|--------|-------|------|-------------|
| 1 | `Type` | — | Always `AMR` |
| 2 | `X` | m | Target X position in map frame |
| 3 | `Y` | m | Target Y position in map frame |
| 4 | `Yaw` | deg | Target heading at waypoint |
| 5 | `Timeout` | sec | Maximum time allowed to reach waypoint |
| 6 | `DriveMode` | — | Motion strategy (see table below) |
| 7 | `TurnRadius` | m | Arc radius for TURN mode; `0.0` otherwise |
| 8 | `Speed` | m/s | Travel speed (or wait duration for WAIT mode) |

Lines starting with `#` are comments and are ignored.

---

## Drive Mode Reference

| Mode | Code | Use Case | Notes |
|------|------|----------|-------|
| AUTO | 0 | General waypoint navigation | Spins in place to face target, then translates straight. Handles 90-degree corners cleanly. Default for most missions. |
| TRANSLATE | 1 | Straight-line motion without pre-spin | Drives directly toward target XY while holding the specified yaw throughout. No pre-spin. |
| PURE_STANLEY | 2 | Path-following with Stanley controller | Follows a smooth path curve; suitable for curved trajectories. |
| TURN | 3 | Arc/circular turn | Executes a curved arc. `TurnRadius` (m) must be set. |
| SPIN | 4 | In-place yaw rotation only | Changes heading without moving XY. Waypoint XY should match current robot position. |
| YAWCTRL | 5 | Translation with continuous yaw correction | Translates toward target while actively correcting heading throughout motion. Use when precise alignment during motion is required. |
| WAIT | 6 | Pause at current position | Robot stops. `Speed` field = wait duration in seconds. |

---

## Scenario Catalog

### 1. `job_test_gazebo.txt`

| Property | Value |
|----------|-------|
| Purpose | Basic integration test: 2x2 m square with a mid-route pause |
| Path shape | Square: (0,0) → (2,0) → (2,2) → (0,2) → (0,0) |
| Drive modes | AUTO, WAIT |
| Speed | 0.3 m/s |
| Est. duration | ~45 s |

Tests AUTO mode navigation and WAIT arrival action. A 3-second WAIT is inserted at the first corner (2,0) to verify arrival handling.

```bash
ros2 launch acs_waypoint_gui acs_test.launch.py \
  job_file:=$(ros2 pkg prefix acs_waypoint_gui)/share/acs_waypoint_gui/config/job_test_gazebo.txt
# or via run script:
./acs_run_test.sh job_test_gazebo
```

---

### 2. `job_test_straight.txt`

| Property | Value |
|----------|-------|
| Purpose | Verify TRANSLATE mode: straight-line motion without pre-spin |
| Path shape | Back-and-forth along X axis: (0,0) ↔ (5,0) |
| Drive modes | TRANSLATE |
| Speed | 0.4 m/s |
| Est. duration | ~30 s |

Confirms the robot holds its specified yaw throughout motion without spinning first. Useful for verifying differential-drive lateral behavior.

```bash
./acs_run_test.sh job_test_straight
```

---

### 3. `job_test_large_loop.txt`

| Property | Value |
|----------|-------|
| Purpose | High-speed stability test over a larger area |
| Path shape | 5x5 m CCW square: (0,0) → (5,0) → (5,5) → (0,5) → (0,0) + heading reset |
| Drive modes | AUTO |
| Speed | 0.5 m/s |
| Est. duration | ~60 s |

Verifies motion control accuracy and timeout margins at elevated speed. 5 waypoints including a heading-reset step at origin.

```bash
./acs_run_test.sh job_test_large_loop
# Optional: loop continuously
ros2 launch acs_waypoint_gui acs_test.launch.py \
  job_file:=<path>/job_test_large_loop.txt loop:=true max_speed:=0.5
```

---

### 4. `job_test_multistop.txt`

| Property | Value |
|----------|-------|
| Purpose | Simulated warehouse pickup/delivery mission with dwell times |
| Path shape | 4-station route: (0,0) → A(2,0) → B(2,3) → C(5,3) → (0,0) |
| Drive modes | AUTO, WAIT |
| Speed | 0.3 m/s |
| Est. duration | ~90 s (including 3+5+3 s waits) |

Tests multi-stop sequencing with variable dwell times (3 s at A, 5 s at B, 3 s at C). Represents a realistic warehouse AMR use case.

```bash
./acs_run_test.sh job_test_multistop
```

---

### 5. `job_test_drive_modes.txt`

| Property | Value |
|----------|-------|
| Purpose | Single-mission showcase of all major drive modes |
| Path shape | (0,0) → (3,0) → spin → (3,3) → (0,3) → (0,0) |
| Drive modes | AUTO, SPIN, TRANSLATE, YAWCTRL |
| Speed | 0.3 m/s |
| Est. duration | ~60 s |

Each leg uses a different drive mode. Run this first when validating a new motion control build to confirm all modes are operational.

```bash
./acs_run_test.sh job_test_drive_modes
```

---

### 6. `job_test_figure_l.txt`

| Property | Value |
|----------|-------|
| Purpose | AUTO mode corner handling in an L-shaped corridor |
| Path shape | L-shape: (0,0) → (6,0) [east, 0.3 m/s] → (6,4) [north, 0.25 m/s] |
| Drive modes | AUTO |
| Speed | 0.3 m/s (horizontal), 0.25 m/s (vertical) |
| Est. duration | ~40 s |

Validates spin-then-translate at a 90-degree corner and tests variable per-leg speed. Simulates narrow corridor caution with reduced speed on the vertical segment.

```bash
./acs_run_test.sh job_test_figure_l
```

---

### 7. `job_test_yawctrl.txt`

| Property | Value |
|----------|-------|
| Purpose | Validate YAWCTRL mode — continuous yaw feedback during straight-line translation |
| Path shape | Rectangular U: (0,0)→(1,0)→(4,0)→(4,1)→(4,4)→(0,4)→(0,1) |
| Drive modes | AUTO (corners), YAWCTRL (straight segments) |
| Speed | 0.3 m/s |
| Est. duration | ~65 s |

Tests heading accuracy on three long straight segments (3 m each) using YAWCTRL, with AUTO for 90° corner turns. Demonstrates improved straight-line tracking compared to open-loop TRANSLATE.

```bash
./acs_run_test.sh job_test_yawctrl
```

---

### 8. `job_test_precision.txt`

| Property | Value |
|----------|-------|
| Purpose | Slow-speed multi-waypoint precision navigation with WAIT stop |
| Path shape | Grid: (0,0)→(1,0)→(2,0 WAIT 2s)→(2,1)→(1,1)→(0,1)→(0,2) |
| Drive modes | AUTO, WAIT |
| Speed | 0.15 m/s |
| Est. duration | ~130 s |

Tests positional accuracy at low speed with short inter-waypoint distances (~1 m). Includes an AUTO move to (2,0) followed by a 2-second WAIT at the same position to verify stop-and-hold behavior. All timeouts are 60 s.

```bash
./acs_run_test.sh job_test_precision
```

---

## How to Create Custom Scenarios

1. **Copy an existing file** as a starting template:
   ```bash
   cp job_test_gazebo.txt job_my_scenario.txt
   ```

2. **Plan your path** — robot starts at (0, 0) in `my_world`. Coordinates are in the map frame (meters).

3. **Add waypoint lines** in order. Each line:
   ```
   AMR  <X>  <Y>  <Yaw_deg>  <Timeout_sec>  <DriveMode>  <TurnRadius>  <Speed>
   ```

4. **Set timeouts conservatively** — recommended formula: `distance / speed * 3`. The mission aborts if a waypoint is not reached within its timeout.

5. **Insert WAIT lines** for dwell time at a station:
   ```
   AMR  <same X>  <same Y>  <Yaw>  10.0  WAIT  0.0  <wait_seconds>
   ```
   The `Speed` field is the wait duration in seconds for WAIT mode.

6. **Test with low speed first** (`max_speed:=0.15`) before increasing.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `TF lookup failed` / `TF not available` | `map->odom->base_link` chain not publishing | Verify SLAM/localization node is running; check `ros2 run tf2_tools view_frames` |
| Action server timeout immediately | Motion control action server not started | Confirm `amr_motion_control_2wd` node is running: `ros2 node list` |
| Robot overshoots waypoint | Speed too high or `xy_goal_tolerance` too loose | Reduce `max_speed` parameter or tighten tolerance in `motion_params_gazebo.yaml` |
| Mission aborts mid-route | Waypoint timeout too short | Increase `Timeout` column value (use `distance / speed * 3` rule) |
| SPIN waypoint moves robot | Waypoint XY does not match current position | Ensure SPIN waypoint XY equals robot's current position exactly |
| WAIT never completes | `Speed` field set to 0 for WAIT mode | Set `Speed` to the desired wait duration in seconds (e.g., `3.0`) |
| All waypoints skipped | Job file has Windows line endings (CRLF) | Convert with `dos2unix job_file.txt` |
