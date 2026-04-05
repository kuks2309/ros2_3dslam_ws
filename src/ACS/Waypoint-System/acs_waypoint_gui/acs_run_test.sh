#!/bin/bash

WORKSPACE_DIR=~/Study/ros2_3dslam_ws
CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/config"

# Scenario definitions: name -> "job_file|description"
declare -A SCENARIOS
SCENARIOS["square"]="job_test_gazebo.txt|2x2m square with WAIT stops"
SCENARIOS["straight"]="job_test_straight.txt|TRANSLATE back-and-forth 5m"
SCENARIOS["large_loop"]="job_test_large_loop.txt|5x5m loop at 0.5 m/s"
SCENARIOS["multistop"]="job_test_multistop.txt|Warehouse stations multi-stop"
SCENARIOS["drive_modes"]="job_test_drive_modes.txt|All drive mode demo"
SCENARIOS["figure_l"]="job_test_figure_l.txt|L-shaped corridor"
SCENARIOS["yawctrl"]="job_test_yawctrl.txt|YAWCTRL precision test"
SCENARIOS["precision"]="job_test_precision.txt|Slow precision grid"

usage() {
    echo "Usage: $0 <scenario> [OPTIONS]"
    echo ""
    echo "Available scenarios:"
    printf "  %-14s %s\n" "square"      "${SCENARIOS[square]#*|}"
    printf "  %-14s %s\n" "straight"    "${SCENARIOS[straight]#*|}"
    printf "  %-14s %s\n" "large_loop"  "${SCENARIOS[large_loop]#*|}"
    printf "  %-14s %s\n" "multistop"   "${SCENARIOS[multistop]#*|}"
    printf "  %-14s %s\n" "drive_modes" "${SCENARIOS[drive_modes]#*|}"
    printf "  %-14s %s\n" "figure_l"    "${SCENARIOS[figure_l]#*|}"
    printf "  %-14s %s\n" "yawctrl"     "${SCENARIOS[yawctrl]#*|}"
    printf "  %-14s %s\n" "precision"   "${SCENARIOS[precision]#*|}"
    echo ""
    echo "Options:"
    echo "  --speed FLOAT    Max speed in m/s (default: 0.3)"
    echo "  --accel FLOAT    Acceleration in m/s^2 (default: 0.3)"
    echo "  --loop           Enable looping (default: false)"
    echo "  -h, --help       Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 straight"
    echo "  $0 large_loop --speed 0.5 --loop"
    echo "  $0 precision --speed 0.1 --accel 0.1"
}

# Defaults
SPEED=0.3
ACCEL=0.3
LOOP=false
SCENARIO=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --speed)
            SPEED="$2"
            shift 2
            ;;
        --accel)
            ACCEL="$2"
            shift 2
            ;;
        --loop)
            LOOP=true
            shift
            ;;
        -*)
            echo "Error: Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            if [[ -z "$SCENARIO" ]]; then
                SCENARIO="$1"
            else
                echo "Error: Unexpected argument: $1"
                usage
                exit 1
            fi
            shift
            ;;
    esac
done

if [[ -z "$SCENARIO" ]]; then
    echo "Error: No scenario specified."
    usage
    exit 1
fi

if [[ -z "${SCENARIOS[$SCENARIO]}" ]]; then
    echo "Error: Unknown scenario '$SCENARIO'"
    echo ""
    echo "Available scenarios: ${!SCENARIOS[*]}"
    exit 1
fi

JOB_FILE="${SCENARIOS[$SCENARIO]%%|*}"
JOB_PATH="${CONFIG_DIR}/${JOB_FILE}"

if [[ ! -f "$JOB_PATH" ]]; then
    echo "Error: Job file not found: $JOB_PATH"
    exit 1
fi

# Source ROS2 and workspace
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/iron/setup.bash ]]; then
    source /opt/ros/iron/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Error: Could not find ROS2 setup.bash in /opt/ros/"
    exit 1
fi

WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
if [[ -f "$WORKSPACE_SETUP" ]]; then
    source "$WORKSPACE_SETUP"
else
    echo "Error: Workspace not built. Run 'colcon build' in ${WORKSPACE_DIR} first."
    exit 1
fi

echo "Running scenario: $SCENARIO"
echo "  Job file : $JOB_PATH"
echo "  Max speed: $SPEED m/s"
echo "  Accel    : $ACCEL m/s^2"
echo "  Loop     : $LOOP"
echo ""

ros2 launch acs_waypoint_gui acs_test.launch.py \
    job_file:="$JOB_PATH" \
    max_speed:="$SPEED" \
    acceleration:="$ACCEL" \
    loop:="$LOOP"
