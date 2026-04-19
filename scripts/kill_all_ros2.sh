#!/bin/bash
# Kill all ROS2 nodes and related processes (Universal version)

echo "=== Killing all ROS2 and simulation processes ==="

# Step 1: Stop ROS2 daemon first
echo "Stopping ROS2 daemon..."
ros2 daemon stop 2>/dev/null

# Step 2: Kill all processes under /opt/ros/ path (covers all ROS2 nodes)
echo "Killing all /opt/ros/ processes..."
pkill -9 -f "/opt/ros/" 2>/dev/null

# Step 3: Kill all Gazebo/Ignition related processes (find dynamically)
echo "Killing all Gazebo/Ignition processes..."
pkill -9 -f "gazebo" 2>/dev/null
pkill -9 -f "ign" 2>/dev/null
pkill -9 -f "gz" 2>/dev/null

# Step 4: Kill ROS2 daemon process
pkill -9 -f "ros2-daemon" 2>/dev/null
pkill -9 -f "ros2" 2>/dev/null

sleep 1

# Step 5: Final verification
echo ""
echo "=== Verification ==="
REMAINING=$(ps aux | grep -E "/opt/ros/|gazebo|ign |gz " | grep -v grep | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "All processes killed successfully"
else
    echo "Remaining processes:"
    ps aux | grep -E "/opt/ros/|gazebo|ign |gz " | grep -v grep

    echo ""
    echo "Force killing remaining..."
    ps aux | grep -E "/opt/ros/|gazebo|ign |gz " | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null

    sleep 1
    echo ""
    echo "=== Final Check ==="
    FINAL=$(ps aux | grep -E "/opt/ros/|gazebo|ign |gz " | grep -v grep | wc -l)
    if [ "$FINAL" -eq 0 ]; then
        echo "All processes killed successfully"
    else
        echo "Still remaining (may need manual kill):"
        ps aux | grep -E "/opt/ros/|gazebo|ign |gz " | grep -v grep
    fi
fi
