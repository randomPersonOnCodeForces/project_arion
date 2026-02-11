#!/bin/bash

# Source the workspace
source install/setup.bash

echo "========================================================"
echo "   PROJECT ARION: TELEOP CONTROL"
echo "========================================================"
echo "   [Instructions]"
echo "   1. Ensure the simulation (run_lab.sh) is running in another terminal."
echo "   2. Keep this window active (click inside it)."
echo "   3. Use the keyboard to drive:"
echo ""
echo "      i"
echo "   j  k  l"
echo "      ,"
echo ""
echo "   (Press 'q' to quit)"
echo "========================================================"

# Check if the package is installed
if ! ros2 pkg list | grep -q teleop_twist_keyboard; then
    echo "[ERROR] teleop_twist_keyboard not found!"
    echo "Installing it now..."
    sudo apt-get update && sudo apt-get install -y ros-humble-teleop-twist-keyboard
fi

# Run the node
ros2 run teleop_twist_keyboard teleop_twist_keyboard