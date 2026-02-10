#!/bin/bash

echo "========================================================"
echo "   PROJECT ARION: STUDENT SETUP SCRIPT"
echo "========================================================"

# 1. Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null
then
    echo "[ERROR] ROS 2 is not detected!"
    echo "Please ensure you are running this on Ubuntu 22.04 with ROS 2 Humble installed."
    exit 1
fi

echo "[INFO] ROS 2 detected. Installing build tools..."

# 2. Install dependencies (The part students usually forget)
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "[INFO] Installing project dependencies..."
# This magic command installs any missing libraries listed in package.xml
rosdep install --from-paths src --ignore-src -r -y

# 3. Build the workspace
echo "[INFO] Building Project Arion..."
colcon build --symlink-install

# 4. Auto-source setup (So they don't have to type it every time)
if ! grep -q "source $(pwd)/install/setup.bash" ~/.bashrc; then
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
    echo "[INFO] Added project to your ~/.bashrc"
else
    echo "[INFO] Project already in ~/.bashrc"
fi

echo "========================================================"
echo "   SETUP COMPLETE!"
echo "   Please close this terminal and open a new one."
echo "   Then run: ./run_lab.sh"
echo "========================================================"