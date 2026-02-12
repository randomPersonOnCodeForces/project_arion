#!/bin/bash

source install/setup.bash

echo "========================================================"
echo "   ROBOT LIVE POSITION TRACKER"
echo "========================================================"
echo "Waiting for robot data..."
echo "Press 'Ctrl+C' to exit."
echo ""

stdbuf -oL ros2 topic echo /odom | grep -E --line-buffered "(sec:|position:|orientation:|  x:|  y:|  z:|  w:)"