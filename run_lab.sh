#!/bin/bash

# Ensure the workspace is sourced
source install/setup.bash

echo "Starting Project Arion Simulation..."
echo "Use 'Ctrl+C' to stop the simulation."

# Launch the main simulation file
ros2 launch arion_simulation start_world.launch.py