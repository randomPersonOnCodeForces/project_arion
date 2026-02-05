# 🎯 AI Academy Warehouse Demo - Complete Setup Guide

## Overview

This document provides step-by-step instructions for running the AI Academy Warehouse demonstration. The demo showcases:

1. **Clean Room Environment**: AWS Small Warehouse world loaded from the local `arion_simulation` package
2. **Robot Spawn**: TurtleBot3 Waffle Pi with a Lidar sensor
3. **Physics & Collision**: Real collision detection and physics simulation
4. **Rviz2 Integration**: Live visualization of Lidar scans (LaserScan data)
5. **Teleop Control**: Keyboard-controlled robot movement

---

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (or compatible)
- ROS 2 Humble (or compatible)
- Gazebo (Ignition Gazebo)
- Python 3.10+

### Required Packages
The following packages must be installed:
- `ross_gz_bridge` - ROS 2 to Gazebo communication bridge
- `gazebo_ros_pkgs` - Gazebo ROS integration
- `rviz2` - ROS 2 visualization tool
- `teleop_twist_keyboard` - Keyboard control node

---

## 📋 Preparation Checklist for Thursday

### Step 1: Clean Build

Before the demo, perform a clean build to ensure everything compiles correctly:

```bash
# Navigate to the workspace root
cd ~/path/to/project_arion

# IMPORTANT: Clean previous build artifacts
rm -rf build install log

# Build with symlink-install for faster development cycles
colcon build --symlink-install --executor sequential

# Take a break - this takes 5-10 minutes
```

### Step 2: Source the Environment

After the build completes, source the setup file:

```bash
source install/setup.bash
```

**Pro tip**: Add this to your `~/.bashrc` to auto-source on terminal startup:
```bash
echo "source ~/path/to/project_arion/install/setup.bash" >> ~/.bashrc
```

### Step 3: Verify Installation

Check that all required packages are available:

```bash
# Should print the path to arion_simulation
ros2 pkg prefix arion_simulation

# Should print the path to aws warehouse world
ros2 pkg prefix aws_robomaker_small_warehouse_world

# Check if teleop package is available
ros2 pkg prefix teleop_twist_keyboard
```

---

## 🚀 Launching the Demo

### Terminal 1: Main Demo Launch

```bash
# Make sure you've sourced the environment
source install/setup.bash

# Launch the warehouse demo (includes Gazebo + Rviz2)
ros2 launch arion_simulation warehouse.launch.py
```

**What happens on launch:**
- Gazebo opens with the AWS Small Warehouse world
- TurtleBot3 Waffle Pi robot appears at the origin (0, 0, 0)
- Rviz2 window opens showing:
  - Grid reference frame
  - Transform frames (TF tree)
  - Lidar scan data (blue/red points showing the warehouse structure)
  - Robot odometry

**Wait 15-30 seconds for full initialization**.

### Terminal 2: Keyboard Control (Optional)

Once Gazebo and Rviz2 are fully loaded:

```bash
# In a NEW terminal, source the environment
source ~/path/to/project_arion/install/setup.bash

# Launch keyboard teleop node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard controls:**
- **i** / **,** : Move forward / backward
- **j** / **l** : Turn left / right
- **u** / **o** : Move forward-left / forward-right
- **m** / **.** : Move backward-left / backward-right
- **k** : Stop
- **q** : Quit

### Demonstration Script

1. **Start the simulation** → Show Gazebo with the warehouse and robot
   - Point out: "This is the Amazon AWS open-source warehouse"
   - Highlight robot with red Lidar sensor on top

2. **Check Rviz2 visualization** → Show the Lidar scan data
   - Pan the view to show the scan points outlining the shelves
   - Explain: "The red dots are from the Lidar sensor - they map out the warehouse structure"

3. **Drive the robot** → Use keyboard teleop to move the robot
   - Drive slowly toward a shelf
   - Drive into the shelf to demonstrate collision detection
   - Explain: "Notice the robot stops - it detects the collision. This is essential for Phase 2 (mapping)"

4. **Check the code** → Show VS Code with the file structure
   - Explain the world is loaded from `src/arion_simulation/worlds/warehouse.sdf`
   - Show the robot URDF in `src/arion_simulation/urdf/`
   - Show the launch file: `src/arion_simulation/launch/warehouse.launch.py`

---

## 📁 File Structure

```
src/arion_simulation/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package dependencies
├── launch/
│   ├── start_world.launch.py      # Old simple launch
│   └── warehouse.launch.py        # ⭐ NEW: Complete demo launch
├── worlds/
│   ├── simple_floor.sdf           # Basic world
│   └── warehouse.sdf              # ⭐ NEW: Full warehouse with robot & pallet
├── urdf/
│   └── turtlebot3_waffle_pi.urdf.xacro  # ⭐ NEW: TurtleBot3 robot definition
├── config/
│   ├── default.rviz               # Basic visualization config
│   └── warehouse.rviz             # ⭐ NEW: Advanced config with LaserScan
├── meshes/                # Mesh files for visualizations
└── ...
```

---

## 🔧 Troubleshooting

### Issue: "Command not found: ros2 launch"

**Solution:** You haven't sourced the environment. Run:
```bash
source install/setup.bash
```

### Issue: Gazebo doesn't start

**Possible causes:**
- Gazebo is already running from a previous session
- Display server issues (common in WSL)
- Missing graphics drivers

**Solutions:**
```bash
# Kill any existing Gazebo instances
pkill -9 ign gazebo
pkill -9 rviz2

# Try again with the launch command
ros2 launch arion_simulation warehouse.launch.py
```

### Issue: World loads but robot doesn't appear

**Solution:** The robot should spawn at (0, 0, 0). If it's not visible:
1. In Rviz2, check the "Fixed Frame" is set to `odom`
2. Use the mouse to pan/rotate the view
3. Check the Gazebo window - the robot should be visible as a dark square

### Issue: Lidar scan not appearing in Rviz2

**Solution:**
1. In the Rviz2 Displays panel, check the "LaserScan" item is enabled (checkmark)
2. Make sure the Topic is set to `/scan`
3. Check that the Gazebo plugins are publishing data:
   ```bash
   ros2 topic list | grep scan
   ```

### Issue: Robot doesn't respond to keyboard commands

**Make sure:**
1. The teleop node is running in Terminal 2
2. The terminal window with teleop is **in focus** (selected)
3. Press 'q' to quit teleop, then restart it
4. Try with lowercase letters (not uppercase)

---

## 🎓 Educational Talking Points

### Goal 1: Local World Loading
> "We're using the Amazon AWS open-source warehouse, loaded directly from our arion_simulation package. This is important because it means we have full commercial rights to use it. We're not downloading or depending on external DroneBlocks scripts."

**Technical Proof:**
- Show the warehouse.sdf file in VS Code
- Show the AWS models are in `src/aws-robomaker-small-warehouse-world/models/`

### Goal 2: Standard Robot Platform
> "We're using the TurtleBot3 Waffle Pi. It's the industry standard for robotics education. Because of that, our students can find dozens of tutorials online if they get stuck."

**Technical Proof:**
- Show the robot URDF definition
- Show the Lidar sensor (red cylinder on top in Gazebo view)
- Show the Lidar spinning in the Gazebo 3D view

### Goal 3: Real Physics Simulation
> "This simulation has proper collision physics. When the robot drives into a shelf, it bounces or stops - it doesn't pass through like a ghost. This is critical for Phase 2, where we'll use Lidar to map the environment."

**Technical Proof:**
- Drive the robot into a shelf
- Show the robot stops/bounces in Gazebo
- Explain: "The ODE physics engine is handling collisions in real-time"

### Goal 4: Sensor Integration (Bonus)
> "The Lidar sensor is active and publishing real data. You can see the scan points in Rviz2 - the red dots outline the shelves. This validates that our sensor simulation is working."

**Technical Proof:**
- Show the LaserScan display in Rviz2
- Pan the camera to show the warehouse structure outlined by red dots
- Check `ros2 topic echo /scan` in a terminal to show raw sensor data

---

## 📊 On-the-Fly Debugging Commands

If something goes wrong during the demo:

```bash
# Check if Gazebo is running and what topics it's publishing
ros2 topic list

# Check if the robot is publishing odometry
ros2 topic echo /odom

# Check if the Lidar is publishing scans
ros2 topic echo /scan

# Check ROS time synchronization (should see timestamps increasing)
ros2 topic echo /clock

# Check if transforms are being published
ros2 run tf2_ros tf2_monitor

# View the TF tree graphically
ros2 run rqt_tf_tree rqt_tf_tree

# Record a bag file for later analysis
ros2 bag record -a -o demo_recording.bag
```

---

## 🎯 Success Criteria

Your demo is successful if:

✅ Gazebo window opens and shows the warehouse  
✅ Robot appears in Gazebo (dark square at center)  
✅ Rviz2 window opens next to Gazebo  
✅ Lidar scan points appear in Rviz2 (red dots outlining shelves)  
✅ Keyboard teleop commands move the robot  
✅ Robot collides with shelf instead of passing through  
✅ No error messages in terminal windows  

If all checkmarks are true, you're ready for Thursday's demo! 🎉

---

## 📝 Quick Reference Card for Demo Day

**Before the demo starts:**
```bash
# Terminal 1
cd ~/path/to/project_arion
colcon build --symlink-install
source install/setup.bash
ros2 launch arion_simulation warehouse.launch.py
```

**After Gazebo/Rviz2 loads (new terminal):**
```bash
# Terminal 2
source ~/path/to/project_arion/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Demo sequence:**
1. Show Gazebo warehouse and robot
2. Show Rviz2 Lidar scans
3. Drive robot around with keyboard
4. Drive into shelf to demonstrate collision
5. Show code in VS Code (warehouse.launch.py, warehouse.sdf)

---

## 🚀 Next Steps

After successfully running this demo:

- **Phase 2**: Implement SLAM (Simultaneous Localization and Mapping)
- **Phase 3**: Add ROS 2 navigation stack
- **Phase 4**: Integrate with AWS Robomaker for cloud deployment

Good luck with your Thursday presentation! 🎉
