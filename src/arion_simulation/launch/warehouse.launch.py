#!/usr/bin/env python3
"""
Comprehensive launch file for the AI Academy Warehouse Demo.

This launch file does the following:
1. Loads the AWS Small Warehouse world with a TurtleBot3 Waffle Pi robot
2. Starts Gazebo simulation with physics and sensor plugins enabled
3. Creates ROS 2 bridges for Gazebo <-> ROS 2 communication
4. Launches Rviz2 for visualization of sensor data (LaserScan, transforms)
5. Makes the robot ready for teleop control

Usage:
    ros2 launch arion_simulation warehouse.launch.py
    
Then in another terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
import shutil
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""
    
    # Get the arion_simulation package directory
    arion_pkg = get_package_share_directory('arion_simulation')
    aws_warehouse_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')
    
    # Paths to world and config files
    world_src = os.path.join(arion_pkg, 'worlds', 'warehouse.sdf')
    world_file = '/tmp/warehouse.sdf'
    
    rviz_config = os.path.join(arion_pkg, 'config', 'warehouse.rviz')
    
    # Copy world file to /tmp to handle any path issues (especially with spaces in WSL)
    shutil.copyfile(world_src, world_file)
    
    # Print debug information
    print("\n" + "="*80)
    print("🎯 AI ACADEMY WAREHOUSE DEMO - LAUNCHING")
    print("="*80)
    print(f"[INFO] Using world file: {world_file}")
    print(f"[INFO] Arion package: {arion_pkg}")
    print(f"[INFO] AWS Warehouse package: {aws_warehouse_pkg}")
    print("\n[NEXT STEPS]")
    print("1. Wait for Gazebo and Rviz2 to fully load (15-30 seconds)")
    print("2. Open another terminal and run:")
    print("   $ ros2 run teleop_twist_keyboard teleop_twist_keyboard")
    print("3. Use arrow keys to drive the robot")
    print("4. Try to drive into a shelf to verify collision physics")
    print("="*80 + "\n")
    
    return LaunchDescription([
        # ========================================================================
        # Environment Variables for Gazebo and ROS 2
        # ========================================================================
        
        # Set Gazebo resource path to include both arion and AWS packages
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=f"{arion_pkg}:{aws_warehouse_pkg}/models:{aws_warehouse_pkg}/worlds"
        ),
        
        # Isolate this simulation on a unique partition to avoid interference
        SetEnvironmentVariable(
            name='IGN_PARTITION',
            value='arion_simulation'
        ),
        
        # Set OpenGL version for compatibility
        SetEnvironmentVariable(
            name='MESA_GL_VERSION_OVERRIDE',
            value='4.1'
        ),
        
        SetEnvironmentVariable(
            name='GALLIUM_DRIVER',
            value='llvmpipe'
        ),
        
        # ========================================================================
        # Gazebo Simulation (Ignition Gazebo)
        # ========================================================================
        
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file, '-v', '4'],
            output='screen',
            additional_env={
                'MESA_GL_VERSION_OVERRIDE': '4.1',
                'GALLIUM_DRIVER': 'llvmpipe',
            },
            name='gazebo'
        ),
        
        # ========================================================================
        # ROS 2 <-> Gazebo Bridge (for clock, transforms, and odometry)
        # ========================================================================
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Clock bridge: Gazebo simulation time -> ROS 2
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Transform frames (static and dynamic)
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                # Odometry from the differential drive plugin
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
            remappings=[],
            output='screen',
            name='ros_gz_bridge'
        ),
        
        # ========================================================================
        # Rviz2 Visualization
        # ========================================================================
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            name='rviz2'
        ),
        
    ])


if __name__ == '__main__':
    generate_launch_description()