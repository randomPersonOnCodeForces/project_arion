#!/usr/bin/env python3
"""
𝗬𝘂𝗶𝗰𝗸 𝗥𝗲𝗳𝗲𝗿𝗲𝗻𝗰𝗲: 𝗔𝗜 𝗔𝗰𝗮𝗱𝗲𝗺𝘆 𝗪𝗮𝗿𝗲𝗵𝗼𝘂𝘀𝗲 𝗗𝗼𝖆𝗼
Run this script once to generate formatted demo commands.
"""

commands = {
    "SETUP": [
        ("Clean build", "rm -rf build install log && colcon build --symlink-install"),
        ("Source environment", "source install/setup.bash"),
        ("Verify ready", "bash check-demo-ready.sh"),
    ],
    "LAUNCH": [
        ("Terminal 1", "source install/setup.bash && ros2 launch arion_simulation warehouse.launch.py"),
        ("Terminal 2 (after Gazebo loads)", "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"),
    ],
    "DEMO SEQUENCE": [
        ("1. Show Gazebo", "Warehouse world with TurtleBot3 visible at center"),
        ("2. Show Rviz2", "Lidar scan points (red dots) outlining warehouse structure"),
        ("3. Drive robot", "Use keyboard arrow keys to move robot around"),
        ("4. Collision test", "Drive robot into a shelf; robot should stop/bounce"),
        ("5. Show code", "Open VS Code; show warehouse.launch.py, warehouse.sdf"),
    ],
    "TROUBLESHOOTING": [
        ("Robot not visible", "In Rviz2: Set Fixed Frame to 'odom'; use mouse to pan camera"),
        ("LaserScan not showing", "In Rviz2: Check LaserScan box is enabled; Topic = '/scan'"),
        ("Teleop not working", "Make sure teleop terminal is in focus; try 'q' then restart"),
        ("Gazebo won't start", "Run: pkill -9 ign; pkill -9 rviz2; then launch again"),
        ("Command not found", "Run: source install/setup.bash"),
    ],
    "KEYBOARD CONTROLS": [
        ("Move forward", "Press: i"),
        ("Move backward", "Press: ,"),
        ("Turn left", "Press: j"),
        ("Turn right", "Press: l"),
        ("Stop", "Press: k"),
        ("Quit teleop", "Press: q"),
    ]
}

def print_section(title, items):
    """Print a formatted section."""
    print(f"\n{'='*70}")
    print(f"  {title}")
    print(f"{'='*70}")
    for label, command in items:
        print(f"\n  {label}:")
        print(f"  $ {command}")

print("\n")
print("╔" + "═"*68 + "╗")
print("║" + " "*12 + "🤖 AI ACADEMY WAREHOUSE DEMO - QUICK REFERENCE" + " "*11 + "║")
print("╚" + "═"*68 + "╝")

for section, items in commands.items():
    print_section(section, items)

print(f"\n" + "="*70)
print("  📋 DEMO CHECKLIST")
print("="*70)

checklist = [
    ("✓", "Run clean build before Thursday"),
    ("✓", "Source environment: source install/setup.bash"),
    ("✓", "Test launch: ros2 launch arion_simulation warehouse.launch.py"),
    ("✓", "Verify Gazebo opens and robot appears"),
    ("✓", "Verify Rviz2 opens with Lidar scans visible"),
    ("✓", "Test keyboard control with teleop_twist_keyboard"),
    ("✓", "Drive robot into shelf to test collision physics"),
    ("✓", "Have VS Code ready to show code files"),
]

for check, item in checklist:
    print(f"\n  {check} {item}")

print(f"\n" + "="*70)
print("  🎯 DEMO TALKING POINTS")
print("="*70)

talking_points = [
    ("Clean Room", "AWS warehouse loaded locally from arion_simulation package"),
    ("Robot", "TurtleBot3 Waffle Pi - industry standard for robotics education"),
    ("Lidar", "Red dots in Rviz2 are real sensor data from simulation"),
    ("Physics", "Robot stops when hitting shelf - proves collision detection works"),
    ("Integration", "Full ROS 2 pipeline from Gazebo to Rviz2 visualization"),
]

for title, point in talking_points:
    print(f"\n  {title}:")
    print(f"    '{point}'")

print(f"\n" + "="*70)
print("  ℹ️  USEFUL DEBUG COMMANDS DURING DEMO")
print("="*70)

debug_commands = [
    ("Check topics", "ros2 topic list"),
    ("See Lidar data", "ros2 topic echo /scan"),
    ("See odometry", "ros2 topic echo /odom"),
    ("See transforms", "ros2 run tf2_ros tf2_monitor"),
    ("View TF tree", "ros2 run rqt_tf_tree rqt_tf_tree"),
]

for task, cmd in debug_commands:
    print(f"\n  {task}:")
    print(f"  $ {cmd}")

print(f"\n" + "="*70 + "\n")
