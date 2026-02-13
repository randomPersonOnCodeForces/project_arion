import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    arion_pkg = get_package_share_directory('arion_simulation')
    # Use simple floor world to save resources while providing a ground plane
    # Workaround: Copy file to /tmp to avoid issues with spaces in the path (common in WSL)
    world_src = os.path.join(arion_pkg, 'worlds', 'simple_floor.sdf')
    world_file = '/tmp/simple_floor.sdf'
    shutil.copyfile(world_src, world_file)
    xacro_file = os.path.join(arion_pkg, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    # Add the package share directory to the resource path so Gazebo can find worlds/models
    resource_paths = arion_pkg

    # DEBUG: Print the world file being loaded to verify changes
    print(f"\n[DEBUG] VERIFICATION: Launching modified start_world.launch.py with world: {world_file}\n")

    urdf_path = os.path.join(arion_pkg, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    # Process the Xacro file to get plain XML
    robot_description_config = xacro.process_file(urdf_path)
    robot_description_xml = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=resource_paths
        ),
        # Isolate this simulation on a unique partition to prevent "zombie" processes
        # or other Gazebo instances from interfering (e.g. loading the wrong world).
        SetEnvironmentVariable(
            name='IGN_PARTITION',
            value='arion_simulation'
        ),

        # Run Gazebo with GUI enabled for debugging
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file, '-v', '4'],
            output='screen',
            additional_env={
                'MESA_GL_VERSION_OVERRIDE': '4.1',
                'GALLIUM_DRIVER': 'llvmpipe',
            }
        ),

        # Bridge to connect Gazebo to ROS 2 (Essential for Rviz2 simulation time and transforms)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            remappings=[],
            output='screen'
        ),
        robot_state_publisher_node,

        Node(
            package='ros_gz_sim',
            executable='create',
            name='create_entity',
            arguments=[
                '-name', 'turtlebot3_waffle_pi',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),

        # Launch RViz with full path using ros2 command (more reliable in launch context)
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', os.path.join(arion_pkg, 'config', 'default.rviz')],
            output='screen'
        )
    ])
