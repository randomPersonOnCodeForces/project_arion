import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    arion_pkg = get_package_share_directory('arion_simulation')
    aws_warehouse_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')

    world_file = os.path.join(arion_pkg, 'worlds', 'warehouse.sdf')

 
    gazebo_resource_path = os.path.join(aws_warehouse_pkg, '..')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=gazebo_resource_path
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),
    ])