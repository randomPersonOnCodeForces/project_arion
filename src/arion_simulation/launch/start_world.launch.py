import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    arion_pkg = get_package_share_directory('arion_simulation')
    aws_warehouse_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')
    # Prefer the provided AWS small_warehouse.world which references available models
    aws_world_file = os.path.join(aws_warehouse_pkg, 'worlds', 'small_warehouse', 'small_warehouse.world')
    arion_world_file = os.path.join(arion_pkg, 'worlds', 'warehouse.sdf')
    world_file = aws_world_file if os.path.exists(aws_world_file) else arion_world_file

 
    gazebo_models_path = os.path.join(aws_warehouse_pkg, 'models')
    gazebo_worlds_path = os.path.join(aws_warehouse_pkg, 'worlds')
    arion_worlds_path = os.path.join(arion_pkg, 'worlds')

    # Compose a platform-correct path list (':' on Unix/WSL, ';' on Windows)
    resource_paths = os.pathsep.join([gazebo_models_path, gazebo_worlds_path, arion_worlds_path])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=resource_paths
        ),
        # also set legacy GAZEBO_MODEL_PATH for compatibility
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.pathsep.join([gazebo_models_path, gazebo_worlds_path])
        ),
        # Disable GPU hardware acceleration to avoid OGRE crashes in WSL
        SetEnvironmentVariable(
            name='MESA_GL_VERSION_OVERRIDE',
            value='4.1'
        ),
        SetEnvironmentVariable(
            name='GALLIUM_DRIVER',
            value='llvmpipe'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file} -v 4'}.items(),
        )
    ])