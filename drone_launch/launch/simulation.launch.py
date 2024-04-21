#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------
    
    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')
    
    simulation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('drone_simulation'), 'launch/simulation.launch.py')),
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
    )
    
    controller_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('drone_control'), 'launch/control.launch.py')),
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns')
        }.items(),
    )

    gui_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('drone_gui'), 'launch/gui.launch.py')),
        launch_arguments={
            'vehicle_id': LaunchConfiguration('vehicle_id'), 
            'vehicle_ns': LaunchConfiguration('vehicle_ns')
        }.items(),
    )

        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        # Launch files
        simulation_launch_file,
        controller_launch_file,
        gui_launch_file
        ])