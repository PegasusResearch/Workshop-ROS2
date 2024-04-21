#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------
    
    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='vehicle', description='Namespace to append to every topic and node name')
    
    gui_node = Node(
        package='drone_gui',
        namespace=[
            LaunchConfiguration('vehicle_ns'), 
            LaunchConfiguration('vehicle_id')],
        executable='drone_gui',
        name='drone_gui',
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'vehicle_ns': LaunchConfiguration('vehicle_ns')
            }
        ]
    )
        
    # Return the node to be launched by ROS2
    return LaunchDescription([
        # Launch arguments
        id_arg,
        namespace_arg,
        gui_node])