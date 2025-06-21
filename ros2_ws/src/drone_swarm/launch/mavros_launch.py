#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    print("Running from directory:", os.getcwd())
    """Generate launch description for drone swarm"""
    
    # Launch configuration
    # num_drones = 10
    num_drones = 4

    # MAVROS nodes for each drone
    mavros_nodes = []
    drone_controllers = []
    base_port =14550
    # base_port = 5762
    
    for i in range(num_drones):
        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            namespace=f'drone_{i}',
            output='screen',
            parameters=[{
                'fcu_url': f'udp://127.0.0.1:{base_port + i * 10}@',
                "tgt_system" : i + 1
            }]
        )
        mavros_nodes.append(mavros_node)


        controller = Node(
            package='drone_swarm',
            executable='drone_controller',
            name=f'drone_controller_{i}',
            output='screen',
            arguments=[str(i)]
        )
        drone_controllers.append(controller)
    
    launch_description = LaunchDescription([
    ])
    
    # Add all MAVROS nodes
    for node in mavros_nodes:
        launch_description.add_action(node)
    
    return launch_description