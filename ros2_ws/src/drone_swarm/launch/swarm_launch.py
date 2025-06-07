#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

world_file_arg = DeclareLaunchArgument(
    'world_file',
    default_value=PathJoinSubstitution([
        FindPackageShare('drone_swarm'),
        'worlds',
        'swarm_world.sdf'
    ]),
    description='Gazebo world file'
)

def generate_launch_description():
    print("Running from directory:", os.getcwd())
    """Generate launch description for drone swarm"""
    
    # Launch configuration
    # num_drones = 10
    num_drones = 2
    
    # Gazebo launch
    gazebo_launch = ExecuteProcess(
    cmd=['gz', 'sim', LaunchConfiguration('world_file')],
    output='screen'
    )
    
    # ArduPilot SITL launcher
    sitl_launch = ExecuteProcess(
    cmd=[
        'bash',
        PathJoinSubstitution([
            FindPackageShare('drone_swarm'),
            'scripts',
            'launch_swarm_sitl.sh'
        ])
    ],
    output='screen'
    )
    
    # Swarm coordinator node
    swarm_coordinator = Node(
        package='drone_swarm',
        executable='swarm_coordinator',
        name='swarm_coordinator',
        output='screen',
        parameters=[{'num_drones': num_drones}]
    )
    
    # Target selection node
    target_selector = Node(
        package='drone_swarm',
        executable='target_selector',
        name='target_selector',
        output='screen'
    )
    
    # MAVROS nodes for each drone
    mavros_nodes = []
    drone_controllers = []
    base_port =5760
    
    for i in range(num_drones):
        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            namespace=f'drone_{i}',
            output='screen',
            parameters=[{
                'fcu_url': f'tcp://127.0.0.1:{base_port + i * 10}'
            }]
        )
        mavros_nodes.append(mavros_node)
        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", len(mavros_nodes))
        # for node in mavros_nodes:
        #     print(f'Node name: {node}')
        # Drone controller

        controller = Node(
            package='drone_swarm',
            executable='drone_controller',
            name=f'drone_controller_{i}',
            output='screen',
            arguments=[str(i)]
        )
        drone_controllers.append(controller)
    
    # # Obstacle detection nodes (using simulated lidar)
    # obstacle_nodes = []
    # for i in range(num_drones):
    #     obstacle_node = Node(
    #         package='drone_swarm',
    #         executable='obstacle_detector',
    #         name=f'obstacle_detector_{i}',
    #         output='screen',
    #         arguments=[str(i)]
    #     )
    #     obstacle_nodes.append(obstacle_node)
    
    # Build launch description
    launch_description = LaunchDescription([
        world_file_arg,
        gazebo_launch,
        sitl_launch,
        swarm_coordinator,
        # target_selector
    ])
    
    # Add all MAVROS nodes
    for node in mavros_nodes:
        launch_description.add_action(node)
    
    # Add all drone controllers
    for controller in drone_controllers:
        launch_description.add_action(controller)
    
    # Add obstacle detection nodes
    # for obs_node in obstacle_nodes:
    #     launch_description.add_action(obs_node)
    
    return launch_description