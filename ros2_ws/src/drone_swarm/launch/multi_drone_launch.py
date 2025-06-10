# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""Launch a swarm of iris quadcopters in Gazebo and Rviz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Generate a launch description for a swarm of iris quadcopters."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_drone_swarm = get_package_share_directory("drone_swarm")


    # Launch arguments
    num_drones_arg = DeclareLaunchArgument(
        "num_drones", 
        default_value="3", 
        description="Number of drones in the swarm"
    )

    # Get number of drones from launch configuration
    num_drones = LaunchConfiguration("num_drones")
    
    # Create drone instances
    drone_launches = []
    
    # Predefined positions for up to 10 drones in a grid pattern
    drone_positions = [
        (0, 0),    # drone 0
        (2, 0),    # drone 1
        (4, 0),    # drone 2
        (0, 2),    # drone 3
        (2, 2),    # drone 4
        (4, 2),    # drone 5
        (0, 4),    # drone 6
        (2, 4),    # drone 7
        (4, 4),    # drone 8
        (6, 0),    # drone 9
    ]

    # Create individual iris drone launches for each drone
    for i in range(2):  # Support up to 10 drones
        iris_drone = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("drone_swarm"),
                            "launch",
                            "drone_individual.launch.py",
                        ]
                    ),
                ]
            ),
            launch_arguments={
                "drone_id": str(i),
                "x_pos": str(drone_positions[i][0]),
                "y_pos": str(drone_positions[i][1]),
            }.items(),
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('num_drones'), ' > 0']))
        )
        drone_launches.append(iris_drone)

    # Gazebo simulation
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_drone_swarm) / "worlds" / "drone_runway_swarm.sdf"}'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # RViz
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris_swarm.rviz"}'],
    #     condition=IfCondition(LaunchConfiguration("rviz")),
    # )

    return LaunchDescription(
        [
            num_drones_arg,
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim_server,
            gz_sim_gui,
        ] + drone_launches + [
            # rviz,
        ]
    )

# # Copyright 2023 ArduPilot.org.
# #
# # This program is free software: you can redistribute it and/or modify
# # it under the terms of the GNU General Public License as published by
# # the Free Software Foundation, either version 3 of the License, or
# # (at your option) any later version.
# #
# # This program is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# # GNU General Public License for more details.

# """
# Launch multiple ArduPilot SITL instances for drone swarm simulation.

# Usage:
# ros2 launch your_package multi_drone_sitl.launch.py num_drones:=3
# ros2 launch your_package multi_drone_sitl.launch.py num_drones:=5 use_dds:=false
# """

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node, PushRosNamespace
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     """Generate launch description for multiple drone SITL instances."""
    
#     # Launch arguments
#     num_drones_arg = DeclareLaunchArgument(
#         'num_drones',
#         default_value='2',
#         description='Number of drone instances to launch'
#     )
    
#     use_dds_arg = DeclareLaunchArgument(
#         'use_dds',
#         default_value='true',
#         description='Enable DDS communication'
#     )
    
#     use_gazebo_arg = DeclareLaunchArgument(
#         'use_gazebo',
#         default_value='false',
#         description='Launch Gazebo simulation'
#     )
    
#     speedup_arg = DeclareLaunchArgument(
#         'speedup',
#         default_value='1',
#         description='Simulation speedup factor'
#     )

#     # Get launch configuration values
#     num_drones = LaunchConfiguration('num_drones')
#     use_dds = LaunchConfiguration('use_dds')
#     use_gazebo = LaunchConfiguration('use_gazebo')
#     speedup = LaunchConfiguration('speedup')
    
#     # Base configuration
#     base_sysid = 1
#     base_mavlink_port = 14550
#     base_dds_port = 2019
#     base_sim_port = 9000
    
#     # Store all launch actions
#     launch_actions = []
    
#     # Add launch arguments
#     launch_actions.extend([
#         num_drones_arg,
#         use_dds_arg,
#         use_gazebo_arg,
#         speedup_arg
#     ])
    
#     # Generate drone instances dynamically
#     # Note: In practice, you might want to limit this to a reasonable number
#     # and generate the launch file statically for better performance
    
#     # For demonstration, we'll create a few instances statically
#     # In a real implementation, you'd generate this dynamically or use a loop
    
#     for i in range(5):  # Support up to 5 drones
#         drone_actions = create_drone_instance(
#             drone_id=i,
#             base_sysid=base_sysid + i,
#             mavlink_port=base_mavlink_port + i * 10,
#             dds_port=base_dds_port + i * 10,
#             sim_port=base_sim_port + i * 10,
#             speedup=speedup,
#             use_dds=use_dds,
#             condition_check=f'$(eval {i} < {num_drones})'  # Only launch if i < num_drones
#         )
#         launch_actions.extend(drone_actions)
    
#     return LaunchDescription(launch_actions)


# def create_drone_instance(drone_id, base_sysid, mavlink_port, dds_port, sim_port, speedup, use_dds, condition_check):
#     """Create launch actions for a single drone instance."""
    
#     drone_namespace = f'drone_{drone_id}'
    
#     # SITL process
#     sitl_process = ExecuteProcess(
#         cmd=[
#             'sim_vehicle.py',
#             '-v', 'ArduCopter',
#             '-I', str(drone_id),
#             f'--sysid={base_sysid}',
#             f'--speedup={speedup}',
#             '--no-mavproxy',
#             f'--out=udp:127.0.0.1:{mavlink_port}',
#             f'--sim-address=127.0.0.1:{sim_port}'
#         ],
#         output='screen',
#         name=f'sitl_drone_{drone_id}',
#         # condition=IfCondition(condition_check)  # You'd need to implement this condition properly
#     )
    
#     # MAVProxy process for this drone
#     mavproxy_process = ExecuteProcess(
#         cmd=[
#             'mavproxy.py',
#             f'--master=udp:127.0.0.1:{mavlink_port}',
#             '--sitl=127.0.0.1:5501',
#             f'--out=udp:127.0.0.1:{mavlink_port + 1}',  # Output port for external connections
#             '--aircraft', f'drone_{drone_id}'
#         ],
#         output='screen',
#         name=f'mavproxy_drone_{drone_id}',
#         # condition=IfCondition(condition_check)
#     )
    
#     # DDS Agent (if enabled)
#     dds_agent = Node(
#         package='micro_ros_agent',
#         executable='micro_ros_agent',
#         name=f'micro_ros_agent_drone_{drone_id}',
#         arguments=['udp4', '--port', str(dds_port)],
#         output='screen',
#         # condition=IfCondition(AndSubstitution(use_dds, condition_check))
#     )
    
#     # Group all drone-related nodes under a namespace
#     drone_group = GroupAction([
#         PushRosNamespace(drone_namespace),
#         sitl_process,
#         mavproxy_process,
#         dds_agent,
#     ])
    
#     return [drone_group]


# # Alternative simpler approach for static configuration
# def generate_static_launch_description():
#     """Generate a static launch description for exactly N drones."""
    
#     # Launch arguments
#     launch_args = [
#         DeclareLaunchArgument('speedup', default_value='1'),
#         DeclareLaunchArgument('use_dds', default_value='true'),
#     ]
    
#     # Configuration
#     num_drones = 3  # Fixed number for this example
#     base_sysid = 1
#     base_port = 14550
    
#     drone_processes = []
    
#     for i in range(num_drones):
#         sysid = base_sysid + i
#         mavlink_port = base_port + i * 10
        
#         # SITL instance
#         sitl = ExecuteProcess(
#             cmd=[
#                 'sim_vehicle.py',
#                 '-v', 'ArduCopter',
#                 '-I', str(i),
#                 f'--sysid={sysid}',
#                 '--speedup=1',
#                 '--no-mavproxy',
#                 f'--out=udp:127.0.0.1:{mavlink_port}',
#             ],
#             output='screen',
#             name=f'sitl_drone_{i}',
#         )
        
#         # MAVProxy for external connections
#         mavproxy = ExecuteProcess(
#             cmd=[
#                 'mavproxy.py',
#                 f'--master=udp:127.0.0.1:{mavlink_port}',
#                 '--aircraft', f'drone_{i}',
#                 '--load-module', 'console',
#             ],
#             output='screen',
#             name=f'mavproxy_drone_{i}',
#         )
        
#         drone_processes.extend([sitl, mavproxy])
    
#     return LaunchDescription(launch_args + drone_processes)