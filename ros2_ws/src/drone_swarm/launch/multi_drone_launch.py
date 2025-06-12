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
    for i in range(4):  # Support up to 10 drones
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
