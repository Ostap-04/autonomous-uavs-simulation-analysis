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

"""Launch an individual iris quadcopter for swarm operations."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition

from launch.event_handlers import OnProcessStart

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Generate a launch description for an individual iris quadcopter."""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Launch arguments
    drone_id_arg = DeclareLaunchArgument(
        "drone_id", default_value="0", description="Drone ID"
    )
    x_pos_arg = DeclareLaunchArgument(
        "x_pos", default_value="0.0", description="X position"
    )
    y_pos_arg = DeclareLaunchArgument(
        "y_pos", default_value="0.0", description="Y position"
    )

    # Get launch configurations
    drone_id = LaunchConfiguration("drone_id")
    x_pos = LaunchConfiguration("x_pos")
    y_pos = LaunchConfiguration("y_pos")

    # Calculate ports for each drone instance
    # Base ports: SITL=5760, DDS=2019, sim_address=5501
    # Each drone gets +10 offset from base
    sitl_port_base = 5760
    dds_port_base = 2019
    sim_port_base = 5501

    # Include SITL DDS component for this drone
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_dds_udp.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "transport": "udp4",
            "port": PythonExpression([str(dds_port_base), " + int(", drone_id, ") * 10"]),
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": drone_id,
            "defaults": os.path.join(
                pkg_ardupilot_gazebo,
                "config",
                "gazebo-iris-gimbal.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": PythonExpression(['"tcp:127.0.0.1:" + str(', str(sitl_port_base), ' + int(', drone_id, ') * 10)']),

            "sitl": PythonExpression(['"127.0.0.1:" + str(', str(sim_port_base), ' + int(', drone_id, ') * 10)']),
        }.items(),
    )

    # Ensure `SDF_PATH` is populated
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file
    sdf_file = os.path.join(
        pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Robot state publisher with namespace
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=["drone_", drone_id],
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ["drone_", drone_id, "/"]},
        ],
    )

    # Bridge with namespace
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=["drone_", drone_id],
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "iris_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # TF relay for this drone
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        namespace=["drone_", drone_id],
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    return LaunchDescription(
        [
            drone_id_arg,
            x_pos_arg,
            y_pos_arg,
            DeclareLaunchArgument(
                "use_gz_tf", default_value="true", description="Use Gazebo TF."
            ),
            sitl_dds,
            robot_state_publisher,
            bridge,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=bridge,
                    on_start=[topic_tools_tf]
                )
            ),
        ]
    )