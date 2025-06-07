# autonomous-uavs-simulation-analysis

BUILD: In: ~/autonomous-uavs-simulation-analysis/ros2_ws$ : colcon build --packages-select drone_swarm

RUN all: ros2 launch drone_swarm swarm_launch.py

NUM_drones: in swarm_launch.py and launch_swarm_sitl.sh 
!!!! In swarm_worls.sdf adjust drone models.


