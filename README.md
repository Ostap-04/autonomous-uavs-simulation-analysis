# autonomous-uavs-simulation-analysis

BUILD: In: ~/autonomous-uavs-simulation-analysis/ros2_ws$ : colcon build --packages-select drone_swarm

RUN drones: ros2 launch drone_swarm multi_drone_launch.py

RUN mavros: ros2 launch drone_swarm mavros_launch.py

Add to ~/.bashrc: 
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/<your_username>/autonomous-uavs-simulation-analysis/ros2_ws/src/drone_swarm/models
source ~/ros2_ws/install/setup.bash
source ~/ardu_ws/install/setup.bash


NUM_drones: in multi_drone_launch.py and mavros_launch.py 
!!!! In drone_runway_swarm.sdf adjust drone models.


