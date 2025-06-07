#!/bin/bash

# Launch multiple ArduPilot SITL instances for drone swarm
# Usage: ./launch_swarm_sitl.sh

# NUM_DRONES=10
NUM_DRONES=2
BASE_SYSID=1
BASE_PORT=14550
BASE_SIM_PORT=9000

# Kill existing instances
pkill -f "arducopter"
pkill -f "mavproxy"

# Wait a moment for cleanup
sleep 5

echo "Launching $NUM_DRONES ArduPilot SITL instances..."

for i in $(seq 0 $((NUM_DRONES-1))); do
    SYSID=$((BASE_SYSID + i))
    SIM_PORT=$((BASE_SIM_PORT + i*10))
    MAVLINK_PORT=$((BASE_PORT + i*10))
    
    INSTANCE_DIR="$HOME/ardupilot_instances/drone_$i"
    mkdir -p "$INSTANCE_DIR"
    
    echo "Starting drone $i (SYSID: $SYSID, Ports: $MAVLINK_PORT, $SIM_PORT)"
    
    cd "$INSTANCE_DIR"
    sim_vehicle.py \
        -v ArduCopter \
        --console \
        --map \
        -I $i \
        --sysid=$SYSID \
        --no-mavproxy \
        --speedup=1 \
        > "$INSTANCE_DIR/log.txt" 2>&1 &
    
    sleep 5
done
        # --out=udp:127.0.0.1:$MAVLINK_PORT \
        # --sim-address=127.0.0.1:$SIM_PORT \

echo "All SITL instances launched!"
echo "You can connect to individual drones using:"
for i in $(seq 0 $((NUM_DRONES-1))); do
    echo "  Drone $i: mavproxy.py --master=udp:127.0.0.1:$((14550 + i))"
done





# !/bin/bash

# Launch multiple ArduPilot SITL instances for drone swarm
# Usage: ./launch_swarm_sitl.sh

# !/bin/bash

# Launch multiple ArduPilot SITL instances for drone swarm
# Usage: ./launch_swarm_sitl.sh


# NUM_DRONES=1
# BASE_SYSID=1
# BASE_MAVLINK_PORT=5760
# BASE_SIM_PORT=9000

# # Kill existing instances
# pkill -f "arducopter"
# pkill -f "mavproxy"
# pkill -f "sim_vehicle"

# # Wait a moment for cleanup
# sleep 5

# echo "Launching $NUM_DRONES ArduPilot SITL instances..."

# for i in $(seq 0 $((NUM_DRONES-1))); do
#     SYSID=$((BASE_SYSID + i))
#     SIM_PORT=$((BASE_SIM_PORT + i*10))
#     MAVLINK_PORT=$((BASE_MAVLINK_PORT + i*10))
    
#     INSTANCE_DIR="$HOME/ardupilot_instances/drone_$i"
#     mkdir -p "$INSTANCE_DIR"
    
#     echo "Starting drone $i (SYSID: $SYSID, MAVLINK Port: $MAVLINK_PORT, SIM Port: $SIM_PORT)"
    
#     cd "$INSTANCE_DIR"
    
#     # Method 1: Using tcpin (TCP server mode - MAVROS connects as client)
#     echo "Trying Method 1: tcpin format"
#     sim_vehicle.py \
#         -v ArduCopter \
#         -I $i \
#         --sysid=$SYSID \
#         --no-mavproxy \
#         --speedup=1 \
#         --out=tcpin:0.0.0.0:$MAVLINK_PORT \
#         > "$INSTANCE_DIR/log.txt" 2>&1 &
    
#     SITL_PID=$!
    
#     # Wait a moment and check if process is still running
#     sleep 3
#     if ! kill -0 $SITL_PID 2>/dev/null; then
#         echo "Method 1 failed, trying Method 2: tcp format"
        
#         # Method 2: Using tcp (different format)
#         sim_vehicle.py \
#             -v ArduCopter \
#             -I $i \
#             --sysid=$SYSID \
#             --no-mavproxy \
#             --speedup=1 \
#             --out=tcp:127.0.0.1:$MAVLINK_PORT \
#             > "$INSTANCE_DIR/log.txt" 2>&1 &
        
#         SITL_PID=$!
#         sleep 3
        
#         if ! kill -0 $SITL_PID 2>/dev/null; then
#             echo "Method 2 failed, trying Method 3: UDP format"
            
#             # Method 3: Using UDP (most compatible)
#             sim_vehicle.py \
#                 -v ArduCopter \
#                 -I $i \
#                 --sysid=$SYSID \
#                 --no-mavproxy \
#                 --speedup=1 \
#                 --out=udp:127.0.0.1:$MAVLINK_PORT \
#                 > "$INSTANCE_DIR/log.txt" 2>&1 &
            
#             echo "Using UDP connection for drone $i"
#         else
#             echo "Using TCP client connection for drone $i"
#         fi
#     else
#         echo "Using TCP server connection for drone $i"
#     fi
    
#     # Wait for SITL to start up properly
#     sleep 8
# done

# echo "All SITL instances launched!"
# echo "Connection details:"
# for i in $(seq 0 $((NUM_DRONES-1))); do
#     MAVLINK_PORT=$((BASE_MAVLINK_PORT + i*10))
#     echo "  Drone $i TCP: tcp://127.0.0.1:$MAVLINK_PORT"
#     echo "  Drone $i UDP: udp://127.0.0.1:$MAVLINK_PORT@127.0.0.1:14550"
# done

# echo ""
# echo "To test connection manually:"
# echo "  mavproxy.py --master=tcp:127.0.0.1:5760"
# echo "  OR"
# echo "  mavproxy.py --master=udp:127.0.0.1:5760"

# echo ""
# echo "To connect MAVROS nodes:"
# echo "For TCP: ros2 run mavros mavros_node --ros-args -r __node:=mavros_drone_0 -p fcu_url:=tcp://127.0.0.1:5760"
# echo "For UDP: ros2 run mavros mavros_node --ros-args -r __node:=mavros_drone_0 -p fcu_url:=udp://:14550@127.0.0.1:5760"