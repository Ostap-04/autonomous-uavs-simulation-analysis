#!/bin/bash

# Launch multiple ArduPilot SITL instances for drone swarm
# Usage: ./launch_swarm_sitl.sh [num_drones]

NUM_DRONES=${1:-2}  # Default to 2 drones if no argument provided
BASE_SYSID=1
BASE_MAVLINK_PORT=14550
BASE_SIM_PORT=9000

echo "Launching $NUM_DRONES ArduPilot SITL instances..."

# Kill existing instances
echo "Cleaning up existing processes..."
pkill -f "arducopter"
pkill -f "mavproxy"
pkill -f "sim_vehicle"

# Wait for cleanup
sleep 3

# Create base directory for instances
mkdir -p "$HOME/ardupilot_instances"

for i in $(seq 0 $((NUM_DRONES-1))); do
    SYSID=$((BASE_SYSID + i))
    SIM_PORT=$((BASE_SIM_PORT + i*10))
    MAVLINK_PORT=$((BASE_MAVLINK_PORT + i*10))
    
    INSTANCE_DIR="$HOME/ardupilot_instances/drone_$i"
    mkdir -p "$INSTANCE_DIR"
    
    echo "Starting drone $i:"
    echo "  - System ID: $SYSID"
    echo "  - MAVLink Port: $MAVLINK_PORT" 
    echo "  - Sim Port: $SIM_PORT"
    echo "  - Instance Dir: $INSTANCE_DIR"
    
    cd "$INSTANCE_DIR"
    
    # Launch SITL instance
    sim_vehicle.py \
        -v ArduCopter \
        -I $i \
        --sysid=$SYSID \
        --speedup=1 \
        --no-mavproxy \
        --out=udp:127.0.0.1:$MAVLINK_PORT \
        --sim-address=127.0.0.1:$SIM_PORT \
        > "$INSTANCE_DIR/sitl_log.txt" 2>&1 &
    
    SITL_PID=$!
    echo "  - Process ID: $SITL_PID"
    
    # Wait for SITL to initialize
    sleep 8
    
    # Check if process is still running
    if ! kill -0 $SITL_PID 2>/dev/null; then
        echo "  - ERROR: Failed to start drone $i"
        cat "$INSTANCE_DIR/sitl_log.txt"
    else
        echo "  - SUCCESS: Drone $i started"
    fi
    
    echo ""
done

echo "========================================="
echo "All SITL instances launched!"
echo "========================================="

echo ""
echo "Connection details:"
for i in $(seq 0 $((NUM_DRONES-1))); do
    MAVLINK_PORT=$((BASE_MAVLINK_PORT + i*10))
    echo "Drone $i:"
    echo "  - UDP: udp://127.0.0.1:$MAVLINK_PORT"
    echo "  - MAVProxy: mavproxy.py --master=udp:127.0.0.1:$MAVLINK_PORT"
    echo ""
done

echo "To kill all instances: pkill -f 'sim_vehicle'"
echo "Log files are in: $HOME/ardupilot_instances/drone_*/sitl_log.txt"