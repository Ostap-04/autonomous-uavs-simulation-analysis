#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import numpy as np
import math
from pymavlink import mavutil

class DroneController(Node):
    def __init__(self, drone_id):
        super().__init__(f'drone_controller_{drone_id}')
        
        self.drone_id = drone_id
        self.current_position = Point()
        self.target_waypoint = Point()
        self.target_position = Point()
        self.current_state = "IDLE"
        
        # Obstacle avoidance parameters
        self.obstacle_distance_threshold = 5.0
        self.avoidance_strength = 2.0
        self.max_velocity = 5.0
        
        # Attack parameters
        self.attack_distance = 2.0
        self.is_attacking = False
        
        # MAVROS connection
        # self.mavros_ns = f'/drone_{drone_id}/mavros'
        self.mavros_ns = f'/drone_{drone_id}'

        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Twist, f'{self.mavros_ns}/setpoint_velocity/cmd_vel_unstamped', 10)
        self.position_pub = self.create_publisher(
            PoseStamped, f'/drone_{drone_id}/position', 10)
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, f'/drone_{drone_id}/waypoint',
            self.waypoint_callback, 10)
        
        self.command_sub = self.create_subscription(
            String, f'/drone_{drone_id}/command',
            self.command_callback, 10)
        
        self.target_sub = self.create_subscription(
            Point, '/swarm/target',
            self.target_callback, 10)
        
        self.state_sub = self.create_subscription(
            State, f'{self.mavros_ns}/state',
            self.state_callback, 10)
        
        self.position_sub = self.create_subscription(
            PoseStamped, f'{self.mavros_ns}/local_position/pose',
            self.position_callback, 10)
        
        # Lidar/obstacle detection (simulated)
        self.laser_sub = self.create_subscription(
            LaserScan, f'/drone_{drone_id}/scan',
            self.laser_callback, 10)
        
        # Services
        self.arming_client = self.create_client(
            CommandBool, f'{self.mavros_ns}/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, f'{self.mavros_ns}/set_mode')
        self.takeoff_client = self.create_client(
            CommandTOL, f'{self.mavros_ns}/cmd/takeoff')
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        # Obstacle data
        self.obstacles = []
        self.mavros_state = None
        
        self.get_logger().info(f"Drone {drone_id} controller initialized")
    
    def waypoint_callback(self, msg):
        """Receive new waypoint from swarm coordinator"""
        self.target_waypoint.x = msg.pose.position.x
        self.target_waypoint.y = msg.pose.position.y
        self.target_waypoint.z = msg.pose.position.z
    
    def command_callback(self, msg):
        """Receive commands from swarm coordinator"""
        command = msg.data
        
        if command == "ARM_AND_TAKEOFF":
            self.arm_and_takeoff()
        elif command == "ATTACK_MODE":
            self.is_attacking = True
            self.current_state = "ATTACKING"
        elif command == "SEARCH_MODE":
            self.current_state = "SEARCHING"
        elif command == "RTL":
            self.return_to_launch()
    
    def target_callback(self, msg):
        """Receive target position"""
        self.target_position = msg
    
    def state_callback(self, msg):
        """MAVROS state callback"""
        self.mavros_state = msg
    
    def position_callback(self, msg):
        """Update current position"""
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z
        
        # Publish position for swarm coordinator
        self.position_pub.publish(msg)
    
    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.obstacles = []
        
        for i, distance in enumerate(msg.ranges):
            if distance < self.obstacle_distance_threshold and distance > 0.1:
                angle = msg.angle_min + i * msg.angle_increment
                
                # Convert to cartesian coordinates relative to drone
                obstacle_x = distance * math.cos(angle)
                obstacle_y = distance * math.sin(angle)
                
                # Convert to global coordinates
                global_x = self.current_position.x + obstacle_x
                global_y = self.current_position.y + obstacle_y
                
                self.obstacles.append((global_x, global_y, distance))
    
    def arm_and_takeoff(self):
        """Arm drone and takeoff"""
        if not self.mavros_state:
            return
        
        # Set GUIDED mode
        mode_req = SetMode.Request()
        mode_req.custom_mode = "GUIDED"
        self.set_mode_client.call_async(mode_req)
        
        # Arm
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arming_client.call_async(arm_req)
        
        # Takeoff
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = 5.0
        self.takeoff_client.call_async(takeoff_req)
        
        self.current_state = "ARMED"
        self.get_logger().info(f"Drone {self.drone_id} armed and taking off")
    
    def calculate_repulsive_force(self):
        """Calculate repulsive force from obstacles"""
        force_x = 0.0
        force_y = 0.0
        
        for obs_x, obs_y, distance in self.obstacles:
            if distance > 0:
                # Direction from obstacle to drone
                dx = self.current_position.x - obs_x
                dy = self.current_position.y - obs_y
                
                # Normalize
                magnitude = math.sqrt(dx*dx + dy*dy)
                if magnitude > 0:
                    dx /= magnitude
                    dy /= magnitude
                    
                    # Repulsive force inversely proportional to distance
                    force_magnitude = self.avoidance_strength / (distance * distance)
                    force_x += dx * force_magnitude
                    force_y += dy * force_magnitude
        
        return force_x, force_y
    
    def calculate_attractive_force(self, target):
        """Calculate attractive force towards target"""
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        dz = target.z - self.current_position.z
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance > 0:
            # Normalize and scale
            attraction_strength = min(2.0, distance * 0.5)
            return (dx/distance * attraction_strength, 
                   dy/distance * attraction_strength,
                   dz/distance * attraction_strength)
        
        return 0.0, 0.0, 0.0
    
    def calculate_swarm_separation(self):
        """Calculate separation force from other drones (simplified)"""
        # This would require position sharing between drones
        # For now, return zero - implement inter-drone communication for full effect
        return 0.0, 0.0
    
    def control_loop(self):
        """Main control loop"""
        if not self.mavros_state or not self.mavros_state.armed:
            return
        
        # Calculate forces
        repulsive_x, repulsive_y = self.calculate_repulsive_force()
        attractive_x, attractive_y, attractive_z = self.calculate_attractive_force(
            self.target_waypoint)
        separation_x, separation_y = self.calculate_swarm_separation()
        
        # Combine forces
        total_x = attractive_x + repulsive_x + separation_x
        total_y = attractive_y + repulsive_y + separation_y
        total_z = attractive_z
        
        # Apply velocity limits
        velocity_magnitude = math.sqrt(total_x*total_x + total_y*total_y + total_z*total_z)
        if velocity_magnitude > self.max_velocity:
            scale = self.max_velocity / velocity_magnitude
            total_x *= scale
            total_y *= scale
            total_z *= scale
        
        # Attack behavior
        if self.is_attacking and self.target_position:
            distance_to_target = math.sqrt(
                (self.target_position.x - self.current_position.x)**2 +
                (self.target_position.y - self.current_position.y)**2 +
                (self.target_position.z - self.current_position.z)**2
            )
            
            if distance_to_target < self.attack_distance:
                # Execute attack (this could trigger payload release, etc.)
                self.execute_attack()
        
        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = total_x
        cmd_vel.linear.y = total_y
        cmd_vel.linear.z = total_z
        
        self.velocity_pub.publish(cmd_vel)
    
    def execute_attack(self):
        """Execute attack on target"""
        self.get_logger().info(f"Drone {self.drone_id} executing attack!")
        # Here you would implement the actual attack logic
        # This could involve:
        # - Payload release
        # - Precision positioning
        # - Confirmation of hit
        # - RTL command
        
        self.is_attacking = False
        self.current_state = "ATTACK_COMPLETE"
    
    def return_to_launch(self):
        """Return to launch position"""
        mode_req = SetMode.Request()
        mode_req.custom_mode = "RTL"
        self.set_mode_client.call_async(mode_req)

def main(args=None):
    rclpy.init(args=args)
    
    # Get drone ID from command line argument
    import sys
    if len(sys.argv) < 2:
        print("Usage: drone_controller.py <drone_id>")
        return
    
    drone_id = int(sys.argv[1])
    controller = DroneController(drone_id)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()