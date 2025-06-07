#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Int32
import numpy as np
import math
from enum import Enum
import threading
import time

class SwarmState(Enum):
    INITIALIZING = 1
    FORMATION = 2
    SEARCHING = 3
    TARGET_ACQUIRED = 4
    ATTACKING = 5
    RTL = 6

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        
        # Swarm parameters
        # self.num_drones = 10
        self.num_drones = 1
        self.formation_radius = 15.0
        self.attack_radius = 5.0
        self.safe_distance = 3.0
        
        # State management
        self.swarm_state = SwarmState.INITIALIZING
        self.target_position = None
        self.formation_center = Point()
        self.formation_center.x = 0.0
        self.formation_center.y = 0.0
        self.formation_center.z = 10.0
        
        # Drone tracking
        self.drone_positions = {}
        self.drone_states = {}
        self.formation_positions = {}
        
        # Publishers for each drone
        self.waypoint_publishers = {}
        self.command_publishers = {}
        
        # Subscribers for drone feedback
        self.position_subscribers = {}
        
        # Global command publisher
        self.swarm_command_pub = self.create_publisher(String, '/swarm/command', 10)
        self.target_pub = self.create_publisher(Point, '/swarm/target', 10)
        
        # Target selection subscriber
        self.target_sub = self.create_subscription(
            Point, '/target_selection', self.target_callback, 10)
        
        # Initialize drone communication
        self.setup_drone_communication()
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Generate formation positions
        self.generate_formation_positions()
        
        self.get_logger().info("Swarm Coordinator initialized")
    
    def setup_drone_communication(self):
        """Setup publishers and subscribers for each drone"""
        for i in range(self.num_drones):
            # Waypoint publishers
            topic_waypoint = f'/drone_{i}/waypoint'
            self.waypoint_publishers[i] = self.create_publisher(
                PoseStamped, topic_waypoint, 10)
            
            # Command publishers
            topic_command = f'/drone_{i}/command'
            self.command_publishers[i] = self.create_publisher(
                String, topic_command, 10)
            
            # Position subscribers
            topic_position = f'/drone_{i}/position'
            self.position_subscribers[i] = self.create_subscription(
                PoseStamped, topic_position, 
                lambda msg, drone_id=i: self.position_callback(msg, drone_id), 10)
            
            # Initialize tracking
            self.drone_positions[i] = Point()
            self.drone_states[i] = "IDLE"
    
    def generate_formation_positions(self):
        """Generate circular formation positions around center"""
        for i in range(self.num_drones):
            angle = 2 * math.pi * i / self.num_drones
            self.formation_positions[i] = Point()
            self.formation_positions[i].x = self.formation_center.x + \
                self.formation_radius * math.cos(angle)
            self.formation_positions[i].y = self.formation_center.y + \
                self.formation_radius * math.sin(angle)
            self.formation_positions[i].z = self.formation_center.z + \
                np.random.uniform(-2, 2)  # Add height variation
    
    def position_callback(self, msg, drone_id):
        """Update drone position"""
        self.drone_positions[drone_id].x = msg.pose.position.x
        self.drone_positions[drone_id].y = msg.pose.position.y
        self.drone_positions[drone_id].z = msg.pose.position.z
    
    def target_callback(self, msg):
        """Receive target position from target selection node"""
        self.target_position = msg
        self.swarm_state = SwarmState.TARGET_ACQUIRED
        self.get_logger().info(f"Target acquired at: {msg.x}, {msg.y}, {msg.z}")
        
        # Publish target to all drones
        self.target_pub.publish(msg)
    
    def send_waypoint_to_drone(self, drone_id, position):
        """Send waypoint to specific drone"""
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = "map"
        waypoint.pose.position = position
        
        # Set orientation (facing target if available)
        if self.target_position:
            dx = self.target_position.x - position.x
            dy = self.target_position.y - position.y
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0
        
        waypoint.pose.orientation.z = math.sin(yaw / 2)
        waypoint.pose.orientation.w = math.cos(yaw / 2)
        
        self.waypoint_publishers[drone_id].publish(waypoint)
    
    def send_command_to_drone(self, drone_id, command):
        """Send command to specific drone"""
        cmd_msg = String()
        cmd_msg.data = command
        self.command_publishers[drone_id].publish(cmd_msg)
    
    def calculate_attack_positions(self):
        """Calculate attack positions around target"""
        if not self.target_position:
            return {}
        
        attack_positions = {}
        for i in range(self.num_drones):
            angle = 2 * math.pi * i / self.num_drones
            attack_positions[i] = Point()
            attack_positions[i].x = self.target_position.x + \
                self.attack_radius * math.cos(angle)
            attack_positions[i].y = self.target_position.y + \
                self.attack_radius * math.sin(angle)
            attack_positions[i].z = self.target_position.z + 3.0
        
        return attack_positions
    
    def check_formation_complete(self):
        """Check if all drones are in formation"""
        for i in range(self.num_drones):
            target_pos = self.formation_positions[i]
            current_pos = self.drone_positions[i]
            
            distance = math.sqrt(
                (target_pos.x - current_pos.x)**2 +
                (target_pos.y - current_pos.y)**2 +
                (target_pos.z - current_pos.z)**2
            )
            
            if distance > 2.0:  # 2m tolerance
                return False
        return True
    
    def control_loop(self):
        """Main control loop for swarm coordination"""
        if self.swarm_state == SwarmState.INITIALIZING:
            # Send formation commands
            for i in range(self.num_drones):
                self.send_waypoint_to_drone(i, self.formation_positions[i])
                self.send_command_to_drone(i, "ARM_AND_TAKEOFF")
            
            self.swarm_state = SwarmState.FORMATION
            self.get_logger().info("Swarm moving to formation")
        
        elif self.swarm_state == SwarmState.FORMATION:
            if self.check_formation_complete():
                self.swarm_state = SwarmState.SEARCHING
                self.get_logger().info("Formation complete, searching for target")
                
                # Broadcast search command
                cmd_msg = String()
                cmd_msg.data = "SEARCH_MODE"
                self.swarm_command_pub.publish(cmd_msg)
        
        elif self.swarm_state == SwarmState.TARGET_ACQUIRED:
            # Calculate attack positions
            attack_positions = self.calculate_attack_positions()
            
            # Send attack waypoints
            for i in range(self.num_drones):
                self.send_waypoint_to_drone(i, attack_positions[i])
                self.send_command_to_drone(i, "ATTACK_MODE")
            
            self.swarm_state = SwarmState.ATTACKING
            self.get_logger().info("Coordinated attack initiated")
        
        elif self.swarm_state == SwarmState.ATTACKING:
            # Monitor attack progress
            # Could implement attack completion logic here
            pass

def main(args=None):
    rclpy.init(args=args)
    swarm_coordinator = SwarmCoordinator()
    
    try:
        rclpy.spin(swarm_coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        swarm_coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()