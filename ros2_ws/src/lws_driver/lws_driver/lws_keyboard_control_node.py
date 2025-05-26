#!/usr/bin/env python3
"""
Keyboard control for turret joints in Gazebo Harmonic
Controls:
- Q/A: Rotate turret base left/right
- W/S: Tilt turret head up/down
- ESC: Exit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

class TurretKeyboardControl(Node):
    def __init__(self):
        super().__init__('turret_keyboard_control')
        
        # Publishers for joint position commands
        self.turret_base_pub = self.create_publisher(
            Float64, 
            '/turret_base_position_cmd', 
            10
        )
        self.turret_head_pub = self.create_publisher(
            Float64, 
            '/turret_head_position_cmd', 
            10
        )
        
        # Current joint positions (in radians)
        self.turret_base_position = 0.0
        self.turret_head_position = 0.0
        
        # Joint limits (in radians)
        self.turret_base_min = -40 * 3.14159 / 180.0  # -40 degrees
        self.turret_base_max = 40 * 3.14159 / 180.0   # 40 degrees
        self.turret_head_min = -45 * 3.14159 / 180.0  # -45 degrees
        self.turret_head_max = 0.0                     # 0 degrees
        
        # Movement step size (in radians)
        self.step_size = 0.1  # ~5.7 degrees
        
        # Store original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Turret Keyboard Control Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  A/Q: Rotate turret base left/right')
        self.get_logger().info('  S/W: Tilt turret head up/down')
        self.get_logger().info('  ESC: Exit')
        self.get_logger().info('Press keys to control the turret...')
        
    def get_key(self):
        """Get a single keypress without pressing Enter"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def publish_positions(self):
        """Publish current joint positions"""
        base_msg = Float64()
        base_msg.data = self.turret_base_position
        self.turret_base_pub.publish(base_msg)
        
        head_msg = Float64()
        head_msg.data = self.turret_head_position
        self.turret_head_pub.publish(head_msg)
        
    def clamp_value(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))
        
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC key
                    break
                elif key.lower() == 'a':
                    # Rotate turret base left
                    self.turret_base_position += self.step_size
                    self.turret_base_position = self.clamp_value(
                        self.turret_base_position, 
                        self.turret_base_min, 
                        self.turret_base_max
                    )
                    self.get_logger().info(f'Turret base: {self.turret_base_position:.2f} rad')
                    
                elif key.lower() == 'q':
                    # Rotate turret base right
                    self.turret_base_position -= self.step_size
                    self.turret_base_position = self.clamp_value(
                        self.turret_base_position, 
                        self.turret_base_min, 
                        self.turret_base_max
                    )
                    self.get_logger().info(f'Turret base: {self.turret_base_position:.2f} rad')
                    
                elif key.lower() == 's':
                    # Tilt turret head up
                    self.turret_head_position += self.step_size
                    self.turret_head_position = self.clamp_value(
                        self.turret_head_position, 
                        self.turret_head_min, 
                        self.turret_head_max
                    )
                    self.get_logger().info(f'Turret head: {self.turret_head_position:.2f} rad')
                    
                elif key.lower() == 'w':
                    # Tilt turret head down
                    self.turret_head_position -= self.step_size
                    self.turret_head_position = self.clamp_value(
                        self.turret_head_position, 
                        self.turret_head_min, 
                        self.turret_head_max
                    )
                    self.get_logger().info(f'Turret head: {self.turret_head_position:.2f} rad')
                
                # Publish the updated positions
                self.publish_positions()
                
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
def main(args=None):
    rclpy.init(args=args)
    
    controller = TurretKeyboardControl()
    
    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()