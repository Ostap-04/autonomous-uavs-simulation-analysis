#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class TargetSelector(Node):
    def __init__(self):
        super().__init__('target_selector')
        
        # Target detection parameters
        self.target_detected = False
        self.target_position = Point()
        self.detection_confidence = 0.0
        self.min_confidence = 0.7
        
        # Computer vision
        self.bridge = CvBridge()
        
        # Publishers
        self.target_pub = self.create_publisher(Point, '/target_selection', 10)
        self.status_pub = self.create_publisher(String, '/target_status', 10)
        
        # Subscribers - can receive from multiple sources
        self.manual_target_sub = self.create_subscription(
            Point, '/manual_target', self.manual_target_callback, 10)
        
        # Image-based target detection (from drone cameras)
        self.image_subscribers = {}
        for i in range(10):  # 10 drones
            topic = f'/drone_{i}/camera/image_raw'
            self.image_subscribers[i] = self.create_subscription(
                Image, topic, 
                lambda msg, drone_id=i: self.image_callback(msg, drone_id), 10)
        
        # Timer for target search coordination
        self.search_timer = self.create_timer(1.0, self.search_coordination)
        
        # Target search patterns
        self.search_areas = self.generate_search_areas()
        self.current_search_area = 0
        
        self.get_logger().info("Target Selector initialized")
    
    def generate_search_areas(self):
        """Generate search areas for systematic target search"""
        areas = []
        # Define search grid
        x_range = range(-50, 51, 20)  # -50 to 50 in steps of 20
        y_range = range(-50, 51, 20)
        
        for x in x_range:
            for y in y_range:
                area = {
                    'center': Point(x=float(x), y=float(y), z=15.0),
                    'radius': 15.0,
                    'searched': False
                }
                areas.append(area)
        
        return areas
    
    def manual_target_callback(self, msg):
        """Receive manually specified target"""
        self.target_position = msg
        self.target_detected = True
        self.detection_confidence = 1.0
        
        # Publish target immediately
        self.target_pub.publish(msg)
        
        # Update status
        status = String()
        status.data = f"MANUAL_TARGET_SET:{msg.x},{msg.y},{msg.z}"
        self.status_pub.publish(status)
        
        self.get_logger().info(f"Manual target set: {msg.x}, {msg.y}, {msg.z}")
    
    def image_callback(self, msg, drone_id):
        """Process images from drone cameras for target detection"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Target detection using color/shape detection
            target_detected, confidence, position = self.detect_target_in_image(
                cv_image, drone_id)
            
            if target_detected and confidence > self.min_confidence:
                if not self.target_detected or confidence > self.detection_confidence:
                    self.target_position = position
                    self.target_detected = True
                    self.detection_confidence = confidence
                    
                    # Publish new target
                    self.target_pub.publish(position)
                    
                    # Update status
                    status = String()
                    status.data = f"TARGET_DETECTED:DRONE_{drone_id}:{confidence:.2f}"
                    self.status_pub.publish(status)
                    
                    self.get_logger().info(
                        f"Target detected by drone {drone_id} at {position.x}, {position.y}")
        
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
    
    def detect_target_in_image(self, image, drone_id):
        """Detect target in image using computer vision"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define red color range (assuming target is red)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, 0.0, Point()
        
        # Find largest contour (assumed to be target)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Minimum area threshold
        if area < 500:  # Adjust based on expected target size
            return False, 0.0, Point()
        
        # Calculate confidence based on area and shape
        confidence = min(1.0, area / 5000.0)  # Normalize area to confidence
        
        # Get center of contour
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return False, 0.0, Point()
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Convert image coordinates to world coordinates
        # This requires camera calibration and drone position
        # For now, use simplified conversion
        world_position = self.image_to_world_coordinates(cx, cy, drone_id)
        
        return True, confidence, world_position
    
    def image_to_world_coordinates(self, img_x, img_y, drone_id):
        """Convert image coordinates to world coordinates"""
        # This is a simplified conversion
        # In practice, you'd need:
        # - Camera intrinsic parameters
        # - Drone position and orientation
        # - Altitude information for ground plane projection
        
        # Placeholder implementation
        # Assume camera is looking down and convert relative to drone position
        position = Point()
        
        # Simple offset based on image center
        img_center_x = 320  # Assuming 640x480 image
        img_center_y = 240
        
        offset_x = (img_x - img_center_x) * 0.01  # Scale factor
        offset_y = (img_y - img_center_y) * 0.01
        
        # Add to estimated drone position (would get from drone state)
        # For now, use search area center
        search_area = self.search_areas[self.current_search_area % len(self.search_areas)]
        position.x = search_area['center'].x + offset_x
        position.y = search_area['center'].y + offset_y
        position.z = 2.0  # Ground level
        
        return position
    
    def search_coordination(self):
        """Coordinate search pattern if no target detected"""
        if self.target_detected:
            return
        
        # Publish current search area for swarm to investigate
        current_area = self.search_areas[self.current_search_area % len(self.search_areas)]
        
        if not current_area['searched']:
            # Publish search area as temporary target
            search_point = current_area['center']
            
            status = String()
            status.data = f"SEARCHING_AREA:{search_point.x},{search_point.y}"
            self.status_pub.publish(status)
            
            # Mark as being searched
            current_area['searched'] = True
            
            self.get_logger().info(
                f"Searching area {self.current_search_area}: {search_point.x}, {search_point.y}")
        
        # Move to next search area
        self.current_search_area += 1
        
        # Reset search if all areas covered
        if self.current_search_area >= len(self.search_areas):
            self.current_search_area = 0
            # Reset all areas
            for area in self.search_areas:
                area['searched'] = False
            
            self.get_logger().info("Search pattern reset - starting new sweep")

def main(args=None):
    rclpy.init(args=args)
    target_selector = TargetSelector()
    
    try:
        rclpy.spin(target_selector)
    except KeyboardInterrupt:
        pass
    finally:
        target_selector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()