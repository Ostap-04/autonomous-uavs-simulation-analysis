import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import sys
import threading
import math
import time

from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, PositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class DroneControllerNode(Node):
    def __init__(self, drone_id: int, target_altitude: float = 5.0):
        super().__init__(f'drone_{drone_id}_controller_node')

        self.drone_id = drone_id
        self.ns = f"/drone_{drone_id}"
        self.altitude = target_altitude
        self.gps_fix_received = False
        self.current_state = "IDLE"  # IDLE, ARMED, TAKEOFF, FORMATION, ATTACKING
        self.current_position = None
        self.target_position = None
        self.formation_position = None

        # QoS profiles
        self.qos = QoSProfile(depth=10)
        self.qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Service clients
        self.set_mode_client = self.create_client(SetMode, f"{self.ns}/set_mode")
        self.arming_client = self.create_client(CommandBool, f"{self.ns}/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, f"{self.ns}/cmd/takeoff")

        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped, 
            f"{self.ns}/setpoint_position/local", 
            self.qos
        )
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            f"{self.ns}/setpoint_velocity/cmd_vel",
            self.qos
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f"{self.ns}/global_position/global",
            self.gps_callback,
            self.qos
        )
        
        self.position_sub = self.create_subscription(
            PoseStamped,
            f"{self.ns}/local_position/pose",
            self.position_callback,
            self.qos
        )

        self.state_sub = self.create_subscription(
            State,
            f"{self.ns}/state",
            self.state_callback,
            self.qos
        )

        # Command subscriber (global for all drones)
        self.command_sub = self.create_subscription(
            String,
            "/drone_command",
            self.command_callback,
            self.qos
        )

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"[drone_{self.drone_id}] 🤖 Controller initialized, waiting for commands...")

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0 and not self.gps_fix_received:
            self.gps_fix_received = True
            self.get_logger().info(f"[drone_{self.drone_id}] ✅ GPS fix acquired.")

    def position_callback(self, msg: PoseStamped):
        self.current_position = msg

    def state_callback(self, msg: State):
        pass  # Can be used to monitor drone state

    def command_callback(self, msg: String):
        command = msg.data.strip().upper()
        self.get_logger().info(f"[drone_{self.drone_id}] 📢 Received command: {command}")
        
        if command == "ARM_TAKEOFF":
            self.handle_arm_takeoff()
        elif command.startswith("FORM_FIGURE"):
            self.handle_form_figure(command)
        elif command.startswith("ATTACK_TARGET"):
            self.handle_attack_target(command)
        elif command == "LAND":
            self.handle_land()
        elif command == "RTL":
            self.handle_rtl()

    def handle_arm_takeoff(self):
        if not self.gps_fix_received:
            self.get_logger().warn(f"[drone_{self.drone_id}] ⚠️ No GPS fix, cannot takeoff")
            return
            
        if self.current_state != "IDLE":
            self.get_logger().warn(f"[drone_{self.drone_id}] ⚠️ Already in state: {self.current_state}")
            return

        self.current_state = "ARMING"
        self.get_logger().info(f"[drone_{self.drone_id}] ⏳ Starting ARM_TAKEOFF sequence")
        
        # Start takeoff sequence in a separate thread
        threading.Thread(target=self.execute_takeoff_sequence, daemon=True).start()

    def handle_form_figure(self, command):
        if self.current_state not in ["TAKEOFF", "FORMATION"]:
            self.get_logger().warn(f"[drone_{self.drone_id}] ⚠️ Must be airborne to form figure")
            return

        # Parse figure type and parameters
        parts = command.split()
        if len(parts) < 2:
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Invalid FORM_FIGURE command")
            return

        figure_type = parts[1]
        self.current_state = "FORMATION"
        
        if figure_type == "CIRCLE":
            radius = float(parts[2]) if len(parts) > 2 else 10.0
            self.form_circle_formation(radius)
        elif figure_type == "LINE":
            spacing = float(parts[2]) if len(parts) > 2 else 5.0
            self.form_line_formation(spacing)
        elif figure_type == "TRIANGLE":
            size = float(parts[2]) if len(parts) > 2 else 10.0
            self.form_triangle_formation(size)

    def handle_attack_target(self, command):
        if self.current_state not in ["TAKEOFF", "FORMATION"]:
            self.get_logger().warn(f"[drone_{self.drone_id}] ⚠️ Must be airborne to attack target")
            return

        # Parse target coordinates
        parts = command.split()
        if len(parts) < 4:  # ATTACK_TARGET X Y Z
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Invalid ATTACK_TARGET command")
            return

        try:
            target_x = float(parts[1])
            target_y = float(parts[2])
            target_z = float(parts[3])
            
            self.current_state = "ATTACKING"
            self.attack_target(target_x, target_y, target_z)
            
        except ValueError:
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Invalid target coordinates")

    def handle_land(self):
        self.current_state = "LANDING"
        self.set_mode("LAND")

    def handle_rtl(self):
        self.current_state = "RTL"
        self.set_mode("RTL")

    def execute_takeoff_sequence(self):
        try:
            # Set mode to GUIDED
            if not self.set_mode("GUIDED"):
                return

            time.sleep(1)

            # Arm the drone
            if not self.arm_drone():
                return

            time.sleep(2)

            # Send takeoff command
            if not self.send_takeoff():
                return

            self.current_state = "TAKEOFF"
            self.get_logger().info(f"[drone_{self.drone_id}] 🚀 Takeoff sequence complete!")

        except Exception as e:
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Takeoff sequence failed: {e}")
            self.current_state = "IDLE"

    def set_mode(self, mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Set mode service not available")
            return False

        mode_req = SetMode.Request(base_mode=0, custom_mode=mode)
        future = self.set_mode_client.call_async(mode_req)
        
        # Don't block the main thread - check periodically
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 10.0:
            time.sleep(0.1)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"[drone_{self.drone_id}] ✅ Mode set to {mode}")
            return True
        else:
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Failed to set mode to {mode}")
            return False

    def arm_drone(self):
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Arming service not available")
            return False

        arm_req = CommandBool.Request(value=True)
        future = self.arming_client.call_async(arm_req)
        
        # Don't block the main thread - check periodically
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 10.0:
            time.sleep(0.1)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"[drone_{self.drone_id}] ✅ Armed successfully")
            return True
        else:
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Failed to arm")
            return False

    def send_takeoff(self):
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Takeoff service not available")
            return False

        takeoff_req = CommandTOL.Request(
            min_pitch=0.0,
            yaw=0.0,
            latitude=0.0,
            longitude=0.0,
            altitude=self.altitude
        )
        future = self.takeoff_client.call_async(takeoff_req)
        
        # Don't block the main thread - check periodically
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 10.0:
            time.sleep(0.1)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"[drone_{self.drone_id}] ✅ Takeoff command sent")
            return True
        else:
            self.get_logger().error(f"[drone_{self.drone_id}] ❌ Failed to send takeoff command")
            return False

    def form_circle_formation(self, radius):
        # Calculate position in circle based on drone_id
        angle = (self.drone_id * 2 * math.pi) / 8  # Assuming max 8 drones
        
        self.formation_position = PoseStamped()
        self.formation_position.header.frame_id = "base_link"
        self.formation_position.pose.position.x = radius * math.cos(angle)
        self.formation_position.pose.position.y = radius * math.sin(angle)
        self.formation_position.pose.position.z = self.altitude
        
        self.get_logger().info(f"[drone_{self.drone_id}] 🔵 Moving to circle formation position")

    def form_line_formation(self, spacing):
        # Form a line with drones spaced apart
        self.formation_position = PoseStamped()
        self.formation_position.header.frame_id = "base_link"
        self.formation_position.pose.position.x = 0.0
        self.formation_position.pose.position.y = (self.drone_id - 1) * spacing
        self.formation_position.pose.position.z = self.altitude
        
        self.get_logger().info(f"[drone_{self.drone_id}] 📏 Moving to line formation position")

    def form_triangle_formation(self, size):
        # Form triangle formation (supports up to 3 drones effectively)
        positions = [
            (0, 0),  # Top
            (-size/2, -size * 0.866),  # Bottom left
            (size/2, -size * 0.866),   # Bottom right
        ]
        
        if self.drone_id <= len(positions):
            pos = positions[(self.drone_id - 1) % len(positions)]
            self.formation_position = PoseStamped()
            self.formation_position.header.frame_id = "base_link"
            self.formation_position.pose.position.x = pos[0]
            self.formation_position.pose.position.y = pos[1]
            self.formation_position.pose.position.z = self.altitude
            
            self.get_logger().info(f"[drone_{self.drone_id}] 🔺 Moving to triangle formation position")

    def attack_target(self, target_x, target_y, target_z):
        # Create attack pattern - approach target in coordinated manner
        self.target_position = PoseStamped()
        self.target_position.header.frame_id = "base_link"
        
        # Each drone approaches from slightly different angle
        angle_offset = (self.drone_id - 1) * (math.pi / 4)  # 45 degree offset per drone
        approach_distance = 2.0  # Stay 2m from target
        
        self.target_position.pose.position.x = target_x + approach_distance * math.cos(angle_offset)
        self.target_position.pose.position.y = target_y + approach_distance * math.sin(angle_offset)
        self.target_position.pose.position.z = target_z
        
        self.get_logger().info(f"[drone_{self.drone_id}] ⚔️ Attacking target at ({target_x}, {target_y}, {target_z})")

    def control_loop(self):
        # Publish setpoints based on current state
        if self.current_state == "FORMATION" and self.formation_position:
            self.formation_position.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.formation_position)
            
        elif self.current_state == "ATTACKING" and self.target_position:
            self.target_position.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_position)


class CommandInterface:
    """Simple command line interface for sending commands to drones"""
    def __init__(self, node):
        self.node = node
        self.command_pub = node.create_publisher(String, "/drone_command", 10)
        self.running = True
        
    def run(self):
        print("\n🎮 DRONE COMMAND INTERFACE")
        print("Available commands:")
        print("  ARM_TAKEOFF                    - Arm and takeoff all drones")
        print("  FORM_FIGURE CIRCLE [radius]   - Form circle formation")
        print("  FORM_FIGURE LINE [spacing]    - Form line formation") 
        print("  FORM_FIGURE TRIANGLE [size]   - Form triangle formation")
        print("  ATTACK_TARGET x y z           - Attack target at coordinates")
        print("  LAND                          - Land all drones")
        print("  RTL                           - Return to launch")
        print("  quit                          - Exit\n")
        
        import sys
        
        while self.running and rclpy.ok():
            try:
                # Use sys.stdout.write and sys.stdout.flush for better control
                sys.stdout.write("Enter command: ")
                sys.stdout.flush()
                
                # Read input with proper handling
                command = input().strip()
                
                if command.lower() == 'quit':
                    self.running = False
                    break
                    
                if command:
                    msg = String()
                    msg.data = command
                    self.command_pub.publish(msg)
                    print(f"📤 Sent command: {command}")
                    # Small delay to ensure message is sent
                    time.sleep(0.1)
                else:
                    print("⚠️ Empty command, please try again")
                    
            except KeyboardInterrupt:
                print("\n🛑 Interrupted by user")
                self.running = False
                break
            except EOFError:
                print("\n🛑 Input ended")
                self.running = False
                break
            except Exception as e:
                print(f"❌ Input error: {e}")
                continue


def main(args=None):
    if args is None:
        args = sys.argv

    if len(args) < 2:
        print("Usage: ros2 run <package> drone_controller <drone_id_1> [<drone_id_2> ...] [--alt <altitude>]")
        return

    if "--alt" in args:
        alt_index = args.index("--alt")
        altitude = float(args[alt_index + 1])
        drone_ids = [int(arg) for arg in args[1:alt_index]]
    else:
        altitude = 5.0
        drone_ids = [int(arg) for arg in args[1:]]

    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    nodes = []

    # Create drone controller nodes
    for drone_id in drone_ids:
        node = DroneControllerNode(drone_id, altitude)
        nodes.append(node)
        executor.add_node(node)

    # Create command interface
    command_node = Node('command_interface')
    executor.add_node(command_node)
    cmd_interface = CommandInterface(command_node)

    # Start executor in separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # Give some time for nodes to initialize
        time.sleep(1)
        # Run command interface in main thread
        cmd_interface.run()
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        # Cleanup
        cmd_interface.running = False
        executor.shutdown()
        
        for node in nodes:
            try:
                node.destroy_node()
            except:
                pass
        try:
            command_node.destroy_node()
        except:
            pass
            
        rclpy.shutdown()
        print("✅ Drone controller shutdown complete.")


if __name__ == '__main__':
    main()