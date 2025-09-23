#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import SetBool
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
import serial
import time
import threading
import re


class PipetteDriverNode(Node):
    """
    ROS2 node for controlling pipette hardware using standard interfaces.
    Provides FollowJointTrajectory action server and SetBool service for LED.
    """

    def __init__(self):
        super().__init__('pipette_driver_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUR')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('plunger_max_m', 1.0)    # Percentage range [0, 1)
        self.declare_parameter('tip_max_m', 1.0)        # Percentage range [0, 1)
        self.declare_parameter('use_fake_hardware', False)  # Enable fake hardware for testing without Arduino
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.plunger_max_m = self.get_parameter('plunger_max_m').value
        self.tip_max_m = self.get_parameter('tip_max_m').value
        self.use_fake_hardware = self.get_parameter('use_fake_hardware').value
        
        # Serial connection
        self.serial_connection = None
        self.running = False
        self.response_thread = None
        
        # Current state (percentage values [0, 1))
        self.plunger_position_pct = 0.0
        self.tip_position_pct = 0.0
        self.led_on = False
        
        # Joint limits (percentage range [0, 1)) - now configurable via parameters
        self.PLUNGER_MAX = self.plunger_max_m
        self.TIP_MAX = self.tip_max_m
        
        # Joint names for trajectory
        self.joint_names = ['plunger_joint', 'tip_eject_joint']
        
        # Create action server for trajectory following
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.follow_joint_trajectory_callback
        )
        self.get_logger().info('FollowJointTrajectory action server ready')
        
        # Create service for LED control
        self.led_service = self.create_service(
            SetBool,
            'set_led',
            self.set_led_callback
        )
        
        # Create color subscription for LED color control
        self.color_subscription = self.create_subscription(
            ColorRGBA,
            'set_color',
            self.set_color_callback,
            10
        )
        
        # Create joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Timer for periodic joint state publishing (10Hz)
        self.joint_state_timer = self.create_timer(0.1, self._publish_joint_states)
        
        # Initialize hardware connection
        if self.use_fake_hardware:
            self.get_logger().info('Pipette driver started with FAKE HARDWARE - no Arduino needed')
            self.init_fake_hardware()
        else:
            self.get_logger().info(f'Pipette driver node started on {self.serial_port}')
            self.init_hardware()

    def init_hardware(self):
        """Initialize serial connection to Arduino"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if not self.serial_connection.is_open:
                self.get_logger().error(f"Failed to open serial port {self.serial_port}")
                return False
                
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baudrate} baud")
            
            # Start response reading thread
            self.running = True
            self.response_thread = threading.Thread(target=self._read_responses, daemon=True)
            self.response_thread.start()
            
            # Initialize Arduino
            time.sleep(2)  # Give Arduino time to initialize
            self._send_command("INIT")
            
            # Read initial positions
            self._update_position_feedback()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize hardware: {e}")
            return False

    def init_fake_hardware(self):
        """Initialize fake hardware for testing without Arduino"""
        try:
            self.get_logger().info("Fake hardware initialized successfully - ready for testing")
            
            # No serial connection needed in fake hardware mode
            self.serial_connection = None
            
            # Start fake hardware response system
            self.running = True
            
            # Initialize fake hardware positions
            self._update_position_feedback()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize fake hardware: {e}")
            return False

    def follow_joint_trajectory_callback(self, goal_handle):
        """Handle FollowJointTrajectory action requests"""
        self.get_logger().info('Received joint trajectory goal')
        
        trajectory = goal_handle.request.trajectory
        
        # Validate joint names
        if len(trajectory.joint_names) != 2:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "Expected 2 joints: plunger_joint, tip_eject_joint"
            return result
        
        # Execute trajectory points
        goal_handle.succeed()
        
        for point in trajectory.points:
            if len(point.positions) != 2:
                continue
                
            plunger_pos = point.positions[0]
            tip_pos = point.positions[1]
            
            # Validate positions (percentage range [0, 1))
            if not (0.0 <= plunger_pos < self.PLUNGER_MAX):
                self.get_logger().warn(f"Plunger position {plunger_pos:.3f} out of range [0, 1)")
                continue
                
            if not (0.0 <= tip_pos < self.TIP_MAX):
                self.get_logger().warn(f"Tip position {tip_pos:.3f} out of range [0, 1)")
                continue
            
            # Send combined command to Arduino (direct percentage values)
            if self.use_fake_hardware:
                # In fake hardware mode, update positions immediately
                self.plunger_position_pct = plunger_pos
                self.tip_position_pct = tip_pos
                self.get_logger().debug(f"Fake HW: Set positions to plunger={plunger_pos:.3f}, tip={tip_pos:.3f}")
            else:
                self._send_command(f"SETPOSITION {plunger_pos:.3f} {tip_pos:.3f}")
            
            # Wait for movement to complete
            time.sleep(1.0)  # Simple wait - could be improved with feedback
            
            # Send feedback
            feedback = FollowJointTrajectory.Feedback()
            self._update_position_feedback()
            feedback.actual.positions = [
                self.plunger_position_pct,
                self.tip_position_pct
            ]
            goal_handle.publish_feedback(feedback)
        
        # Return result
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory completed successfully"
        
        self.get_logger().info('Joint trajectory completed successfully')
        return result

    def set_led_callback(self, request, response):
        """Handle SetBool service requests for LED control"""
        self.get_logger().info(f'Received LED request: {request.data}')
        
        # Send LED command to Arduino
        if self.use_fake_hardware:
            # In fake hardware mode, just simulate LED control
            self.led_on = request.data
            success = True
            self.get_logger().debug(f"Fake HW: LED set to {'ON' if request.data else 'OFF'}")
        else:
            cmd = "SETCOLOR 255 255 255" if request.data else "SETCOLOR 00 00 00"
            success = self._send_command(cmd)
        
        if success:
            self.led_on = request.data
            
        response.success = success
        response.message = f"LED {'on' if request.data else 'off'}" if success else "LED command failed"
        
        self.get_logger().info(f'LED service completed: success={response.success}')
        return response

    def set_color_callback(self, msg):
        """Handle ColorRGBA topic messages for LED color control"""
        # Convert float values (0.0-1.0) to int values (0-255)
        r = int(msg.r * 255)
        g = int(msg.g * 255) 
        b = int(msg.b * 255)
        
        # Clamp values to valid range
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        
        self.get_logger().info(f'Received color command: R={r}, G={g}, B={b}')
        
        # Send SETCOLOR command to Arduino
        if self.use_fake_hardware:
            # In fake hardware mode, just simulate color control
            success = True
            self.get_logger().debug(f"Fake HW: Color set to R={r}, G={g}, B={b}")
        else:
            cmd = f"SETCOLOR {r} {g} {b}"
            success = self._send_command(cmd)
        
        if success:
            self.get_logger().info(f'Color set successfully')
        else:
            self.get_logger().error(f'Failed to set color')

    def _send_command(self, command: str) -> bool:
        """Send command to Arduino"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error("Serial connection not available")
            return False
            
        try:
            # Add newline for Arduino line parsing
            if not command.endswith('\n'):
                command += '\n'
                
            # Convert to bytes and send
            cmd_bytes = command.encode('utf-8')
            bytes_written = self.serial_connection.write(cmd_bytes)
            self.serial_connection.flush()
            
            self.get_logger().debug(f"Sent command: {command.strip()}")
            return bytes_written == len(cmd_bytes)
            
        except Exception as e:
            self.get_logger().error(f"Error sending command '{command.strip()}': {e}")
            return False

    def _read_responses(self):
        """Continuously read responses from Arduino"""
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode('utf-8').strip()
                    if response:
                        self._process_response(response)
            except Exception:
                # Silent exit when main thread closes serial port
                break
            time.sleep(0.01)

    def _process_response(self, response: str):
        """Process responses from Arduino to extract position feedback"""
        self.get_logger().debug(f"Arduino response: {response}")
        
        try:
            # Look for position information in responses
            # Supports multiple formats:
            # - STATUS: PLUNGER=0.500, TIP=0.300
            # - POS:0.500,0.300
            # - PLUNGER=0.500 TIP=0.300
            
            # Try new compact format first: POS:plunger,tip
            pos_match = re.search(r'POS:([0-9]*\.?[0-9]+),([0-9]*\.?[0-9]+)', response)
            if pos_match:
                plunger_pos = float(pos_match.group(1))
                tip_pos = float(pos_match.group(2))
                
                # Validate range
                if 0.0 <= plunger_pos < self.PLUNGER_MAX:
                    self.plunger_position_pct = plunger_pos
                if 0.0 <= tip_pos < self.TIP_MAX:
                    self.tip_position_pct = tip_pos
                    
                self.get_logger().debug(f"Parsed positions: plunger={plunger_pos:.3f}, tip={tip_pos:.3f}")
                return
            
            # Fall back to individual parsing (legacy format)
            plunger_match = re.search(r'PLUNGER=([0-9]*\.?[0-9]+)', response)
            if plunger_match:
                plunger_pos = float(plunger_match.group(1))
                if 0.0 <= plunger_pos < self.PLUNGER_MAX:
                    self.plunger_position_pct = plunger_pos
                    
            tip_match = re.search(r'TIP=([0-9]*\.?[0-9]+)', response)
            if tip_match:
                tip_pos = float(tip_match.group(1))
                if 0.0 <= tip_pos < self.TIP_MAX:
                    self.tip_position_pct = tip_pos
                    
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse Arduino response '{response}': {e}")

    def _update_position_feedback(self):
        """Request position status from Arduino"""
        if self.use_fake_hardware:
            # In fake hardware mode, positions are already updated when commands are sent
            self.get_logger().debug(f"Fake HW: Current positions - plunger={self.plunger_position_pct:.3f}, tip={self.tip_position_pct:.3f}")
        else:
            self._send_command("STATUS")
            time.sleep(0.05)  # Small delay to allow response

    def _publish_joint_states(self):
        """Publish current joint states for RViz/MoveIt feedback"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        
        # Joint names must match URDF
        msg.name = ['plunger_joint', 'tip_eject_joint']
        
        # Current positions (from Arduino feedback or mock values)
        msg.position = [
            float(self.plunger_position_pct),
            float(self.tip_position_pct)
        ]
        
        # Velocities (could be enhanced with actual velocity feedback later)
        msg.velocity = [0.0, 0.0]
        
        # Efforts (not used for pipette)
        msg.effort = [0.0, 0.0]
        
        self.joint_state_publisher.publish(msg)
        
        # Debug logging for joint state publishing
        self.get_logger().debug(
            f"Publishing joint states: plunger={self.plunger_position_pct:.3f}, "
            f"tip={self.tip_position_pct:.3f}"
        )

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down pipette driver node")
        
        self.running = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        if self.response_thread:
            self.response_thread.join(timeout=1.0)
            
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = PipetteDriverNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            executor.shutdown()
            
    except Exception as e:
        print(f"Failed to start pipette driver node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()