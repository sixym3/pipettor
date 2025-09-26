#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from pipette_driver.action import PipettorOperation
from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import SetBool
import serial
import time
import threading
import re


class PipetteDriverNode(Node):
    """
    ROS2 node for controlling pipette hardware using standard interfaces.
    Provides FollowJointTrajectory action server for pipette control.
    """

    def __init__(self):
        super().__init__('pipette_driver_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/tmp/ttyUR')
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
        
        # Joint limits (percentage range [0, 1))
        self.PLUNGER_MAX = self.plunger_max_m
        self.TIP_MAX = self.tip_max_m

        # Track command acknowledgments and completion with sequence numbers
        self.command_sequence = 0
        self.pending_command = None
        self.pending_sequence = None
        self.command_acked = False
        self.command_done = False
        
        # Joint names for trajectory
        self.joint_names = ['plunger_joint', 'tip_eject_joint']
        
        # Create action server for trajectory following (legacy/internal interface)
        self.trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.follow_joint_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('FollowJointTrajectory action server ready')

        # Create high-level pipettor operation action server
        self.pipettor_server = ActionServer(
            self,
            PipettorOperation,
            'pipettor_operation',
            self.pipettor_operation_callback,
            goal_callback=self.pipettor_goal_callback,
            cancel_callback=self.pipettor_cancel_callback
        )
        self.get_logger().info('PipettorOperation action server ready')

        # LED control interfaces
        self.led_service = self.create_service(
            SetBool,
            'led_control',
            self.led_control_callback
        )
        self.led_color_subscriber = self.create_subscription(
            ColorRGBA,
            'led_color',
            self.led_color_callback,
            10
        )
        self.get_logger().info('LED control interfaces ready')

        # Current LED state
        self.led_enabled = False
        self.led_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # Default white
        
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
            
            # Send status command to check connection
            self._send_command("STATUS")
            
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
            
            # No position feedback in fake hardware mode either
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize fake hardware: {e}")
            return False

    def goal_callback(self, goal_request):
        """Handle goal requests - accept all valid goals"""
        self.get_logger().info(f'Received goal request with {len(goal_request.trajectory.points)} points')

        # Basic validation
        if len(goal_request.trajectory.joint_names) != 2:
            self.get_logger().warn('Rejecting goal: Expected 2 joints')
            return GoalResponse.REJECT

        expected_joints = ['plunger_joint', 'tip_eject_joint']
        if goal_request.trajectory.joint_names != expected_joints:
            self.get_logger().warn(f'Rejecting goal: Expected joints {expected_joints}')
            return GoalResponse.REJECT

        self.get_logger().info('Accepting goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancel requests - accept all cancellations"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def follow_joint_trajectory_callback(self, goal_handle):
        """Handle FollowJointTrajectory action requests"""
        import time
        current_time = time.time()
        self.get_logger().info(f'Received joint trajectory goal at {current_time} (goal_id: {goal_handle.goal_id})')

        trajectory = goal_handle.request.trajectory

        # Validate joint names
        if len(trajectory.joint_names) != 2:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "Expected 2 joints: plunger_joint, tip_eject_joint"
            return result

        # Validate expected joint names
        expected_joints = ['plunger_joint', 'tip_eject_joint']
        if trajectory.joint_names != expected_joints:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"Expected joints {expected_joints}, got {trajectory.joint_names}"
            return result

        # Execute trajectory points with proper cancellation checking
        self.get_logger().info(f'Executing trajectory with {len(trajectory.points)} points for goal {goal_handle.goal_id}')

        for i, point in enumerate(trajectory.points):
            # Check for cancellation before each point
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trajectory execution canceled')
                # Send emergency stop to Arduino
                if not self.use_fake_hardware:
                    self._send_command("STOP")
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Trajectory canceled by user"
                return result

            if len(point.positions) != 2:
                self.get_logger().warn(f"Point {i} has {len(point.positions)} positions, expected 2")
                continue

            plunger_pos = point.positions[0]
            tip_pos = point.positions[1]

            # Validate positions (percentage range [0, 1))
            if not (0.0 <= plunger_pos <= self.PLUNGER_MAX):
                self.get_logger().warn(f"Point {i}: Plunger position {plunger_pos:.3f} out of range [0, {self.PLUNGER_MAX}]")
                continue

            if not (0.0 <= tip_pos <= self.TIP_MAX):
                self.get_logger().warn(f"Point {i}: Tip position {tip_pos:.3f} out of range [0, {self.TIP_MAX}]")
                continue

            self.get_logger().info(f'Executing point {i+1}/{len(trajectory.points)}: plunger={plunger_pos:.3f}, tip={tip_pos:.3f}')

            # Send feedback when starting each point
            start_feedback = FollowJointTrajectory.Feedback()
            start_feedback.header.stamp = self.get_clock().now().to_msg()
            start_feedback.joint_names = ['plunger_joint', 'tip_eject_joint']
            start_feedback.desired.positions = [plunger_pos, tip_pos]
            start_feedback.desired.time_from_start = point.time_from_start
            # Note: actual positions not available - no feedback from linear actuators
            goal_handle.publish_feedback(start_feedback)

            # Send combined command to Arduino with enhanced feedback
            if self.use_fake_hardware:
                success = True
                self.get_logger().info(f"Fake HW: Processing point {i+1} - plunger={plunger_pos:.3f}, tip={tip_pos:.3f}")
            else:
                # Convert to percentages and use new protocol with sequence number
                plunger_pct = int(plunger_pos * 100)
                tip_pct = int(tip_pos * 100)

                # Increment sequence number and generate command with sequence
                self.command_sequence = (self.command_sequence + 1) % 1000  # Roll over after 999
                command_with_seq = f"S{plunger_pct:03d}{tip_pct:03d}_{self.command_sequence:03d}"

                # Reset command tracking for new sequence
                self.pending_command = f"S{plunger_pct:03d}{tip_pct:03d}"
                self.pending_sequence = self.command_sequence
                self.command_acked = False
                self.command_done = False

                success = self._send_command(command_with_seq)
                self.get_logger().info(f"Sent command {command_with_seq} (sequence {self.command_sequence})")

            if not success:
                self.get_logger().error(f"Failed to send command for point {i}: {command_with_seq if not self.use_fake_hardware else 'fake command'}")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"Failed to execute point {i}: Command transmission failed"
                return result

            # Non-blocking wait with cancellation checking
            if not self._wait_for_movement_with_cancellation(goal_handle, point.time_from_start):
                # Cancellation occurred during wait
                if not self.use_fake_hardware:
                    self._send_command("STOP")
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Trajectory canceled during execution"
                return result

            # Send completion feedback for this point
            completion_feedback = FollowJointTrajectory.Feedback()
            completion_feedback.header.stamp = self.get_clock().now().to_msg()
            completion_feedback.joint_names = ['plunger_joint', 'tip_eject_joint']
            completion_feedback.desired.positions = [plunger_pos, tip_pos]
            completion_feedback.desired.time_from_start = point.time_from_start
            # Note: We assume movement is complete after waiting the specified time
            # but have no actual position feedback to confirm this
            goal_handle.publish_feedback(completion_feedback)

        # Trajectory completed successfully
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory completed successfully"

        self.get_logger().info(f'Joint trajectory completed successfully for goal {goal_handle.goal_id}')
        return result

    def _wait_for_movement_with_cancellation(self, goal_handle, target_time):
        """
        Wait for Arduino DONE signal or timeout with cancellation checking.
        Returns False if canceled, True if completed.
        """
        import rclpy

        # Convert target_time to seconds
        if hasattr(target_time, 'sec') and hasattr(target_time, 'nanosec'):
            target_seconds = target_time.sec + target_time.nanosec * 1e-9
        else:
            target_seconds = 1.0  # Default fallback

        # Use minimum 1 second, maximum 10 seconds for safety
        timeout = max(1.0, min(10.0, target_seconds))

        # Check for cancellation and DONE signal every 0.1 seconds
        check_interval = 0.1
        elapsed_time = 0.0

        if self.use_fake_hardware:
            # In fake hardware mode, just wait for time
            while elapsed_time < timeout:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f'Cancellation requested during movement wait (elapsed: {elapsed_time:.2f}s)')
                    return False
                time.sleep(check_interval)
                elapsed_time += check_interval
                # Removed rclpy.spin_once() - causes deadlock in action callback
            return True
        else:
            # Real hardware: wait for DONE signal or timeout
            # Note: Cannot call spin_once() from action callback - would cause deadlock
            # Arduino responses are processed by background thread, so command_done will update
            while elapsed_time < timeout:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f'Cancellation requested during movement wait (elapsed: {elapsed_time:.2f}s)')
                    return False

                # Check if Arduino sent DONE signal for current command
                if self.command_done:
                    self.get_logger().info(f'Movement completed via DONE signal for sequence {self.pending_sequence} (elapsed: {elapsed_time:.2f}s)')
                    return True

                time.sleep(check_interval)
                elapsed_time += check_interval
                # Removed rclpy.spin_once() - causes deadlock in action callback

            # Timeout occurred - log warning but continue
            self.get_logger().warn(f'Movement timeout after {timeout:.1f}s for sequence {self.pending_sequence}, assuming complete')
            return True

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
        """Process responses from Arduino (no position feedback available)"""
        self.get_logger().debug(f"Arduino response: {response}")

        # Handle command acknowledgments and completion signals with sequence tracking
        if response.startswith("ACK "):
            full_command = response[4:].strip()
            # Parse command with sequence: S055030_001
            if "_" in full_command:
                try:
                    command_part, seq_part = full_command.rsplit("_", 1)
                    sequence = int(seq_part)

                    if command_part == self.pending_command and sequence == self.pending_sequence:
                        self.command_acked = True
                        self.get_logger().debug(f"Command acknowledged: {full_command}")
                    else:
                        self.get_logger().warn(f"Received ACK for wrong command/sequence: {full_command}, expected: {self.pending_command}_{self.pending_sequence:03d}")
                except (ValueError, AttributeError):
                    self.get_logger().warn(f"Invalid ACK format: {full_command}")
            else:
                # Legacy format without sequence - still support for backwards compatibility
                if full_command == self.pending_command:
                    self.command_acked = True
                    self.get_logger().debug(f"Command acknowledged (legacy): {full_command}")

        elif response.startswith("DONE "):
            full_command = response[5:].strip()
            # Parse command with sequence: S055030_001
            if "_" in full_command:
                try:
                    command_part, seq_part = full_command.rsplit("_", 1)
                    sequence = int(seq_part)

                    if command_part == self.pending_command and sequence == self.pending_sequence:
                        self.command_done = True
                        self.get_logger().debug(f"Command completed: {full_command}")
                    else:
                        self.get_logger().warn(f"Received DONE for wrong command/sequence: {full_command}, expected: {self.pending_command}_{self.pending_sequence:03d}")
                except (ValueError, AttributeError):
                    self.get_logger().warn(f"Invalid DONE format: {full_command}")
            else:
                # Legacy format without sequence - still support for backwards compatibility
                if full_command == self.pending_command:
                    self.command_done = True
                    self.get_logger().debug(f"Command completed (legacy): {full_command}")

    # PipettorOperation Action Server Methods
    def pipettor_goal_callback(self, goal_request):
        """Handle PipettorOperation goal requests"""
        operation = goal_request.operation.upper()
        valid_operations = ["SUCK", "EXPEL", "EJECT_TIP", "SET_LED"]

        if operation not in valid_operations:
            self.get_logger().warn(f'Rejecting goal: Invalid operation "{operation}". Valid: {valid_operations}')
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepting PipettorOperation goal: {operation}')
        return GoalResponse.ACCEPT

    def pipettor_cancel_callback(self, goal_handle):
        """Handle PipettorOperation cancel requests"""
        self.get_logger().info('Received cancel request for PipettorOperation')
        return CancelResponse.ACCEPT

    def pipettor_operation_callback(self, goal_handle):
        """Handle PipettorOperation action requests"""
        operation = goal_handle.request.operation.upper()
        volume_pct = goal_handle.request.volume_pct
        led_color = goal_handle.request.led_color

        self.get_logger().info(f'Executing PipettorOperation: {operation}')

        # Initialize result
        result = PipettorOperation.Result()
        result.final_plunger_position = 0.0
        result.final_tip_position = 0.0

        try:
            if operation == "SUCK":
                success = self._execute_suck_sequence(goal_handle)
                result.final_plunger_position = 0.0  # Always returns to 0 after suck
                result.final_tip_position = 0.0

            elif operation == "EXPEL":
                success = self._execute_expel_sequence(goal_handle)
                result.final_plunger_position = 0.0  # Always returns to 0 after expel
                result.final_tip_position = 0.0

            elif operation == "EJECT_TIP":
                success = self._execute_eject_sequence(goal_handle)
                result.final_plunger_position = 0.0
                result.final_tip_position = 0.0  # Always returns to 0 after eject

            elif operation == "SET_LED":
                success = self._execute_led_operation(led_color)
                result.final_plunger_position = 0.0  # LED operation doesn't affect actuators
                result.final_tip_position = 0.0

            else:
                success = False
                result.message = f"Unknown operation: {operation}"

            if success:
                goal_handle.succeed()
                result.success = True
                result.message = f"{operation} operation completed successfully"
                self.get_logger().info(f'PipettorOperation {operation} completed successfully')
            else:
                goal_handle.abort()
                result.success = False
                if not result.message:
                    result.message = f"Failed to execute {operation} operation"
                self.get_logger().error(f'PipettorOperation {operation} failed: {result.message}')

        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f"Error during {operation}: {str(e)}"
            self.get_logger().error(f'PipettorOperation {operation} error: {e}')

        return result

    def _execute_suck_sequence(self, goal_handle):
        """Execute hardcoded suck sequence: 0→55→0 over 5 seconds"""
        sequence_points = [
            (0.0, 0.0, 0.0),     # Start position (plunger, tip, time)
            (0.55, 0.0, 2.5),    # Extend to suction position at 2.5 seconds
            (0.0, 0.0, 5.0)      # Return to start at 5 seconds
        ]

        return self._execute_sequence(goal_handle, sequence_points, "SUCK",
                                     ["EXTENDING", "RETRACTING", "COMPLETE"])

    def _execute_expel_sequence(self, goal_handle):
        """Execute hardcoded expel sequence: 0→77→0 over 5 seconds"""
        sequence_points = [
            (0.0, 0.0, 0.0),     # Start position (plunger, tip, time)
            (0.77, 0.0, 2.5),    # Extend to expel position at 2.5 seconds
            (0.0, 0.0, 5.0)      # Return to start at 5 seconds
        ]

        return self._execute_sequence(goal_handle, sequence_points, "EXPEL",
                                     ["EXTENDING", "RETRACTING", "COMPLETE"])

    def _execute_eject_sequence(self, goal_handle):
        """Execute hardcoded tip eject sequence: 0→30→0 over 5 seconds"""
        sequence_points = [
            (0.0, 0.0, 0.0),     # Start position (plunger, tip, time)
            (0.0, 0.30, 2.5),    # Extend tip ejector at 2.5 seconds
            (0.0, 0.0, 5.0)      # Return to start at 5 seconds
        ]

        return self._execute_sequence(goal_handle, sequence_points, "EJECT_TIP",
                                     ["EXTENDING", "RETRACTING", "COMPLETE"])

    def _execute_sequence(self, goal_handle, sequence_points, operation_name, phases):
        """Execute a trajectory sequence with feedback"""
        total_points = len(sequence_points)

        for i, (plunger_pos, tip_pos, target_time) in enumerate(sequence_points):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'{operation_name} sequence canceled')
                if not self.use_fake_hardware:
                    self._send_command("STOP")
                goal_handle.canceled()
                return False

            # Determine current phase
            if i < len(phases):
                current_phase = phases[i]
            else:
                current_phase = phases[-1]

            # Calculate progress (0.0 to 1.0)
            progress = (i + 1) / total_points

            # Send feedback
            feedback = PipettorOperation.Feedback()
            feedback.current_phase = current_phase
            feedback.plunger_position = plunger_pos
            feedback.tip_position = tip_pos
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f'{operation_name} point {i+1}/{total_points}: '
                                 f'plunger={plunger_pos:.3f}, tip={tip_pos:.3f}, phase={current_phase}')

            # Execute the trajectory point using internal FollowJointTrajectory
            success = self._execute_trajectory_point(plunger_pos, tip_pos, target_time, goal_handle)
            if not success:
                return False

        return True

    def _execute_trajectory_point(self, plunger_pos, tip_pos, target_time, goal_handle):
        """Execute a single trajectory point using the internal trajectory system"""
        if self.use_fake_hardware:
            # In fake hardware mode, just wait for the target time
            if target_time > 0:
                time.sleep(min(target_time, 2.0))  # Cap at 2 seconds for safety
            return True

        # Real hardware: send command to Arduino
        plunger_pct = int(plunger_pos * 100)
        tip_pct = int(tip_pos * 100)

        # Increment sequence number and generate command
        self.command_sequence = (self.command_sequence + 1) % 1000
        command_with_seq = f"S{plunger_pct:03d}{tip_pct:03d}_{self.command_sequence:03d}"

        # Reset command tracking
        self.pending_command = f"S{plunger_pct:03d}{tip_pct:03d}"
        self.pending_sequence = self.command_sequence
        self.command_acked = False
        self.command_done = False

        success = self._send_command(command_with_seq)
        if not success:
            self.get_logger().error(f"Failed to send command: {command_with_seq}")
            return False

        # Wait for completion with cancellation checking
        return self._wait_for_movement_with_cancellation_simple(goal_handle, target_time)

    def _wait_for_movement_with_cancellation_simple(self, goal_handle, target_time):
        """Simplified wait for movement completion with cancellation checking"""
        timeout = max(1.0, min(10.0, target_time if target_time > 0 else 1.0))
        check_interval = 0.1
        elapsed_time = 0.0

        while elapsed_time < timeout:
            if goal_handle.is_cancel_requested:
                return False

            # Check if Arduino sent DONE signal
            if self.command_done:
                self.get_logger().debug(f'Movement completed via DONE signal (elapsed: {elapsed_time:.2f}s)')
                return True

            time.sleep(check_interval)
            elapsed_time += check_interval

        # Timeout - assume complete
        self.get_logger().warn(f'Movement timeout after {timeout:.1f}s, assuming complete')
        return True

    def _execute_led_operation(self, led_color):
        """Execute LED color change operation"""
        self.led_color = led_color
        self.led_enabled = True

        if self.use_fake_hardware:
            self.get_logger().info(f'Fake HW: Setting LED color to R={led_color.r:.2f}, '
                                  f'G={led_color.g:.2f}, B={led_color.b:.2f}')
            return True

        # Convert RGB to Arduino format (0-255)
        r = int(led_color.r * 255)
        g = int(led_color.g * 255)
        b = int(led_color.b * 255)

        # Send LED command to Arduino
        led_command = f"LED{r:03d}{g:03d}{b:03d}"
        success = self._send_command(led_command)

        if success:
            self.get_logger().info(f'LED color set to R={r}, G={g}, B={b}')
        else:
            self.get_logger().error(f'Failed to set LED color')

        return success

    # LED Service and Topic Callbacks
    def led_control_callback(self, request, response):
        """Handle LED on/off service requests"""
        self.led_enabled = request.data

        if self.use_fake_hardware:
            self.get_logger().info(f'Fake HW: LED {"enabled" if self.led_enabled else "disabled"}')
            response.success = True
            response.message = f'LED {"enabled" if self.led_enabled else "disabled"}'
            return response

        if self.led_enabled:
            # Turn on LED with current color
            success = self._execute_led_operation(self.led_color)
        else:
            # Turn off LED (set to black)
            off_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            success = self._execute_led_operation(off_color)

        response.success = success
        response.message = f'LED {"enabled" if self.led_enabled else "disabled"}' if success else 'LED control failed'
        return response

    def led_color_callback(self, msg):
        """Handle LED color topic messages"""
        self.led_color = msg
        if self.led_enabled:
            # Only update LED if it's currently enabled
            self._execute_led_operation(msg)
            self.get_logger().info(f'Updated LED color: R={msg.r:.2f}, G={msg.g:.2f}, B={msg.b:.2f}')

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