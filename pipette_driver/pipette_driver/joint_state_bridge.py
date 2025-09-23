#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class JointStateBridge(Node):
    """
    Bridge node that converts joint_states from GUI sliders 
    to FollowJointTrajectory actions for the pipette driver.
    """

    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # Track previous positions to avoid spamming
        self.last_positions = {}
        self.last_command_time = 0
        self.command_timeout = 0.1  # Minimum time between commands (100ms)
        
        # Expected joint names
        self.expected_joints = ['plunger_joint', 'tip_eject_joint']
        
        # Action client to send trajectory commands
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory'
        )
        
        self.get_logger().info('Joint State Bridge starting - waiting for action server...')
        
        # Wait for action server to be available before subscribing to joint states
        self._wait_for_action_server()
        
        # Subscribe to joint states from GUI only after action server is ready
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Joint State Bridge ready - GUI sliders will control hardware')

    def _wait_for_action_server(self):
        """Wait for the FollowJointTrajectory action server to become available"""
        max_attempts = 30  # 30 seconds max wait
        attempt = 0
        
        while not self.trajectory_client.wait_for_server(timeout_sec=1.0):
            attempt += 1
            if attempt >= max_attempts:
                self.get_logger().error(
                    'Timeout waiting for FollowJointTrajectory action server after 30 seconds'
                )
                raise RuntimeError('Action server not available')
            
            self.get_logger().info(f'Waiting for action server... (attempt {attempt}/30)')
        
        self.get_logger().info('Action server found and ready')

    def joint_state_callback(self, msg):
        """Process joint state messages from GUI and send to hardware"""
        
        # Check if we have the expected joints
        if not all(joint in msg.name for joint in self.expected_joints):
            return
            
        # Extract positions for our joints
        positions = {}
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.expected_joints:
                positions[joint_name] = msg.position[i]
        
        # Check if positions changed significantly (avoid spam)
        if self._positions_changed(positions):
            current_time = time.time()
            if current_time - self.last_command_time > self.command_timeout:
                self._send_trajectory_goal(positions)
                self.last_command_time = current_time
                self.last_positions = positions.copy()

    def _positions_changed(self, new_positions):
        """Check if positions changed significantly from last command"""
        threshold = 0.0005  # 0.5mm threshold
        
        for joint_name in self.expected_joints:
            if joint_name not in new_positions:
                return False
                
            old_pos = self.last_positions.get(joint_name, 0.0)
            new_pos = new_positions[joint_name]
            
            if abs(new_pos - old_pos) > threshold:
                return True
                
        return False

    def _send_trajectory_goal(self, positions):
        """Send trajectory goal to pipette driver"""
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Trajectory action server not available')
            return
            
        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.expected_joints
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [
            positions.get('plunger_joint', 0.0),
            positions.get('tip_eject_joint', 0.0)
        ]
        point.time_from_start = Duration(sec=1, nanosec=0)  # 1 second to reach
        
        goal.trajectory.points = [point]
        
        # Send goal (fire and forget - don't wait for result)
        future = self.trajectory_client.send_goal_async(goal)
        
        self.get_logger().debug(
            f'Sent trajectory: plunger={point.positions[0]:.4f}m, '
            f'tip={point.positions[1]:.4f}m'
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = JointStateBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()