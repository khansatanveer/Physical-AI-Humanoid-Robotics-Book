#!/usr/bin/env python3
"""
AI Agent Bridge Example

This example demonstrates how to connect an AI agent with a ROS 2 system.
The AI agent processes sensor data and makes decisions for robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import random


class AIAgentBridgeNode(Node):
    """
    A bridge node that connects an AI agent with ROS 2 for robot control.
    """

    def __init__(self):
        """
        Initialize the AI agent bridge node.
        """
        super().__init__('ai_agent_bridge')

        # Subscribe to sensor data (laser scan)
        self.sensor_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.sensor_callback,
            10
        )

        # Publish commands to robot
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publish AI status
        self.status_publisher = self.create_publisher(String, 'ai_status', 10)

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.1, self.ai_decision_loop)

        # Store latest sensor data
        self.latest_scan = None

        # AI agent state
        self.ai_state = {
            'position': [0.0, 0.0],
            'orientation': 0.0,
            'obstacles_detected': False,
            'target_reached': False,
            'current_behavior': 'exploring'
        }

        self.get_logger().info('AI Agent Bridge node initialized')

    def sensor_callback(self, msg):
        """
        Callback for handling sensor data from the robot.

        Args:
            msg: LaserScan message containing sensor data
        """
        # Store the latest laser scan
        self.latest_scan = msg
        self.get_logger().debug('Received sensor data')

    def ai_decision_loop(self):
        """
        Main AI decision-making loop.
        Processes sensor data and makes decisions for robot control.
        """
        if self.latest_scan is None:
            return

        # Process sensor data and make AI decisions
        try:
            # Analyze the environment based on laser scan
            environment_analysis = self.analyze_environment(self.latest_scan)

            # Make a decision based on the analysis
            decision = self.make_decision(environment_analysis)

            # Execute the decision
            self.execute_decision(decision)

            # Publish status
            status_msg = String()
            status_msg.data = f"Behavior: {self.ai_state['current_behavior']}, Obstacles: {environment_analysis['obstacle_count']}"
            self.status_publisher.publish(status_msg)

            self.get_logger().info(f'AI Decision: {status_msg.data}')

        except Exception as e:
            self.get_logger().error(f'Error in AI decision loop: {e}')

    def analyze_environment(self, scan_msg):
        """
        Analyze the environment based on laser scan data.

        Args:
            scan_msg: LaserScan message containing sensor data

        Returns:
            Dictionary containing environment analysis
        """
        # Convert ROS LaserScan to usable data
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        # Analyze the scan data
        obstacle_distances = []
        obstacle_angles = []
        obstacle_count = 0

        # Define thresholds
        min_distance_threshold = 1.0  # meters
        max_distance_threshold = 10.0  # meters

        for i, distance in enumerate(ranges):
            if min_distance_threshold <= distance <= max_distance_threshold:
                angle = angle_min + i * angle_increment
                obstacle_distances.append(distance)
                obstacle_angles.append(angle)
                obstacle_count += 1

        # Find closest obstacle
        closest_distance = min(obstacle_distances) if obstacle_distances else float('inf')
        closest_angle = obstacle_angles[obstacle_distances.index(closest_distance)] if obstacle_distances else 0.0

        # Determine if there's a clear path ahead
        front_range_start = len(ranges) // 2 - 10
        front_range_end = len(ranges) // 2 + 10
        front_distances = ranges[front_range_start:front_range_end]
        front_clear = all(d > 1.5 for d in front_distances)  # Clear if all distances > 1.5m

        # Analyze left and right sides
        left_distances = ranges[:len(ranges)//4]
        right_distances = ranges[3*len(ranges)//4:]
        left_clear = all(d > 1.0 for d in left_distances)
        right_clear = all(d > 1.0 for d in right_distances)

        # Return analysis
        return {
            'obstacle_distances': obstacle_distances,
            'obstacle_angles': obstacle_angles,
            'obstacle_count': obstacle_count,
            'closest_distance': closest_distance,
            'closest_angle': closest_angle,
            'front_clear': front_clear,
            'left_clear': left_clear,
            'right_clear': right_clear,
            'scan_range': scan_msg.range_max - scan_msg.range_min
        }

    def make_decision(self, environment_analysis):
        """
        Make a decision based on environment analysis.

        Args:
            environment_analysis: Dictionary containing environment analysis

        Returns:
            Dictionary containing decision parameters
        """
        # Simple AI decision logic based on environment analysis
        decision = {
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'behavior': 'idle'
        }

        # If there's an obstacle very close in front, stop and turn
        if environment_analysis['closest_distance'] < 0.5:
            decision['linear_velocity'] = 0.0
            # Turn away from the closest obstacle
            if environment_analysis['closest_angle'] > 0:
                decision['angular_velocity'] = -0.8  # Turn right
            else:
                decision['angular_velocity'] = 0.8   # Turn left
            decision['behavior'] = 'avoiding_obstacle'

        # If front is clear, move forward
        elif environment_analysis['front_clear']:
            decision['linear_velocity'] = 0.6
            decision['angular_velocity'] = 0.0
            decision['behavior'] = 'moving_forward'

        # If front is blocked but sides are clear, turn toward clearer side
        else:
            if environment_analysis['left_clear'] and not environment_analysis['right_clear']:
                decision['linear_velocity'] = 0.1
                decision['angular_velocity'] = 0.5  # Turn left
                decision['behavior'] = 'turning_left'
            elif environment_analysis['right_clear'] and not environment_analysis['left_clear']:
                decision['linear_velocity'] = 0.1
                decision['angular_velocity'] = -0.5  # Turn right
                decision['behavior'] = 'turning_right'
            else:
                # Both sides blocked, or both clear - turn randomly
                decision['linear_velocity'] = 0.0
                decision['angular_velocity'] = random.choice([-0.5, 0.5])
                decision['behavior'] = 'random_turn'

        # Update AI state based on decision
        self.ai_state['current_behavior'] = decision['behavior']

        return decision

    def execute_decision(self, decision):
        """
        Execute the AI decision by publishing commands.

        Args:
            decision: Dictionary containing decision parameters
        """
        # Create Twist message for robot commands
        cmd_msg = Twist()
        cmd_msg.linear.x = decision['linear_velocity']
        cmd_msg.angular.z = decision['angular_velocity']

        # Publish the command
        self.cmd_publisher.publish(cmd_msg)

        # Log the command if it's significant
        if abs(cmd_msg.linear.x) > 0.01 or abs(cmd_msg.angular.z) > 0.01:
            self.get_logger().debug(
                f'Executing: linear={cmd_msg.linear.x:.2f}, angular={cmd_msg.angular.z:.2f}, '
                f'behavior={decision["behavior"]}'
            )

    def shutdown(self):
        """
        Perform cleanup operations before shutting down.
        """
        # Stop the robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_publisher.publish(stop_msg)
        self.get_logger().info('AI Agent Bridge stopped robot before shutdown')


def main(args=None):
    """
    Main function to initialize and run the AI agent bridge node.

    Args:
        args: Command line arguments (typically None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the AI agent bridge node
    ai_agent_bridge = AIAgentBridgeNode()

    # Keep the node running until interrupted
    try:
        rclpy.spin(ai_agent_bridge)
    except KeyboardInterrupt:
        ai_agent_bridge.get_logger().info('Interrupted, shutting down...')
    finally:
        # Perform cleanup
        ai_agent_bridge.shutdown()

        # Clean up resources
        ai_agent_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()