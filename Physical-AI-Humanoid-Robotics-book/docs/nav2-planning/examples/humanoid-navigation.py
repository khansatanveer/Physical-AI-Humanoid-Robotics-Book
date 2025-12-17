#!/usr/bin/env python3
"""
Humanoid Navigation Example with Nav2
Demonstrates navigation with humanoid-specific constraints and behaviors
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


class HumanoidNavigationNode(Node):
    """
    Node to demonstrate humanoid navigation with Nav2
    """

    def __init__(self):
        super().__init__('humanoid_navigation_node')

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.footstep_pub = self.create_publisher(MarkerArray, 'footsteps', 10)
        self.path_pub = self.create_publisher(Marker, 'navigation_path', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Navigation state
        self.current_pose = None
        self.current_goal = None
        self.navigation_active = False
        self.footsteps = []
        self.navigation_path = []

        # Humanoid-specific parameters
        self.step_size = 0.3  # Maximum distance between steps
        self.foot_separation = 0.2  # Distance between feet for stability
        self.max_step_height = 0.1  # Maximum obstacle height to step over
        self.stance_time = 0.1  # Time to maintain stable stance
        self.balance_threshold = 0.05  # Threshold for balance maintenance

        # Safety parameters
        self.collision_threshold = 0.3  # Minimum distance to obstacles
        self.max_navigation_attempts = 3  # Number of retry attempts

        self.get_logger().info("Humanoid Navigation Node initialized")

    def scan_callback(self, msg):
        """
        Callback for laser scan messages
        """
        # Process laser scan for obstacle detection
        self.detect_obstacles(msg)

    def detect_obstacles(self, scan_msg):
        """
        Detect obstacles from laser scan data
        """
        obstacles = []
        angle = scan_msg.angle_min

        for range_val in scan_msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)) and range_val < 2.0:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                obstacles.append((x, y))
            angle += scan_msg.angle_increment

        # Check if any obstacles are in the navigation path
        if self.navigation_path:
            for point in self.navigation_path:
                for obs_x, obs_y in obstacles:
                    dist = math.sqrt(
                        (point[0] - obs_x) ** 2 + (point[1] - obs_y) ** 2
                    )
                    if dist < self.collision_threshold:
                        self.get_logger().warn("Obstacle detected in navigation path!")

    def set_initial_pose(self, x, y, theta=0.0):
        """
        Set the initial pose of the humanoid robot
        """
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = R.from_euler('z', theta).as_quat()
        initial_pose.pose.pose.orientation.x = quat[0]
        initial_pose.pose.pose.orientation.y = quat[1]
        initial_pose.pose.pose.orientation.z = quat[2]
        initial_pose.pose.pose.orientation.w = quat[3]

        # Add small covariance values
        initial_pose.pose.covariance[0] = 0.25  # x
        initial_pose.pose.covariance[7] = 0.25  # y
        initial_pose.pose.covariance[35] = 0.0685  # theta

        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info(f"Initial pose set to: ({x}, {y}, {theta})")

    def send_navigation_goal(self, x, y, theta=0.0):
        """
        Send a navigation goal to Nav2
        """
        # Wait for the action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return False

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = R.from_euler('z', theta).as_quat()
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Send the goal
        self.current_goal = goal_msg.pose.pose
        self.navigation_active = True

        self.get_logger().info(f"Sending navigation goal: ({x}, {y}, {theta})")

        # Send async goal
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        future.add_done_callback(self.navigation_result_callback)
        return True

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback
        """
        feedback = feedback_msg.feedback
        # In a real implementation, this would handle feedback from Nav2
        self.get_logger().debug("Received navigation feedback")

    def navigation_result_callback(self, future):
        """
        Callback for navigation result
        """
        try:
            goal_handle = future.result()
            if goal_handle is not None and goal_handle.status == 4:  # SUCCEEDED
                self.get_logger().info("Navigation succeeded!")
                self.navigation_active = False
            else:
                self.get_logger().info("Navigation failed or was canceled")
                self.navigation_active = False
        except Exception as e:
            self.get_logger().error(f"Exception in navigation result callback: {e}")
            self.navigation_active = False

    def plan_footsteps(self, start_pose, goal_pose):
        """
        Plan footsteps for humanoid navigation from start to goal
        """
        footsteps = []

        # Calculate the path as a series of waypoints
        start_pos = np.array([start_pose.position.x, start_pose.position.y])
        goal_pos = np.array([goal_pose.position.x, goal_pose.position.y])

        # Calculate direction vector
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            # Goal is very close, just take one step
            footsteps.append(goal_pos)
            return footsteps

        # Normalize direction
        direction = direction / distance

        # Calculate number of steps needed
        num_steps = int(distance / self.step_size)

        # Generate footsteps
        for i in range(1, num_steps + 1):
            step_pos = start_pos + direction * (i * self.step_size)
            footsteps.append(step_pos)

        # Add the final goal position
        if np.linalg.norm(footsteps[-1] - goal_pos) > 0.05:  # If goal not already included
            footsteps.append(goal_pos)

        return footsteps

    def visualize_footsteps(self, footsteps):
        """
        Publish footsteps as visualization markers
        """
        if not footsteps:
            return

        marker_array = MarkerArray()
        for i, step in enumerate(footsteps):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'footsteps'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = step[0]
            marker.pose.position.y = step[1]
            marker.pose.position.z = 0.01  # Slightly above ground
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1  # Foot size
            marker.scale.y = 0.05  # Foot thickness
            marker.scale.z = 0.02  # Height

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.footstep_pub.publish(marker_array)

    def check_balance_feasibility(self, pose1, pose2):
        """
        Check if moving from pose1 to pose2 maintains humanoid balance
        """
        # Calculate distance between poses
        dist = math.sqrt(
            (pose2[0] - pose1[0]) ** 2 + (pose2[1] - pose1[1]) ** 2
        )

        # Check if the step is within feasible limits
        if dist > self.step_size:
            return False, f"Step too large: {dist:.2f} > {self.step_size:.2f}"

        # Check if the step maintains balance
        # This is a simplified check - in reality, balance depends on many factors
        return True, "Step is feasible"

    def execute_humanoid_navigation(self, start_x, start_y, goal_x, goal_y):
        """
        Execute complete humanoid navigation from start to goal
        """
        self.get_logger().info(f"Starting humanoid navigation from ({start_x}, {start_y}) to ({goal_x}, {goal_y})")

        # Set initial pose
        self.set_initial_pose(start_x, start_y)

        # Plan footsteps
        start_pose = PoseStamped()
        start_pose.pose.position.x = start_x
        start_pose.pose.position.y = start_y

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y

        footsteps = self.plan_footsteps(start_pose.pose, goal_pose.pose)
        self.get_logger().info(f"Planned {len(footsteps)} footsteps")

        # Validate footsteps for balance
        valid_footsteps = []
        for i, step in enumerate(footsteps):
            if i == 0:
                # First step, use start position
                is_valid, reason = self.check_balance_feasibility(
                    np.array([start_x, start_y]), step
                )
            else:
                # Check from previous step
                is_valid, reason = self.check_balance_feasibility(
                    footsteps[i-1], step
                )

            if is_valid:
                valid_footsteps.append(step)
                self.get_logger().debug(f"Step {i} is valid: {reason}")
            else:
                self.get_logger().warn(f"Step {i} is invalid: {reason}")

        # Visualize valid footsteps
        self.visualize_footsteps(valid_footsteps)

        # Send navigation goal if there are valid footsteps
        if valid_footsteps:
            final_goal = valid_footsteps[-1]
            success = self.send_navigation_goal(final_goal[0], final_goal[1])
            return success
        else:
            self.get_logger().error("No valid footsteps found for navigation")
            return False

    def run_navigation_demo(self):
        """
        Run a demonstration of humanoid navigation
        """
        self.get_logger().info("Starting humanoid navigation demonstration...")

        # Example navigation: move from (0, 0) to (5, 5)
        success = self.execute_humanoid_navigation(0.0, 0.0, 5.0, 5.0)

        if success:
            self.get_logger().info("Navigation demonstration started successfully")
        else:
            self.get_logger().error("Failed to start navigation demonstration")


def main(args=None):
    """
    Main function to run the humanoid navigation example
    """
    rclpy.init(args=args)

    nav_node = HumanoidNavigationNode()

    try:
        # Run the navigation demonstration
        nav_node.run_navigation_demo()

        # Keep the node running
        rclpy.spin(nav_node)

    except KeyboardInterrupt:
        nav_node.get_logger().info("Navigation node interrupted by user")
    except Exception as e:
        nav_node.get_logger().error(f"Error in navigation node: {e}")
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()