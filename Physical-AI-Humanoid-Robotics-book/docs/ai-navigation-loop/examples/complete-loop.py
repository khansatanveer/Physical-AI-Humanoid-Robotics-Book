#!/usr/bin/env python3
"""
Complete AI-Driven Navigation Loop Example
Integrates perception, planning, and control systems for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import time
from threading import Lock


class CompleteNavigationLoop(Node):
    """
    Complete navigation loop integrating perception, planning, and control
    """

    def __init__(self):
        super().__init__('complete_navigation_loop')

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.visualization_pub = self.create_publisher(MarkerArray, 'navigation_visualization', 10)

        # Subscribers for perception
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile
        )
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, qos_profile
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, qos_profile
        )

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Navigation state
        self.current_pose = None
        self.current_goal = None
        self.navigation_active = False
        self.obstacle_detected = False
        self.environment_map = {}
        self.navigation_path = []

        # Perception data
        self.latest_image = None
        self.camera_info = None
        self.laser_data = None

        # Control parameters
        self.update_rate = 10.0  # Hz
        self.safety_distance = 0.5  # meters
        self.max_detection_range = 3.0  # meters
        self.goal_tolerance = 0.25  # meters

        # Threading lock for data access
        self.data_lock = Lock()

        # Timer for main navigation loop
        self.navigation_timer = self.create_timer(1.0/self.update_rate, self.navigation_loop)

        self.get_logger().info("Complete Navigation Loop initialized")
        self.get_logger().info("Integration of perception, planning, and control systems ready")

    def scan_callback(self, msg):
        """
        Callback for laser scan data (perception component)
        """
        with self.data_lock:
            self.laser_data = msg
            self.process_laser_data(msg)

    def image_callback(self, msg):
        """
        Callback for camera image data (perception component)
        """
        with self.data_lock:
            try:
                self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                self.process_image_data(self.latest_image)
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

    def camera_info_callback(self, msg):
        """
        Callback for camera information (perception component)
        """
        with self.data_lock:
            self.camera_info = msg

    def process_laser_data(self, scan_msg):
        """
        Process laser scan data for obstacle detection (perception component)
        """
        obstacles = []
        angle = scan_msg.angle_min

        for range_val in scan_msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)) and range_val < self.max_detection_range:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                obstacles.append((x, y))
            angle += scan_msg.angle_increment

        # Update environment map with obstacle information
        with self.data_lock:
            self.environment_map['obstacles'] = obstacles
            self.obstacle_detected = len(obstacles) > 0

    def process_image_data(self, image):
        """
        Process camera image for object detection (perception component)
        """
        # Simple color-based object detection for demonstration
        # In a real system, this would use deep learning models
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color detection (for demonstration)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                detected_objects.append((center_x, center_y, w, h))

        with self.data_lock:
            self.environment_map['detected_objects'] = detected_objects

    def plan_path(self, start_pose, goal_pose):
        """
        Plan a path from start to goal (planning component)
        This is a simplified path planner for demonstration
        """
        if start_pose is None or goal_pose is None:
            return []

        # Calculate straight-line path with intermediate waypoints
        start_pos = np.array([start_pose.position.x, start_pose.position.y])
        goal_pos = np.array([goal_pose.position.x, goal_pose.position.y])

        # Calculate direction vector
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)

        # Create intermediate waypoints
        num_waypoints = max(2, int(distance / 0.5))  # 0.5m between waypoints
        waypoints = []

        for i in range(num_waypoints + 1):
            ratio = i / num_waypoints if num_waypoints > 0 else 0
            waypoint_pos = start_pos + direction * ratio

            waypoint = PoseStamped()
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = float(waypoint_pos[0])
            waypoint.pose.position.y = float(waypoint_pos[1])
            waypoint.pose.position.z = 0.0

            # Keep same orientation as goal for simplicity
            waypoint.pose.orientation = goal_pose.orientation

            waypoints.append(waypoint)

        return waypoints

    def check_path_safety(self, path):
        """
        Check if path is safe given current obstacle information (planning component)
        """
        with self.data_lock:
            obstacles = self.environment_map.get('obstacles', [])

        for waypoint in path:
            for obs_x, obs_y in obstacles:
                dist = math.sqrt(
                    (waypoint.pose.position.x - obs_x) ** 2 +
                    (waypoint.pose.position.y - obs_y) ** 2
                )
                if dist < self.safety_distance:
                    return False, f"Path blocked at ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})"

        return True, "Path is clear"

    def send_navigation_goal(self, x, y, theta=0.0):
        """
        Send navigation goal to Nav2 (planning/control component)
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return False

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

        self.current_goal = goal_msg.pose.pose
        self.navigation_active = True

        self.get_logger().info(f"Sending navigation goal: ({x}, {y}, {theta})")

        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        future.add_done_callback(self.navigation_result_callback)
        return True

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback (control component)
        """
        self.get_logger().debug("Received navigation feedback")
        # In a real implementation, this would handle feedback from Nav2

    def navigation_result_callback(self, future):
        """
        Callback for navigation result (control component)
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

    def control_step(self):
        """
        Execute control actions based on current state (control component)
        """
        if not self.navigation_active:
            return

        # In a real system, this would send velocity commands to the robot
        # For simulation, we'll just log the control actions
        self.get_logger().debug("Executing navigation control commands")

    def navigation_loop(self):
        """
        Main navigation loop that integrates perception, planning, and control
        """
        if self.current_goal is None:
            self.get_logger().info("No goal set, waiting for navigation goal...")
            return

        # Perception: Update environment understanding
        self.get_logger().debug("Perception: Updating environment understanding")

        # Planning: Check if current path is still valid
        self.get_logger().debug("Planning: Checking path validity")

        # Check if we need to replan due to obstacles
        with self.data_lock:
            if self.obstacle_detected and self.current_pose:
                self.get_logger().warn("Obstacle detected, considering replanning...")

                # Create temporary goal near current position to avoid obstacle
                temp_goal = PoseStamped()
                temp_goal.header.frame_id = 'map'
                temp_goal.pose.position.x = self.current_pose.position.x + 0.5
                temp_goal.pose.position.y = self.current_pose.position.y + 0.5
                temp_goal.pose.orientation = self.current_pose.orientation

                # Plan path to temporary goal
                temp_path = self.plan_path(self.current_pose, temp_goal.pose)
                is_safe, reason = self.check_path_safety(temp_path)

                if not is_safe:
                    self.get_logger().warn(f"Path to temporary goal not safe: {reason}")
                    # Stop navigation and wait for clearer path
                    self.navigation_active = False
                else:
                    self.get_logger().info("Found safe path around obstacle")

        # Control: Execute navigation commands
        self.get_logger().debug("Control: Executing navigation commands")
        self.control_step()

    def set_initial_pose(self, x, y, theta=0.0):
        """
        Set the initial pose of the robot
        """
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
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

        # Store current pose
        self.current_pose = initial_pose.pose.pose

    def execute_complete_navigation(self, start_x, start_y, goal_x, goal_y):
        """
        Execute complete navigation from start to goal
        """
        self.get_logger().info(f"Starting complete navigation from ({start_x}, {start_y}) to ({goal_x}, {goal_y})")

        # Set initial pose
        self.set_initial_pose(start_x, start_y)

        # Send navigation goal
        success = self.send_navigation_goal(goal_x, goal_y)

        if success:
            self.get_logger().info("Complete navigation loop started successfully")
            return True
        else:
            self.get_logger().error("Failed to start navigation")
            return False

    def run_demonstration(self):
        """
        Run a demonstration of the complete navigation loop
        """
        self.get_logger().info("Starting complete navigation loop demonstration...")

        # Example navigation: move from (0, 0) to (5, 5)
        success = self.execute_complete_navigation(0.0, 0.0, 5.0, 5.0)

        if success:
            self.get_logger().info("Navigation demonstration started successfully")
        else:
            self.get_logger().error("Failed to start navigation demonstration")


def main(args=None):
    """
    Main function to run the complete navigation loop example
    """
    rclpy.init(args=args)

    nav_loop = CompleteNavigationLoop()

    try:
        # Run the navigation demonstration
        nav_loop.run_demonstration()

        # Keep the node running
        rclpy.spin(nav_loop)

    except KeyboardInterrupt:
        nav_loop.get_logger().info("Navigation loop interrupted by user")
    except Exception as e:
        nav_loop.get_logger().error(f"Error in navigation loop: {e}")
    finally:
        nav_loop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()