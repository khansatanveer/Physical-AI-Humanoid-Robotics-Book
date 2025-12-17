#!/usr/bin/env python3
"""
Perception-Planning-Control Pipeline Example
Demonstrates the complete AI-driven navigation pipeline with all three components
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Time

import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import time
from threading import Lock
import tf2_ros
from tf2_ros import TransformException


class PerceptionPlanningControlPipeline(Node):
    """
    Complete pipeline integrating perception, planning, and control components
    """

    def __init__(self):
        super().__init__('perception_planning_control_pipeline')

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, 'pipeline_visualization', 10)
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, 'local_costmap', 10)

        # Subscribers for perception
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile
        )
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, qos_profile
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, qos_profile
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'depth/points', self.pointcloud_callback, qos_profile
        )

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Pipeline state
        self.current_pose = None
        self.current_goal = None
        self.navigation_active = False
        self.pipeline_state = "IDLE"  # IDLE, PERCEIVING, PLANNING, CONTROLLING

        # Perception data
        self.latest_image = None
        self.camera_info = None
        self.laser_data = None
        self.pointcloud_data = None
        self.environment_map = {
            'obstacles': [],
            'free_space': [],
            'unknown': [],
            'objects': []
        }

        # Planning data
        self.global_path = []
        self.local_path = []
        self.current_waypoint = None

        # Control data
        self.velocity_commands = []
        self.balance_state = "STABLE"

        # Pipeline parameters
        self.update_rate = 20.0  # Hz
        self.perception_rate = 30.0  # Hz
        self.planning_rate = 5.0  # Hz
        self.control_rate = 50.0  # Hz

        # Threading locks
        self.perception_lock = Lock()
        self.planning_lock = Lock()
        self.control_lock = Lock()

        # Timers for different pipeline components
        self.perception_timer = self.create_timer(1.0/self.perception_rate, self.perception_step)
        self.planning_timer = self.create_timer(1.0/self.planning_rate, self.planning_step)
        self.control_timer = self.create_timer(1.0/self.control_rate, self.control_step)
        self.pipeline_timer = self.create_timer(1.0/self.update_rate, self.pipeline_update)

        # Initialize pipeline state
        self.initialize_pipeline()

        self.get_logger().info("Perception-Planning-Control Pipeline initialized")
        self.get_logger().info("Complete AI-driven navigation pipeline ready")

    def initialize_pipeline(self):
        """
        Initialize the pipeline components
        """
        self.pipeline_state = "IDLE"
        self.get_logger().info("Pipeline initialized and ready for navigation")

    def laser_callback(self, msg):
        """
        Handle laser scan data (perception component)
        """
        with self.perception_lock:
            self.laser_data = msg

    def image_callback(self, msg):
        """
        Handle camera image data (perception component)
        """
        with self.perception_lock:
            try:
                self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

    def camera_info_callback(self, msg):
        """
        Handle camera information (perception component)
        """
        with self.perception_lock:
            self.camera_info = msg

    def pointcloud_callback(self, msg):
        """
        Handle point cloud data (perception component)
        """
        with self.perception_lock:
            self.pointcloud_data = msg

    def perception_step(self):
        """
        Execute perception step - process sensor data and update environment model
        """
        if self.pipeline_state != "PERCEIVING" and self.pipeline_state != "PLANNING" and self.pipeline_state != "CONTROLLING":
            return

        with self.perception_lock:
            # Process laser data for obstacle detection
            if self.laser_data is not None:
                obstacles = self.process_laser_data(self.laser_data)
                self.environment_map['obstacles'] = obstacles

            # Process image data for object detection
            if self.latest_image is not None:
                detected_objects = self.process_image_data(self.latest_image)
                self.environment_map['objects'] = detected_objects

            # Process point cloud data for 3D environment understanding
            if self.pointcloud_data is not None:
                self.process_pointcloud_data(self.pointcloud_data)

        self.get_logger().debug(f"Perception step completed. Obstacles detected: {len(self.environment_map['obstacles'])}")

    def process_laser_data(self, scan_msg):
        """
        Process laser scan data to detect obstacles
        """
        obstacles = []
        angle = scan_msg.angle_min

        for i, range_val in enumerate(scan_msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)) and range_val < 3.0:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                obstacles.append({
                    'x': x,
                    'y': y,
                    'range': range_val,
                    'angle': angle,
                    'index': i
                })
            angle += scan_msg.angle_increment

        return obstacles

    def process_image_data(self, image):
        """
        Process camera image for object detection and classification
        """
        # Simple color-based detection for demonstration
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red objects (for demonstration)
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

                detected_objects.append({
                    'type': 'red_object',
                    'center': (center_x, center_y),
                    'bbox': (x, y, w, h),
                    'area': cv2.contourArea(contour)
                })

        return detected_objects

    def process_pointcloud_data(self, pointcloud_msg):
        """
        Process point cloud data for 3D environment understanding
        """
        # In a real implementation, this would convert point cloud to 3D map
        # For demonstration, we'll just log the receipt
        self.get_logger().debug(f"Point cloud data received with {pointcloud_msg.height * pointcloud_msg.width} points")

    def planning_step(self):
        """
        Execute planning step - update path based on current environment model
        """
        if self.pipeline_state != "PLANNING" and self.pipeline_state != "CONTROLLING":
            return

        with self.planning_lock:
            # Update global path if we have a goal and current pose
            if self.current_goal is not None and self.current_pose is not None:
                self.global_path = self.update_global_path()

            # Update local path considering current obstacles
            self.local_path = self.update_local_path()

        self.get_logger().debug(f"Planning step completed. Global path points: {len(self.global_path)}, Local path points: {len(self.local_path)}")

    def update_global_path(self):
        """
        Update the global path to the goal
        """
        if self.current_pose is None or self.current_goal is None:
            return []

        # Simple straight-line path for demonstration
        # In a real system, this would use A*, Dijkstra, or other path planning algorithms
        start_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        goal_pos = np.array([self.current_goal.position.x, self.current_goal.position.y])

        # Calculate direction vector
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)

        # Create waypoints along the path
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

            # Keep same orientation as goal
            waypoint.pose.orientation = self.current_goal.orientation

            waypoints.append(waypoint)

        return waypoints

    def update_local_path(self):
        """
        Update the local path considering current obstacles
        """
        # Check for obstacles in the immediate vicinity
        safe_local_path = []

        with self.perception_lock:
            obstacles = self.environment_map['obstacles']

        # If we have a global path, check each segment for obstacles
        if self.global_path:
            for waypoint in self.global_path[:10]:  # Only check first 10 waypoints locally
                is_safe = True
                for obs in obstacles:
                    dist = math.sqrt(
                        (waypoint.pose.position.x - obs['x']) ** 2 +
                        (waypoint.pose.position.y - obs['y']) ** 2
                    )
                    if dist < 0.5:  # Safety margin
                        is_safe = False
                        break

                if is_safe:
                    safe_local_path.append(waypoint)
                else:
                    # Found obstacle, stop local path planning
                    break

        return safe_local_path

    def control_step(self):
        """
        Execute control step - generate velocity commands to follow the path
        """
        if self.pipeline_state != "CONTROLLING":
            return

        with self.control_lock:
            # Generate velocity commands to follow the local path
            cmd_vel = self.generate_velocity_commands()

            # Publish velocity commands
            if cmd_vel is not None:
                self.cmd_vel_pub.publish(cmd_vel)

        self.get_logger().debug("Control step completed")

    def generate_velocity_commands(self):
        """
        Generate velocity commands to follow the path
        """
        if not self.local_path or self.current_pose is None:
            return None

        # Get the next waypoint to follow
        if self.current_waypoint is None and self.local_path:
            self.current_waypoint = self.local_path[0]
        elif self.local_path:
            # Check if we're close to current waypoint, then move to next
            dist_to_waypoint = math.sqrt(
                (self.current_pose.position.x - self.current_waypoint.pose.position.x) ** 2 +
                (self.current_pose.position.y - self.current_waypoint.pose.position.y) ** 2
            )

            if dist_to_waypoint < 0.25:  # Waypoint reached
                if len(self.local_path) > 1:
                    self.local_path.pop(0)
                    self.current_waypoint = self.local_path[0]

        if self.current_waypoint is None:
            return None

        # Calculate direction to waypoint
        dx = self.current_waypoint.pose.position.x - self.current_pose.position.x
        dy = self.current_waypoint.pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Simple proportional controller
        if distance > 0.1:  # If not very close to waypoint
            # Calculate angle to waypoint
            target_angle = math.atan2(dy, dx)

            # Get current orientation
            current_quat = self.current_pose.orientation
            current_euler = R.from_quat([
                current_quat.x,
                current_quat.y,
                current_quat.z,
                current_quat.w
            ]).as_euler('xyz')
            current_angle = current_euler[2]  # Z rotation

            # Calculate angle difference
            angle_diff = target_angle - current_angle
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Create velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = min(0.5, distance * 1.0)  # Max 0.5 m/s
            cmd_vel.angular.z = angle_diff * 1.0  # Proportional control

            return cmd_vel

        return None  # Stop when close to goal

    def pipeline_update(self):
        """
        Main pipeline update - coordinate perception, planning, and control
        """
        # Update current pose from TF or other source
        self.update_current_pose()

        # State machine for pipeline coordination
        if self.navigation_active:
            if self.pipeline_state == "IDLE":
                self.pipeline_state = "PERCEIVING"
            elif self.pipeline_state == "PERCEIVING":
                # After perception is done, move to planning
                self.pipeline_state = "PLANNING"
            elif self.pipeline_state == "PLANNING":
                # After planning is done, move to control
                self.pipeline_state = "CONTROLLING"
            elif self.pipeline_state == "CONTROLLING":
                # Check if we've reached the goal
                if self.has_reached_goal():
                    self.pipeline_state = "IDLE"
                    self.navigation_active = False
                    self.get_logger().info("Navigation goal reached!")
        else:
            # If not navigating, stay in idle state
            self.pipeline_state = "IDLE"

        self.get_logger().debug(f"Pipeline state: {self.pipeline_state}")

    def update_current_pose(self):
        """
        Update current robot pose from localization system
        """
        # In a real system, this would get pose from AMCL, odometry, or other localization
        # For demonstration, we'll simulate pose updates if we have basic info
        pass

    def has_reached_goal(self):
        """
        Check if robot has reached the navigation goal
        """
        if self.current_pose is None or self.current_goal is None:
            return False

        distance = math.sqrt(
            (self.current_pose.position.x - self.current_goal.position.x) ** 2 +
            (self.current_pose.position.y - self.current_goal.position.y) ** 2
        )

        return distance < 0.25  # 25cm tolerance

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

    def send_navigation_goal(self, x, y, theta=0.0):
        """
        Send navigation goal to the system
        """
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = R.from_euler('z', theta).as_quat()
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        self.current_goal = goal_pose.pose
        self.navigation_active = True
        self.pipeline_state = "PERCEIVING"  # Start the pipeline

        self.get_logger().info(f"Navigation goal set: ({x}, {y}, {theta})")
        return True

    def execute_pipeline_demo(self):
        """
        Execute a demonstration of the complete pipeline
        """
        self.get_logger().info("Starting Perception-Planning-Control pipeline demonstration...")

        # Set initial pose at (0, 0)
        self.set_initial_pose(0.0, 0.0, 0.0)

        # Set goal at (5, 5)
        success = self.send_navigation_goal(5.0, 5.0, 0.0)

        if success:
            self.get_logger().info("Pipeline demonstration started successfully")
        else:
            self.get_logger().error("Failed to start pipeline demonstration")

    def visualize_pipeline_state(self):
        """
        Visualize the current state of the pipeline
        """
        marker_array = MarkerArray()

        # Create markers for different pipeline components
        # Perception markers
        perception_marker = Marker()
        perception_marker.header.frame_id = "map"
        perception_marker.header.stamp = self.get_clock().now().to_msg()
        perception_marker.ns = "pipeline"
        perception_marker.id = 1
        perception_marker.type = Marker.TEXT_VIEW_FACING
        perception_marker.action = Marker.ADD
        perception_marker.pose.position.x = -2.0
        perception_marker.pose.position.y = 2.0
        perception_marker.pose.position.z = 1.0
        perception_marker.pose.orientation.w = 1.0
        perception_marker.scale.z = 0.3
        perception_marker.color.r = 0.0
        perception_marker.color.g = 1.0
        perception_marker.color.b = 0.0
        perception_marker.color.a = 1.0
        perception_marker.text = f"Perception: {len(self.environment_map['obstacles'])} obstacles"
        marker_array.markers.append(perception_marker)

        # Planning markers
        planning_marker = Marker()
        planning_marker.header.frame_id = "map"
        planning_marker.header.stamp = self.get_clock().now().to_msg()
        planning_marker.ns = "pipeline"
        planning_marker.id = 2
        planning_marker.type = Marker.TEXT_VIEW_FACING
        planning_marker.action = Marker.ADD
        planning_marker.pose.position.x = 0.0
        planning_marker.pose.position.y = 2.0
        planning_marker.pose.position.z = 1.0
        planning_marker.pose.orientation.w = 1.0
        planning_marker.scale.z = 0.3
        planning_marker.color.r = 1.0
        planning_marker.color.g = 1.0
        planning_marker.color.b = 0.0
        planning_marker.color.a = 1.0
        planning_marker.text = f"Planning: {len(self.local_path)} local points"
        marker_array.markers.append(planning_marker)

        # Control markers
        control_marker = Marker()
        control_marker.header.frame_id = "map"
        control_marker.header.stamp = self.get_clock().now().to_msg()
        control_marker.ns = "pipeline"
        control_marker.id = 3
        control_marker.type = Marker.TEXT_VIEW_FACING
        control_marker.action = Marker.ADD
        control_marker.pose.position.x = 2.0
        control_marker.pose.position.y = 2.0
        control_marker.pose.position.z = 1.0
        control_marker.pose.orientation.w = 1.0
        control_marker.scale.z = 0.3
        control_marker.color.r = 0.0
        control_marker.color.g = 0.0
        control_marker.color.b = 1.0
        control_marker.color.a = 1.0
        control_marker.text = f"Control: {self.pipeline_state}"
        marker_array.markers.append(control_marker)

        self.visualization_pub.publish(marker_array)


def main(args=None):
    """
    Main function to run the perception-planning-control pipeline example
    """
    rclpy.init(args=args)

    pipeline = PerceptionPlanningControlPipeline()

    try:
        # Run the pipeline demonstration
        pipeline.execute_pipeline_demo()

        # Keep the node running
        rclpy.spin(pipeline)

    except KeyboardInterrupt:
        pipeline.get_logger().info("Pipeline interrupted by user")
    except Exception as e:
        pipeline.get_logger().error(f"Error in pipeline: {e}")
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()