---
title: Main Project - Complete AI-Driven Navigation Loop
---

# Main Project: Complete AI-Driven Navigation Loop

## Project Overview

In this project, you'll build a complete AI-driven navigation system that integrates perception, planning, and control components. This system will enable a humanoid robot to autonomously navigate through unknown environments using NVIDIA Isaac Sim for simulation and Navigation2 (Nav2) for path planning.

## Bill of Materials (BOM)

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **NVIDIA Isaac Sim**: 2023.1.0 or later
- **Isaac ROS Packages**: Latest compatible version
- **Navigation2 (Nav2)**: Latest stable release
- **Python**: 3.8 or higher
- **NVIDIA GPU**: RTX 3060 or better (for simulation)
- **System RAM**: 16GB or more
- **Storage**: 50GB free space for Isaac Sim

### Optional Hardware (for real robot testing)
- **Humanoid Robot Platform**: Compatible with ROS 2
- **RGB-D Camera**: For perception
- **LIDAR Sensor**: For environment mapping
- **IMU**: For balance and orientation

## System Architecture

The complete navigation system consists of three main components:

1. **Perception**: Sensors gather environmental data (cameras, LIDAR, IMU)
2. **Planning**: Path planning algorithms determine optimal routes (Nav2)
3. **Control**: Motion controllers execute planned movements (humanoid controllers)

## Step-by-Step Instructions

### Step 1: Environment Setup

1. **Install ROS 2 Humble**:
   ```bash
   # Setup locale
   sudo locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

   # Install ROS 2 packages
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

2. **Install Navigation2**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

3. **Install Isaac ROS**:
   ```bash
   sudo apt install ros-humble-isaac-ros-* ros-humble-isaac-ros-gems
   ```

4. **Setup ROS workspace**:
   ```bash
   mkdir -p ~/navigation_ws/src
   cd ~/navigation_ws
   colcon build
   source install/setup.bash
   ```

### Step 2: Create the Complete Navigation Package

1. **Create the navigation package**:
   ```bash
   cd ~/navigation_ws/src
   ros2 pkg create --build-type ament_python navigation_complete_loop
   cd navigation_complete_loop
   ```

2. **Create the package structure**:
   ```bash
   mkdir -p navigation_complete_loop/{perception,planning,control,utils}
   touch navigation_complete_loop/__init__.py
   touch navigation_complete_loop/perception/__init__.py
   touch navigation_complete_loop/planning/__init__.py
   touch navigation_complete_loop/control/__init__.py
   touch navigation_complete_loop/utils/__init__.py
   ```

3. **Update setup.py**:
   ```python
   import os
   from glob import glob
   from setuptools import setup

   package_name = 'navigation_complete_loop'

   setup(
       name=package_name,
       version='0.0.1',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='Complete AI-driven navigation loop example',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'complete_navigation = navigation_complete_loop.complete_navigation:main',
               'perception_node = navigation_complete_loop.perception_node:main',
               'planning_node = navigation_complete_loop.planning_node:main',
               'control_node = navigation_complete_loop.control_node:main',
           ],
       },
   )
   ```

### Step 3: Implement the Complete Navigation Node

Create the main navigation node that integrates all components:

```python
# navigation_complete_loop/complete_navigation.py
#!/usr/bin/env python3
"""
Complete AI-Driven Navigation Loop
Integrates perception, planning, and control systems for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
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
```

### Step 4: Create Perception Node

Create a dedicated perception node:

```python
# navigation_complete_loop/perception_node.py
#!/usr/bin/env python3
"""
Perception Node for Navigation System
Processes sensor data to understand the environment
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header

import cv2
from cv_bridge import CvBridge
import numpy as np
import math
from scipy.spatial import distance


class PerceptionNode(Node):
    """
    Perception node for processing sensor data
    """

    def __init__(self):
        super().__init__('perception_node')

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.obstacle_pub = self.create_publisher(PointStamped, 'obstacles', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, 'perception_visualization', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile
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

        # Perception data
        self.latest_image = None
        self.camera_info = None
        self.laser_data = None
        self.pointcloud_data = None

        # Processing parameters
        self.update_rate = 30.0  # Hz

        # Timer for perception processing
        self.perception_timer = self.create_timer(1.0/self.update_rate, self.process_perception)

        self.get_logger().info("Perception node initialized")

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.laser_data = msg

    def image_callback(self, msg):
        """Handle camera image data"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def camera_info_callback(self, msg):
        """Handle camera information"""
        self.camera_info = msg

    def pointcloud_callback(self, msg):
        """Handle point cloud data"""
        self.pointcloud_data = msg

    def process_perception(self):
        """Main perception processing loop"""
        if self.laser_data is not None:
            obstacles = self.process_laser_scan()
            self.publish_obstacles(obstacles)

        if self.latest_image is not None:
            objects = self.process_camera_image()
            self.publish_objects(objects)

    def process_laser_scan(self):
        """Process laser scan data to detect obstacles"""
        obstacles = []
        if self.laser_data is None:
            return obstacles

        angle = self.laser_data.angle_min
        for range_val in self.laser_data.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)) and range_val < 3.0:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                obstacles.append((x, y))
            angle += self.laser_data.angle_increment

        return obstacles

    def process_camera_image(self):
        """Process camera image for object detection"""
        if self.latest_image is None:
            return []

        # Simple color-based detection for demonstration
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                objects.append({
                    'type': 'red_object',
                    'center': (center_x, center_y),
                    'bbox': (x, y, w, h),
                    'area': cv2.contourArea(contour)
                })

        return objects

    def publish_obstacles(self, obstacles):
        """Publish detected obstacles"""
        for obs_x, obs_y in obstacles:
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'map'
            point.point.x = obs_x
            point.point.y = obs_y
            point.point.z = 0.0
            self.obstacle_pub.publish(point)

    def publish_objects(self, objects):
        """Publish detected objects"""
        # Implementation for publishing detected objects
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Perception node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Create Planning Node

Create a dedicated planning node:

```python
# navigation_complete_loop/planning_node.py
#!/usr/bin/env python3
"""
Planning Node for Navigation System
Creates paths from current location to goal
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray


class PlanningNode(Node):
    """
    Planning node for path generation
    """

    def __init__(self):
        super().__init__('planning_node')

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.path_pub = self.create_publisher(PoseStamped, 'planned_path', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, 'planning_visualization', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 10
        )
        self.obstacle_sub = self.create_subscription(
            PointCloud2, 'obstacles', self.obstacle_callback, 10
        )

        # Planning state
        self.current_pose = None
        self.current_goal = None
        self.obstacles = []

        # Planning parameters
        self.update_rate = 5.0  # Hz

        # Timer for planning
        self.planning_timer = self.create_timer(1.0/self.update_rate, self.plan_path)

        self.get_logger().info("Planning node initialized")

    def goal_callback(self, msg):
        """Handle goal pose"""
        self.current_goal = msg.pose

    def initial_pose_callback(self, msg):
        """Handle initial pose"""
        self.current_pose = msg.pose.pose

    def obstacle_callback(self, msg):
        """Handle obstacle information"""
        # Process obstacle data
        self.obstacles.append(msg)

    def plan_path(self):
        """Plan path from current pose to goal"""
        if self.current_pose is None or self.current_goal is None:
            return

        # Simple path planning implementation
        # In a real system, this would use A*, Dijkstra, or other algorithms
        self.get_logger().debug("Planning path from current pose to goal")


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planning node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 6: Create Control Node

Create a dedicated control node:

```python
# navigation_complete_loop/control_node.py
#!/usr/bin/env python3
"""
Control Node for Navigation System
Executes planned movements on the robot
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState


class ControlNode(Node):
    """
    Control node for executing navigation commands
    """

    def __init__(self):
        super().__init__('control_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, 'planned_path', self.path_callback, 10
        )
        self.velocity_sub = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10
        )

        # Control state
        self.current_path = []
        self.current_velocity = Twist()

        # Control parameters
        self.control_rate = 50.0  # Hz

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_rate, self.execute_control)

        self.get_logger().info("Control node initialized")

    def path_callback(self, msg):
        """Handle planned path"""
        self.current_path = msg.poses

    def velocity_callback(self, msg):
        """Handle velocity commands"""
        self.current_velocity = msg

    def execute_control(self):
        """Execute control commands"""
        # Send velocity commands to robot
        if self.current_velocity.linear.x != 0 or self.current_velocity.angular.z != 0:
            self.cmd_vel_pub.publish(self.current_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Control node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 7: Create Launch File

Create a launch file to start all nodes together:

```xml
# navigation_complete_loop/launch/navigation_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_complete_loop',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
        Node(
            package='navigation_complete_loop',
            executable='planning_node',
            name='planning_node',
            output='screen'
        ),
        Node(
            package='navigation_complete_loop',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        Node(
            package='navigation_complete_loop',
            executable='complete_navigation',
            name='complete_navigation_loop',
            output='screen'
        )
    ])
```

### Step 8: Build and Test the System

1. **Build the package**:
   ```bash
   cd ~/navigation_ws
   colcon build --packages-select navigation_complete_loop
   source install/setup.bash
   ```

2. **Run the complete navigation system**:
   ```bash
   ros2 launch navigation_complete_loop navigation_launch.py
   ```

3. **Test with Isaac Sim**:
   - Launch Isaac Sim with a humanoid robot model
   - Configure the robot to publish sensor data to the expected topics
   - Send navigation goals using the appropriate ROS 2 interfaces

## Configuration Files

### Nav2 Configuration for Humanoid Robots

Create `config/humanoid_nav2_config.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "humanoid_navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["HumanoidController"]

    HumanoidController:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 20
      control_horizon: 10
      model_dt: 0.1
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      publish_cost_grid: False
      speed_regulation_factor: 0.1
      obstacle_cost_weight: 1.0
      goal_cost_weight: 1.0
      control_cost_weight: 0.0
      curvature_cost_weight: 0.0
      # Humanoid-specific parameters
      max_humanoid_vel_x: 0.3
      min_humanoid_vel_x: -0.1
      max_humanoid_vel_theta: 0.5
      max_humanoid_acc_x: 0.5
      max_humanoid_acc_theta: 0.5
      step_size_limit: 0.3
      foot_separation: 0.2
      stance_time: 0.1

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3
```

## Testing and Validation

### Basic Navigation Test
1. Launch the navigation system
2. Set initial pose at (0, 0)
3. Set goal at (5, 5)
4. Verify the robot navigates to the goal
5. Check that obstacles are detected and avoided

### Complex Environment Test
1. Create a complex environment with multiple obstacles
2. Test navigation in this environment
3. Verify path replanning when obstacles are detected
4. Check that the robot can recover from navigation failures

### Performance Metrics
- Navigation success rate
- Average time to reach goal
- Path efficiency (actual path length vs. straight-line distance)
- Obstacle detection accuracy
- System response time

## Troubleshooting

### Common Issues
- **No path found**: Check that the goal is reachable and not in an obstacle
- **Robot not moving**: Verify that velocity commands are being published
- **Oscillating behavior**: Adjust controller parameters for stability
- **Perception errors**: Check sensor calibration and data quality

### Debugging Tips
- Use RViz2 to visualize the navigation system state
- Monitor ROS 2 topics to verify data flow
- Check TF transforms for coordinate frame issues
- Use ROS 2 tools like `ros2 bag` to record and replay data

## Extension Ideas

1. **Learning-based navigation**: Implement reinforcement learning for navigation
2. **Semantic mapping**: Add object recognition and semantic mapping
3. **Multi-robot coordination**: Extend to multiple robots
4. **Dynamic obstacle prediction**: Predict and avoid moving obstacles
5. **Human-aware navigation**: Consider human presence in navigation planning