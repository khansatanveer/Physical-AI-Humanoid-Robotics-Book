#!/usr/bin/env python3
"""
Isaac ROS Navigation Basics Example
Demonstrates fundamental navigation concepts using ROS navigation stack
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import tf_transformations
from std_srvs.srv import Empty
import numpy as np
import math


class NavigationBasicNode(Node):
    """
    Basic navigation node demonstrating path planning and execution
    """

    def __init__(self):
        super().__init__('navigation_basic_node')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/current_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/navigation_marker', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Service clients
        self.clear_costmap_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')

        # TF listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state
        self.current_pose = None
        self.current_goal = None
        self.path = []
        self.obstacles = []

        # Navigation parameters
        self.linear_vel = 0.5  # m/s
        self.angular_vel = 0.5  # rad/s
        self.arrival_threshold = 0.5  # meters
        self.rotation_threshold = 0.1  # radians

        self.get_logger().info("Navigation Basic Node initialized")

    def odom_callback(self, msg):
        """Callback for odometry messages"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Callback for laser scan messages"""
        # Process laser scan to detect obstacles
        self.process_scan_for_obstacles(msg)

    def process_scan_for_obstacles(self, scan_msg):
        """Process laser scan to detect obstacles"""
        # Convert laser scan to obstacle positions
        self.obstacles = []
        angle = scan_msg.angle_min

        for range_val in scan_msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)) and range_val < 2.0:  # Within 2m
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                self.obstacles.append((x, y))
            angle += scan_msg.angle_increment

    def send_goal(self, x, y, theta=0.0):
        """Send a navigation goal to the system"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        goal_msg.pose.orientation.x = quat[0]
        goal_msg.pose.orientation.y = quat[1]
        goal_msg.pose.orientation.z = quat[2]
        goal_msg.pose.orientation.w = quat[3]

        self.current_goal = goal_msg.pose
        self.goal_pub.publish(goal_msg)

        self.get_logger().info(f"Goal sent: ({x}, {y}, {theta})")

    def calculate_path(self):
        """Calculate a simple path to the goal (in a real system, this would use A*, Dijkstra, etc.)"""
        if self.current_pose is None or self.current_goal is None:
            return []

        # Simple straight-line path calculation
        start_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        goal_pos = np.array([self.current_goal.position.x, self.current_goal.position.y])

        # Calculate direction vector
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)

        if distance < self.arrival_threshold:
            return []  # Already at goal

        # Normalize direction
        direction = direction / distance

        # Create path points (every 0.1m)
        path = []
        step_size = 0.1
        num_steps = int(distance / step_size)

        for i in range(num_steps + 1):
            step_pos = start_pos + direction * (i * step_size)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(step_pos[0])
            pose.pose.position.y = float(step_pos[1])
            pose.pose.position.z = 0.0
            path.append(pose)

        return path

    def check_obstacle_collision(self, path):
        """Check if the path collides with any obstacles"""
        if not path or not self.obstacles:
            return False

        for path_point in path:
            for obs_x, obs_y in self.obstacles:
                dist = math.sqrt(
                    (path_point.pose.position.x - obs_x) ** 2 +
                    (path_point.pose.position.y - obs_y) ** 2
                )
                if dist < 0.3:  # Collision threshold
                    return True

        return False

    def simple_navigation_loop(self):
        """Simple navigation loop - in a real system this would be more sophisticated"""
        if self.current_pose is None or self.current_goal is None:
            return

        # Calculate path to goal
        path = self.calculate_path()
        self.path = path

        # Publish current path
        if path:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            path_msg.poses = path
            self.path_pub.publish(path_msg)

        # Check if we're close enough to the goal
        goal_dist = math.sqrt(
            (self.current_pose.position.x - self.current_goal.position.x) ** 2 +
            (self.current_pose.position.y - self.current_goal.position.y) ** 2
        )

        if goal_dist < self.arrival_threshold:
            self.get_logger().info("Goal reached!")
            return

        # Simple control: move toward goal
        goal_angle = math.atan2(
            self.current_goal.position.y - self.current_pose.position.y,
            self.current_goal.position.x - self.current_pose.position.x
        )

        # Current robot angle from quaternion
        current_quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        _, _, current_angle = tf_transformations.euler_from_quaternion(current_quat)

        # Calculate angle difference
        angle_diff = goal_angle - current_angle
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Simple proportional control for rotation
        angular_cmd = max(-self.angular_vel, min(self.angular_vel, angle_diff * 0.5))

        # Move forward if roughly aligned with goal
        linear_cmd = 0.0
        if abs(angle_diff) < self.rotation_threshold:
            linear_cmd = self.linear_vel

        self.get_logger().info(f"Moving: linear={linear_cmd:.2f}, angular={angular_cmd:.2f}")

    def create_visualization_marker(self, x, y, z, marker_type, marker_id):
        """Create a visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def run_navigation_example(self):
        """Run a complete navigation example"""
        self.get_logger().info("Starting navigation example...")

        # Send a goal to navigate to (2.0, 2.0)
        self.send_goal(2.0, 2.0)

        # Run navigation loop for a set number of iterations
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.simple_navigation_loop)


def main(args=None):
    """Main function to run the navigation basic example"""
    rclpy.init(args=args)

    nav_node = NavigationBasicNode()

    try:
        # Start the navigation example
        nav_node.run_navigation_example()

        # Spin to keep the node running
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()