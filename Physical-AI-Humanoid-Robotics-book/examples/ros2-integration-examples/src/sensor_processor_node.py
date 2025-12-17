#!/usr/bin/env python3
"""
Sensor Data Processor Node for Digital Twin

This node processes data from multiple sensors (LiDAR, Camera, IMU)
and prepares it for visualization and analysis in the digital twin system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import math
from collections import deque


class SensorProcessorNode(Node):
    def __init__(self):
        super().__init__('sensor_processor_node')

        # Create subscription for each sensor type
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/basic_humanoid/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/basic_humanoid/camera/image_raw',
            self.camera_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/basic_humanoid/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/basic_humanoid/imu',
            self.imu_callback,
            10
        )

        # Publisher for processed sensor data
        self.obstacle_publisher = self.create_publisher(
            Twist,
            '/basic_humanoid/obstacle_avoidance_cmd',
            10
        )

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Data storage
        self.lidar_ranges = None
        self.lidar_angle_min = 0.0
        self.lidar_angle_max = 0.0
        self.lidar_angle_increment = 0.0
        self.latest_image = None
        self.latest_depth = None
        self.latest_imu = None

        # Processing parameters
        self.min_obstacle_distance = 1.0  # meters
        self.safe_zone_radius = 2.0       # meters

        # Statistics
        self.lidar_update_count = 0
        self.camera_update_count = 0
        self.imu_update_count = 0

        self.get_logger().info('Sensor Processor Node initialized')

    def lidar_callback(self, msg):
        """Process LiDAR scan data"""
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_max = msg.angle_max
        self.lidar_angle_increment = msg.angle_increment

        # Filter out invalid ranges (inf, nan)
        valid_mask = np.isfinite(self.lidar_ranges) & (self.lidar_ranges > 0)
        valid_ranges = self.lidar_ranges[valid_mask]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.lidar_update_count += 1

            # Log statistics periodically
            if self.lidar_update_count % 100 == 0:
                avg_distance = np.mean(valid_ranges)
                self.get_logger().info(
                    f'LiDAR: Min dist={min_distance:.2f}m, '
                    f'Avg dist={avg_distance:.2f}m, '
                    f'Valid points={len(valid_ranges)}'
                )

            # Check for obstacles and publish avoidance command if needed
            self.check_obstacles(min_distance)
        else:
            self.get_logger().warn('LiDAR: No valid range readings')

    def camera_callback(self, msg):
        """Process camera image data"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image

            self.camera_update_count += 1

            # Log statistics periodically
            if self.camera_update_count % 300 == 0:  # Every 300 frames
                height, width, channels = cv_image.shape
                self.get_logger().info(
                    f'Camera: Image received - {width}x{height}x{channels}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def depth_callback(self, msg):
        """Process depth image data"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.latest_depth = cv_depth

            # Process depth data (example: find average depth in center region)
            height, width = cv_depth.shape
            center_h, center_w = height // 2, width // 2
            center_region = cv_depth[
                center_h-50:center_h+50,
                center_w-50:center_w+50
            ]

            # Filter out invalid depth values
            valid_depths = center_region[np.isfinite(center_region) & (center_region > 0)]

            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
                self.get_logger().info(f'Depth: Average center depth = {avg_depth:.2f}m')

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def imu_callback(self, msg):
        """Process IMU data"""
        self.latest_imu = msg
        self.imu_update_count += 1

        # Extract orientation quaternion
        orientation = msg.orientation
        # Convert quaternion to Euler angles for easier interpretation
        euler = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # Extract angular velocity
        ang_vel = msg.angular_velocity
        linear_acc = msg.linear_acceleration

        # Log statistics periodically
        if self.imu_update_count % 100 == 0:
            self.get_logger().info(
                f'IMU: Roll={euler[0]:.3f}, Pitch={euler[1]:.3f}, Yaw={euler[2]:.3f} '
                f'(rad) | AngVel: x={ang_vel.x:.3f}, y={ang_vel.y:.3f}, z={ang_vel.z:.3f}'
            )

    def check_obstacles(self, min_distance):
        """Check for obstacles and publish avoidance command if needed"""
        if min_distance < self.min_obstacle_distance:
            # Create obstacle avoidance command
            cmd = Twist()
            cmd.linear.x = 0.0  # Stop forward motion
            cmd.angular.z = 0.5  # Turn to avoid obstacle

            self.obstacle_publisher.publish(cmd)
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m! Avoidance command published.')
        else:
            # Publish stop command when no obstacles
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.obstacle_publisher.publish(cmd)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def get_robot_state_summary(self):
        """Get a summary of the robot's sensor state"""
        summary = {
            'lidar': {
                'updates': self.lidar_update_count,
                'has_data': self.lidar_ranges is not None,
                'min_distance': float(np.min(self.lidar_ranges)) if self.lidar_ranges is not None and len(self.lidar_ranges) > 0 else None
            },
            'camera': {
                'updates': self.camera_update_count,
                'has_data': self.latest_image is not None
            },
            'imu': {
                'updates': self.imu_update_count,
                'has_data': self.latest_imu is not None
            }
        }
        return summary


def main(args=None):
    rclpy.init(args=args)

    sensor_processor = SensorProcessorNode()

    # Add a timer to periodically log sensor state
    timer = sensor_processor.create_timer(5.0, lambda:
        sensor_processor.get_logger().info(
            f'Sensor Summary: {sensor_processor.get_robot_state_summary()}'
        )
    )

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        sensor_processor.get_logger().info('Sensor Processor Node stopped by user')
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()