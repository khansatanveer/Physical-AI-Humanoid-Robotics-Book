#!/usr/bin/env python3
"""
Sensor Fusion Node for Digital Twin

This node combines data from multiple sensors to create a unified
perception of the environment for the digital twin system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import math
from collections import deque
from scipy.spatial.transform import Rotation as R


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create subscription for each sensor type
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/basic_humanoid/scan',
            self.lidar_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/basic_humanoid/imu',
            self.imu_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/basic_humanoid/odom',
            self.odom_callback,
            10
        )

        # Publishers for fused sensor data
        self.environment_map_publisher = self.create_publisher(
            PointStamped,
            '/basic_humanoid/environment_map_point',
            10
        )

        self.fused_state_publisher = self.create_publisher(
            Float32MultiArray,
            '/basic_humanoid/fused_sensor_state',
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/basic_humanoid/fused_pose',
            10
        )

        # Initialize CvBridge (though we're not using images in this node)
        self.bridge = CvBridge()

        # Data storage with timestamps
        self.lidar_data = {'ranges': None, 'timestamp': None, 'angle_min': 0, 'angle_max': 0, 'angle_increment': 0}
        self.imu_data = {'orientation': None, 'angular_velocity': None, 'linear_acceleration': None, 'timestamp': None}
        self.odom_data = {'pose': None, 'twist': None, 'timestamp': None}

        # Sensor fusion state
        self.fused_position = np.array([0.0, 0.0, 0.0])
        self.fused_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x, y, z, w]
        self.fused_velocity = np.array([0.0, 0.0, 0.0])

        # Time synchronization window (in seconds)
        self.sync_window = 0.1  # 100ms window for sensor synchronization

        # Statistics
        self.fusion_update_count = 0
        self.lidar_sync_count = 0
        self.imu_sync_count = 0

        # Moving averages for smoothing
        self.position_history = deque(maxlen=10)
        self.orientation_history = deque(maxlen=10)

        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        """Process LiDAR scan data"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Store lidar data with timestamp
        self.lidar_data = {
            'ranges': np.array(msg.ranges),
            'timestamp': current_time,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

        self.lidar_sync_count += 1

        # Try to fuse data if other sensors also have recent data
        self.attempt_fusion(current_time)

    def imu_callback(self, msg):
        """Process IMU data"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Store IMU data with timestamp
        self.imu_data = {
            'orientation': np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]),
            'angular_velocity': np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]),
            'linear_acceleration': np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]),
            'timestamp': current_time
        }

        self.imu_sync_count += 1

        # Try to fuse data if other sensors also have recent data
        self.attempt_fusion(current_time)

    def odom_callback(self, msg):
        """Process odometry data"""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Store odometry data with timestamp
        self.odom_data = {
            'pose': np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]),
            'twist': np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ]),
            'timestamp': current_time
        }

        # Update fused position from odometry
        self.fused_position = self.odom_data['pose'][:3]
        self.fused_orientation = self.odom_data['pose'][3:]
        self.fused_velocity = self.odom_data['twist'][:3]

        # Add to history for smoothing
        self.position_history.append(self.fused_position.copy())
        self.orientation_history.append(self.fused_orientation.copy())

    def attempt_fusion(self, current_time):
        """Attempt to fuse sensor data if all required sensors have recent data"""
        # Check if we have data from all sensors within the sync window
        if (self.lidar_data['timestamp'] is not None and
            self.imu_data['timestamp'] is not None and
            self.odom_data['timestamp'] is not None):

            # Check if all data is within the sync window
            lidar_age = abs(current_time - self.lidar_data['timestamp'])
            imu_age = abs(current_time - self.imu_data['timestamp'])
            odom_age = abs(current_time - self.odom_data['timestamp'])

            if all(age <= self.sync_window for age in [lidar_age, imu_age, odom_age]):
                self.perform_sensor_fusion(current_time)

    def perform_sensor_fusion(self, current_time):
        """Perform the actual sensor fusion algorithm"""
        # Extract sensor data
        lidar_ranges = self.lidar_data['ranges']
        imu_orientation = self.imu_data['orientation']
        imu_angular_vel = self.imu_data['angular_velocity']
        imu_linear_acc = self.imu_data['linear_acceleration']

        # 1. Update orientation using IMU data (complementary filter approach)
        # For now, we'll use the IMU orientation directly, but in a real system
        # you might want to fuse it with odometry data
        self.fused_orientation = imu_orientation

        # 2. Process LiDAR data for environment mapping
        if lidar_ranges is not None and len(lidar_ranges) > 0:
            # Filter out invalid ranges
            valid_mask = np.isfinite(lidar_ranges) & (lidar_ranges > 0) & (lidar_ranges < 30.0)
            valid_ranges = lidar_ranges[valid_mask]

            if len(valid_ranges) > 0:
                # Calculate angles for valid ranges
                valid_indices = np.where(valid_mask)[0]
                angles = self.lidar_data['angle_min'] + valid_indices * self.lidar_data['angle_increment']

                # Convert to Cartesian coordinates relative to robot
                x_points = valid_ranges * np.cos(angles)
                y_points = valid_ranges * np.sin(angles)
                z_points = np.zeros_like(x_points)  # 2D LiDAR

                # Transform to global coordinates using robot pose
                # (simplified - in reality, you'd use the full transformation)
                rotation_matrix = self.quaternion_to_rotation_matrix(self.fused_orientation)
                local_points = np.vstack([x_points, y_points, z_points])
                global_points = rotation_matrix @ local_points
                global_points[0, :] += self.fused_position[0]
                global_points[1, :] += self.fused_position[1]
                global_points[2, :] += self.fused_position[2]

                # Publish some environment points (first 10 for visualization)
                if global_points.shape[1] > 0:
                    for i in range(min(10, global_points.shape[1])):
                        point_msg = PointStamped()
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.header.frame_id = 'map'
                        point_msg.point.x = float(global_points[0, i])
                        point_msg.point.y = float(global_points[1, i])
                        point_msg.point.z = float(global_points[2, i])
                        self.environment_map_publisher.publish(point_msg)

        # 3. Update position using odometry with IMU correction
        # In a real system, you'd implement a more sophisticated fusion algorithm
        # like an Extended Kalman Filter or Particle Filter

        # 4. Create fused state array for Unity visualization
        fused_state = Float32MultiArray()
        fused_state.data = [
            float(self.fused_position[0]),  # x
            float(self.fused_position[1]),  # y
            float(self.fused_position[2]),  # z
            float(self.fused_orientation[0]),  # qx
            float(self.fused_orientation[1]),  # qy
            float(self.fused_orientation[2]),  # qz
            float(self.fused_orientation[3]),  # qw
            float(self.fused_velocity[0]),  # vx
            float(self.fused_velocity[1]),  # vy
            float(self.fused_velocity[2])   # vz
        ]

        # Publish fused state
        self.fused_state_publisher.publish(fused_state)

        # Publish fused pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(self.fused_position[0])
        pose_msg.pose.position.y = float(self.fused_position[1])
        pose_msg.pose.position.z = float(self.fused_position[2])
        pose_msg.pose.orientation.x = float(self.fused_orientation[0])
        pose_msg.pose.orientation.y = float(self.fused_orientation[1])
        pose_msg.pose.orientation.z = float(self.fused_orientation[2])
        pose_msg.pose.orientation.w = float(self.fused_orientation[3])

        self.pose_publisher.publish(pose_msg)

        # Update statistics
        self.fusion_update_count += 1

        # Log statistics periodically
        if self.fusion_update_count % 50 == 0:
            self.get_logger().info(
                f'Sensor Fusion: Updates={self.fusion_update_count}, '
                f'LiDAR sync={self.lidar_sync_count}, '
                f'IMU sync={self.imu_sync_count}'
            )

    def quaternion_to_rotation_matrix(self, quat):
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
        x, y, z, w = quat

        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm

        # Create rotation matrix
        rotation_matrix = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

        return rotation_matrix

    def get_fused_state(self):
        """Get the current fused state"""
        return {
            'position': self.fused_position.copy(),
            'orientation': self.fused_orientation.copy(),
            'velocity': self.fused_velocity.copy()
        }


def main(args=None):
    rclpy.init(args=args)

    sensor_fusion = SensorFusionNode()

    # Add a timer to periodically log fusion state
    timer = sensor_fusion.create_timer(2.0, lambda:
        sensor_fusion.get_logger().info(
            f'Fusion State: Pos={sensor_fusion.get_fused_state()["position"]}'
        )
    )

    try:
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        sensor_fusion.get_logger().info('Sensor Fusion Node stopped by user')
    finally:
        sensor_fusion.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()