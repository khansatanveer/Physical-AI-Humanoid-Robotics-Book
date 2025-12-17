#!/usr/bin/env python3
"""
Sensor Data Publisher Example

This example demonstrates how to publish sensor data that can be used
by AI agents in ROS 2 systems. It simulates various sensor types.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import math
import random


class SensorPublisherNode(Node):
    """
    A node that publishes various types of sensor data for AI agent consumption.
    """

    def __init__(self):
        """
        Initialize the sensor publisher node.
        """
        super().__init__('sensor_publisher')

        # Create publishers for different sensor types
        self.laser_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.battery_publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        self.odom_publisher = self.create_publisher(Float32, 'distance_traveled', 10)
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Create timer to publish sensor data at regular intervals
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

        # Robot state for simulation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 100.0
        self.distance_traveled = 0.0
        self.simulation_time = 0.0

        self.get_logger().info('Sensor publisher node initialized')

    def publish_sensor_data(self):
        """
        Publish all sensor data for the current simulation step.
        """
        # Update simulation time
        self.simulation_time += 0.1  # 0.1 seconds per call

        # Publish laser scan data
        self.publish_laser_scan()

        # Publish IMU data
        self.publish_imu_data()

        # Publish battery state
        self.publish_battery_state()

        # Publish odometry (distance traveled)
        self.publish_odom_data()

        # Publish robot status
        self.publish_status()

        # Update robot state based on simulated movement
        self.update_robot_state()

    def publish_laser_scan(self):
        """
        Publish simulated laser scan data.
        """
        msg = LaserScan()

        # Set laser scan parameters
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree increments
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Calculate number of ranges
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1

        # Generate simulated ranges with some obstacles
        ranges = []
        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Simulate some obstacles at specific angles
            distance = 5.0  # Default distance

            # Add some simulated obstacles
            if 0.5 < angle < 0.7:
                distance = 1.5  # Close obstacle on the right
            elif -0.3 < angle < 0.3:
                distance = 2.0  # Medium obstacle in front
            elif -0.8 < angle < -0.6:
                distance = 1.2  # Close obstacle on the left

            # Add some noise to simulate real sensor
            distance += random.uniform(-0.1, 0.1)
            distance = max(msg.range_min, min(msg.range_max, distance))

            ranges.append(distance)

        msg.ranges = ranges
        msg.intensities = []  # Empty intensities

        self.laser_publisher.publish(msg)

    def publish_imu_data(self):
        """
        Publish simulated IMU data.
        """
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_frame'

        # Simulate orientation (with some variation)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.robot_theta / 2)
        msg.orientation.w = math.cos(self.robot_theta / 2)

        # Simulate angular velocity (with some variation)
        msg.angular_velocity.x = random.uniform(-0.01, 0.01)
        msg.angular_velocity.y = random.uniform(-0.01, 0.01)
        msg.angular_velocity.z = random.uniform(-0.1, 0.1)

        # Simulate linear acceleration (with some variation)
        msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.z = 9.81 + random.uniform(-0.1, 0.1)  # Gravity

        self.imu_publisher.publish(msg)

    def publish_battery_state(self):
        """
        Publish simulated battery state.
        """
        msg = BatteryState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'battery_frame'

        # Simulate battery drain over time
        self.battery_level = max(0.0, self.battery_level - 0.01)

        msg.voltage = 12.6 - (12.6 - 11.0) * (100 - self.battery_level) / 100
        msg.temperature = 25.0 + random.uniform(-2, 2)
        msg.current = -1.0 + random.uniform(-0.5, 0.5)  # Negative means discharging
        msg.charge = self.battery_level
        msg.capacity = 10.0
        msg.design_capacity = 10.0
        msg.percentage = self.battery_level / 100.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        self.battery_publisher.publish(msg)

    def publish_odom_data(self):
        """
        Publish simulated odometry data (distance traveled).
        """
        msg = Float32()
        msg.data = self.distance_traveled
        self.odom_publisher.publish(msg)

    def publish_status(self):
        """
        Publish robot status information.
        """
        msg = String()
        msg.data = f"Pos:({self.robot_x:.2f},{self.robot_y:.2f}), Theta:{self.robot_theta:.2f}, Batt:{self.battery_level:.1f}%"
        self.status_publisher.publish(msg)

    def update_robot_state(self):
        """
        Update the simulated robot state based on movement.
        """
        # Simulate movement (this would normally come from motor feedback)
        linear_vel = 0.5 + random.uniform(-0.1, 0.1)  # Simulate movement
        angular_vel = random.uniform(-0.2, 0.2)      # Simulate turning

        dt = 0.1  # Time step (from timer interval)

        # Update position
        self.robot_x += linear_vel * math.cos(self.robot_theta) * dt
        self.robot_y += linear_vel * math.sin(self.robot_theta) * dt
        self.robot_theta += angular_vel * dt

        # Update distance traveled
        self.distance_traveled += linear_vel * dt

        # Keep theta in [-pi, pi] range
        self.robot_theta = math.atan2(
            math.sin(self.robot_theta),
            math.cos(self.robot_theta)
        )


def main(args=None):
    """
    Main function to initialize and run the sensor publisher node.

    Args:
        args: Command line arguments (typically None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the sensor publisher node
    sensor_publisher = SensorPublisherNode()

    # Keep the node running until interrupted
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()