#!/usr/bin/env python3
"""
Gazebo-Unity Bridge Node

This ROS 2 node facilitates communication between Gazebo simulation and Unity visualization,
enabling real-time synchronization of robot states, sensor data, and control commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Time

import numpy as np
import socket
import json
import threading
import time
from collections import deque


class GazeboUnityBridge(Node):
    """
    Bridge node that synchronizes data between Gazebo simulation and Unity visualization.
    """

    def __init__(self):
        super().__init__('gazebo_unity_bridge')

        # Bridge configuration
        self.unity_ip = self.declare_parameter('unity_ip', '127.0.0.1').value
        self.unity_port = self.declare_parameter('unity_port', 5555).value
        self.ros_publish_rate = self.declare_parameter('publish_rate', 30).value  # Hz

        # Socket for Unity communication
        self.unity_socket = None
        self.connect_to_unity()

        # Robot state storage
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

        # Sensor data storage
        self.lidar_data = None
        self.imu_data = None
        self.camera_data = None

        # Unity command storage
        self.unity_commands = deque(maxlen=10)

        # ROS 2 QoS profile for real-time communication
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ROS 2 subscribers for Gazebo data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/basic_humanoid/joint_states',
            self.joint_state_callback,
            qos_profile
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/basic_humanoid/scan',
            self.lidar_callback,
            qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/basic_humanoid/imu',
            self.imu_callback,
            qos_profile
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/basic_humanoid/odom',
            self.odom_callback,
            qos_profile
        )

        # ROS 2 publishers for Unity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/basic_humanoid/cmd_vel',
            10
        )

        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/basic_humanoid/joint_commands',
            10
        )

        # Timer for sending data to Unity
        self.bridge_timer = self.create_timer(1.0/self.ros_publish_rate, self.send_to_unity)

        # Timer for processing Unity commands
        self.command_timer = self.create_timer(0.01, self.process_unity_commands)

        self.get_logger().info(f'Gazebo-Unity Bridge initialized. Connecting to Unity at {self.unity_ip}:{self.unity_port}')

    def connect_to_unity(self):
        """Establish connection to Unity application."""
        try:
            self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_socket.connect((self.unity_ip, self.unity_port))
            self.unity_socket.settimeout(0.1)  # Non-blocking with timeout
            self.get_logger().info(f'Connected to Unity at {self.unity_ip}:{self.unity_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Unity: {e}')
            self.unity_socket = None

    def joint_state_callback(self, msg):
        """Process joint state messages from Gazebo."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_efforts[name] = msg.effort[i]

    def lidar_callback(self, msg):
        """Process LiDAR data from Gazebo."""
        self.lidar_data = {
            'ranges': msg.ranges[:min(360, len(msg.ranges))],  # Take first 360 if available
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            }
        }

    def imu_callback(self, msg):
        """Process IMU data from Gazebo."""
        self.imu_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            }
        }

    def odom_callback(self, msg):
        """Process odometry data from Gazebo."""
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w
        }

    def send_to_unity(self):
        """Send robot state and sensor data to Unity."""
        if not self.unity_socket:
            return

        try:
            # Prepare data packet for Unity
            data_packet = {
                'timestamp': time.time(),
                'robot_state': {
                    'pose': self.robot_pose,
                    'joints': {name: pos for name, pos in self.joint_positions.items()}
                },
                'sensors': {
                    'lidar': self.lidar_data,
                    'imu': self.imu_data
                }
            }

            # Convert to JSON and send
            json_data = json.dumps(data_packet)
            self.unity_socket.send(json_data.encode('utf-8') + b'\n')

        except socket.error as e:
            self.get_logger().warning(f'Socket error when sending to Unity: {e}')
            # Attempt to reconnect
            self.connect_to_unity()
        except Exception as e:
            self.get_logger().error(f'Error sending data to Unity: {e}')

    def process_unity_commands(self):
        """Process commands received from Unity."""
        if not self.unity_socket:
            return

        try:
            # Receive data from Unity (non-blocking)
            self.unity_socket.settimeout(0.001)  # Very short timeout for non-blocking
            data = self.unity_socket.recv(1024)

            if data:
                try:
                    cmd_data = json.loads(data.decode('utf-8'))
                    self.unity_commands.append(cmd_data)

                    # Process different types of commands
                    if 'cmd_vel' in cmd_data:
                        self.publish_velocity_command(cmd_data['cmd_vel'])
                    elif 'joint_commands' in cmd_data:
                        self.publish_joint_commands(cmd_data['joint_commands'])

                except json.JSONDecodeError:
                    self.get_logger().warning('Received invalid JSON from Unity')

        except socket.timeout:
            # This is expected when no data is available
            pass
        except socket.error as e:
            self.get_logger().warning(f'Socket error when receiving from Unity: {e}')
            # Attempt to reconnect
            self.connect_to_unity()
        except Exception as e:
            self.get_logger().error(f'Error processing Unity commands: {e}')

    def publish_velocity_command(self, cmd_vel):
        """Publish velocity command to ROS 2."""
        msg = Twist()
        msg.linear.x = cmd_vel.get('linear_x', 0.0)
        msg.linear.y = cmd_vel.get('linear_y', 0.0)
        msg.linear.z = cmd_vel.get('linear_z', 0.0)
        msg.angular.x = cmd_vel.get('angular_x', 0.0)
        msg.angular.y = cmd_vel.get('angular_y', 0.0)
        msg.angular.z = cmd_vel.get('angular_z', 0.0)

        self.cmd_vel_pub.publish(msg)

    def publish_joint_commands(self, joint_commands):
        """Publish joint commands to ROS 2."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        for joint_name, joint_cmd in joint_commands.items():
            msg.name.append(joint_name)
            msg.position.append(joint_cmd.get('position', 0.0))
            msg.velocity.append(joint_cmd.get('velocity', 0.0))
            msg.effort.append(joint_cmd.get('effort', 0.0))

        self.joint_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bridge_node = GazeboUnityBridge()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Bridge node interrupted by user')
    finally:
        if bridge_node.unity_socket:
            bridge_node.unity_socket.close()
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()