#!/usr/bin/env python3
"""
Isaac ROS VSLAM Basics Example
Demonstrates fundamental VSLAM concepts using Isaac ROS components
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from tf2_ros import TransformBroadcaster
import tf_transformations


class VSLAMBasicNode(Node):
    """
    Basic VSLAM node demonstrating the core concepts of visual SLAM
    Note: This is a conceptual example. A full Isaac ROS VSLAM implementation
    would use the actual Isaac ROS packages which require Isaac Sim.
    """

    def __init__(self):
        super().__init__('vslam_basic_node')

        # Create CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Publishers for VSLAM outputs
        self.pose_pub = self.create_publisher(PoseStamped, '/vslam/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vslam/odometry', 10)

        # Subscribers for camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Store camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Store previous image for feature tracking
        self.prev_image = None
        self.prev_features = None

        # Robot pose tracking
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion

        # Feature tracking parameters
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )

        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        self.get_logger().info("VSLAM Basic Node initialized")

    def camera_info_callback(self, msg):
        """Callback for camera info messages"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info("Camera parameters received")

    def image_callback(self, msg):
        """Callback for image messages"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process the image for VSLAM
            self.process_vslam_frame(cv_image, msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_vslam_frame(self, current_image, timestamp):
        """Process a single frame for VSLAM"""
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)

        if self.prev_image is None:
            # First frame - detect initial features
            self.prev_features = cv2.goodFeaturesToTrack(
                gray,
                mask=None,
                **self.feature_params
            )
            self.prev_image = gray.copy()
            return

        # Track features from previous frame to current frame
        if self.prev_features is not None and len(self.prev_features) > 0:
            # Calculate optical flow
            new_features, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_image, gray,
                self.prev_features, None,
                **self.lk_params
            )

            # Filter out bad matches
            good_new = new_features[status == 1]
            good_old = self.prev_features[status == 1]

            if len(good_new) >= 10:  # Need enough features for pose estimation
                # Estimate motion from feature correspondences
                motion_estimate = self.estimate_motion(good_old, good_new)

                if motion_estimate is not None:
                    # Update robot pose based on estimated motion
                    self.update_robot_pose(motion_estimate)

                    # Publish pose and odometry
                    self.publish_pose_and_odom(timestamp)

            # Update features for next iteration
            self.prev_features = good_new.reshape(-1, 1, 2)

        # Store current image for next iteration
        self.prev_image = gray.copy()

        # Visualize tracked features
        self.visualize_features(current_image, self.prev_features)

    def estimate_motion(self, old_points, new_points):
        """Estimate camera/robot motion from feature correspondences"""
        if len(old_points) < 8 or len(new_points) < 8:
            return None

        # Estimate essential matrix to get relative pose
        essential_matrix, mask = cv2.findEssentialMat(
            new_points, old_points,
            self.camera_matrix,
            method=cv2.RANSAC,
            threshold=1.0,
            prob=0.999
        )

        if essential_matrix is not None:
            # Decompose essential matrix to get rotation and translation
            _, rotation, translation, _ = cv2.recoverPose(
                essential_matrix, new_points, old_points,
                self.camera_matrix
            )

            # Convert to transformation matrix
            transformation = np.eye(4)
            transformation[:3, :3] = rotation
            transformation[:3, 3] = translation.flatten()

            return transformation

        return None

    def update_robot_pose(self, motion):
        """Update robot pose based on estimated motion"""
        # Convert rotation matrix to quaternion
        rotation_matrix = motion[:3, :3]
        quaternion = tf_transformations.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]])
        )

        # Update position
        self.robot_position += motion[:3, 3]

        # Update orientation
        self.robot_orientation = quaternion

    def publish_pose_and_odom(self, timestamp):
        """Publish pose and odometry messages"""
        # Create and publish pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(self.robot_position[0])
        pose_msg.pose.position.y = float(self.robot_position[1])
        pose_msg.pose.position.z = float(self.robot_position[2])
        pose_msg.pose.orientation.x = float(self.robot_orientation[0])
        pose_msg.pose.orientation.y = float(self.robot_orientation[1])
        pose_msg.pose.orientation.z = float(self.robot_orientation[2])
        pose_msg.pose.orientation.w = float(self.robot_orientation[3])

        self.pose_pub.publish(pose_msg)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose_msg.pose
        # Velocity would be estimated from pose changes over time

        self.odom_pub.publish(odom_msg)

    def visualize_features(self, image, features):
        """Visualize tracked features on the image"""
        if features is not None:
            for feature in features:
                x, y = feature.ravel()
                cv2.circle(image, (int(x), int(y)), 3, (0, 255, 0), -1)

        # Display the image (in a real implementation, this might be published)
        cv2.imshow('VSLAM Features', image)
        cv2.waitKey(1)


def main(args=None):
    """Main function to run the VSLAM basic example"""
    rclpy.init(args=args)

    vslam_node = VSLAMBasicNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()

        # Close OpenCV windows
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()