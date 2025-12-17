#!/usr/bin/env python3
"""
Basic Subscriber Example

This is a simple ROS 2 subscriber node that demonstrates the fundamental
concepts of creating a subscriber and receiving messages from a topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node that subscribes to messages from a topic.
    """

    def __init__(self):
        """
        Initialize the subscriber node.
        """
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'topic' topic with String message type
        # The queue size is set to 10
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription  # This line prevents the linter from warning about unused variable

        # Log that the subscriber has started
        self.get_logger().info('Minimal subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received.

        Args:
            msg: The received message (String type)
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the subscriber node.

    Args:
        args: Command line arguments (typically None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the subscriber node
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running until interrupted
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()