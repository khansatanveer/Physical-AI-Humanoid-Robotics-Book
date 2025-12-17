#!/usr/bin/env python3
"""
Basic Publisher Example

This is a simple ROS 2 publisher node that demonstrates the fundamental
concepts of creating a publisher and sending messages on a topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node that publishes messages to a topic.
    """

    def __init__(self):
        """
        Initialize the publisher node.
        """
        super().__init__('minimal_publisher')

        # Create a publisher for the 'topic' topic with String message type
        # The queue size is set to 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer to publish messages every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.i = 0

        # Log that the publisher has started
        self.get_logger().info('Minimal publisher node initialized')

    def timer_callback(self):
        """
        Callback function that is called by the timer.
        Creates and publishes a message with the current counter value.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the publisher node.

    Args:
        args: Command line arguments (typically None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the publisher node
    minimal_publisher = MinimalPublisher()

    # Keep the node running until interrupted
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()