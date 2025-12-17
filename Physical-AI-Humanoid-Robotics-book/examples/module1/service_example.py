#!/usr/bin/env python3
"""
Service Server and Client Example

This example demonstrates both a service server and client in the same file
for educational purposes. In practice, these would typically be in separate files.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServer(Node):
    """
    A service server that adds two integers.
    """

    def __init__(self):
        """
        Initialize the service server node.
        """
        super().__init__('service_server')

        # Create a service server
        # Service name: 'add_two_ints'
        # Service type: AddTwoInts (built-in example interface)
        # Callback function: add_two_ints_callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server initialized')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for handling service requests.

        Args:
            request: The service request containing two integers (a and b)
            response: The service response to be filled with the sum

        Returns:
            The response object with the calculated sum
        """
        # Calculate the sum
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(
            f'Incoming request - a: {request.a}, b: {request.b}, sum: {response.sum}'
        )

        # Return the response
        return response


class ServiceClient(Node):
    """
    A service client that requests addition of two integers.
    """

    def __init__(self):
        """
        Initialize the service client node.
        """
        super().__init__('service_client')

        # Create a service client
        # Service name: 'add_two_ints'
        # Service type: AddTwoInts (must match the server)
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to add two integers.

        Args:
            a: First integer
            b: Second integer

        Returns:
            The response from the service
        """
        # Set the request values
        self.request.a = a
        self.request.b = b

        # Send the request asynchronously
        self.future = self.cli.call_async(self.request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, self.future)

        # Return the response
        return self.future.result()


def run_server():
    """
    Function to run the service server.
    """
    rclpy.init()

    service_server = ServiceServer()

    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass
    finally:
        service_server.destroy_node()
        rclpy.shutdown()


def run_client():
    """
    Function to run the service client.
    """
    rclpy.init()

    service_client = ServiceClient()

    # Send a request to add 4 and 2
    response = service_client.send_request(4, 2)

    # Log the result
    service_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}'
    )

    # Clean up
    service_client.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """
    Main function demonstrating both server and client usage.
    In practice, these would run in separate processes.
    """
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == 'server':
            print("Running as server...")
            run_server()
        elif sys.argv[1] == 'client':
            print("Running as client...")
            run_client()
        else:
            print("Usage: python service_example.py [server|client]")
    else:
        # For demonstration, show how to use both
        print("This example shows both server and client code.")
        print("Run as server: python service_example.py server")
        print("Run as client: python service_example.py client")


if __name__ == '__main__':
    main()