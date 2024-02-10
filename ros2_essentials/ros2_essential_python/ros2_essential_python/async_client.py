# This file is part of the ros2_motion_python package.
#
# Copyright (c) 2023 Anis Koubaa. All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import sys  # Import the sys module for accessing command-line arguments.

from ros2_interfaces_cpp.srv import AddTwoInts  # Import the service interface.
import rclpy  # Import ROS client library for Python.
from rclpy.node import Node  # Import the Node class.


class MinimalClientAsync(Node):
    """
    A minimal asynchronous service client node in ROS 2.
    """

    def __init__(self):
        """
        Class constructor to set up the client node.
        """
        super().__init__('minimal_client_async')  # Initialize the node with the name 'minimal_client_async'.
        self.client = self.create_client(AddTwoInts, 'add_two_ints')  # Create a client for the 'add_two_ints' service.
        while not self.client.wait_for_service(timeout_sec=1.0):  # Wait for the service to be available.
            self.get_logger().info('Service not available, waiting again...')
        self.request = AddTwoInts.Request()  # Create a request object.

    def send_request(self, a, b):
        """
        Sends a request to the 'add_two_ints' service.

        :param a: The first integer to add.
        :param b: The second integer to add.
        :return: The result of the service call.
        """
        self.request.a = a  # Set the first integer in the request.
        self.request.b = b  # Set the second integer in the request.
        self.get_logger().info('Sending request to the server.')  # Log that the request is being sent.
        self.future = self.client.call_async(self.request)  # Call the service asynchronously.
        self.get_logger().info('Waiting to receive the response...')  # Log that the client is waiting for a response.
        rclpy.spin_until_future_complete(self, self.future)  # Wait for the response.
        self.get_logger().info('Service completed.')  # Log that the service call is complete.
        return self.future.result()  # Return the result of the service call.


def main(args=None):
    """
    Main function to execute the service client.
    """
    rclpy.init(args=args)  # Initialize the ROS client library.

    minimal_client = MinimalClientAsync()  # Create an instance of the client node.
    if len(sys.argv) == 3:
        response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))  # Send a request with command-line arguments.
        minimal_client.get_logger().info(
            f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}'
        )  # Log the result of the service call.
    else:
        minimal_client.get_logger().error('Insufficient command-line arguments provided. Please specify two integers.')

    minimal_client.destroy_node()  # Destroy the node explicitly.
    rclpy.shutdown()  # Shutdown the ROS client library.


if __name__ == '__main__':
    main()  # Execute the main function.
