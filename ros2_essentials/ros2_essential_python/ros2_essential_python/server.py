# This file is part of the ros2_motion_python package.
#
# Copyright (c) 2023 Anis Koubaa. All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

# Import the AddTwoInts service definition from the ros2_interfaces_cpp package
from ros2_interfaces_cpp.srv import AddTwoInts
# Import the ROS client library for Python
import rclpy
# Import the Node class from the ROS client library
from rclpy.node import Node
# Import the sleep function from the time module for delays
from time import sleep

class MinimalService(Node):
    """
    Define a MinimalService class, which is a ROS 2 node offering an addition service.
    """

    def __init__(self):
        """
        Class constructor to set up the node and service.
        """
        # Initialize the parent class (Node) with the node name 'minimal_service'
        super().__init__('minimal_service')
        # Create a service that will use the AddTwoInts service definition
        # The 'add_two_ints' is the service name, and add_two_ints_callback is the callback function
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        # Print a message indicating the service is ready and waiting for requests
        self.get_logger().info('Service is waiting for requests...')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for the add_two_ints service.

        :param request: The request message.
        :param response: The response message.
        :return: The response message with the sum of request.a and request.b.
        """
        # Perform the addition operation and store the result in the response
        response.sum = request.a + request.b
        # Delay the response for demonstration purposes
        sleep(5)
        # Log the received request
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        # Indicate that the response is being sent back
        self.get_logger().info('Sending response and waiting for new requests...')
        return response

def main(args=None):
    """
    Entry point for the service server program.
    """
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of the MinimalService node
    minimal_service = MinimalService()
    # Keep the node alive to listen for incoming service requests
    rclpy.spin(minimal_service)
    # Shutdown the ROS client library upon termination
    rclpy.shutdown()

if __name__ == '__main__':
    main()
