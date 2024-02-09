# This file is part of the ros2_motion_python package.
#
# Copyright (c) 2023 Anis Koubaa. All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

# Import the ROS client library for Python
import rclpy
# Import the Node class from the ROS client library
from rclpy.node import Node
# Import the String message type from the standard ROS message library
from std_msgs.msg import String


class TalkerPublisher(Node):
    """A simple ROS 2 node that publishes 'Hello World' messages."""

    def __init__(self):
        """
        Class constructor to set up the node.

        :param None
        """
        # Initialize the parent class (Node) with the node name 'talker_publisher'
        super().__init__('talker_publisher')
        # Create a publisher object to send messages of type String on the 'chatter' topic
        self.publisher = self.create_publisher(String, 'chatter', 10)
        # Define the time period for the timer callback in seconds
        timer_period = 2  # messages will be published at 0.5Hz
        # Create a timer that calls the timer_callback method periodically
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize a counter to keep track of the number of messages sent
        self.counter = 0

    def timer_callback(self):
        """
        Callback function for the timer.

        :param None
        """
        # Create a new String message
        msg = String()
        # Assign the message data with the current value of the counter
        msg.data = 'Hello World: %d' % self.counter
        # Publish the message on the 'chatter' topic
        self.publisher.publish(msg)
        # Log the published message to the console
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # Increment the counter
        self.counter += 1


def main(args=None):
    """
    Entry point for the program.

    :param args: Command-line arguments
    """
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of the TalkerPublisher node
    talker_publisher = TalkerPublisher()
    # Keep the node alive until it's manually stopped or Ctrl+C is pressed
    rclpy.spin(talker_publisher)
    # Destroy the node explicitly
    talker_publisher.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
