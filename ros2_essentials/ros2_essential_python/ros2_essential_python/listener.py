# This file is part of the ros2_motion_python package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

# Import the rclpy library to enable ROS2 functionalities in Python
import rclpy
# Import the Node class from rclpy.node, which is essential for creating ROS2 nodes
from rclpy.node import Node
# Import the String message type from std_msgs.msg, used for sending text messages
from std_msgs.msg import String


class ListenerSubscriber(Node):
    """
    A ROS2 Node for subscribing to a topic and listening to incoming messages.

    Attributes:
        subscription: A subscription object for ROS2 messages.
    """
    
    def __init__(self):
        """
        Initialize the ListenerSubscriber node with a subscription to the 'chatter' topic.
        """
        # Initialize the node with the name 'listen_subscribe'
        super().__init__('listen_subscribe')
        # Create a subscription to the 'chatter' topic with String messages and a callback function
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listen_callback,
            10
        )
        # This line avoids a warning about the unused variable 'subscription'
        self.subscription

    def listen_callback(self, msg):
        """
        Callback function for processing incoming messages.

        Args:
            msg: The incoming message object.
        """
        # Log the received message using the node's logger
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    
    Args:
        args: Optional arguments passed to rclpy.init.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    # Create an instance of the ListenerSubscriber node
    listen_subscribe = ListenerSubscriber()
    # Keep the node running and listening for incoming messages
    rclpy.spin(listen_subscribe)
    # Explicitly destroy the node before shutting down rclpy
    listen_subscribe.destroy_node()
    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
