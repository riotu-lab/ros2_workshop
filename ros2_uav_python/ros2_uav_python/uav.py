import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
import time
import sys

class UAVControlNode(Node):
    def __init__(self):
        super().__init__('uav_control_node')
        # Declare parameters
        self.declare_parameter('takeoff_height', 5.0)
        self.takeoff_height = self.get_parameter('takeoff_height').get_parameter_value().double_value
        
        # Publishers and Clients
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        
        # Wait for services to become available
        self.wait_for_services()

    def wait_for_services(self):
        self.get_logger().info('Waiting for ROS services...')
        self.arm_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()
        self.get_logger().info('All services are available.')

    def arm(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('UAV arming successful.')
            else:
                self.get_logger().info('UAV arming failed.')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

    def takeoff(self):
        request = CommandTOL.Request()
        request.altitude = 10.0#self.takeoff_height
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Takeoff successful.')
            else:
                self.get_logger().info('Takeoff failed.')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

    def land(self):
        request = CommandTOL.Request()
        future = self.land_client.call_async(request)
        future.add_done_callback(self.land_callback)

    def land_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Landing successful.')
            else:
                self.get_logger().info('Landing failed.')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

    def go_to_location(self, x, y, z):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_pub.publish(pose)
        self.get_logger().info('Publishing new position.')

def print_menu():
    print("\nMenu:")
    print("1: Arm UAV")
    print("2: Takeoff")
    print("3: Land")
    print("4: Go to a Location")
    print("5: Exit")

def main(args=None):
    rclpy.init(args=args)
    uav_control_node = UAVControlNode()

    while True:
        print_menu()
        choice = input("Enter your choice: ")

        if choice == '1':
            uav_control_node.arm()
        elif choice == '2':
            uav_control_node.takeoff()
        elif choice == '3':
            uav_control_node.land()
        elif choice == '4':
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            z = float(input("Enter Z altitude: "))
            uav_control_node.go_to_location(x, y, z)
        elif choice == '5':
            break
        else:
            print("Invalid choice. Please try again.")

        rclpy.spin_once(uav_control_node, timeout_sec=0.1)

    uav_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
