#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces.msg import String
from std_msgs.msg import Int64
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.publisher = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Hardware Status Publisher has been started")

    def publish_status(self):
        msg = HardwareStatus()
        msg.temperature = 54
        msg.are_motors_ready = True
        msg.degug_message = "Python publisher is here"
        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()