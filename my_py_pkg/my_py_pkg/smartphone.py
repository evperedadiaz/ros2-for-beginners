#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces.msg import String
from std_msgs.msg import String

class SmartponeNode(Node):
    def __init__(self):
        super().__init__("smartpone")
        
        self.subscriber = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone has been started")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)

def main(args = None):
    rclpy.init(args = args)
    node = SmartponeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()