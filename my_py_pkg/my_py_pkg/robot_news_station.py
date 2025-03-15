#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces.msg import String
from std_msgs.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        self.declare_parameter("robot_name", "C3P2")

        self.counter = 0
        self.robot_name = self.get_parameter("robot_name").value
        self.publisher = self.create_publisher(String, "robot_news", 10)
        self.timer = self.create_timer(5.0, self.publish_news)
        self.get_logger().info("Robot News Station has been started")

    def publish_news(self):
        self.counter += 1
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name) + " from the robot news station - " + str(self.counter)
        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()