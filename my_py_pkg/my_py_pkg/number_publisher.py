#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces.msg import String
from std_msgs.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        self.declare_parameter("number_to_publish", 123)
        self.declare_parameter("frequency", 1.0)

        self.number = self.get_parameter("number_to_publish").value
        self.frequency = self.get_parameter("frequency").value
        self.publisher = self.create_publisher(Int64, "number", 10)
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_number)
        self.get_logger().info("Robot News Station has been started")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number
        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()