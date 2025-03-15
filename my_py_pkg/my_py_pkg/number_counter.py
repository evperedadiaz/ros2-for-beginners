#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from example_interfaces.msg import String
from std_msgs.msg import Int64

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.total = 0
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.subscriber = self.create_subscription(Int64, "number", self.getNumberCallback, 10)
        self.get_logger().info("NumberCounterNode has been started")

    def getNumberCallback(self, msgIn):
        self.total += msgIn.data
        msgOut = Int64()
        msgOut.data = self.total
        self.publisher.publish(msgOut)


def main(args = None):
    rclpy.init(args = args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()