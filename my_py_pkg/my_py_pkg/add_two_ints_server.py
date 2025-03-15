#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.create_service(AddTwoInts, "add_two_ints", self.addTwoIntsCallback)
        self.get_logger().info("AddTwoInts started")

    def addTwoIntsCallback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response

def main(args = None):
    rclpy.init(args = args)

    # node = Node("py_test")
    #node.get_logger().info("Hello Ros2")S
    node = AddTwoIntsServerNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()