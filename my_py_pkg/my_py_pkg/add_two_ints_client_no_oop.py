#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args = None):
    rclpy.init(args = args)

    node = Node("add_two_ints_client_no_oop")
    client = node.create_client(AddTwoInts, "add_two_ints")
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for service add_two_ints")
    request = AddTwoInts.Request()
    request.a = 1
    request.b = 3
    #response
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    try:
        response = future.result()
        node.get_logger().info("response from service add_two_ints: " + str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
    except Exception as e:
        node.get_logger().error("Error getting response from service add_two_ints: %r" % (e,))    

    rclpy.shutdown()

if __name__ == "__main__":
    main()