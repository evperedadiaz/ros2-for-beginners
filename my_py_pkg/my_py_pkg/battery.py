#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial
import threading

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.get_logger().info("Hardware Status Publisher has been started")
        #time.sleep(4.0)
        threading.Timer(4.0, self.send_battery_uncharged).start()
        
    def send_battery_charged(self):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service set_led")
        request = SetLed.Request()
        request.led_number = 3
        request.state = 'on'
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_led_on))
        threading.Timer(4.0, self.send_battery_uncharged).start()

    def send_battery_uncharged(self):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service set_led")
        request = SetLed.Request()
        request.led_number = 3
        request.state = 'off'
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_led_off))
        threading.Timer(6.0, self.send_battery_charged).start()

    def callback_set_led_on(self, future):
        try:
            response = future.result()
            self.get_logger().info("response from service set_led setting on: " + str(response.success))
        except Exception as e:
            self.get_logger().error("Error getting response from service set_led setting on: %r" % (e,))

    def callback_set_led_off(self, future):
        try:
            response = future.result()
            self.get_logger().info("response from service set_led setting off: " + str(response.success))
        except Exception as e:
            self.get_logger().error("Error getting response from service set_led setting off: %r" % (e,))  

def main(args = None):
    rclpy.init(args = args)

    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()