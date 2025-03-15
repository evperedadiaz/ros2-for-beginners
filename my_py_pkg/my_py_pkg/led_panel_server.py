#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedPanelState

class LedPanelServerNode(Node):
    def __init__(self):
        super().__init__("set_led_server")
        self.declare_parameter("led_state", [0, 0, 0])
        self.panel_state = self.get_parameter("led_state").value
        self.create_service(SetLed, "set_led", self.setLedCallback)
        self.publisher = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.get_logger().info("LedPanelServerNode started")

    def setLedCallback(self, request, response):
        self.get_logger().info("Led number: " + str(request.led_number) + ", state: " + request.state)
        # todo: enviar al topic led_panel_state el estado de los 3 leds.
        self.panel_state[request.led_number - 1] = 1 if request.state == 'on' else 0
        ledPanelStateMsg = LedPanelState()
        ledPanelStateMsg.led_state = self.panel_state
        self.publisher.publish(ledPanelStateMsg)
        response.success = True
        return response

def main(args = None):
    rclpy.init(args = args)

    node = LedPanelServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()