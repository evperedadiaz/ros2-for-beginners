#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.msg import TurtleData
from my_robot_interfaces.msg import TurtleList
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import random
import math

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        self.declare_parameter("turtle_spawn_frequency", 1.0)
        self.turtle_spawn_frequency = self.get_parameter("turtle_spawn_frequency").value

        self.create_service(CatchTurtle, "catch_turtle", self.catchTurtle)
        self.get_logger().info("catch_turtle service started")
        self.spawnClient = self.create_client(Spawn, "spawn")
        self.killClient = self.create_client(Kill, "kill")
        self.alive_turtles = {}
        self.spawnTimer = self.create_timer(self.turtle_spawn_frequency, self.spawnTurtle)

        self.alive_turtles_publisher = self.create_publisher(TurtleList, "alive_turtles", 10)

    def spawnTurtle(self):
        spawnRequest = Spawn.Request()
        spawnRequest.x = round(random.uniform(0.00, 11.0), 2)
        spawnRequest.y = round(random.uniform(0.00, 11.0), 2)
        spawnRequest.theta = round(random.uniform(0, 2 * math.pi), 2)
        # todo: almacenar la request junto al nombre de la tortuga. Se podría utilizar una estructura similar a un json, que las keys sean los nombres de las tortugas y los valores las coordenadas.

        while not self.spawnClient.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service /spawn")

        future = self.spawnClient.call_async(spawnRequest)
        future.add_done_callback(partial(self.proccess_spawn_response, spawnRequest=spawnRequest))

    def proccess_spawn_response(self, future, spawnRequest):
        try:
            response = future.result()

            turtle_data = TurtleData()
            turtle_data.turtle_name = response.name
            turtle_data.x = spawnRequest.x
            turtle_data.y = spawnRequest.y

            self.alive_turtles[response.name] = turtle_data
            self.publish_alive_turtles()
            self.get_logger().info("Turtle added: " + response.name + " -> " + str(self.alive_turtles))
        except Exception as e:
            self.get_logger().error("Error getting response from service /spawn: %r" % (e,))

    def publish_alive_turtles(self):
        turtule_list = TurtleList()
        for turtle_data in self.alive_turtles.values():
            turtule_list.turtle_list.append(turtle_data)
        self.alive_turtles_publisher.publish(turtule_list)

    def catchTurtle(self, request, response):
        turtleName = request.turtle_name.lower()
        try:
            turtle_data = self.alive_turtles[turtleName]
        except KeyError:
            response.success = False
            response.error = "The turtle " + turtleName + " isn't in the list of alive Turtles"
            return response

        killRequest = Kill.Request()
        killRequest.name = turtleName

        while not self.killClient.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service /spawn")

        future = self.killClient.call_async(killRequest)
        future.add_done_callback(partial(self.proccess_kill_response, turtleName=turtleName))
        response.success = True

        return response
    
    def proccess_kill_response(self, future, turtleName):
        try:
            killResponse = future.result()
            self.get_logger().info(f"Killed turtle: {turtleName}")
        except Exception as e:
            self.get_logger().error(f"Request to /kill failed: {e}")
            self.get_logger().info("Turtle list: " + str(self.alive_turtles))
            return

        try:
            del self.alive_turtles[turtleName]
        except KeyError:
            self.get_logger().info("Turtle already deleted: " + str(turtleName))
        self.publish_alive_turtles()
        
        self.get_logger().info("Turtle list: " + str(self.alive_turtles))


def main(args = None):
    rclpy.init(args = args)
    node = TurtleSpawner()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()