#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.msg import TurtleData
from my_robot_interfaces.msg import TurtleList
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from functools import partial
import random
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.declare_parameter("check_next_objective_frequency", 0.1)
        self.check_next_objective_frequency = self.get_parameter("check_next_objective_frequency").value

        self.declare_parameter("distance_tolerance", 0.3)
        self.distance_tolerance = self.get_parameter("distance_tolerance").value

        # Parámetros de control (ganancias) para el movimiento.
        self.declare_parameter("linear_speed_gain", 2.0) # Ganancia para la velocidad lineal.
        self.linear_speed_gain = self.get_parameter("linear_speed_gain").value
        self.declare_parameter("angular_speed_gain", 6.0) # Ganancia para la velocidad angular.
        self.angular_speed_gain = self.get_parameter("angular_speed_gain").value
        
        
        self.check_next_objective_frequency = self.get_parameter("check_next_objective_frequency").value

        self.turtle_list = TurtleList().turtle_list
        self.master_turtle_position = Pose()
        self.next_objective = None

        self.alive_turtle_subscriber = self.create_subscription(TurtleList, "alive_turtles", self.callback_alive_turtles, 10)
        self.master_turtle_subscriber = self.create_subscription(Pose, "turtle1/pose", self.master_turtle_pose_callback, 10)
        
        # Publicador para enviar comandos de velocidad a la tortuga.
        # Se publicarán mensajes de tipo Twist en el tópico 'turtle1/cmd_vel'.
        self.master_cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        self.catch_turtle_client = self.create_client(CatchTurtle, "catch_turtle")

        self.check_next_objective_timer = self.create_timer(self.check_next_objective_frequency, self.check_next_objective_callback)

        self.get_logger().info("Node turtle_controller started")

    def callback_alive_turtles(self, turte_list_msg):
        self.turtle_list = turte_list_msg.turtle_list
        self.get_logger().info(f"turte_list: {turte_list_msg.turtle_list}")
    
    def master_turtle_pose_callback(self, master_turtle_pose_msg):
        self.master_turtle_position = master_turtle_pose_msg
        #self.get_logger().info(f"master pose: {master_turtle_pose_msg}")

    def check_next_objective_callback(self):
        # Si ya no hay más objetivos, se detiene la tortuga.
        if len(self.turtle_list) == 0:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.master_cmd_vel_pub.publish(twist)
            self.get_logger().info('Todos los objetivos han sido alcanzados. Deteniendo la tortuga.')
            return

        minor_euclidean_dist = None
        next_objective = None
        
        for index, turtle_data in enumerate(self.turtle_list):
            euclidean_dist = self.euclidean_distance(self.master_turtle_position.x, self.master_turtle_position.y, turtle_data.x, turtle_data.y)
            if minor_euclidean_dist == None or minor_euclidean_dist > euclidean_dist:
                minor_euclidean_dist = euclidean_dist
                next_objective = turtle_data
        
        self.next_objective = next_objective # self.next_objective vale para poder tener en otro temporizador el movimiento hacia el objetivo

        # Si la tortuga está lo suficientemente cerca del objetivo (dentro de la tolerancia),
        # se considera que ha alcanzado dicho objetivo.
        if minor_euclidean_dist < self.distance_tolerance:
            self.get_logger().info(
                f'Objetivo {self.next_objective.turtle_name} alcanzado en ({self.next_objective.x}, {self.next_objective.y})'
            )
            # todo: capturar la cartuga
            self.catch_turtle(self.next_objective.turtle_name)
            return

        # todo:
        # - la aceleración y la desceleración la calculo luego. 
        # - calcular el ángulo al que tiene que girar la master para estar en dirección del next_objective
        # - 
        angle_correction = self.angle_correction()

        # Creamos un mensaje Twist para enviar el comando de velocidad.
        twist = Twist()
        # La velocidad lineal se define proporcional a la distancia al objetivo.
        twist.linear.x = self.linear_speed_gain * minor_euclidean_dist
        # La velocidad angular se define proporcional al error angular.
        twist.angular.z = self.angular_speed_gain * angle_correction

        # Publicamos el comando de velocidad.
        self.master_cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f'Moviendo hacia objetivo {self.next_objective.turtle_name}: ({self.next_objective.x}, {self.next_objective.y}) | '
            f'Distancia: {minor_euclidean_dist:.2f} | Error angular: {angle_correction:.2f}'
        )

    def catch_turtle(self, turtle_name):
        catch_turtle_request = CatchTurtle.Request()
        catch_turtle_request.turtle_name = turtle_name

        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service /catch_turtle")

        future = self.catch_turtle_client.call_async(catch_turtle_request)
        future.add_done_callback(partial(self.proccess_catch_turtle_response, turtle_name=turtle_name))
    
    def proccess_catch_turtle_response(self, future, turtle_name):
        try:
            catch_turtle_response = future.result()
            self.get_logger().info(f"turtle catched: {turtle_name}")
        except Exception as e:
            self.get_logger().error(f"Request to /catch_turtle failed: {e}")

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def angle_correction(self):
        # Calculamos el ángulo deseado para dirigirnos hacia el objetivo.
        # Usamos la función arctan2 para obtener el ángulo entre la línea que une la posición actual con el objetivo.
        desired_angle = math.atan2(self.next_objective.y - self.master_turtle_position.y, self.next_objective.x - self.master_turtle_position.x)
        # Calculamos el error angular (diferencia entre el ángulo deseado y la orientación actual).
        angle_error = desired_angle - self.master_turtle_position.theta
        # Normalizamos el error angular para que se encuentre en el rango [-pi, pi].
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        return angle_error


def main(args = None):
    rclpy.init(args = args)
    node = TurtleController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()