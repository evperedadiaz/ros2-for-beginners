from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_names = ["Giskard", "BB8", "Lander", "Daneel", "C3PO"]
    robot_news_station_nodes = []

    for name in robot_names:
        robot_news_station_nodes.append(
            Node(
                package="my_py_pkg",
                executable="robot_news_station",
                name="robot_news_station_" + name.lower(),
                parameters=[
                    {"robot_name": name}
                ]
            )
        )

    robot_news_station_nodes.append(
        Node(
            package="my_cpp_pkg",
            executable="smartphone",
            name="smartphone"
        )
    )

    ld = LaunchDescription(robot_news_station_nodes)
    
    return ld
