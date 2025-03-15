from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtle_spawner_node = Node(
        package="turtlesim_catch_them_all",
        executable="turtle_spawner",
        name="turtle_spawner",
        remappings=[
        ],
        parameters=[
            {"turtle_spawn_frequency": 1.0},
        ]
    )

    turtle_controller_node = Node(
        package="turtlesim_catch_them_all",
        executable="turtle_controller",
        name="turtle_controller",
        remappings=[
        ],
        parameters=[
            {"check_next_objective_frequency": 0.05},
            {"distance_tolerance": 1.0},
            {"linear_speed_gain": 2.5},
            {"angular_speed_gain": 10.0},
        ]
    )

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_node",
        remappings=[
            ("node", "my_turtle")
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    return ld
