from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_controller = Node(
        package="turtlesim_ctall",
        executable="turtle_controller"
    )

    turtle_spawner = Node(
        package="turtlesim_ctall",
        executable="turtle_spawner",
        parameters=[
            {"spawn_frequency": 2.0}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller)
    ld.add_action(turtle_spawner)

    return ld
