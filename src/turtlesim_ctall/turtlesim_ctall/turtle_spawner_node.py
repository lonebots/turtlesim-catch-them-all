#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from functools import partial
import random
import math

from turtlesim.srv import Spawn


class TurtleSpawnerNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_spawner")
        self.turtle_name_prefix_ = "turtle"

        # variables
        self.turtle_counter_ = 0
        self.alive_turtle_ = []

        # timer for spawning turtle
        self.turtle_spawn_timer_ = self.create_timer(
            2.0, self.spawn_new_turtle)

        self.get_logger().info("turtle_spawner node started")

    def spawn_new_turtle(self):
        # set name
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_+str(self.turtle_counter_)

        # set coordinate and theta
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_spawn_server(turtle_name=name, x=x,
                               y=y, theta=theta)  # spawn call

    def call_spawn_server(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_spawn_server, x=x, y=y,
                    theta=theta, turtle_name=turtle_name)
        )

    def callback_spawn_server(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"turtle {response.name } is now live!")
        except Exception as e:
            self.get_logger().error(f"Exception in service call : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
