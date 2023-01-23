#!/usr/bin/python3

# ros-related
import rclpy
from rclpy.node import Node

# common-python
from functools import partial
import random
import math

# interface
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim_ctall_interfaces.msg import Turtle
from turtlesim_ctall_interfaces.msg import TurtleArray
from turtlesim_ctall_interfaces.srv import CatchTurtle
from std_srvs.srv import Empty


class TurtleSpawnerNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_spawner")
        self.turtle_name_prefix_ = "turtle"

        # parameter
        self.declare_parameter('spawn_frequency', 2.0)

        # variables
        self.turtle_counter_ = 1  # turtle1 is master and it is already spawned
        self.alive_turtles_ = []
        self.spawn_frequency_ = self.get_parameter('spawn_frequency').value

        # timer for spawning turtle
        self.turtle_spawn_timer_ = self.create_timer(
            1.0/self.spawn_frequency_, self.spawn_new_turtle)

        # publish alive turtles
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray,
                                                              'alive_turtles', 10)

        # kill turtle serivce
        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, 'catch_turtle', self.callback_catch_turtle_service)

        self.get_logger().info("turtle_spawner node started")

    def callback_catch_turtle_service(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response

    def publish_alive_turtle(self,):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

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

    def clear_turtle_path(self):
        self.call_clearpath_server()

    # spawn

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

                # add it to list of alive turtles
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta

                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtle()

        except Exception as e:
            self.get_logger().error(f"Exception in service call : {e}")

    # kill

    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, '/kill')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_kill_server, turtle_name=turtle_name)
        )

    def callback_kill_server(self, future, turtle_name):
        try:
            future.result()
            # enumerate over the alive_turtles array and delete the turtle
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtle()
                    self.clear_turtle_path()
                    break

        except Exception as e:
            self.get_logger().error(f"Exception in service call : {e}")

    # clear path

    def call_clearpath_server(self):
        client = self.create_client(Empty, '/clear')
        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(
            self.callback_clearpath_server
        )

    def callback_clearpath_server(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Exception in service call : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
