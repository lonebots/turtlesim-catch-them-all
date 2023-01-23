#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import math

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_ctall_interfaces.msg import Turtle
from turtlesim_ctall_interfaces.msg import TurtleArray


class TurtleControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_controller")

        self.turtle_to_catch_ = None

        # attributes
        self.pose_ = None

        # parameters
        self.declare_parameter('loop_frequency', 0.01)

        # value assgin
        self.loop_frequency_ = self.get_parameter('loop_frequency').value

        # subscription to /turtle1/pose
        self.turtle_pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.callback_turtle_pose, 10)

        # subscription to /alive_turtles
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, '/alive_turtles', self.callback_alive_turtles, 10
        )

        # publisher to /turtle1/cmd_vel
        self.turtle_cmd_vel_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        # timer for control loop
        self.control_loop_timer_ = self.create_timer(
            self.loop_frequency_, self.control_loop)
        self.get_logger().info("turtle_controller node started")

        # client to kill turtle 
        self.kill_turtle_client_ = self.create_client()
        
    def call_kill_turtle_service(self,turtle_name) :
        pass

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch_ = msg.turtles[0]

    def callback_turtle_pose(self, msg):
        # self.get_logger().info(f"turtle position : {msg}")
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ is None:
            return

        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(pow(dist_x, 2)+pow(dist_y, 2))

        # create twist message
        msg = Twist()
        '''
        compute the velocity for linear and angular components

        we will be manipulating only the x
            1. linear x component
            2. angular z component

            until distance <= 0.5

        '''
        if distance > 0.5:
            # update postion components
            msg.linear.x = 2*distance

            # update orientation
            goal_theta = math.atan2(dist_y, dist_x)
            dif_theta = goal_theta - self.pose_.theta

            # make angle within 0 to 2pi
            if dif_theta > math.pi:
                dif_theta -= 2*math.pi
            elif dif_theta < -math.pi:
                dif_theta += 2*math.pi

            msg.angular.z = 6*dif_theta

        else:
            # make them 0
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            # remove the old target from the list 



        self.turtle_cmd_vel_publisher_.publish(msg)

    def catch_turtle(self, alive_turtle):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
