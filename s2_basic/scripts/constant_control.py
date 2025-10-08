#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist


class ConstantControl(Node):
    def __init__(self) -> None:
# initialize base class (must happen before everything else)
        super().__init__("constant_control")

        self.msg = "sending constant control..."
        # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        # self.hb_pub = self.create_publisher(String, "/constant_control", 10)
        self.hb_pub2 = self.create_publisher(Twist, '/cmd_vel', 10)
            
        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(1, self.hb_callback)

        # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.motor_sub = self.create_subscription(Bool, "/constant_control/motor", self.health_callback, 10)
        self.motor_sub2 = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

    def kill_callback(self, msg: Bool) -> None:
        if msg.data:
            self.hb_timer.cancel()
            msg_control = Twist()
            msg_control.linear.x = 0.0
            msg_control.angular.z = 0.0
            self.hb_pub2.publish(msg_control)

    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0

        # publish heartbeat counter
        self.hb_pub2.publish(msg)

    def health_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if not msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = ConstantControl()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context
