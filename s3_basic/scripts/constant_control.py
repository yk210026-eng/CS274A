#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class ConstantVelocityController(Node):
    """
    A simple ROS 2 node that publishes a constant forward velocity
    until stopped via a /kill or /motor topic.
    """

    def __init__(self) -> None:
        super().__init__("constant_velocity_controller")

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Publish velocity every 1 second
        self.timer = self.create_timer(1.0, self.publish_constant_velocity)

        # Subscribers for stop signals
        self.create_subscription(Bool, "/kill", self.kill_callback, 10)
        self.create_subscription(Bool, "/constant_control/motor", self.motor_callback, 10)

        self.get_logger().info("ConstantVelocityController started. Publishing constant velocity...")

    def publish_constant_velocity(self) -> None:
        """Publish a constant forward velocity command."""
        msg = Twist()
        msg.linear.x = 1.0  # move forward at 1 m/s
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().debug("Published forward velocity: linear.x = 1.0")

    def kill_callback(self, msg: Bool) -> None:
        """Stop all motion when /kill topic publishes True."""
        if msg.data:
            self.timer.cancel()
            stop_msg = Twist()  # all zeros by default
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().warn("Kill signal received. Robot stopped.")

    def motor_callback(self, msg: Bool) -> None:
        """Stop publishing if motor health fails."""
        if not msg.data:
            self.timer.cancel()
            self.get_logger().fatal("Motor health failure detected. Velocity publishing stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = ConstantVelocityController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
