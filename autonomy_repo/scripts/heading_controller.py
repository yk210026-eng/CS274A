#!/usr/bin/env python3
import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):

    def __init__(self):
        super().__init__()

        self.declare_parameter("kp", 2)

    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value

    def compute_control_with_goal(self, current: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        err = wrap_angle(goal.theta - current.theta)
        omega = self.kp*err

        # ROS2 logging
        self.get_logger().info(f"err={err:.3f}, omega_cmd={omega:.3f}")

        result = TurtleBotControl()
        result.omega = omega
        return result
            
if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()
