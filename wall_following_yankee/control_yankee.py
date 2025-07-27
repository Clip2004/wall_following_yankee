#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class ControlYankeeNode(Node): # Redefine node class
    def __init__(self):
        super().__init__("control_yankee") # Redefine node name

        # create a topic subscriber

        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.error_sub = self.create_subscription(Float32,'/error',self.error_callback,10)

        # publisher obj (msg_type, topic_name, queue==buffer)
        self.cmd_vel_ctrl_pub = self.create_publisher(Twist,'cmd_vel_ctrl',10)
        # create a timer function to send msg
        self.timer_period = 0.1 # in [s]
        self.timer = self.create_timer(self.timer_period,self.control_callback)
        # Initialize variables
        self.error = 0.0
        self.error_1 = 0.0
        self.th = 0.0  # Current angle
        self.th_d = 0.0  # Desired angle
        self.kp = 1.5  # Proportional gain
        self.kd = 3 # Derivative gain
        self.max_angle_rad = math.radians(30)
        self.min_angle_rad = math.radians(-30)

    def error_callback(self, msg):
        # Example: Accessing error data
        self.error = msg.data
        self.th_d = self.kp * self.error + self.kd * ((self.error - self.error_1) / self.timer_period)
        self.th = self.th_d
        self.error_1 = self.error
    def control_callback(self):
        cmd_vel_ctrl = Twist()
        cmd_vel_ctrl.linear.x = 0.5
        # Saturate th to be within -90 to 90 degrees (converted to radians)
        self.th = max(min(self.th, self.max_angle_rad), self.min_angle_rad)
        cmd_vel_ctrl.angular.z = self.th
        self.cmd_vel_ctrl_pub.publish(cmd_vel_ctrl)


def main(args=None):
    rclpy.init(args=args)
    node = ControlYankeeNode() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()