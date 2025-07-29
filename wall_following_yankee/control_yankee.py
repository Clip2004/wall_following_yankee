#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class ControlYankeeNode(Node): # Redefine node class
    def __init__(self):
        super().__init__("control_yankee") # Redefine node name
        # Declarar parámetros y valores por defecto
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('kd', 3.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.5)

        # read parameters
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        # create a topic subscriber

        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.error_sub = self.create_subscription(Float32,'/error',self.error_callback,10)

        # publisher obj (msg_type, topic_name, queue==buffer)
        self.cmd_vel_ctrl_pub = self.create_publisher(Twist,'cmd_vel_ctrl',10)
        # create a timer function to send msg
        self.timer_period = 0.1 # in [s]
        self.timer = self.create_timer(self.timer_period,self.control_callback)
        # self.timer_linear_velocity = self.create_timer(self.timer_period, self.linear_velocity_callback)
        # Initialize variables
        self.error = 0.0
        self.error_1 = 0.0
        self.th = 0.0  # Current angle
        self.th_d = 0.0  # Desired angle
        # self.kp = 1.5  # Proportional gain
        # self.kd = 3 # Derivative gain
        self.max_angle_rad = math.radians(30)  # Maximum angle in radians (30 degrees)
        self.min_angle_rad = math.radians(-30)
        self.linear_velocity = 0.5
    def error_callback(self, msg):
        # Example: Accessing error data
        self.error = msg.data
        self.th_d = self.kp * self.error + self.kd * ((self.error - self.error_1) / self.timer_period)
        self.th = self.th_d
        # Adjust linear velocity based on error magnitude
        error_abs = abs(self.error)
        # Clamp error_abs to a reasonable range to avoid negative speeds
        error_abs = min(error_abs, 1.0)
        if self.th < 0.005 and self.th > -0.005:
            self.linear_velocity = self.max_speed - (self.max_speed - self.min_speed) * error_abs
        else:
            self.linear_velocity = self.min_speed
    def control_callback(self):
        cmd_vel_ctrl = Twist()
        cmd_vel_ctrl.linear.x = self.linear_velocity
        if (self.error - self.error_1) == 0:
            if self.error > 0:
                self.th = math.radians(-30)  
            else:
                self.th = math.radians(30)
        else:
            # Saturate th to be within -90 to 90 degrees (converted to radians)
            self.th = max(min(self.th, self.max_angle_rad), self.min_angle_rad)
        cmd_vel_ctrl.angular.z = self.th
        self.cmd_vel_ctrl_pub.publish(cmd_vel_ctrl)
        # Update error_1 after using it for comparison
        self.error_1 = self.error

def main(args=None):
    rclpy.init(args=args)
    node = ControlYankeeNode() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()