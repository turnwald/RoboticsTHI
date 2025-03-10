#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose2D, PoseArray, PoseStamped
from builtin_interfaces.msg import Time

import demo_diffdrive_control.kin_controllers as myc
import numpy as np
from tf2_geometry_msgs import tf2_geometry_msgs


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose2D, '/robot/pose2d', self.pose2d_callback, 10)
        self.cur_pose2D = Pose2D()
        self.cur_cmd = TwistStamped()
        self.declare_parameter('xd', 10.0)
        self.declare_parameter('yd', 0.0)
        self.declare_parameter('phi_deg', 0.0)
        self.des_pose = Pose2D()
        self.des_pose.x = self.get_parameter('xd').get_parameter_value().double_value
        self.des_pose.y = self.get_parameter('yd').get_parameter_value().double_value
        self.des_pose.theta = np.pi/180. *  self.get_parameter('phi_deg').get_parameter_value().double_value
        self.Ts = 0.2  # seconds
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        self.get_logger().info('Robot Controller node has been started.')

        # Create a separate timer for logging
        self.T_log = 1.0  # seconds
        self.logging_timer = self.create_timer(self.T_log, self.logging_callback)

    def pose2d_callback(self,msg:Pose2D):
        self.cur_pose2D = msg

    def timer_callback(self):
        self.cur_cmd = myc.simple_xy_controller(self.cur_pose2D,self.des_pose)
        self.cur_cmd.header.stamp = self.get_clock().now().to_msg()
        self.cur_cmd.header.frame_id = 'chassis'
        #print(f"Current command = {self.cur_cmd.twist.linear.x}")
        self.publisher_.publish(self.cur_cmd)


    def logging_callback(self):
        self.des_pose.x = self.get_parameter('xd').get_parameter_value().double_value
        self.des_pose.y = self.get_parameter('yd').get_parameter_value().double_value

        #self.get_logger().info(f'Current Position -> X: {self.cur_odom.pose.pose.position.x}, Y: {self.cur_odom.pose.pose.position.y}')
        #self.get_logger().info(f'Current Target -> X: {self.des_pose.x}, Y: {self.des_pose.y}')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
