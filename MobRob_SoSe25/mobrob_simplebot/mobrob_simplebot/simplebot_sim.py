import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math

class MobileRobot(Node):
    def __init__(self):
        super().__init__('simplebot_sim')
        
        # Initial position and orientation using Pose2D
        self.pose = Pose2D()
        self.twist = Twist()

        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0  # Orientation in radians

        self.twist.linear.x = 0.
        self.twist.angular.z = 0.
        
        # Create a subscriber to receive velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create a publisher for the pose
        self.pose_publisher = self.create_publisher(Pose2D, 'robot_pose2d', 10)

        # Timer to update the robot's position
        self.timer = self.create_timer(0.1, self.update_position)

    def cmd_vel_callback(self, msg):
        # Store the received linear and angular velocities
        self.twist = msg

    def update_position(self):
        # Update the robot's position based on the velocities
        dt = 0.1  # Time step
        v = self.twist.linear.x
        omege = self.twist.angular.z
        self.pose.x += v * math.cos(self.pose.theta) * dt
        self.pose.y += v* math.sin(self.pose.theta) * dt
        self.pose.theta += omege * dt

        # Log the current position and orientation
        self.get_logger().info(
            f'Position: ({self.pose.x:.2f}, {self.pose.y:.2f}),\
                Orientation: {self.pose.theta:.2f} rad')
        
        # Publish the current pose
        self.pose_publisher.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    robot = MobileRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
