import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose2D
import demo_diffdrive_control.kin_controllers as myc

class SimpleBotController(Node):
    def __init__(self):
        super().__init__('simple_bot_controller')
         # Publisher for TwistStamped messages
        self.publisher = self.create_publisher(TwistStamped, 'simplebot/cmd_vel', 10)

        # Subscription to Pose2D messages
        self.pose_subscription = self.create_subscription(
            Pose2D,
            'simplebot/robot_pose2d',
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback) 

        #--- 
        self.pose2d = Pose2D()
        self.twist = TwistStamped()

        self.declare_parameter('xd', 10.0)
        self.declare_parameter('yd', 10.0)
        self.des_pose2d = Pose2D()
        self.des_pose2d.x = self.get_parameter('xd').get_parameter_value().double_value
        self.des_pose2d.y = self.get_parameter('yd').get_parameter_value().double_value

    def timer_callback(self):
        self.twist.twist.linear.x = 0.0
        self.twist.twist.angular.z = 0.0
        self.des_pose2d.x = self.get_parameter('xd').get_parameter_value().double_value
        self.des_pose2d.y = self.get_parameter('yd').get_parameter_value().double_value
        #self.cur_cmd = myc.simple_x_controller(self.cur_pose,self.des_pose)
        self.twist = myc.simple_xy_controller(self.pose2d,self.des_pose2d)
        self.twist.header.frame_id = 'robot_base'
        self.publisher.publish(self.twist)
        #self.get_logger().info('Publishing: "%s"' % str(msg))

    def pose_callback(self, msg):
        # Print the received Pose2D message
        #self.get_logger().info(f'Received Pose2D - x: {msg.x}, y: {msg.y}, theta: {msg.theta}')
        self.pose2d = msg
        

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
