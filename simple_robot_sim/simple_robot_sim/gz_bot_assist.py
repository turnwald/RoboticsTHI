import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose2D, PoseStamped, TransformStamped, PoseArray
from nav_msgs.msg import Odometry
import math
import demo_diffdrive_control.kin_controllers as myc
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class AssistGZBotControl(Node):
    def __init__(self):
        super().__init__('gz_robot_assist')
        self.subscription_gt = self.create_subscription(PoseArray, '/world/empty/pose/info', self.gt_callback, 10)
        #self.subscription = self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.odom_callback, 10)
        self.publisher_gt = self.create_publisher(PoseStamped, '/robot/ground_truth', 10)
        self.publisher_2d = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose2d = Pose2D()
        self.pose_gt = PoseStamped()

        # Create a timer to update pose and publish it every 10ms (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        

    def gt_callback(self, msg:PoseArray):
        self.pose_gt.pose = msg.poses[1]
        self.pose_gt.header.stamp = self.get_clock().now().to_msg()
        self.pose_gt.header.frame_id = 'world'
        self.publisher_gt.publish(self.pose_gt)
        self.pose2d = myc.pose_to_pose2d(self.pose_gt)
        self.publisher_2d.publish(self.pose2d)
        #self.get_logger().info(f'Current Position Publisehd -> X: {self.pose2d.x}, Y: {self.pose2d.y}')


    def timer_callback(self):
        #--- Publish the transform
        # Use the same data to publish the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 1.
        t.transform.translation.y = 0.
        t.transform.translation.z = 0.0

        #self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AssistGZBotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()