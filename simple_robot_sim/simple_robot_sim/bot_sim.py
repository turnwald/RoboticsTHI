import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Pose2D, PoseStamped, TransformStamped
from rclpy.qos import qos_profile_system_default
import math
import demo_diffdrive_control.kin_controllers as myc
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')

        # Initialize robot pose as Pose2D
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        

        self.pose2d = Pose2D
        self.pose2d = myc.pose_to_pose2d(self.pose)

        # Initialize last received TwistStamped message
        self.current_twist = TwistStamped()
        self.current_twist.twist.linear.x = 0.1
        self.current_twist.twist.angular.z = 1e-3

        # Create a publisher for Pose2D
        self.pose_publisher = self.create_publisher(PoseStamped, 'simplebot/robot_pose', 10)
        self.pose2d_publisher = self.create_publisher(Pose2D, 'simplebot/robot_pose2d', 10)

        # Create a subscription for TwistStamped
        self.twist_subscription = self.create_subscription(
            TwistStamped,
            'simplebot/cmd_vel',
            self.twist_callback,
            qos_profile_system_default
        )

        # Create a timer to update pose and publish it every 10ms (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.frame_id = 'robot_base'
        static_t.child_frame_id = 'chassis'
        self.tf_static_broadcaster.sendTransform(static_t)
        #------------------
        static_t.header.frame_id = 'chassis'
        static_t.child_frame_id = 'left_wheel'
        static_t.transform.translation.x = 0.554283
        static_t.transform.translation.y = 0.625029
        static_t.transform.translation.z = 0.3
        self.tf_static_broadcaster.sendTransform(static_t)
        #------------------
        static_t.child_frame_id = 'right_wheel'
        static_t.transform.translation.y = -0.625029
        self.tf_static_broadcaster.sendTransform(static_t)


    def twist_callback(self, msg):
        #print(f"----- twist callback -----")
        # Update the current_twist with the latest message
        msg.header.stamp = self.get_clock().now().to_msg()
        self.current_twist = msg
        #print(f"==== current state = [{self.pose.x:.1f},{self.pose.y:.1f},{self.pose.theta:.1f}] and current input = [{self.current_twist.twist.linear.x:.1f},{self.current_twist.twist.angular.z:.1f}] ====")

    def timer_callback(self):
        # Extract velocity commands from the current_twist
        #print(f"----- timer callback -----")
        v = self.current_twist.twist.linear.x
        omega = self.current_twist.twist.angular.z

        # Simple Euler integration for state update
        dt = 0.01  # Time step (10 ms)
        self.pose2d.x += v * math.cos(self.pose2d.theta) * dt
        self.pose2d.y += v * math.sin(self.pose2d.theta) * dt
        self.pose2d.theta += omega * dt
        # Normalize theta to the range [-pi, pi]
        self.pose2d.theta = math.atan2(math.sin(self.pose2d.theta), math.cos(self.pose2d.theta))

        self.pose2d_publisher.publish(self.pose2d)

        self.pose = myc.pose2d_to_pose(self.pose2d)
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = 'world'  # Set the frame_id to "map"
        self.pose_publisher.publish(self.pose)

        #--- Publish the transform
        # Use the same data to publish the transform
        t = TransformStamped()
        t.header = self.pose.header
        t.child_frame_id = 'robot_base'
        t.transform.translation.x = self.pose.pose.position.x
        t.transform.translation.y = self.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
