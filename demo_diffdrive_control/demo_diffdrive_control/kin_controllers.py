from geometry_msgs.msg import TwistStamped, Pose2D, PoseStamped
from math import atan2
import numpy as np
import math

def simple_x_controller(cur_pose,des_pose):
    msg = TwistStamped()
    msg.twist.linear.x = -0.5*(cur_pose.x - des_pose.x) # Forward with 0.5 m/s
    #print(f"Current pose = {cur_pose.x} and des pose = {des_pose.x}. vel_com = {msg.twist.linear.x}")
    msg.twist.angular.z = 0.0  # No rotation
    return msg

def simple_xy_controller(cur_pose,des_pose):
    msg = TwistStamped()
    ep = 1 * np.pi/180.
    diff_x = cur_pose.x - des_pose.x
    diff_y = cur_pose.y - des_pose.y
    diff_theta = cur_pose.theta  - des_pose.theta
    err_x, err_y, err_theta = rotated_error(diff_x,diff_y,diff_theta, cur_pose.theta)

    Kx = 0.4
    Ky = 0.5 
    Ktheta = 0.1  
    v_r = 0.
    omeg_r = 0.


    msg.twist.linear.x = -Kx*err_x + v_r*np.cos(err_theta)
    msg.twist.angular.z = -Ktheta*np.sin(err_theta) + omeg_r - Ky*err_y #Ky*v_r*err_y # loosing this for v_r = 0? 
    
    print(f"Pose = [{cur_pose.x:.1f},{cur_pose.y:.1f},{cur_pose.theta * 180./np.pi:.1f} deg], and rot. error = [{err_x:.1f},{err_y:.1f},{err_theta * 180./np.pi:.1f} deg], and  cmd = [{msg.twist.linear.x:.1f},{msg.twist.angular.z:.2f}] ")
    
    return msg

def Odometry2Pose2D(Od) -> Pose2D:
    Pose = Pose2D()
    Pose.x = Od.pose.pose.position.x
    Pose.y = Od.pose.pose.position.y
    Pose.theta = yaw_from_quat(Od.pose.pose.orientation.x,Od.pose.pose.orientation.y,Od.pose.pose.orientation.z,Od.pose.pose.orientation.w)
    return Pose

def pose2d_to_pose(pose2d) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = pose2d.x
    pose.pose.position.y = pose2d.y
    pose.pose.position.z = 0.0  # Assuming z is 0 in the 2D case
    # Convert theta to a quaternion (yaw-only rotation)
    qz = math.sin(pose2d.theta / 2.0)
    qw = math.cos(pose2d.theta / 2.0)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def pose_to_pose2d(pose:PoseStamped) -> Pose2D:
    pose2d = Pose2D()
    pose2d.x = pose.pose.position.x
    pose2d.y = pose.pose.position.y
    # Extract yaw (theta) from the quaternion
    siny_cosp = 2 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y)
    cosy_cosp = 1 - 2 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z)
    pose2d.theta = math.atan2(siny_cosp, cosy_cosp)
    return pose2d

def yaw_from_quat(x,y,z,w):
     # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
    # We assume roll and pitch are zero for this conversion to 2D (yaw only)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)

def rotated_error(del_x,del_y,del_theta,theta):
    dx =  np.array([del_x,del_y,del_theta])
    # Define the rotation matrix about the z-axis
    Rot = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])
    # Rotate the vector
    x_rotated = np.dot(Rot, dx)
    return x_rotated[0],x_rotated[1],x_rotated[2]