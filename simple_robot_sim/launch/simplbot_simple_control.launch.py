import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    return launch.LaunchDescription([
        # Declare the argument to specify the path to the URDF file
        DeclareLaunchArgument(
            'urdf_file',
            default_value='/opt/ros_ws/src/gz_ros2_diffdrive/urdf/diff_drive.xacro.urdf',
            description='Full path to the URDF file to be loaded'),

        # -- start node 1
        launch_ros.actions.Node(
            package='simple_robot_sim',
            executable='bot_sim',
            name='bot_sim'),
        # -- start node 2
        launch_ros.actions.Node(
            package='demo_diffdrive_control',
            executable='simple_control_node',
            name='simple_control_node'),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])
            }],
        ),
  ])