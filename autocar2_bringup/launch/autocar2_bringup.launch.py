import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

  map_dir = os.path.join(get_package_share_directory('autocar2_bringup')
                                                       ,'map','map01.yaml')

  return LaunchDescription([
    DeclareLaunchArgument('map'
                          , default_value=map_dir),
    Node(
      package='joystick_ros2',
      node_executable='joystick_ros2',
      name='joystick_ros2',
      output='screen'),
    Node(
      package='autocar2_teleop',
      node_executable='joy2teleop',
      name='joy2teleop',
      output='log'),
    Node(
      package='rf2o_laser_odometry',
      node_executable='rf2o_laser_odometry_node',
      name='rf2o_laser_odometry',
      output='log',
      parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],),
    Node(
      package='pop_ros',
      node_executable='pop',
      name='pop',
      output='log'),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('autocar2_bringup'),
        'launch', 'autocar2_state_publisher.launch.py')),
    ),
  ])
