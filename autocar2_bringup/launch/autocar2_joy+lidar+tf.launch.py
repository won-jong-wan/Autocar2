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
    # LaunchConfiguration('map')
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('sllidar_ros2'),
        'launch', 'sllidar_a2m12_launch.py'))),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('rf2o_laser_odometry'),
        'launch', 'rf2o.laser_odometry.launch.py'))),
    Node(
      package='joystick_ros2',
      node_executable='joystick_ros2',
      name='joystick_ros2',
      output='screen'),
    Node(
      package='pop_ros',
      node_executable='pop',
      name='pop',
      output='screen'),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('autocar2_bringup'),
        'launch', 'autocar2_state_publisher.launch.py'))),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('nav2_bringup'),
        'launch', 'nav2_localization_launch.py')),
      launch_arguments={'map': map_dir}.items(),
    ),
  ])
