import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  bringup_dir = get_package_share_directory('autocar2_bringup')
  params_file = LaunchConfiguration('params_file')
  
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  rviz_config_dir = os.path.join(get_package_share_directory('autocar2_bringup'),
                                   'rviz', 'ac2_amcl.rviz')
  
  map_name = LaunchConfiguration('map_name')

  #map_dir_0 = os.path.join('/home', 'soda','dev','2d_lidar_maps')
  #launch_map_dir_0 = TextSubstitution(text=map_dir_0)
  #launch_map_dir_1 = TextSubstitution(text='.yaml')

  map_dir = os.path.join('/home', 'soda','dev','2d_lidar_maps', 'test0', 'test0'+'.yaml')

  return LaunchDescription([
    Node(
            package='rviz2',
            node_executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'),
    DeclareLaunchArgument('map_name', default_value=map_dir),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('nav2_bringup'),
        'launch', 'nav2_localization_launch.py')),
      launch_arguments={ 'params_file': params_file,
                          'map_subscribe_transient_local': 'true',
                          'map': map_name,}.items(),
    ),
  ])
