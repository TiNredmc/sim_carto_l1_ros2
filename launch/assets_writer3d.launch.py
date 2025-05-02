"""
  Copyright 2016 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value = FindPackageShare('sim_carto_l1_ros2').find('sim_carto_l1_ros2') + '/param')
    config_file_arg = DeclareLaunchArgument('config_file', default_value = 'assets_writer_3d.lua')
    urdf_filename_arg = DeclareLaunchArgument('urdf_filename', default_value = FindPackageShare('sim_carto_l1_ros2').find('sim_carto_l1_ros2') + '/description/robot.urdf')
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames', default_value = FindPackageShare('sim_carto_l1_ros2').find('sim_carto_l1_ros2') + '/bag/L1_ros2.db3')
    pose_graph_filename_arg = DeclareLaunchArgument('pose_graph_filename', default_value = FindPackageShare('sim_carto_l1_ros2').find('sim_carto_l1_ros2') + '/map/l1.pbstream')
    output_file_prefix_arg = DeclareLaunchArgument('output_file_prefix', default_value = "output")
    ## ***** Nodes *****
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_assets_writer',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('config_file'),
            '-urdf_filename', LaunchConfiguration('urdf_filename'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-pose_graph_filename', LaunchConfiguration('pose_graph_filename'),
            '-output_file_prefix', LaunchConfiguration('output_file_prefix')],
        output = 'screen'
        )

    return LaunchDescription([
        configuration_directory_arg,
        config_file_arg,
        urdf_filename_arg,
        bag_filenames_arg,
        pose_graph_filename_arg,
        output_file_prefix_arg,
        # Nodes
        cartographer_node,
    ])
