# Launch file for ABU Robocon 2025 Robot
# Coded by TinLethax
import os

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions
import xacro

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

     # Specify the name of the package and path to xacro file within the package
    pkg_name = 'sim_carto_l1_ros2'
    file_subpath = 'description/robot.urdf'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Configure node launch information 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory', default= os.path.join(get_package_share_directory(pkg_name), 'param') )
    # Configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', default='R1_3D_mapping.lua')
    
    # Configure the node
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # ROS2 bag
    ros2_bag_instant = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--rate', '1.0', os.path.join(get_package_share_directory(pkg_name), 'bag', )],
        output='screen',
    )
    
    delayed_ros2bag = launch.actions.TimerAction(period=2.0, actions=[ros2_bag_instant])

    # Cartographer SLAM
    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        remappings=[
                ('imu', 'unilidar/imu'),
                ('points2', 'unilidar/cloud')
                ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                  ]
    )

    cartographer_occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    return launch.LaunchDescription([
        node_robot_state_publisher,
        delayed_ros2bag,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
