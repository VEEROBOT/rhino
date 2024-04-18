#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo ,IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Constants for paths to different files and folders
    package_name = 'four_w_amr'
    package_joy = 'teleop_twist_joy'

    rviz_config_file_path = 'config/rviz.rviz'
    urdf_file_path = 'urdf/four_w_amr.urdf'
    joy_file_path = 'launch/teleop-launch.py'

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    pkg_share_joy = FindPackageShare(package=package_joy).find(package_joy)
    joy_path = os.path.join(pkg_share_joy, joy_file_path)

    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

    # Declare the launch arguments
    gui = DeclareLaunchArgument(
        'gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui')

    rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    use_robot_state_pub = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Debug statement to print launch arguments

    with open(default_urdf_model_path, 'r') as infp:
        robot_desc = infp.read()

    urdf_params = {'robot_description': robot_desc, 'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[urdf_params])
    
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[{'use_gui': LaunchConfiguration('gui')}])
    
    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')])
    static_transform_publisher_cmd_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0', '0.01', '1.57', '0', '0.0', 'base_link', 'imu_link_1'])


    static_transform_publisher_cmd = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_transform_publisher',
    output='screen',
    arguments=['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'])

    static_transform_publisher_cmd_2 = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_transform_publisher',
    output='screen',
    arguments=['-0.09', '0', '0.15', '0', '0', '0', 'top_plate_link', 'camera_link'])

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([joy_path])
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(gui)
    ld.add_action(rviz_config_file)
    ld.add_action(use_rviz)
    ld.add_action(use_sim_time)

    # Debug print launch arguments

    # # Start the robot state publisher
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    # Launch RViz
    ld.add_action(start_rviz_cmd)
    ld.add_action(static_transform_publisher_cmd_1)
    ld.add_action(static_transform_publisher_cmd)
    ld.add_action(static_transform_publisher_cmd_2)
    ld.add_action(included_launch)


    return ld
