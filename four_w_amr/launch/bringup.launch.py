#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  LaunchConfiguration, PythonExpression
import time
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    # Constants for paths to different files and folders
    package_name = 'four_w_amr'
    lidar_pkg= 'rplidar_ros'
    imu_pkg= 'imu_filter_madgwick'

    pkg_share = FindPackageShare(package=lidar_pkg).find(lidar_pkg)

    pkg_share2 = FindPackageShare(package=package_name).find(package_name)

    pkg_share3 = FindPackageShare(package=imu_pkg).find(imu_pkg)

    ekf_file_path='config/ekf.yaml'

    # Declare the transport_type argument
    transport_type = DeclareLaunchArgument('transport_type', default_value='serial', description='Transport type (e.g., udp4, udp6, tcp4, tcp6, canfd, serial, multiserial, pseudoterminal)')
    # Declare the serial device argument
    serial_dev = DeclareLaunchArgument('serial_dev', default_value='/dev/ttyesp32', description='Serial device')
    # Declare the baudrate argument
    baudrate = DeclareLaunchArgument('baudrate', default_value='115200', description='Baudrate for serial communication')

    ekf_config_path = os.path.join(pkg_share2, ekf_file_path)


    default_odom_topic = DeclareLaunchArgument(
        name='odom_topic', 
        default_value='/odom',
        description='EKF out odometry topic'
    )

                
    
    start_twist_to_pwm = Node(
        package='four_w_amr',
        executable='twist_2_pwm.py',
        name='twist_2_pwm',
        output='both')
    
    start_odom_publisher = Node(
        package='four_w_amr',
        executable='odom.py',
        name='odom_pub',
        output='both')
    


    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path
        ],
        remappings=[("odometry/filtered", LaunchConfiguration('odom_topic'))]
    )
    

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        output='screen',
        arguments=[LaunchConfiguration('transport_type'), '--dev', LaunchConfiguration('serial_dev'), '-b', LaunchConfiguration('baudrate')],  # Include the --dev and -b/--baudrate arguments, and -h/--help option
        respawn=False)
    

    
    static_transform_publisher_cmd_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0', '0.01', '0', '0', '3.14', 'base_link', 'imu_link'])


    lidar_launch_file_foxy = IncludeLaunchDescription(PythonLaunchDescriptionSource([pkg_share , '/launch/rplidar.launch.py']),)
    
    lidar_launch_file_humble = IncludeLaunchDescription(PythonLaunchDescriptionSource([pkg_share2, '/launch/lidar.launch.py']),)

    imu_tool = IncludeLaunchDescription(PythonLaunchDescriptionSource([pkg_share3, '/launch/imu_filter.launch.py']),)
    
    # Create LaunchDescription object
    ld = LaunchDescription()

                # HUMBLE
    # ld.add_action(lidar_launch_file_humble)

                # FOXY
    # ld.add_action(lidar_launch_file_foxy)

    ld.add_action(transport_type)
    ld.add_action(serial_dev)
    ld.add_action(baudrate)
    ld.add_action(micro_ros_agent_node)
    ld.add_action(default_odom_topic)

    # Add the nodes to the LaunchDescription
    ld.add_action(start_twist_to_pwm)
    ld.add_action(start_odom_publisher)
    # ld.add_action(imu_tool)

    # ld.add_action(start_ekf_node)

    # ld.add_action(static_transform_pu2blisher_cmd)

    return ld
