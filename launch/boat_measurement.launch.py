#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    microsd_path_arg = DeclareLaunchArgument(
        'microsd_path',
        default_value='/media/microsd',
        description='Path to microSD card mount point'
    )
    
    measurement_duration_arg = DeclareLaunchArgument(
        'measurement_duration',
        default_value='300',
        description='Measurement duration in seconds (default: 300 = 5 minutes)'
    )
    
    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyACM0',
        description='GPS serial port (default: /dev/ttyACM0)'
    )
    
    tds_port_arg = DeclareLaunchArgument(
        'tds_port',
        default_value='/dev/ttyACM1',
        description='TDS Arduino serial port (default: /dev/ttyACM1)'
    )
    
    # GPS Driver Node
    gps_driver = Node(
        package='your_package_name',  # CHANGE THIS to your package name
        executable='gps_driver.py',
        name='gps_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('gps_port')
        }]
    )
    
    # TDS Publisher Node
    tds_publisher = Node(
        package='your_package_name',  # CHANGE THIS to your package name
        executable='TDS_publisher.py',
        name='tds_driver',
        output='screen',
        parameters=[{
            # You may need to add port parameter if TDS uses different port
            # 'port': LaunchConfiguration('tds_port')
        }]
    )
    
    # Boat Measurement Driver Node
    boat_driver = Node(
        package='your_package_name',  # CHANGE THIS to your package name
        executable='boat_driver.py',
        name='boat_measurement_driver',
        output='screen',
        parameters=[{
            'microsd_path': LaunchConfiguration('microsd_path'),
            'measurement_duration': LaunchConfiguration('measurement_duration')
        }]
    )
    
    return LaunchDescription([
        microsd_path_arg,
        measurement_duration_arg,
        gps_port_arg,
        tds_port_arg,
        gps_driver,
        tds_publisher,
        boat_driver
    ])