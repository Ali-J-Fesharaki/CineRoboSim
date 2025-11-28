#!/usr/bin/env python3
"""
Launch file for sdf_to_gif node (Gazebo Camera Orbit)

Usage:
    ros2 launch sdf_to_gif sdf_to_gif.launch.py center_x:=5.0 center_y:=5.0 radius:=25.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments - center point
        DeclareLaunchArgument(
            'center_x',
            default_value='0.0',
            description='X coordinate of orbit center'
        ),
        DeclareLaunchArgument(
            'center_y',
            default_value='0.0',
            description='Y coordinate of orbit center'
        ),
        DeclareLaunchArgument(
            'center_z',
            default_value='0.0',
            description='Z coordinate of orbit center'
        ),
        
        # Declare arguments - orbit
        DeclareLaunchArgument(
            'radius',
            default_value='20.0',
            description='Orbit radius in meters'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='15.0',
            description='Camera height above center_z'
        ),
        DeclareLaunchArgument(
            'frames',
            default_value='30',
            description='Number of frames for orbit'
        ),
        
        # Declare arguments - output
        DeclareLaunchArgument(
            'output_file',
            default_value='output.gif',
            description='Output GIF filename'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='gifs',
            description='Output directory for GIF files'
        ),
        
        # Declare arguments - misc
        DeclareLaunchArgument(
            'cmd',
            default_value='ign',
            description='Gazebo command prefix (ign or gz)'
        ),
        DeclareLaunchArgument(
            'settle_time',
            default_value='0.3',
            description='Time between frames in seconds'
        ),
        DeclareLaunchArgument(
            'gif_duration',
            default_value='3.0',
            description='Total GIF duration in seconds'
        ),
        
        # Node
        Node(
            package='sdf_to_gif',
            executable='sdf_to_gif_node',
            name='sdf_to_gif_node',
            output='screen',
            parameters=[{
                'center_x': LaunchConfiguration('center_x'),
                'center_y': LaunchConfiguration('center_y'),
                'center_z': LaunchConfiguration('center_z'),
                'radius': LaunchConfiguration('radius'),
                'height': LaunchConfiguration('height'),
                'frames': LaunchConfiguration('frames'),
                'output_file': LaunchConfiguration('output_file'),
                'output_dir': LaunchConfiguration('output_dir'),
                'cmd': LaunchConfiguration('cmd'),
                'settle_time': LaunchConfiguration('settle_time'),
                'gif_duration': LaunchConfiguration('gif_duration'),
            }]
        ),
    ])
