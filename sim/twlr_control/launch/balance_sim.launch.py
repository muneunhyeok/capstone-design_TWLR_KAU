"""Gazebo + balance controller in one command.
ros2 launch twlr_control balance_sim.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory('twlr_description')
    ctrl = get_package_share_directory('twlr_control')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'sim.launch.py')),
        launch_arguments={'acrylic_mass': LaunchConfiguration('acrylic_mass'),
                          'spawn_pitch': LaunchConfiguration('spawn_pitch')}.items())
    controller = Node(
        package='twlr_control', executable='balance_node', name='balance_controller',
        parameters=[os.path.join(ctrl, 'config', 'balance_sim.yaml'), {'use_sim_time': True}],
        output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('acrylic_mass', default_value='0.515'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.101'),
        sim,
        TimerAction(period=3.0, actions=[controller]),   # let Gazebo come up first
    ])
