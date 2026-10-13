"""Gazebo Harmonic simulation of the TWLR.
ros2 launch twlr_description sim.launch.py [acrylic_mass:=0.515] [legs_fixed:=true]
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('twlr_description')
    xacro_file = os.path.join(pkg, 'urdf', 'twlr.urdf.xacro')
    world = os.path.join(pkg, 'worlds', 'twlr_world.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'gz_bridge.yaml')

    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' acrylic_mass:=', LaunchConfiguration('acrylic_mass'),
        ' legs_fixed:=', LaunchConfiguration('legs_fixed')]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('acrylic_mass', default_value='0.515',
                              description='acrylic cover mass [kg], 0.001 = no cover'),
        DeclareLaunchArgument('legs_fixed', default_value='true'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.0',
                              description='initial body pitch [rad], + = nose down'),
        gz,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
             output='screen'),
        # wheel bottom is 0.422 m below base_link, spawn 8 mm above ground
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'twlr',
                        '-z', '0.430', '-P', LaunchConfiguration('spawn_pitch')],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
             output='screen'),
    ])
