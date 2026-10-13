"""View the model in RViz with sliders for the joints (no physics).
ros2 launch twlr_description display.launch.py legs_fixed:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('twlr_description')
    xacro_file = os.path.join(pkg, 'urdf', 'twlr.urdf.xacro')
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' acrylic_mass:=', LaunchConfiguration('acrylic_mass'),
        ' legs_fixed:=', LaunchConfiguration('legs_fixed')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('acrylic_mass', default_value='0.515'),
        DeclareLaunchArgument('legs_fixed', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'twlr.rviz')]),
    ])
