from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # 启动节点
    move_node = Node(
        package='coffee_detect',
        executable='move2panel.py',
        name='control_panel_follower_tf',
        output='screen',
    )

    return LaunchDescription([
        move_node
    ])