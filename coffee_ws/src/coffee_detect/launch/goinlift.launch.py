from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # 启动节点
    goin_node = Node(
        package='coffee_detect',
        executable='goinlift.py',
        name='motion_commander',
        output='screen',
    )

    return LaunchDescription([
        goin_node
    ])