from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('coffee_detect').find('coffee_detect')

    # 启动节点
    avoid_node = Node(
        package='coffee_detect',
        executable='avoid.py',
        name='pc_avoid',
        output='screen'
    )

    return LaunchDescription([
        avoid_node
    ])