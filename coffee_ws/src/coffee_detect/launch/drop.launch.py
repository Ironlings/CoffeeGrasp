from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('coffee_detect').find('coffee_detect')

    config_file_arg = DeclareLaunchArgument(
        name='config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'drop_params.yaml']),
        description='Path to parameter config file'
    )

    # 启动节点
    grasp_node = Node(
        package='coffee_detect',
        executable='drop.py',
        name='coffee_drop',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        #sam_checkpoint_arg,
        config_file_arg,
        grasp_node
    ])