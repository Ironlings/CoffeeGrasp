from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('coffee_detect').find('coffee_detect')

    # # 默认 SAM 路径（可放在包内 models/ 目录）
    # default_sam_path = PathJoinSubstitution([
    #     pkg_share,
    #     'sam',
    #     'sam_vit_h_4b8939.pth'
    # ])

    # # 声明 launch 参数
    # sam_checkpoint_arg = DeclareLaunchArgument(
    #     name='sam_checkpoint',
    #     default_value=default_sam_path,
    #     description='Path to SAM model checkpoint (.pth)'
    # )

    config_file_arg = DeclareLaunchArgument(
        name='config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
        description='Path to parameter config file'
    )

    # 启动节点
    grasp_node = Node(
        package='coffee_detect',
        executable='coffee.py',
        name='coffee_detect',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        #sam_checkpoint_arg,
        config_file_arg,
        grasp_node
    ])