import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    
    return LaunchDescription([
    Node(
        package = 'mpc_rbt_student',
        executable = 'Localization',
        name = 'Localization'
    ),

    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
        )
    ])
