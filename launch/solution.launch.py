import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    sim_dir = get_package_share_directory('mpc_rbt_simulator')
    simulator_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim_dir, 'launch', 'simulation.launch.py')))

    localNode = Node(
        package = 'mpc_rbt_student',
        executable = 'Localization',
        name = 'Localization'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        ),

        TimerAction(
            period = 3.0,
            actions = [simulator_launch]
        ),

        TimerAction(
            period = 15.0,
            actions = [localNode]
        )
    ])
