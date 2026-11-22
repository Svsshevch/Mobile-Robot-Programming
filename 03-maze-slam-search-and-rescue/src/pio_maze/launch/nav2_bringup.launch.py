"""
nav2_bringup.launch.py - the Nav2 action servers + lifecycle manager.
Included by explore.launch.py (with slam providing map->odom) and by
navigation.launch.py (with amcl/map_server providing it).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pio_maze')
    params = os.path.join(pkg, 'config', 'nav2.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    nav_nodes = ['controller_server', 'smoother_server', 'planner_server',
                 'behavior_server', 'bt_navigator']

    def srv(pkg_name, exe, name):
        return Node(package=pkg_name, executable=exe, name=name, output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    remappings=[('cmd_vel', '/cmd_vel')])

    nodes = [
        srv('nav2_controller', 'controller_server', 'controller_server'),
        srv('nav2_smoother', 'smoother_server', 'smoother_server'),
        srv('nav2_planner', 'planner_server', 'planner_server'),
        srv('nav2_behaviors', 'behavior_server', 'behavior_server'),
        srv('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': nav_nodes}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params),
        *nodes,
    ])
