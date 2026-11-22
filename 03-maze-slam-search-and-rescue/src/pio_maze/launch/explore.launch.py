"""
explore.launch.py - Part 1: autonomous frontier exploration.
  sim + slam_toolbox + Nav2 + frontier_explorer.

  ros2 launch pio_maze explore.launch.py                 # headless
  ros2 launch pio_maze explore.launch.py gui:=true rviz:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pio_maze')
    slam_cfg = os.path.join(pkg, 'config', 'slam.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    explore = LaunchConfiguration('explore')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'gui': gui, 'rviz': rviz}.items())

    slam_share = get_package_share_directory('slam_toolbox')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': slam_cfg,
                          'autostart': 'true'}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # start exploring after slam + nav2 are up
    explorer = TimerAction(period=45.0, actions=[
        Node(package='pio_maze', executable='frontier_explorer.py',
             name='frontier_explorer', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'start_x': 0.0, 'start_y': 0.0}],
             condition=IfCondition(explore))])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('explore', default_value='true'),
        sim, slam, nav2, explorer,
    ])
