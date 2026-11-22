"""
navigation.launch.py - Part 2: navigation on the saved map + search & rescue.
  sim + map_server + amcl + Nav2 + (optional) rescue behavior tree.

  ros2 launch pio_maze navigation.launch.py                 # headless, auto rescue
  ros2 launch pio_maze navigation.launch.py gui:=true rviz:=true
  ros2 launch pio_maze navigation.launch.py rescue:=false   # just bring up nav
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
    params = os.path.join(pkg, 'config', 'nav2.yaml')
    default_map = os.path.join(pkg, 'maps', 'generated_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rescue = LaunchConfiguration('rescue')
    map_yaml = LaunchConfiguration('map')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'gui': gui, 'rviz': rviz}.items())

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time,
                             'yaml_filename': map_yaml}])

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}])

    loc_mgr = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server', 'amcl']}])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # run the rescue behavior tree once everything is up + localized
    rescue_bt = TimerAction(period=20.0, actions=[
        Node(package='pio_maze', executable='rescue_bt',
             name='rescue_bt', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(rescue))])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rescue', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        sim, map_server, amcl, loc_mgr, nav2, rescue_bt,
    ])
