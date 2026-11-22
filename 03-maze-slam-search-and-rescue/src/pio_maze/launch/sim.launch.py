"""
sim.launch.py - base simulation:
  Gazebo (maze world) + robot_state_publisher + ros_gz bridge +
  robot spawn at (-7.0, -6.5) + odom->tf node + (optional) RViz.

Args:
  use_sim_time (true)   gui (true)   rviz (false)   headless (false)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, AppendEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pio_maze')
    world = os.path.join(pkg, 'worlds', 'maze.world')
    urdf_xacro = os.path.join(pkg, 'urdf', 'pio.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'maze.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')

    robot_desc = ParameterValue(
        Command(['xacro ', urdf_xacro], on_stderr='ignore'), value_type=str)

    # gz needs to find the maze model
    set_res = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))

    # server-only (headless) when headless:=true or gui:=false
    server_only = PythonExpression(
        ["'true' if '", headless, "' == 'true' or '", gui,
         "' == 'false' else 'false'"])
    gz_env = {'GZ_SIM_RESOURCE_PATH': os.path.join(pkg, 'models')}
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '2', world],
        output='screen', additional_env=gz_env,
        condition=IfCondition(server_only))
    gz_full = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world],
        output='screen', additional_env=gz_env,
        condition=UnlessCondition(server_only))

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim_time}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': use_sim_time}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description', '-name', 'pio',
                   '-x', '-7.0', '-y', '-6.5', '-z', '0.08'])

    # EKF fuses wheel odometry + IMU and publishes odom -> base_footprint
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}])

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        set_res,
        gz_headless, gz_full, rsp, bridge, spawn, ekf, rviz_node,
    ])
