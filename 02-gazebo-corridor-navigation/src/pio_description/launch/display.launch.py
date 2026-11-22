import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = get_package_share_directory('pio_description')
    # Get the Gazebo launch file from ros_gz_sim package
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(
        ros_gz_sim_share,
        'launch',
        'gz_spawn_model.launch.py'
    )

    default_model_path = os.path.join(
        pkg_share,
        'src',
        'description',
        'pio_description.sdf'
    )

    default_rviz_config_path = os.path.join(
        pkg_share,
        'rviz',
        'config.rviz'
    )

    world_path = os.path.join(
        pkg_share,
        'world',
        'corridor_world.sdf'
    )

    bridge_config_path = os.path.join(
        pkg_share,
        'config',
        'bridge_config.yaml'
    )
    # Gazebo needs this environment variable to find the model and world files
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )
    #open Gazebo GUI
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )
    #run simulation
    #container is needed to run gz_server and ros_gz_bridge in the same process for better performance
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container', 
        create_own_container='True',
        use_composition='True',
    )
    #start the bridge to connect ROS 2 and Gazebo topics
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')], #loads saved RViz config file
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'corridor_world',
            'file': LaunchConfiguration('model'),
            'entity_name': 'pio_robot',

            #Initial pose of the robot in the Gazebo world, right top corner
            'x': '2.0',
            'y': '-3.0',
            'z': '0.12',

            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )
    #make sure the robot is spawned after Gazebo server is ready, otherwise the spawn will fail because Gazebo topics are not available yet
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    navigation_node = Node(
        package='pio_description',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen', #print the node's log messages to the console
        parameters=[{'use_sim_time': True}] #tell the node to use simulation time from Gazebo not the real system time
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot SDF file'
        ),

        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to RViz config file'
        ),

        set_gz_resource_path, #find package resources like models and worlds
        navigation_node, #start the navigation node
        gz_gui, #open Gazebo GUI
        gz_server, #start Gazebo server to run the simulation
        ros_gz_bridge, #start the bridge to connect ROS 2 and Gazebo topics
        rviz_node, #start RViz to visualize the robot and its sensor data
        delayed_spawn, #delay the spawning of the robot until Gazebo server is ready
    ])