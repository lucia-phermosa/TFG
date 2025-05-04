#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import random
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Paquetes y rutas
    pkg_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    pkg_robot_gazebo = FindPackageShare('robot_gazebo').find('robot_gazebo')
    pkg_robot_description = FindPackageShare('robot_description').find('robot_description')
    description_install_dir = get_package_prefix('robot_description')

    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf')
    controller_config = os.path.join(pkg_robot_description, 'config', 'ros2_control.yaml')
    world_file = os.path.join(pkg_robot_gazebo, 'worlds', 'demo.sdf')

    # Leer URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Read the controller YAML config
    with open(controller_config, 'r') as file:
        controller_params = yaml.safe_load(file)


    # Nombre random para evitar conflictos
    robot_base_name = "robot_quadruped"
    entity_name = robot_base_name + "-" + str(int(random.random()*100000))

    # 1. Lanzar Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # 2. Setear paths de GAZEBO
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f'{description_install_dir}/share:{os.path.join(pkg_robot_gazebo, "models")}'
    )

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=f'{description_install_dir}/lib'
    )

    # 3. Publicar URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }],
        output='screen'
    )

    # 4. Spawnear el robot en Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-entity', entity_name,
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.2',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_params,
            {'use_sim_time': True}   # ✅ Make sure controller_manager uses sim time
        ],
        output='screen',
    )

    # spawn_joint_state_bcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'joint_state_broadcaster',
    #         '--controller-manager', '/controller_manager',
    #     ],
    #     parameters=[{'use_sim_time': True}],  # ✅ Clean, no --params-file, just parameters
    #     output='screen'
    # )

    # spawn_position_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'revolution1_position_controller',
    #         '--controller-manager', '/controller_manager',
    #     ],
    #     parameters=[{'use_sim_time': True}],  # ✅ Again, just parameters
    #     output='screen'
    # )

    # delayed_spawner = TimerAction(
    #     period=15.0,  # Adjust this if needed
    #     actions=[spawn_joint_state_bcaster, spawn_position_controller]
    # )

    # 7. Bridge between ros and gazebo
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[
            os.path.join(pkg_robot_description, 'config', 'controller.yaml'),
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        launch_gazebo,
        robot_state_publisher,
        spawn_robot,
        controller_manager_node,
        # delayed_spawner,
        bridge_node
    ])
