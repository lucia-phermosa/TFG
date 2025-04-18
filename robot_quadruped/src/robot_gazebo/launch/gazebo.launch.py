#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import random

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
    controller_config = '/home/lucia/TFG/robot_quadruped/src/robot_description/config/controller.yaml'
    world_file = os.path.join(pkg_robot_gazebo, 'worlds', 'demo.sdf')

    # Leer URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

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
            'use_sim_time': True
        }],
        output='screen'
    )

    # 4. Spawnear el robot en Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', entity_name,
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.2',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # 5. Lanza ros2_control_node 
    ros2_control_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"use_sim_time": True},
                    controller_config
                ],
                output="screen"
            )
        ]
    )

    # 6. Spawners de controladores 
    controller_spawners = []
    for i in range(1, 13):
        controller_name = f"Revolution_{i}_controller"
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name],
            output="screen"
        )
        controller_spawners.append(spawner)

    delayed_controller_spawners = TimerAction(
        period=15.0,
        actions=controller_spawners
    )

    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        launch_gazebo,
        robot_state_publisher,
        spawn_robot,
        ros2_control_node,
        delayed_controller_spawners
    ])
