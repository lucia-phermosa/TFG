#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import random
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # --- locate packages and files ---
    pkg_gz_sim         = get_package_share_directory('ros_gz_sim')
    pkg_robot_gazebo   = get_package_share_directory('robot_gazebo')
    pkg_robot_desc     = get_package_share_directory('robot_description')

    urdf_file          = os.path.join(pkg_robot_desc,   'urdf',    'robot.urdf')
    controller_yaml    = os.path.join(pkg_robot_desc,   'config',  'ros2_control.yaml')
    world_file         = os.path.join(pkg_robot_gazebo, 'worlds',  'demo.sdf')

    # read URDF into a string
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # random name to avoid conflicts
    entity_name = f"robot_quadruped_{random.randint(0,99999)}"

    # --- Gazebo launch ---
    launch_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # --- environment for models & plugins ---
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_robot_gazebo, 'models')
    )
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(get_package_prefix('robot_description'), 'lib')
    )

    # --- publish the URDF ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # --- spawn into Gazebo using the URDF topic ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-entity', entity_name,
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # --- bridge /clock, /tf, joint_states, IMU, and send commands back to Gazebo ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/joint_group_position_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- ros2_control manager: give it the URDF + your controller YAML ---
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            {   # robot_description _and_ sim_time
                'robot_description': robot_description_content,
                'use_sim_time': True,
            },
            controller_yaml   # load your ros2_control.yaml
        ],
        output='screen'
    )

    # --- spawn joint_state_broadcaster after a delay ---
    spawner_js = TimerAction(
        period=15.0,
        actions=[ 
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_joint_state_broadcaster',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    # --- then spawn the position‐trajectory controller ---
    spawner_pos = TimerAction(
        period=18.0,  # a few seconds after the broadcaster
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_position_controller',
                arguments=[
                    'joint_group_position_controller',
                    '--controller-manager', 'controller_manager'
                ],
                output='screen',
            )
        ]
    )

    # --- finally, check list of loaded controllers ---
    check_controllers = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    "echo '=== controller list ===' && ros2 control list_controllers"
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        launch_gz,
        robot_state_publisher,
        spawn_entity,
        bridge,
        controller_manager,   # <— make sure this is actually launched!
        spawner_js,
        spawner_pos,
        check_controllers,
    ])
