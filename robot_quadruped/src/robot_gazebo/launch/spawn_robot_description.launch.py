#!/usr/bin/python3
# -*- coding: utf-8 -*-

import random

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    #[X, Y, Z]
    position = [0.0, 0.0, 0.2]
    #[Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]

    robot_base_name = "robot_quadruped"

    entity_name = robot_base_name + "-" + str(int(random.random()*100000))

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),  
            '-R', str(orientation[0]),
            '-P', str(orientation[1]),
            '-Y', str(orientation[2]),
            '-topic', '/robot_description' 
        ]
    )


    return LaunchDescription([
                spawn_robot,       
    ])
