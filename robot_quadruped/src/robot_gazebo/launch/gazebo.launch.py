#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    #print(pkg_gazebo_ros)
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    #print(pkg_robot_gazebo)

    description_package_name = "robot_description"
    install_dir = get_package_prefix(description_package_name)
    #print(install_dir)

    gazebo_models_path = os.path.join(pkg_robot_gazebo, 'models')
    #os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    #print(gazebo_models_path)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'+ ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share' + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    #print(os.environ['GAZEBO_MODEL_PATH'])

    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')  # Update the actual launch file name
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf', 'on_exit_shutdown': 'true'}.items()
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazebo)

    return launchDescriptionObject


