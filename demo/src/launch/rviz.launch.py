import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare(package='demo').find('demo')
    rviz_config_path = os.path.join(pkg_share, 'launch', 'urdf.rviz')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('demo'),
        'urdf',
        'robot.xacro.urdf',   # asegúrate de que este nombre de fichero existe
    ])

    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description
        }],
        output='screen'
    )

    # 6) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)
    return ld
