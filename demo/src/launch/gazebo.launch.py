import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gz_args = LaunchConfiguration('gz_args', default='')

    # Paths
    xacro_file = PathJoinSubstitution([
        FindPackageShare('demo'),
        'urdf',
        'robot.xacro.urdf',   # asegúrate de que este nombre de fichero existe
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('demo'),
        'config',
        'ros2_control.yaml',
    ])

    pkg_robot_gazebo = get_package_share_directory('demo')

    world_sdf = os.path.join(pkg_robot_gazebo, 'worlds', 'demo.sdf')

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('demo'),
        'launch',
        'urdf.rviz',
    ])

    # Run xacro -> robot_description (forzar string)
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'demo',            # <- NOMBRE DEL MODELO
                   '-allow_renaming', 'true'
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.26',
                   '-R', '0',
                   '-P', '0',
                   '-Y', '0',
                ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file', robot_controllers,
        ],
        output='screen'
    )

    # joint_trajectory_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'joint_group_position_controller',
    #         '--param-file', robot_controllers,
    #     ],
    #     output='screen'
    # )

    # Bridge (coherente con -name robot)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/demo/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    joint_pub = Node(
        package='demo',
        executable='main',                  # nombre del binario
        name='joint_pub_node',
        output='screen',
        parameters=[{
            'model_name': 'demo',          # <- NOMBRE DEL MODELO
            'use_sim_time': True,
            'start_after_seconds': 0.0,
            'publish_rate_hz': 50.0,
            'controller_name': 'joint_trajectory_controller',
            'trajectory_lookahead_s': 0.05,
            'publish_joint_states': False,
            'imu_topic': '/imu',
            'odom_topic': '/odom',
            'joint_names': [
                'FR_elbow','FR_shoulder','FR_hip',
                'RR_elbow','RR_shoulder','RR_hip',
                'RL_elbow','RL_shoulder','RL_hip',
                'FL_elbow','FL_shoulder','FL_hip'
            ],
        }],
    )

    # joint_pub = Node(
    #     package='demo',
    #     executable='main',   # nombre del ejecutable en CMake
    #     name='joint_pub_node',
    #     output='screen',
    #     parameters=[
    #         {'odom_topic': '/odom'},
    #     ],
    # )

    pose_to_odom = Node(
        package='demo',
        executable='entitypose_to_odom.py',   
        name='entitypose_to_odom',
        output='screen',
        parameters=[
            {'pose_topic': '/model/demo/pose'},
            {'odom_topic': '/odom'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
        ],
    )

    gait = Node(
        package='demo',
        executable='gait_duty_autoadapt',
        name='gait_duty_autoadapt',
        output='screen',
        parameters=[
            {'odom_topic': '/odom'},
        ],
    )

    # Arrancar joint_pub después de que esté listo el controlador de trayectoria
    start_joint_pub_after_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[joint_pub],
        )
    )

    start_gait_after_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[gait],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        # Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                      'launch', 'gz_sim.launch.py'])
            ]),
            # launch_arguments={'gz_args': [gz_args, f' -r -v 1 empty.sdf']}.items()
            launch_arguments={'gz_args': [gz_args, f' -r -v 1 {world_sdf}']}.items()
        ),

        # Orden de spawners
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        start_joint_pub_after_jtc,
        # body_planner,
        # start_gait_after_jtc,

        # Otros nodos
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # pose_to_odom,
        # RViz si quieres:
        # Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen',
        #      parameters=[{'use_sim_time': use_sim_time}]),

        # Launch arg
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
    ])
