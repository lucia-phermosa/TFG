from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='robot_controller_gazebo',
            name='robot_controller',
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=[
                '/world/robot_world/model/robot/link/base_link/sensor/imu_sensor/i'
                + '@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ],
            output='screen'
        ),
    ])
