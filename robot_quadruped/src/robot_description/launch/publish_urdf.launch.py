import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_description = "robot_description"
    
    # Directly use the URDF file
    urdf_file = "robot.urdf"
    #If using a xacro file
    #xacro_file = "robot.xacro" 

    # Get full path to the URDF file
    robot_desc_path = PathJoinSubstitution([
        FindPackageShare(package_description), "urdf", urdf_file
    ])

    # Read URDF file content
    with open(os.path.join(get_package_share_directory(package_description), "urdf", urdf_file), "r") as file:
        urdf_content = file.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": True,
            "robot_description": urdf_content  # Directly pass URDF content
            #"robot_description": Command([FindExecutable(name="xacro"), robot_desc_path])
        }],
        output="screen"
    )

    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare(package_description), "config", "rviz_config.rviz"  # Ensure correct path
    ])

    #rviz_node = Node(
    #    package="rviz2",
    #    executable="rviz2",
    #    output="screen",
    #    name="rviz_node",
    #    parameters=[{"use_sim_time": True}],
    #    arguments=["-d", rviz_config_dir]
    #)

    return LaunchDescription([
        robot_state_publisher_node
        #rviz_node
    ])
