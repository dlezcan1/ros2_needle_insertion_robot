from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'needle_insertion_robot_translation'

    ld = LaunchDescription()

    # arguments
    namespace_arg = DeclareLaunchArgument('ns', default_value='',description="ROS Namespace for robot translation node.")

    # Nodes
    robot_translation_node = Node(
            package=package_name,
            namespace=LaunchConfiguration('ns'),
            executable='insertion_robot_translation_node',
            output='screen',
            emulate_tty=True,
            
    )
    
    # add to launch description
    ld.add_action( namespace_arg )

    ld.add_action( robot_translation_node )

    return ld

# generate_launch_description

