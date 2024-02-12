from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    python_node = Node(
            package='ros_encoder',
            executable='python_node.py',
        )

    ld.add_action(python_node)

    return ld