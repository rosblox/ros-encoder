from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    multiturn_encoder_node = Node(
            package='ros_encoder',
            executable='multiturn_encoder_node.py',
        )

    ld.add_action(multiturn_encoder_node)

    return ld