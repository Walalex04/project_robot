
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return(LaunchDescription(
        Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"], 
        output="screen")
    ))