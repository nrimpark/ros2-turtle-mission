"""Launch turtlesim and the goal navigation nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the simulator, goal publisher, and controller."""
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        Node(
            package='turtle_goal',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen',
        ),
        Node(
            package='turtle_goal',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen',
        ),
    ])
