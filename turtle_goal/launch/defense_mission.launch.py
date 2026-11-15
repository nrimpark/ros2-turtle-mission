"""Start one simulator and three custom nodes for the mission exercise."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch all nodes together; subscribers wait for complete input data."""
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='turtle_goal', executable='mission_goal_publisher',
             name='mission_goal_publisher', output='screen'),
        Node(package='turtle_goal', executable='mission_environment',
             name='mission_environment', output='screen'),
        Node(package='turtle_goal', executable='mission_controller',
             name='mission_controller', output='screen'),
    ])
