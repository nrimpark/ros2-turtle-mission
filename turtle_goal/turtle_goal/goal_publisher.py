"""Publish the target position for turtlesim."""

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GoalPublisher(Node):
    """Send the goal position once per second."""

    def __init__(self):
        super().__init__('goal_publisher')

        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)

        self.goal_pub = self.create_publisher(
            Point, '/goal_position', 10
        )
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        """Create and publish a goal message."""
        msg = Point()
        msg.x = self.get_parameter('goal_x').value
        msg.y = self.get_parameter('goal_y').value
        msg.z = 0.0

        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'Publishing goal: ({msg.x:.1f}, {msg.y:.1f})'
        )


def main(args=None):
    """Start the goal publisher node."""
    rclpy.init(args=args)
    node = GoalPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
