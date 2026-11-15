"""Publish the same final target used in the original exercise."""

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MissionGoalPublisher(Node):
    """Publish a target every second so late subscribers also receive it."""

    def __init__(self):
        super().__init__('mission_goal_publisher')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.publisher = self.create_publisher(Point, '/goal_position', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.get_logger().info('Mission target: (8, 8).')

    def publish_goal(self):
        """Publish a planar coordinate (z remains zero)."""
        msg = Point()
        msg.x = float(self.get_parameter('goal_x').value)
        msg.y = float(self.get_parameter('goal_y').value)
        msg.z = 0.0
        self.publisher.publish(msg)


def main(args=None):
    """Run the target publisher."""
    rclpy.init(args=args)
    node = MissionGoalPublisher()
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
