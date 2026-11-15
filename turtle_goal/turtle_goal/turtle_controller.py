"""Drive turtlesim to a target position."""

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleController(Node):
    """Calculate velocity commands from the pose and goal."""

    def __init__(self):
        super().__init__('turtle_controller')
        self.pose = None
        self.goal = None
        self.reached = False

        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )
        self.goal_sub = self.create_subscription(
            Point, '/goal_position', self.goal_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Controller ready.')

    def pose_callback(self, msg):
        """Save the latest turtle pose."""
        self.pose = msg

    def goal_callback(self, msg):
        """Save the target and detect a changed goal."""
        if (self.goal is None or msg.x != self.goal.x
                or msg.y != self.goal.y):
            self.reached = False
        self.goal = msg

    def control_loop(self):
        """Turn toward the target, approach it, and stop."""
        cmd = Twist()

        if self.pose is None or self.goal is None:
            self.cmd_pub.publish(cmd)
            return

        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y
        distance = math.hypot(dx, dy)

        if distance < 0.1:
            if not self.reached:
                self.get_logger().info('Goal reached. Stopping.')
            self.reached = True
        else:
            self.reached = False
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.pose.theta
            angle_error = math.atan2(
                math.sin(angle_error), math.cos(angle_error)
            )

            cmd.angular.z = max(-3.0, min(3.0, 4.0 * angle_error))
            if abs(angle_error) < 0.2:
                cmd.linear.x = min(2.0, 1.5 * distance)

        self.cmd_pub.publish(cmd)


def main(args=None):
    """Start the controller node."""
    rclpy.init(args=args)
    node = TurtleController()

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
