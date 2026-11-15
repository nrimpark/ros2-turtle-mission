"""Follow safe waypoints and slow/stop in a synthetic low-visibility zone."""

import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String
from turtlesim.msg import Pose

from turtle_goal.mission_logic import (
    DATA_TIMEOUT, GOAL_TIMEOUT, LOW_VISIBILITY_SPEED, MAX_SPEED, SAFE_RADIUS,
    distance, plan_route, segment_clearance, velocity_toward,
)


class MissionController(Node):
    """A finite-state teaching controller for one fixed, known-size obstacle."""

    def __init__(self):
        super().__init__('mission_controller')
        self.pose = self.goal = self.obstacle = None
        self.confidence = None
        self.received = {}
        self.route = None
        self.last_state = None
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(
            Point, '/goal_position', self.goal_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Point, '/obstacle_position', self.obstacle_callback, 10)
        self.vision_sub = self.create_subscription(
            Float32, '/vision_confidence', self.vision_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_callback(self, msg):
        """Pose continues during fog, standing in for available odometry."""
        self.pose = (msg.x, msg.y, msg.theta)
        self.received['pose'] = time.monotonic()

    def goal_callback(self, msg):
        """Replan only when the goal actually changes."""
        new_goal = (msg.x, msg.y)
        if new_goal != self.goal:
            self.route = None
        self.goal = new_goal
        self.received['goal'] = time.monotonic()

    def obstacle_callback(self, msg):
        """Save the position of the one circular obstacle."""
        new_obstacle = (msg.x, msg.y)
        if new_obstacle != self.obstacle:
            self.route = None
        self.obstacle = new_obstacle
        self.received['obstacle'] = time.monotonic()

    def vision_callback(self, msg):
        """A separate virtual vision signal controls the speed/stop decision."""
        self.confidence = msg.data
        self.received['vision'] = time.monotonic()

    def publish(self, state, linear=0.0, angular=0.0):
        """Publish a command and report state transitions once."""
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        if state != self.last_state:
            self.get_logger().info(f'Mission: {state}')
            self.last_state = state

    def control_loop(self):
        """Check data, visibility, obstacle clearance, and the next waypoint."""
        now = time.monotonic()
        for name, timeout in (('pose', DATA_TIMEOUT), ('goal', GOAL_TIMEOUT),
                              ('obstacle', DATA_TIMEOUT), ('vision', DATA_TIMEOUT)):
            if name not in self.received or now - self.received[name] > timeout:
                self.publish('STOP_MISSING_DATA')
                return
        values = (*self.pose, *self.goal, *self.obstacle, self.confidence)
        if not all(math.isfinite(value) for value in values):
            self.publish('STOP_INVALID_DATA')
            return
        position = self.pose[:2]
        if distance(position, self.obstacle) < SAFE_RADIUS:
            self.publish('STOP_OBSTACLE_TOO_CLOSE')
            return
        if self.confidence < 0.2:
            self.publish('STOP_VISION_LOST')
            return
        if self.route is None:
            if self.confidence < 0.8:
                self.publish('WAIT_CLEAR_VIEW_TO_PLAN')
                return
            self.route = plan_route(position, self.goal, self.obstacle)
            if self.route is None:
                self.publish('STOP_NO_SAFE_ROUTE')
                return
            self.get_logger().info(f'Confirmed route: {self.route}')

        # During fog the confirmed route is retained instead of being replanned.
        while len(self.route) > 1 and distance(position, self.route[0]) < 0.12:
            self.route.pop(0)
        target = self.route[0]
        if len(self.route) == 1 and distance(position, target) < 0.1:
            self.publish('ARRIVED')
            return
        low_visibility = self.confidence < 0.8
        speed_limit = LOW_VISIBILITY_SPEED if low_visibility else MAX_SPEED
        linear, angular = velocity_toward(self.pose, target, speed_limit)

        # A short prediction catches a departure toward the inflated obstacle.
        future = (position[0] + linear * 0.2 * math.cos(self.pose[2]),
                  position[1] + linear * 0.2 * math.sin(self.pose[2]))
        if segment_clearance(position, future, self.obstacle) < SAFE_RADIUS:
            self.publish('STOP_OBSTACLE_TOO_CLOSE')
            return
        state = 'SLOW_FOG' if low_visibility else (
            'AVOID_OBSTACLE' if len(self.route) > 1 else 'GO_TO_GOAL')
        self.publish(state, linear, angular)


def main(args=None):
    """Run the mission controller."""
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
