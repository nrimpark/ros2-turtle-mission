"""Publish virtual obstacle/vision information and spawn an obstacle marker."""

import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

from turtle_goal.mission_logic import DATA_TIMEOUT, OBSTACLE, FogScenario


class MissionEnvironment(Node):
    """Provide synthetic signals; this node does not perform perception."""

    def __init__(self):
        super().__init__('mission_environment')
        self.pose = None
        self.pose_time = None
        self.marker_pose = None
        self.marker_time = None
        self.spawn_future = None
        self.last_spawn_attempt = -10.0
        self.last_state = None
        self.fog = FogScenario()
        self.obstacle_pub = self.create_publisher(Point, '/obstacle_position', 10)
        self.vision_pub = self.create_publisher(Float32, '/vision_confidence', 10)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.marker_sub = self.create_subscription(
            Pose, '/obstacle/pose', self.marker_callback, 10)
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.timer = self.create_timer(0.1, self.update)

    def pose_callback(self, msg):
        """Remember ground-truth pose for triggering the virtual fog zone."""
        self.pose, self.pose_time = msg, time.monotonic()

    def marker_callback(self, msg):
        """Check that the obstacle marker really exists at the intended place."""
        self.marker_pose, self.marker_time = msg, time.monotonic()

    def update(self):
        """Publish at 10 Hz after the stationary obstacle marker is ready."""
        now = time.monotonic()
        marker_fresh = (self.marker_time is not None
                        and now - self.marker_time < DATA_TIMEOUT)
        if not marker_fresh:
            if (self.spawn_future is not None and self.spawn_future.done()):
                try:
                    self.spawn_future.result()
                except Exception as error:
                    self.get_logger().warning(f'Obstacle marker: {error}')
                self.spawn_future = None
            if (self.spawn_future is None and now - self.last_spawn_attempt > 2.0
                    and self.spawn_client.service_is_ready()):
                request = Spawn.Request()
                request.x, request.y = OBSTACLE
                request.theta, request.name = 0.0, 'obstacle'
                self.spawn_future = self.spawn_client.call_async(request)
                self.last_spawn_attempt = now
            return

        if math.hypot(self.marker_pose.x - OBSTACLE[0],
                      self.marker_pose.y - OBSTACLE[1]) > 0.05:
            self.set_state('MARKER_MOVED: stop and restart the launch.')
            return

        msg = Point()
        msg.x, msg.y = OBSTACLE
        msg.z = 0.0
        self.obstacle_pub.publish(msg)

        if self.pose_time is None or now - self.pose_time > DATA_TIMEOUT:
            self.set_state('WAITING_FOR_POSE')
            return
        confidence, state = self.fog.update(self.pose.x, self.pose.y, now)
        self.vision_pub.publish(Float32(data=confidence))
        self.set_state(state)

    def set_state(self, state):
        """Log transitions without flooding the terminal."""
        if state != self.last_state:
            self.get_logger().info(f'Environment: {state}')
            self.last_state = state


def main(args=None):
    """Run the environment publisher."""
    rclpy.init(args=args)
    node = MissionEnvironment()
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
