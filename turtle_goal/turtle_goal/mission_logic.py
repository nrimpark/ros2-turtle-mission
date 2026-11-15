"""Small, ROS-independent planning and control helpers for one static circle."""

import math


OBSTACLE = (6.5, 6.5)
OBSTACLE_RADIUS = 0.45
TURTLE_RADIUS = 0.25
SAFETY_MARGIN = 0.20
SAFE_RADIUS = OBSTACLE_RADIUS + TURTLE_RADIUS + SAFETY_MARGIN
LOW_VISIBILITY_SPEED = 0.35
MAX_SPEED = 1.25
DATA_TIMEOUT = 1.0
GOAL_TIMEOUT = 3.0


def distance(a, b):
    """Return Euclidean distance in turtlesim coordinates."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def segment_clearance(a, b, center):
    """Return the shortest distance between a line segment and a point."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    length_squared = vx * vx + vy * vy
    if length_squared < 1e-12:
        return distance(a, center)
    projection = ((center[0] - a[0]) * vx
                  + (center[1] - a[1]) * vy) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = (a[0] + projection * vx, a[1] + projection * vy)
    return distance(closest, center)


def inside_map(point):
    """Keep the turtle center away from the turtlesim walls."""
    return all(0.5 <= value <= 10.5 for value in point)


def plan_route(start, goal, obstacle=OBSTACLE):
    """Return [goal] or [bypass_point, goal], or None if no safe route exists.

    This is a one-obstacle teaching example. It checks several possible
    intermediate points; it is not a general-purpose navigation planner.
    """
    if not all(inside_map(point) for point in (start, goal, obstacle)):
        return None
    # Extra clearance absorbs waypoint tolerance and discrete control updates.
    required = SAFE_RADIUS + 0.20
    if min(distance(start, obstacle), distance(goal, obstacle)) < required:
        return None
    if segment_clearance(start, goal, obstacle) >= required:
        return [goal]

    length = distance(start, goal)
    if length < 1e-9:
        return [goal]
    perpendicular = (-(goal[1] - start[1]) / length,
                     (goal[0] - start[0]) / length)
    candidates = []
    for side in (1.0, -1.0):
        for scale in (1.5, 2.0, 2.5, 3.0, 4.0):
            waypoint = (
                obstacle[0] + side * scale * required * perpendicular[0],
                obstacle[1] + side * scale * required * perpendicular[1],
            )
            if not inside_map(waypoint):
                continue
            if min(segment_clearance(start, waypoint, obstacle),
                   segment_clearance(waypoint, goal, obstacle)) >= required:
                cost = distance(start, waypoint) + distance(waypoint, goal)
                candidates.append((cost, waypoint))
    if not candidates:
        return None
    # Preserve the first side on equal costs, making the demo reproducible.
    best_cost = min(item[0] for item in candidates)
    waypoint = next(point for cost, point in candidates
                    if cost <= best_cost + 1e-9)
    return [waypoint, goal]


def velocity_toward(pose, target, speed_limit):
    """Proportional position/heading control. This is not RPP."""
    x, y, theta = pose
    dx, dy = target[0] - x, target[1] - y
    heading_error = math.atan2(dy, dx) - theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
    angular = max(-2.5, min(2.5, 4.0 * heading_error))
    linear = 0.0
    if abs(heading_error) < 0.18:
        linear = min(speed_limit, 1.5 * math.hypot(dx, dy))
    return linear, angular


class FogScenario:
    """Inject a repeatable virtual vision failure on first entering a y band."""

    def __init__(self):
        self.entry_time = None

    def update(self, x, y, now):
        """Return (confidence, label); recovery uses time while stopped."""
        in_band = 0.5 <= x <= 10.5 and 7.0 <= y <= 7.7
        if self.entry_time is None:
            if not in_band:
                return 1.0, 'CLEAR'
            self.entry_time = now
        elapsed = now - self.entry_time
        if elapsed < 1.2:
            return 0.4, 'FOG_SLOW'
        if elapsed < 3.2:
            return 0.0, 'VISION_LOST'
        if in_band:
            return 0.4, 'FOG_RECOVERED_SLOW'
        return 1.0, 'CLEAR'
