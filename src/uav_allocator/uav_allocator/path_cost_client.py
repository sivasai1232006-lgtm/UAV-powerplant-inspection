import math


class PathCostClient:
    def __init__(self, allow_straight_line=True):
        self.allow_straight_line = allow_straight_line
        self._distances = {}

    @staticmethod
    def _key(start, goal, start_yaw=0.0, goal_yaw=0.0):
        return (
            tuple(round(float(value), 6) for value in start),
            tuple(round(float(value), 6) for value in goal),
            round(float(start_yaw), 6),
            round(float(goal_yaw), 6),
        )

    def set_distance(self, start, goal, distance, start_yaw=0.0, goal_yaw=0.0):
        self._distances[self._key(start, goal, start_yaw, goal_yaw)] = distance

    def estimate_distance(self, start, goal, start_yaw=0.0, goal_yaw=0.0):
        try:
            if len(start) != 3 or len(goal) != 3:
                return None
            finite_coordinates = all(
                math.isfinite(value) for value in (*start, *goal)
            )
        except (TypeError, ValueError):
            return None
        if not finite_coordinates:
            return None
        key = self._key(start, goal, start_yaw, goal_yaw)
        if key in self._distances:
            return self._distances[key]
        if not self.allow_straight_line:
            return None
        return math.dist(start, goal)

    @property
    def is_provisional(self):
        return self.allow_straight_line
