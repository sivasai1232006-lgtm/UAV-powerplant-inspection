import math


class PathCostClient:
    def __init__(
        self, distance_safety_factor=1.3, cruise_altitude_m=7.0,
    ):
        if (
            not math.isfinite(distance_safety_factor)
            or not 1.25 <= distance_safety_factor <= 1.30
        ):
            raise ValueError(
                'Distance safety factor must be between 1.25 and 1.30'
            )
        if not math.isfinite(cruise_altitude_m):
            raise ValueError('Cruise altitude must be finite')
        self.distance_safety_factor = float(distance_safety_factor)
        self.cruise_altitude_m = float(cruise_altitude_m)

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
        horizontal_distance = math.hypot(
            float(goal[0]) - float(start[0]),
            float(goal[1]) - float(start[1]),
        )
        vertical_distance = (
            abs(float(start[2]) - self.cruise_altitude_m)
            + abs(float(goal[2]) - self.cruise_altitude_m)
        )
        return (
            horizontal_distance + vertical_distance
        ) * self.distance_safety_factor

    @property
    def is_provisional(self):
        return True
