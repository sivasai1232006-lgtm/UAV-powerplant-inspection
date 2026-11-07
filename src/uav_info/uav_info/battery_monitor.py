import math


class BatteryMonitor:
    def __init__(
        self,
        initial_percentage,
        consumption_per_meter,
        minimum_reserve_percentage=0.0,
    ):
        if (
            not math.isfinite(initial_percentage)
            or not 0.0 <= initial_percentage <= 100.0
        ):
            raise ValueError(
                'Initial battery percentage must be between 0 and 100'
            )
        if (
            not math.isfinite(consumption_per_meter)
            or consumption_per_meter < 0.0
        ):
            raise ValueError(
                'Battery consumption rate must be finite and non-negative'
            )
        if (
            not math.isfinite(minimum_reserve_percentage)
            or not 0.0 <= minimum_reserve_percentage <= 100.0
        ):
            raise ValueError('Battery reserve must be between 0 and 100')

        self.initial_percentage = float(initial_percentage)
        self.consumption_per_meter = float(consumption_per_meter)
        self.minimum_reserve_percentage = float(
            minimum_reserve_percentage
        )
        self.cumulative_distance_m = 0.0
        self.has_path_distance = False

    def update_path_distance(self, cumulative_distance_m):
        if (
            not math.isfinite(cumulative_distance_m)
            or cumulative_distance_m < 0.0
        ):
            return False
        if cumulative_distance_m < self.cumulative_distance_m:
            return False

        self.cumulative_distance_m = float(cumulative_distance_m)
        self.has_path_distance = True
        return True

    def estimated_percentage(self):
        consumed = self.consumption_per_meter * self.cumulative_distance_m
        return max(0.0, self.initial_percentage - consumed)

    def estimated_range_m(self):
        if self.consumption_per_meter == 0.0:
            return math.inf
        available_percentage = max(
            0.0,
            self.estimated_percentage() - self.minimum_reserve_percentage,
        )
        return available_percentage / self.consumption_per_meter
