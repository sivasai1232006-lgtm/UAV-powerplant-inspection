from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FeasibilityResult:
    feasible: bool
    required_percentage: float
    available_percentage: float
    reason: str


class BatteryFeasibility:
    def __init__(
        self,
        consumption_per_meter,
        inspection_consumption_per_minute,
        minimum_reserve_percentage,
    ):
        values = (
            consumption_per_meter,
            inspection_consumption_per_minute,
            minimum_reserve_percentage,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError('Battery model values cannot be negative')
        if minimum_reserve_percentage > 100.0:
            raise ValueError('Battery reserve cannot exceed 100 percent')
        self.consumption_per_meter = float(consumption_per_meter)
        self.inspection_consumption_per_minute = float(
            inspection_consumption_per_minute
        )
        self.minimum_reserve_percentage = float(
            minimum_reserve_percentage
        )

    def evaluate(
        self,
        battery_percentage,
        outbound_distance_m,
        return_distance_m,
        inspection_duration_sec,
    ):
        inputs = (
            battery_percentage,
            outbound_distance_m,
            return_distance_m,
            inspection_duration_sec,
        )
        if any(not math.isfinite(value) for value in inputs):
            return FeasibilityResult(
                False,
                math.inf,
                float(battery_percentage),
                'battery feasibility inputs are invalid',
            )
        if any(value < 0.0 for value in inputs[1:]):
            return FeasibilityResult(
                False,
                math.inf,
                float(battery_percentage),
                'battery feasibility inputs cannot be negative',
            )
        required = (
            (outbound_distance_m + return_distance_m)
            * self.consumption_per_meter
            + inspection_duration_sec / 60.0
            * self.inspection_consumption_per_minute
            + self.minimum_reserve_percentage
        )
        available = float(battery_percentage)
        feasible = 0.0 <= available <= 100.0 and available >= required
        reason = (
            'battery sufficient for outbound, inspection, return, and reserve'
            if feasible
            else f'requires {required:.2f}% battery, has {available:.2f}%'
        )
        return FeasibilityResult(feasible, required, available, reason)
