import math


class CostFunction:
    def __init__(
        self,
        weights,
        distance_scale_m,
        time_scale_sec,
        energy_scale_percentage,
        minimum_priority,
        maximum_priority,
    ):
        self.weights = dict(weights)
        if set(self.weights) != {'distance', 'time', 'energy', 'priority'}:
            raise ValueError(
                'Weights must define distance, time, energy, priority'
            )
        if any(not math.isfinite(value) or value < 0.0
               for value in self.weights.values()):
            raise ValueError('Cost weights must be finite and non-negative')
        if sum(self.weights.values()) <= 0.0:
            raise ValueError('At least one cost weight must be positive')
        scales = (distance_scale_m, time_scale_sec, energy_scale_percentage)
        if any(not math.isfinite(value) or value <= 0.0 for value in scales):
            raise ValueError('Cost normalization scales must be positive')
        if maximum_priority <= minimum_priority:
            raise ValueError('Maximum priority must exceed minimum priority')
        self.distance_scale_m = float(distance_scale_m)
        self.time_scale_sec = float(time_scale_sec)
        self.energy_scale_percentage = float(energy_scale_percentage)
        self.minimum_priority = int(minimum_priority)
        self.maximum_priority = int(maximum_priority)

    def calculate(self, distance_m, duration_sec, energy_percentage,
                  priority):
        priority_value = min(
            self.maximum_priority,
            max(self.minimum_priority, int(priority)),
        )
        priority_penalty = (
            self.maximum_priority - priority_value
        ) / (self.maximum_priority - self.minimum_priority)
        return (
            self.weights['distance'] * distance_m / self.distance_scale_m
            + self.weights['time'] * duration_sec / self.time_scale_sec
            + self.weights['energy'] * energy_percentage
            / self.energy_scale_percentage
            + self.weights['priority'] * priority_penalty
        )
