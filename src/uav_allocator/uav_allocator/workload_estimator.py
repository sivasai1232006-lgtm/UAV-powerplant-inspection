from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WorkloadEstimate:
    outbound_travel_sec: float
    inspection_sec: float
    total_task_sec: float
    return_travel_sec: float


class WorkloadEstimator:
    def __init__(self, nominal_speed_mps):
        if not math.isfinite(nominal_speed_mps) or nominal_speed_mps <= 0.0:
            raise ValueError('Nominal speed must be greater than zero')
        self.nominal_speed_mps = float(nominal_speed_mps)

    def estimate(self, outbound_distance_m, inspection_duration_sec,
                 return_distance_m):
        values = (
            outbound_distance_m,
            inspection_duration_sec,
            return_distance_m,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError('Workload inputs must be finite and non-negative')
        outbound_time = outbound_distance_m / self.nominal_speed_mps
        inspection_time = float(inspection_duration_sec)
        return_time = return_distance_m / self.nominal_speed_mps
        return WorkloadEstimate(
            outbound_travel_sec=outbound_time,
            inspection_sec=inspection_time,
            total_task_sec=outbound_time + inspection_time,
            return_travel_sec=return_time,
        )
