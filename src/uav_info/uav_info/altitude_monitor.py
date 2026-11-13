import math

class AltitudeMonitor:
    def __init__(self, minimum_m, maximum_m):
        if not math.isfinite(minimum_m) or not math.isfinite(maximum_m):
            raise ValueError('Altitude limits must be finite')
        if minimum_m > maximum_m:
            raise ValueError('Minimum altitude must not exceed maximum altitude')
        self.minimum_m = float(minimum_m)
        self.maximum_m = float(maximum_m)

    def within_limits(self, altitude_m):
        return (math.isfinite(altitude_m) and self.minimum_m <= altitude_m <= self.maximum_m)
