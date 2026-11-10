"""Operating-region bounds relative to each UAV's initial map position."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OperatingBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @classmethod
    def from_initial_position(cls, position, configuration):
        config = dict(configuration or {})
        absolute_keys = (
            'x_min_m', 'x_max_m', 'y_min_m', 'y_max_m', 'z_min_m', 'z_max_m'
        )
        if any(key in config for key in absolute_keys):
            values = tuple(
                float(config[key]) for key in absolute_keys
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError('Operating-region values must be finite')
            x_min, x_max, y_min, y_max, z_min, z_max = values
            if x_min >= x_max or y_min >= y_max or z_min >= z_max:
                raise ValueError(
                    'Operating-region minimums must be below maximums'
                )
            if z_min < 0.0:
                raise ValueError(
                    'Operating-region altitude cannot be below ground'
                )
            return cls(
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=z_min,
                z_max=z_max,
            )

        x0, y0, _ = (float(value) for value in position)
        values = (
            x0,
            y0,
            float(configuration['x_min_offset_m']),
            float(configuration['x_max_offset_m']),
            float(configuration['y_min_offset_m']),
            float(configuration['y_max_offset_m']),
            float(configuration['z_min_m']),
            float(configuration['z_max_m']),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Operating-region values must be finite')
        (
            x0,
            y0,
            x_min_offset,
            x_max_offset,
            y_min_offset,
            y_max_offset,
            z_min,
            z_max,
        ) = values
        if (
            x_min_offset >= x_max_offset
            or y_min_offset >= y_max_offset
            or z_min >= z_max
        ):
            raise ValueError(
                'Operating-region minimums must be below maximums'
            )
        if z_min < 0.0:
            raise ValueError(
                'Operating-region altitude cannot be below ground'
            )
        return cls(
            x_min=x0 + x_min_offset,
            x_max=x0 + x_max_offset,
            y_min=y0 + y_min_offset,
            y_max=y0 + y_max_offset,
            z_min=z_min,
            z_max=z_max,
        )

    def contains(self, position):
        try:
            x, y, z = (float(value) for value in position)
        except (TypeError, ValueError):
            return False
        return (
            all(math.isfinite(value) for value in (x, y, z))
            and self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )

    def contains_xy(self, position):
        try:
            x, y = (float(value) for value in position[:2])
        except (TypeError, ValueError):
            return False
        return (
            math.isfinite(x)
            and math.isfinite(y)
            and self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
        )

    def as_parameters(self):
        return {
            'x_min': self.x_min,
            'x_max': self.x_max,
            'y_min': self.y_min,
            'y_max': self.y_max,
            'z_min': self.z_min,
            'z_max': self.z_max,
        }
