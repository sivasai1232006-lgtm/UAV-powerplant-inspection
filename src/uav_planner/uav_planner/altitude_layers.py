import math


def altitude_label(altitude_m):
    altitude = float(altitude_m)
    if not math.isfinite(altitude) or altitude <= 0.0:
        raise ValueError('Flight altitude must be finite and positive')
    return f'{altitude:g}'.replace('.', 'p')


def structural_map_name(altitude_m):
    altitude = float(altitude_m)
    if math.isclose(altitude, 10.0, abs_tol=1e-9):
        # Preserve the existing 10 m map-server filename for compatibility.
        return 'static_map'
    return f'static_map_{altitude_label(altitude)}m'


def planner_name(altitude_m):
    return f'planner_{altitude_label(altitude_m)}m'


def altitude_layers(altitudes_m):
    altitudes = tuple(float(value) for value in altitudes_m)
    if not altitudes:
        raise ValueError('At least one flight altitude must be configured')
    if len(set(altitudes)) != len(altitudes):
        raise ValueError('Flight altitudes must be unique')
    return tuple(
        (altitude, structural_map_name(altitude), planner_name(altitude))
        for altitude in altitudes
    )
