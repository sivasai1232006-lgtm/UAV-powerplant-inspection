import math

def altitude_label(altitude):
    altitude = float(altitude)
    if not math.isfinite(altitude) or altitude   <= 0.0:
        raise ValueError('Altitude must be > 0 and finite')
    return f'{altitude:g}'.replace('.', 'p')

def structural_map_name(altitude):
    altitude = float(altitude)
    if math.isclose(altitude, 10.0, abs_tol=1e-9):
        return 'static_map'
    return f'static_map_{altitude_label(altitude)}m'

def planner_name(altitude):
    return f'planner_{altitude_label(altitude)}m'

def altitude_layers(altitudes):
    altitudes = tuple(float(value) for value in altitudes)
    if len(altitudes) == 0:
        raise ValueError('At least one flight altitude must be given')
    if len(set(altitudes)) != len(altitudes):
        raise ValueError('Flight altitudes must be unique')
    return tuple((altitude, structural_map_name(altitude), planner_name(altitude)) for altitude in altitudes)
