import math


def gps_to_gazebo_enu(
    latitude_deg,
    longitude_deg,
    altitude_m,
    reference_latitude_deg,
    reference_longitude_deg,
    reference_altitude_m,
):
    values = (
        latitude_deg,
        longitude_deg,
        altitude_m,
        reference_latitude_deg,
        reference_longitude_deg,
        reference_altitude_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('GPS coordinates and reference must be finite')
    if not -90.0 <= latitude_deg <= 90.0:
        raise ValueError('Latitude must be between -90 and 90 degrees')
    if not -90.0 <= reference_latitude_deg <= 90.0:
        raise ValueError(
            'Reference latitude must be between -90 and 90 degrees'
        )
    if not -180.0 <= longitude_deg <= 180.0:
        raise ValueError('Longitude must be between -180 and 180 degrees')
    if not -180.0 <= reference_longitude_deg <= 180.0:
        raise ValueError(
            'Reference longitude must be between -180 and 180 degrees'
        )

    earth_radius_m = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = flattening * (2.0 - flattening)

    def to_ecef(latitude, longitude, altitude):
        latitude_rad = math.radians(latitude)
        longitude_rad = math.radians(longitude)
        sin_latitude = math.sin(latitude_rad)
        cos_latitude = math.cos(latitude_rad)
        prime_vertical_radius = earth_radius_m / math.sqrt(
            1.0 - eccentricity_squared * sin_latitude**2
        )
        return (
            (prime_vertical_radius + altitude)
            * cos_latitude
            * math.cos(longitude_rad),
            (prime_vertical_radius + altitude)
            * cos_latitude
            * math.sin(longitude_rad),
            (prime_vertical_radius * (1.0 - eccentricity_squared) + altitude)
            * sin_latitude,
        )

    point_ecef = to_ecef(latitude_deg, longitude_deg, altitude_m)
    reference_ecef = to_ecef(
        reference_latitude_deg,
        reference_longitude_deg,
        reference_altitude_m,
    )
    delta_x, delta_y, delta_z = (
        point - reference
        for point, reference in zip(point_ecef, reference_ecef)
    )

    reference_latitude_rad = math.radians(reference_latitude_deg)
    reference_longitude_rad = math.radians(reference_longitude_deg)
    sin_latitude = math.sin(reference_latitude_rad)
    cos_latitude = math.cos(reference_latitude_rad)
    sin_longitude = math.sin(reference_longitude_rad)
    cos_longitude = math.cos(reference_longitude_rad)

    east = -sin_longitude * delta_x + cos_longitude * delta_y
    north = (
        -sin_latitude * cos_longitude * delta_x
        - sin_latitude * sin_longitude * delta_y
        + cos_latitude * delta_z
    )
    up = (
        cos_latitude * cos_longitude * delta_x
        + cos_latitude * sin_longitude * delta_y
        + sin_latitude * delta_z
    )
    return east, north, up
