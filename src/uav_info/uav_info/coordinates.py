import math

_WGS84_SEMI_MAJOR_M = 6378137.0
_WGS84_FLATTENING = 1.0 / 298.257223563
_WGS84_ECCENTRICITY_SQUARED = (
    _WGS84_FLATTENING * (2.0 - _WGS84_FLATTENING)
)

def _geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = _WGS84_SEMI_MAJOR_M / math.sqrt(
        1.0 - _WGS84_ECCENTRICITY_SQUARED * sin_latitude**2
    )
    return (
        (radius + altitude_m) * cos_latitude * math.cos(longitude),
        (radius + altitude_m) * cos_latitude * math.sin(longitude),
        (radius * (1.0 - _WGS84_ECCENTRICITY_SQUARED) + altitude_m)
        * sin_latitude,
    )

def _validate_geodetic(latitude_deg, longitude_deg):
    if not -90.0 <= latitude_deg <= 90.0:
        raise ValueError('Latitude must be between -90 and 90 degrees')
    if not -180.0 <= longitude_deg <= 180.0:
        raise ValueError('Longitude must be between -180 and 180 degrees')

def _enu_basis(latitude_deg, longitude_deg):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    return (
        (-sin_longitude, -sin_latitude * cos_longitude,
         cos_latitude * cos_longitude),
        (cos_longitude, -sin_latitude * sin_longitude,
         cos_latitude * sin_longitude),
        (0.0, cos_latitude, sin_latitude),
    )

def _mat_vec(matrix, vector):
    return tuple(
        sum(row[index] * vector[index] for index in range(3))
        for row in matrix
    )

def _transpose(matrix):
    return tuple(zip(*matrix))

def rotation_matrix_to_quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = (
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(
            1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]
        ) * 2.0
        result = (
            (matrix[2][1] - matrix[1][2]) / scale,
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
        )
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(
            1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]
        ) * 2.0
        result = (
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
        )
    else:
        scale = math.sqrt(
            1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]
        ) * 2.0
        result = (
            (matrix[1][0] - matrix[0][1]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
        )
    norm = math.sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)

def local_enu_yaw_in_map(rotation):
    return math.atan2(rotation[1][0], rotation[0][0])

def _rotate_basis_vector(basis, vector):
    return tuple(
        sum(basis[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )

def local_enu_to_map_rotation(reference_latitude_deg, reference_longitude_deg, map_latitude_deg, map_longitude_deg,):
    _validate_geodetic(reference_latitude_deg, reference_longitude_deg)
    _validate_geodetic(map_latitude_deg, map_longitude_deg)
    map_basis = _enu_basis(map_latitude_deg, map_longitude_deg)
    local_basis = _enu_basis(reference_latitude_deg, reference_longitude_deg)
    return tuple(
        tuple(
            sum(map_basis[axis][row] * local_basis[axis][column]
                for axis in range(3))
            for column in range(3)
        )
        for row in range(3)
    )

def local_origin_to_map(reference_latitude_deg, reference_longitude_deg, reference_altitude_m, map_latitude_deg, map_longitude_deg, map_altitude_m):
    return gps_to_gazebo_enu(reference_latitude_deg,reference_longitude_deg,reference_altitude_m,map_latitude_deg,map_longitude_deg,map_altitude_m,)

def local_ned_to_map(
    north_m,east_m, down_m, reference_latitude_deg, reference_longitude_deg, reference_altitude_m, map_latitude_deg, map_longitude_deg, map_altitude_m,):
    coordinates = ( north_m, east_m, down_m, reference_latitude_deg, reference_longitude_deg, reference_altitude_m, map_latitude_deg, map_longitude_deg, map_altitude_m,)
    if not all(math.isfinite(float(value)) for value in coordinates):
        raise ValueError('NED coordinates and geodetic references must be finite')
    _validate_geodetic(reference_latitude_deg, reference_longitude_deg)
    _validate_geodetic(map_latitude_deg, map_longitude_deg)
    local_origin = _geodetic_to_ecef( reference_latitude_deg, reference_longitude_deg, reference_altitude_m )
    map_origin = _geodetic_to_ecef( map_latitude_deg, map_longitude_deg, map_altitude_m)
    local_enu = (east_m, north_m, -down_m)
    local_basis = _enu_basis(reference_latitude_deg, reference_longitude_deg)
    point_ecef = tuple(
        local_origin[axis] + sum(local_basis[axis][column] * local_enu[column] for column in range(3)) - map_origin[axis] for axis in range(3) )
    return _mat_vec(_transpose(_enu_basis(map_latitude_deg, map_longitude_deg)), point_ecef,)


def map_to_local_ned(east_m, north_m, up_m, reference_latitude_deg, reference_longitude_deg, reference_altitude_m, map_latitude_deg, map_longitude_deg,  map_altitude_m,):
    coordinates = (
        east_m, north_m, up_m, reference_latitude_deg,
        reference_longitude_deg, reference_altitude_m,
        map_latitude_deg, map_longitude_deg, map_altitude_m,
    )
    if not all(math.isfinite(float(value)) for value in coordinates):
        raise ValueError('Map coordinates and geodetic references must be finite' )
    _validate_geodetic(reference_latitude_deg, reference_longitude_deg)
    _validate_geodetic(map_latitude_deg, map_longitude_deg)
    map_origin = _geodetic_to_ecef(map_latitude_deg, map_longitude_deg, map_altitude_m)
    local_origin = _geodetic_to_ecef(reference_latitude_deg, reference_longitude_deg, reference_altitude_m,)
    map_basis = _enu_basis(map_latitude_deg, map_longitude_deg)
    local_basis = _enu_basis(reference_latitude_deg, reference_longitude_deg)
    map_offset_ecef = _rotate_basis_vector( map_basis, (east_m, north_m, up_m))
    delta_ecef = tuple( map_origin[axis] + map_offset_ecef[axis] - local_origin[axis] for axis in range(3))
    local_enu = _mat_vec(_transpose(local_basis), delta_ecef)
    return local_enu[1], local_enu[0], -local_enu[2]

def gps_to_gazebo_enu(latitude_deg, longitude_deg, altitude_m, reference_latitude_deg, reference_longitude_deg, reference_altitude_m):
    values = (latitude_deg, longitude_deg, altitude_m, reference_latitude_deg, reference_longitude_deg, reference_altitude_m,)
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
        prime_vertical_radius = earth_radius_m / math.sqrt(1.0 - eccentricity_squared * sin_latitude**2)
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
    reference_ecef = to_ecef(reference_latitude_deg,reference_longitude_deg,reference_altitude_m,)
    delta_x, delta_y, delta_z = (point - reference for point, reference in zip(point_ecef, reference_ecef))

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
