import math

from uav_info.coordinate_transform import (
    enu_to_ned,
    gps_to_gazebo_enu,
    local_ned_to_map,
    map_to_local_ned,
    ned_to_enu,
)


MAP_REFERENCE = (47.397971057728974, 8.546163739800146, 0.0)


def test_ned_to_enu_swaps_horizontal_axes_and_inverts_down():
    assert ned_to_enu((2.0, 3.0, 4.0)) == (3.0, 2.0, -4.0)
    assert enu_to_ned(ned_to_enu((2.0, 3.0, 4.0))) == (2.0, 3.0, 4.0)


def test_gps_origin_maps_to_zero_and_small_gps_displacement_is_metric():
    latitude, longitude, altitude = MAP_REFERENCE
    assert all(abs(value) < 1e-8 for value in gps_to_gazebo_enu(
        latitude, longitude, altitude, *MAP_REFERENCE
    ))

    north_displacement = gps_to_gazebo_enu(
        latitude + 0.00001, longitude, altitude, *MAP_REFERENCE
    )
    assert 1.0 < north_displacement[1] < 1.2
    assert abs(north_displacement[0]) < 1e-4


def test_local_ned_and_map_point_round_trip_with_distinct_origins():
    ref_lat, ref_lon, ref_alt = (
        MAP_REFERENCE[0] + 0.00004,
        MAP_REFERENCE[1] + 0.00005,
        1.5,
    )
    ned = (12.0, -7.0, -5.0)
    point_map = local_ned_to_map(
        *ned, ref_lat, ref_lon, ref_alt, *MAP_REFERENCE
    )
    recovered_ned = map_to_local_ned(
        *point_map, ref_lat, ref_lon, ref_alt, *MAP_REFERENCE
    )
    for actual, expected in zip(recovered_ned, ned):
        assert math.isclose(actual, expected, abs_tol=1e-6)


def test_different_uav_local_origins_share_the_same_map_coordinates():
    reference_a = MAP_REFERENCE
    reference_b = (
        MAP_REFERENCE[0] + 0.00002,
        MAP_REFERENCE[1] - 0.00003,
        0.0,
    )
    point_a = local_ned_to_map(
        0.0, 0.0, 0.0, *reference_a, *MAP_REFERENCE
    )
    point_b = local_ned_to_map(
        0.0, 0.0, 0.0, *reference_b, *MAP_REFERENCE
    )
    expected_b = gps_to_gazebo_enu(*reference_b, *MAP_REFERENCE)
    for actual, expected in zip(point_b, expected_b):
        assert math.isclose(actual, expected, abs_tol=1e-6)
    assert math.dist(point_a, point_b) > 3.0


def test_map_reference_and_local_reference_rotation_are_inverse():
    from uav_info.coordinate_transform import local_enu_to_map_rotation

    rotation = local_enu_to_map_rotation(
        MAP_REFERENCE[0] + 0.001,
        MAP_REFERENCE[1] - 0.001,
        MAP_REFERENCE[0],
        MAP_REFERENCE[1],
    )
    for row in range(3):
        norm = sum(rotation[row][column] ** 2 for column in range(3))
        assert math.isclose(norm, 1.0, abs_tol=1e-12)
