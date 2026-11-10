import pytest

from uav_mission.operating_bounds import OperatingBounds


REGION = {
    'x_min_offset_m': -40.0,
    'x_max_offset_m': 20.0,
    'y_min_offset_m': -80.0,
    'y_max_offset_m': 80.0,
    'z_min_m': 0.0,
    'z_max_m': 10.0,
}


@pytest.mark.parametrize(
    ('axis', 'edge', 'outside'),
    [
        ('x', -45.0, -45.000001),
        ('x', 15.0, 15.000001),
        ('y', -80.0, -80.000001),
        ('y', 80.0, 80.000001),
        ('z', 0.0, -0.000001),
        ('z', 10.0, 10.000001),
    ],
)
def test_operating_bounds_include_exact_edges_and_reject_outside(
    axis, edge, outside
):
    bounds = OperatingBounds.from_initial_position((-5.0, 0.0, 3.0), REGION)
    point = {'x': 0.0, 'y': 0.0, 'z': 5.0}
    point[axis] = edge
    assert bounds.contains((point['x'], point['y'], point['z']))

    point[axis] = outside
    assert not bounds.contains((point['x'], point['y'], point['z']))


def test_each_uav_gets_independent_bounds_from_latched_map_position():
    first = OperatingBounds.from_initial_position((-5.0, 2.0, 0.0), REGION)
    second = OperatingBounds.from_initial_position((5.0, -3.0, 0.0), REGION)

    assert first.as_parameters() == {
        'x_min': -45.0, 'x_max': 15.0,
        'y_min': -78.0, 'y_max': 82.0,
        'z_min': 0.0, 'z_max': 10.0,
    }
    assert second.as_parameters() == {
        'x_min': -35.0, 'x_max': 25.0,
        'y_min': -83.0, 'y_max': 77.0,
        'z_min': 0.0, 'z_max': 10.0,
    }


def test_z_bounds_are_absolute_not_offset_by_initial_altitude():
    bounds = OperatingBounds.from_initial_position((1.0, 2.0, 8.0), REGION)

    assert bounds.z_min == 0.0
    assert bounds.z_max == 10.0
    assert bounds.contains((1.0, 2.0, 10.0))
