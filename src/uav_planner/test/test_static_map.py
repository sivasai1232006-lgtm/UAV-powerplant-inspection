from pathlib import Path

from uav_planner.map_generator import generate_grid
from uav_planner.sdf_parser import _compose, load_world_collisions


def _config(altitude=10.0):
    return {
        'flight_altitude': altitude,
        'resolution': 1.0,
        'uav_safety_radius': 0.5,
        'additional_clearance': 0.25,
        'x_min': -2.0,
        'x_max': 2.0,
        'y_min': -2.0,
        'y_max': 2.0,
        'world_frame': 'map',
        'occupied_value': 100,
        'free_value': 0,
        'unknown_value': -1,
    }


def test_compose_rotates_child_translation():
    result = _compose((1, 2, 3, 0, 0, 1.5707963267948966), (2, 0, 0, 0, 0, 0))
    assert abs(result[0] - 1.0) < 1e-8
    assert abs(result[1] - 4.0) < 1e-8
    assert abs(result[2] - 3.0) < 1e-8


def test_grid_marks_only_obstacles_in_flight_band():
    bounds = {'bounds': [-0.5, -0.5, 8.0, 0.5, 0.5, 8.5]}
    grid = generate_grid([bounds], _config())
    assert all(value == 0 for value in grid['data'])
    lower_grid = generate_grid(
        [bounds], _config(8.0)
    )
    assert any(value == 100 for value in lower_grid['data'])


def test_current_world_has_no_collision_at_ten_meter_slice():
    source_root = Path(__file__).resolve().parents[2]
    obstacles = load_world_collisions(
        source_root / 'uav_sim/worlds/electrical_substation.world',
        source_root / 'uav_sim/models',
    )
    grid = generate_grid(obstacles, {
        **_config(),
        'resolution': 0.25,
        'x_min': -40.0,
        'x_max': 45.0,
        'y_min': -70.0,
        'y_max': 40.0,
    })
    assert len(obstacles) == 10
    assert not any(value == 100 for value in grid['data'])


def test_eight_meter_layer_contains_tall_world_obstacles():
    source_root = Path(__file__).resolve().parents[2]
    obstacles = load_world_collisions(
        source_root / 'uav_sim/worlds/electrical_substation.world',
        source_root / 'uav_sim/models',
    )
    grid = generate_grid(obstacles, {
        **_config(8.0),
        'resolution': 0.25,
        'x_min': -40.0,
        'x_max': 45.0,
        'y_min': -70.0,
        'y_max': 40.0,
    })
    assert any(value == 100 for value in grid['data'])
