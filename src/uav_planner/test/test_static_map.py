import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import yaml

from uav_planner.altitude_layers import altitude_layers
from uav_planner.grid_exporter import export_altitude_maps
from uav_planner.map_generator import (
    fuse_occupancy_grids,
    generate_grid,
)
from uav_planner.map_fuser import apply_xy_bounds
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
    bounds = {'bounds': [-0.5, -0.5, 7.0, 0.5, 0.5, 7.5]}
    grid = generate_grid([bounds], _config())
    assert all(value == 0 for value in grid['data'])
    lower_grid = generate_grid(
        [bounds], _config(7.0)
    )
    assert any(value == 100 for value in lower_grid['data'])


def test_grid_leaves_horizontal_clearance_to_nav2_costmap_once():
    bounds = {'bounds': [-0.5, -0.5, 7.0, 0.5, 0.5, 7.5]}
    grid = generate_grid([bounds], _config(7.0))
    column = int((-1.5 - grid['origin'][0]) / grid['resolution'])
    row = int((0.0 - grid['origin'][1]) / grid['resolution'])
    assert grid['data'][row * grid['width'] + column] == 0


def test_height_filter_ignores_beam_above_and_keeps_intersecting_pole():
    beam = {'bounds': [0.0, 0.0, 3.0, 1.0, 0.25, 3.3]}
    pole = {'bounds': [2.0, 0.0, 0.0, 2.25, 0.25, 5.0]}
    grid = generate_grid(
        [beam, pole], {**_config(2.0), 'x_max': 3.0}
    )
    beam_column = int((0.5 - grid['origin'][0]) / grid['resolution'])
    pole_column = int((2.0 - grid['origin'][0]) / grid['resolution'])
    row = int((0.0 - grid['origin'][1]) / grid['resolution'])
    assert grid['data'][row * grid['width'] + beam_column] == 0
    assert grid['data'][row * grid['width'] + pole_column] == 100


def test_fusion_preserves_slam_cells_and_overlays_structural_obstacles():
    base = {
        'frame_id': 'map', 'resolution': 1.0, 'width': 2, 'height': 2,
        'origin': [0.0, 0.0, 0.0], 'data': [0, -1, 0, 100],
    }
    structure = {
        'frame_id': 'map', 'resolution': 0.5, 'width': 4, 'height': 4,
        'origin': [0.0, 0.0, 0.0],
        'data': [0, 0, 0, 0, 0, 100, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0],
    }
    fused = fuse_occupancy_grids(base, structure)
    assert fused['data'][0] == 100
    assert fused['data'][1] == -1
    assert fused['data'][2] == 0
    assert fused['data'][3] == 100
    assert fused['width'] == 2 and fused['height'] == 2


def test_powerplant_assembly_collision_uses_its_world_pose_and_mesh_bounds():
    source_root = Path(__file__).resolve().parents[2]
    obstacles = load_world_collisions(
        source_root / 'uav_sim/worlds/electrical_substation.world',
        source_root / 'uav_sim/models',
    )
    assembly = next(
        item for item in obstacles
        if item['name'] == 'powerplant_assembly_1'
    )
    expected = (
        -28.88, 10.21, -2.20,
        14.89, 64.71, 17.05,
    )
    assert all(
        abs(actual - target) < 0.03
        for actual, target in zip(assembly['bounds'], expected)
    )


def test_powerplant_mesh_preserves_open_space_inside_its_bounding_box():
    source_root = Path(__file__).resolve().parents[2]
    obstacles = load_world_collisions(
        source_root / 'uav_sim/worlds/electrical_substation.world',
        source_root / 'uav_sim/models',
    )
    grid = generate_grid(obstacles, {
        **_config(7.0),
        'resolution': 0.25,
        'x_min': -40.0,
        'x_max': 45.0,
        'y_min': -70.0,
        'y_max': 40.0,
    })

    def occupied(x, y):
        column = int((x - grid['origin'][0]) / grid['resolution'])
        row = int((y - grid['origin'][1]) / grid['resolution'])
        return grid['data'][row * grid['width'] + column] >= 50

    # The old AABB raster marked the whole assembly rectangle as lethal.
    # This point is inside that rectangle but outside the collision mesh.
    assert not occupied(-4.0, 13.0)
    assert not occupied(16.0, 13.0)
    assert not occupied(-5.0, 0.0)

    start = (int((16.0 + 40.0) / 0.25), int((13.0 + 70.0) / 0.25))
    goal = (int((-5.0 + 40.0) / 0.25), int((0.0 + 70.0) / 0.25))
    pending = [start]
    visited = {start}
    while pending:
        column, row = pending.pop()
        if (column, row) == goal:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                next_cell = (column + dx, row + dy)
                if next_cell in visited:
                    continue
                x, y = next_cell
                if not (0 <= x < grid['width'] and 0 <= y < grid['height']):
                    continue
                if grid['data'][y * grid['width'] + x] >= 50:
                    continue
                visited.add(next_cell)
                pending.append(next_cell)
    assert goal in visited


def test_seven_meter_layer_contains_world_obstacles():
    source_root = Path(__file__).resolve().parents[2]
    obstacles = load_world_collisions(
        source_root / 'uav_sim/worlds/electrical_substation.world',
        source_root / 'uav_sim/models',
    )
    grid = generate_grid(obstacles, {
        **_config(7.0),
        'resolution': 0.25,
        'x_min': -40.0,
        'x_max': 45.0,
        'y_min': -70.0,
        'y_max': 40.0,
    })
    assert any(value == 100 for value in grid['data'])


def test_powerplant_assembly_is_present_in_each_flight_altitude_layer():
    source_root = Path(__file__).resolve().parents[2]
    obstacles = load_world_collisions(
        source_root / 'uav_sim/worlds/electrical_substation.world',
        source_root / 'uav_sim/models',
    )
    for altitude in (3.0, 5.0, 7.0, 10.0):
        grid = generate_grid(obstacles, {
            **_config(altitude),
            'resolution': 0.25,
            'x_min': -40.0,
            'x_max': 45.0,
            'y_min': -70.0,
            'y_max': 40.0,
        })
        x_min, y_min, _, x_max, y_max, _ = next(
            item['bounds'] for item in obstacles
            if item['name'] == 'powerplant_assembly_1'
        )
        first_column = int((x_min - grid['origin'][0]) / grid['resolution'])
        last_column = int((x_max - grid['origin'][0]) / grid['resolution'])
        first_row = int((y_min - grid['origin'][1]) / grid['resolution'])
        last_row = int((y_max - grid['origin'][1]) / grid['resolution'])
        assert any(
            grid['data'][row * grid['width'] + column] == 100
            for row in range(max(0, first_row), min(grid['height'], last_row + 1))
            for column in range(max(0, first_column),
                                min(grid['width'], last_column + 1))
        )


def test_exporter_generates_all_four_map_files_with_correct_altitude_data(
    tmp_path,
):
    config = {
        **_config(),
        'flight_altitudes': [3.0, 5.0, 7.0, 10.0],
    }

    generated = export_altitude_maps(
        [{'bounds': [-0.5, -0.5, 4.0, 0.5, 0.5, 4.5]}],
        config,
        tmp_path,
    )

    layers = altitude_layers(config['flight_altitudes'])
    assert tuple(generated) == (3.0, 5.0, 7.0, 10.0)
    assert [map_name for _, map_name, _ in layers] == [
        'static_map_3m',
        'static_map_5m',
        'static_map_7m',
        'static_map',
    ]
    for altitude, map_name, _ in layers:
        metadata = json.loads(
            (tmp_path / f'{map_name}.json').read_text(encoding='utf-8')
        )
        map_yaml = yaml.safe_load(
            (tmp_path / f'{map_name}.yaml').read_text(encoding='utf-8')
        )
        assert metadata['flight_altitude'] == altitude
        assert (tmp_path / map_yaml['image']).is_file()


def test_both_uavs_have_planners_using_the_matching_fused_map():
    package_root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load(
        (package_root / 'config/static_map.yaml').read_text(encoding='utf-8')
    )
    layers = altitude_layers(config['flight_altitudes'])
    assert config['flight_altitude'] == 7.0
    assert tuple(config['flight_altitudes']) == (3.0, 5.0, 7.0, 10.0)

    for altitude, _, planner_name in layers:
        for uav_id in ('uav_1', 'uav_2'):
            params = yaml.safe_load(
                (
                    package_root / 'config'
                    / f'nav2_planner_{uav_id}_{altitude:g}m.yaml'
                ).read_text(encoding='utf-8')
            )
            costmap = params[
                f'/{uav_id}/{planner_name}/global_costmap/global_costmap'
            ]['ros__parameters']
            assert costmap['static_layer']['map_topic'] == (
                f'/{uav_id}/{planner_name}/map'
            )
            scan_name = f'{uav_id}_scan'
            scan = costmap['obstacle_layer'][scan_name]
            assert scan['topic'] == (
                f'/{planner_name}/{uav_id}/scan'
            )
            assert scan['data_type'] == 'LaserScan'


def test_static_planner_launch_connects_maps_fusers_and_both_uavs(
    monkeypatch,
):
    package_root = Path(__file__).resolve().parents[1]
    launch_path = package_root / 'launch/static_planner.launch.py'
    spec = importlib.util.spec_from_file_location(
        'static_planner_launch_test', launch_path
    )
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    monkeypatch.setattr(
        launch_module,
        'get_package_share_directory',
        lambda _package: str(package_root),
    )
    monkeypatch.setattr(
        launch_module, 'LaunchDescription', lambda entities: entities
    )
    monkeypatch.setattr(
        launch_module,
        'Node',
        lambda **kwargs: SimpleNamespace(**kwargs),
    )

    nodes = launch_module.generate_launch_description()
    map_servers = [
        node for node in nodes if node.package == 'nav2_map_server'
    ]
    fusers = [
        node for node in nodes
        if node.package == 'uav_planner'
        and node.executable == 'altitude_map_fuser'
    ]
    planners = [
        node for node in nodes
        if node.package == 'nav2_planner'
        and node.executable == 'planner_server'
    ]

    assert len(map_servers) == 4
    assert len(fusers) == 8
    assert len(planners) == 8
    for node in map_servers:
        map_config = Path(node.parameters[0]['yaml_filename'])
        assert map_config.is_file()
        assert node.remappings == [('map', 'structural_map')]
    for node in fusers:
        params = node.parameters[0]
        assert params['structural_map_topic'].endswith(
            '/structural_map'
        )
        assert params['output_map_topic'] == (
            f"/{node.namespace}/"
            f"{params['structural_map_topic'].split('/')[1]}/map"
        )
    for node in planners:
        config_path = Path(node.parameters[0])
        assert config_path.is_file()
        uav_id, planner_name = node.namespace.split('/')
        assert uav_id in ('uav_1', 'uav_2')
        with config_path.open(encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)
        costmap = config[
            f'/{node.namespace}/global_costmap/global_costmap'
        ]['ros__parameters']
        assert costmap['static_layer']['map_topic'] == (
            f'/{uav_id}/{planner_name}/map'
        )


def test_apply_xy_bounds_marks_cells_outside_region_occupied():
    bounded = apply_xy_bounds(
        [0] * 16,
        width=4,
        height=4,
        resolution=1.0,
        origin=(0.0, 0.0),
        bounds={
            'x_min': 1.0,
            'x_max': 3.0,
            'y_min': 1.0,
            'y_max': 3.0,
        },
    )

    assert bounded == [
        100, 100, 100, 100,
        100, 0, 0, 100,
        100, 0, 0, 100,
        100, 100, 100, 100,
    ]
