"""Generate altitude-filtered 2.5D structural occupancy maps from an SDF world."""
import argparse
import json
from pathlib import Path

import yaml

from uav_planner.altitude_layers import altitude_layers
from uav_planner.map_generator import generate_grid
from uav_planner.sdf_parser import load_world_collisions


def _read_config(path):
    """Read the map configuration as YAML (the old hand parser was fragile)."""
    with Path(path).open(encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    if not isinstance(config, dict):
        raise ValueError(f'Map configuration must be a YAML mapping: {path}')
    required = {
        'world_frame', 'resolution', 'x_min', 'x_max', 'y_min', 'y_max',
        'free_value', 'occupied_value',
    }
    missing = sorted(required.difference(config))
    if missing:
        raise ValueError(f'Map configuration is missing: {", ".join(missing)}')
    if 'flight_altitudes' not in config and 'flight_altitude' not in config:
        raise ValueError('Configure flight_altitudes or flight_altitude')
    return config


def export_map(grid, output_dir, name='static_map'):
    """Export a Nav2 map-server-compatible PGM/YAML plus full JSON metadata."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    width, height = int(grid['width']), int(grid['height'])
    pgm_values = []
    # OccupancyGrid data starts at bottom-left; PGM rows start at top-left.
    for row in range(height - 1, -1, -1):
        for column in range(width):
            value = int(grid['data'][row * width + column])
            pgm_values.append(0 if value >= 50 else 127 if value < 0 else 254)
    pgm_path = output_dir / f'{name}.pgm'
    pgm_path.write_text(
        f'P2\n{width} {height}\n255\n'
        + '\n'.join(
            ' '.join(map(str, pgm_values[index:index + width]))
            for index in range(0, len(pgm_values), width)
        ) + '\n',
        encoding='ascii',
    )
    yaml_data = {
        'image': pgm_path.name,
        'resolution': float(grid['resolution']),
        'origin': [float(grid['origin'][0]), float(grid['origin'][1]), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'mode': 'scale',
    }
    (output_dir / f'{name}.yaml').write_text(
        yaml.safe_dump(yaml_data, sort_keys=False), encoding='utf-8'
    )
    (output_dir / f'{name}.json').write_text(
        json.dumps(grid, indent=2), encoding='utf-8'
    )


def export_altitude_maps(obstacles, config, output_dir, altitudes=None):
    configured_altitudes = (
        config.get('flight_altitudes', [config.get('flight_altitude')])
        if altitudes is None else altitudes
    )
    if not configured_altitudes or any(
        value is None for value in configured_altitudes
    ):
        raise ValueError('At least one flight altitude must be configured')
    layers = altitude_layers(configured_altitudes)
    generated = {}
    for altitude, map_name, _ in layers:
        grid = generate_grid(
            obstacles, dict(config, flight_altitude=altitude)
        )
        export_map(grid, output_dir, map_name)
        occupied = sum(value >= 50 for value in grid['data'])
        print(
            f'{map_name}: {occupied}/{len(grid["data"])} occupied cells; '
            f'flight band [{grid["vertical_band"][0]:.2f}, '
            f'{grid["vertical_band"][1]:.2f}] m; '
            f'{len(obstacles)} collision geometries'
        )
        generated[altitude] = grid
    return generated


def export_obstacle_database(obstacles, output_dir):
    """Save 3D collision extents and triangulated geometry for auditing/reuse."""
    output = []
    for index, obstacle in enumerate(obstacles):
        output.append({
            'id': index,
            'name': obstacle.get('name', f'obstacle_{index}'),
            'bounds_xyzxyz_m': [float(v) for v in obstacle['bounds']],
            'min_height_m': float(obstacle['bounds'][2]),
            'max_height_m': float(obstacle['bounds'][5]),
            'triangle_count': len(obstacle.get('triangles', ())),
        })
    path = Path(output_dir) / 'structural_obstacles.json'
    path.write_text(json.dumps({
        'schema': 'uav_structural_obstacles_v1',
        'coordinate_frame': 'world',
        'units': 'm',
        'obstacles': output,
    }, indent=2), encoding='utf-8')
    return path


def main():
    parser = argparse.ArgumentParser(
        description='Export 2D structural occupancy maps for multiple UAV flight altitudes.'
    )
    parser.add_argument('--world')
    parser.add_argument('--models')
    parser.add_argument('--config')
    parser.add_argument('--output', default='maps')
    parser.add_argument(
        '--altitudes', nargs='+', type=float,
        help='Override flight_altitudes from the map config (metres).'
    )
    args = parser.parse_args()

    from ament_index_python.packages import get_package_share_directory

    planner_share = Path(get_package_share_directory('uav_planner'))
    sim_share = Path(get_package_share_directory('uav_sim'))
    world = Path(args.world) if args.world else sim_share / 'worlds/electrical_substation.world'
    models = Path(args.models) if args.models else sim_share / 'models'
    config_path = Path(args.config) if args.config else planner_share / 'config/static_map.yaml'
    config = _read_config(config_path)
    obstacles = load_world_collisions(world, models)
    output_dir = Path(args.output)
    export_altitude_maps(obstacles, config, output_dir, args.altitudes)
    database = export_obstacle_database(obstacles, output_dir)
    print(f'3D obstacle metadata: {database.resolve()}')
    print(f'Maps exported to {output_dir.resolve()}')


if __name__ == '__main__':
    main()