import json
import argparse
import ast
from pathlib import Path

from uav_planner.map_generator import generate_grid
from uav_planner.sdf_parser import load_world_collisions


def _read_config(path):
    config = {}
    for line in Path(path).read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        key, value = (part.strip() for part in line.split(':', 1))
        if key == 'world_frame':
            config[key] = value
        else:
            try:
                config[key] = ast.literal_eval(value)
            except (ValueError, SyntaxError):
                config[key] = float(value)
    return config


def export_map(grid, output_dir, name='static_map'):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    width, height = grid['width'], grid['height']
    pgm_values = []
    free_value = min(grid['data']) if grid['data'] else 0
    for row in range(height - 1, -1, -1):
        for column in range(width):
            value = grid['data'][row * width + column]
            pgm_values.append(0 if value >= 50 else 254 if value == free_value else 205)
    (output_dir / f'{name}.pgm').write_text(
        f'P2\n{width} {height}\n255\n' +
        '\n'.join(' '.join(map(str, pgm_values[index:index + width]))
                  for index in range(0, len(pgm_values), width)) + '\n'
    )
    (output_dir / f'{name}.yaml').write_text(
        f'image: {name}.pgm\n'
        f'resolution: {grid["resolution"]}\n'
        f'origin: [{grid["origin"][0]}, {grid["origin"][1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
        f'mode: scale\n# frame_id: {grid["frame_id"]}\n'
    )
    (output_dir / f'{name}.json').write_text(json.dumps(grid))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world')
    parser.add_argument('--models')
    parser.add_argument('--config')
    parser.add_argument('--output', default='maps')
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory

    planner_share = Path(get_package_share_directory('uav_planner'))
    sim_share = Path(get_package_share_directory('uav_sim'))
    world = Path(args.world) if args.world else sim_share / 'worlds/electrical_substation.world'
    models = Path(args.models) if args.models else sim_share / 'models'
    config_path = Path(args.config) if args.config else planner_share / 'config/static_map.yaml'
    config = _read_config(config_path)
    obstacles = load_world_collisions(world, models)
    altitudes = config.pop('flight_altitudes', [config['flight_altitude']])
    for altitude in altitudes:
        layer_config = dict(config, flight_altitude=float(altitude))
        grid = generate_grid(obstacles, layer_config)
        name = f'static_map_{int(altitude)}m'
        if int(altitude) == 10:
            name = 'static_map'
        export_map(grid, args.output, name)
        occupied = sum(value >= 50 for value in grid['data'])
        print(f'{name}: {occupied} occupied cells from {len(obstacles)} collisions')
    print(f'Maps exported to {Path(args.output).resolve()}')


if __name__ == '__main__':
    main()
