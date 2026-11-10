"""Combine per-run benchmark JSON files into cross-scenario tables."""

import argparse
import csv
import json
from pathlib import Path

from benchmarks.runner import _source_workspace_root


def collect(results_dir):
    rows = []
    pattern = '*tasks*waypoints/run_*/comparison.json'
    for path in sorted(Path(results_dir).glob(pattern)):
        try:
            comparison = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            continue
        if comparison.get('comparison_model_version') != 'coverage_bounds_v3':
            # Older files used a different MILP route model and included
            # unavailable flight-only columns. Do not mix those estimates into
            # current cross-run summaries.
            continue
        for algorithm, metrics in comparison.get('algorithms', {}).items():
            rows.append({
                'run_id': comparison.get('run_id', path.parent.name),
                'task_count': comparison.get('task_count'),
                'waypoint_count': comparison.get('waypoint_count'),
                'algorithm': algorithm,
                'status': metrics.get('status'),
                'assigned_tasks': metrics.get(
                    'assigned_task_count', metrics.get('assigned_tasks')
                ),
                'unassigned_tasks': metrics.get(
                    'unassigned_task_count', metrics.get('unassigned_tasks')
                ),
                'allocation_time_sec': metrics.get(
                    'allocation_computation_time_sec',
                    metrics.get('allocation_time_sec'),
                ),
                'estimated_distance_m': metrics.get(
                    'estimated_total_distance_m',
                    metrics.get('estimated_distance_m'),
                ),
                'estimated_energy_percent': metrics.get(
                    'estimated_energy_consumption_percent',
                    metrics.get('estimated_energy_percent'),
                ),
                'solver_status': metrics.get('solver', {}).get('status'),
                'feasibility_status': metrics.get('feasibility_status'),
                'battery_violation_count': metrics.get('battery_violation_count'),
                'safety_violation_count': metrics.get('safety_violation_count'),
                'milp_optimality_gap': metrics.get('milp_optimality_gap'),
                'uav_velocity_multiplier': comparison.get(
                    'uav_velocity_multiplier'
                ),
            })
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', default='')
    parser.add_argument('--output-csv', default='')
    args = parser.parse_args(argv)
    root = (
        Path(args.results_dir)
        if args.results_dir
        else _source_workspace_root() / 'sim_results'
    )
    rows = collect(root)
    if not rows:
        print(f'No benchmark comparison files found under {root}')
        return 1
    output = Path(args.output_csv) if args.output_csv else root / 'comparison_all.csv'
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    json_path = output.with_suffix('.json')
    json_path.write_text(json.dumps(rows, indent=2) + '\n', encoding='utf-8')
    print(f'Wrote {len(rows)} algorithm/run rows to {output} and {json_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
