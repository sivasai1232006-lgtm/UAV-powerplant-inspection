# UAV task allocation benchmarks

The benchmark compares three offline task allocation methods using estimates.
All methods receive the same mission tasks, UAV starts and battery levels,
return locations, and allocator parameters. Their proposed plans are checked
and scored with the same route evaluator, distance model, battery and landing
energy model, workload/capacity limits, minimum-separation rule, and base/UGV
return policy. The benchmark does not command UAVs or use measured flight
data. See [`docs/MILP_FORMULATION.md`](docs/MILP_FORMULATION.md) for the
objective and constraints.

Build from the workspace root and run:

```bash
colcon build --packages-up-to benchmarks
source install/setup.bash
ros2 launch benchmarks benchmark.launch.py
```

The launch file accepts `mission_config`, `allocator_config`, `results_dir`,
`milp_time_limit_sec`, and `max_tasks_per_uav` (`0` means the task-set size).
To combine historical run summaries:

```bash
ros2 run benchmarks compare_benchmarks
```

Run artifacts are created under `src/sim_results/[N]tasks[M]waypoints/` in a
unique `run_YYYYMMDD_HHMMSS_microseconds/` folder. The primary side-by-side
comparison is written directly in that folder as `algorithm_comparison.csv`,
`algorithm_comparison.json`, and `comparison_summary.txt`. The CSV has exactly
one aggregate row per algorithm and contains no per-task assignments. The
older `comparison.csv`, `comparison.json`, and `summary.txt` files remain for
compatibility. Per-algorithm aggregate `metrics.json` and `summary.txt` files
are stored in algorithm subfolders. Per-task assignment CSVs are omitted unless
`--write-debug-task-details` is passed to `run_benchmark`.

The algorithm comparison contains estimated mission time, distance, and battery
use, plus measured offline allocation computation time. It contains no actual
flight or task-completion columns, since only one algorithm is flown in the
Gazebo mission. Estimated winners are labeled as estimates.

The runtime mission accepts inspection tasks only. It transits at the configured
cruise altitude, inspects at `object_height_m + 1.5 m`, and returns to cruise
altitude before continuing. Mission loading rejects targets that violate the
configured vertical clearance or altitude ceiling. Battery feasibility uses a
10% post-landing reserve; return priorities are the configured UAV home, the
configured UGV, then any explicitly validated nearby landing option. Navigation
failures are retried at most twice before a task is marked failed.
Auction bids use a provisional direct-horizontal estimate with the cruise
climb/descent distances and configured safety factor; Nav2 remains the
authority for executable task and return paths.

The global Gazebo mission launch runs this offline comparison alongside the
simulation. Separate mission telemetry from the flown run is saved to
`src/sim_results/mission_runs/run_*/`. After both UAVs confirm a successful
landing, it waits the configurable `simulation_shutdown_delay_sec` (default
`2.0`), saves final mission telemetry, and waits until the comparison's CSV,
JSON, summaries, and per-algorithm metrics are complete before requesting a
graceful launch shutdown. The standalone mission launch does not require a
benchmark result. A graceful manual shutdown saves a partial run with
`run_status: INTERRUPTED`. You can override the destination with:

```bash
ros2 launch uav_planner global_mission.launch.py \
  results_dir:=/path/to/sim_results
```

Mission task-level logs are omitted by default. Opt in when debugging with
`save_task_level_results:=true`; this writes `task_results.csv` separately.
Each actual run contains `mission_metrics.json` and `summary.txt`. A forced
process kill cannot run the shutdown save hook.

CBC and PuLP are required to solve the MILP. See
[`docs/MILP_FORMULATION.md`](docs/MILP_FORMULATION.md) for the model and solver
setup. If CBC is unavailable, a failure record is written and the baseline and
Greedy evaluations still complete.
