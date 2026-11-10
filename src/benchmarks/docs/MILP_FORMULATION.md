# Shared allocation and route benchmark

The comparison is offline and estimate-only. All algorithms receive the same
mission tasks, UAV starting positions and battery levels, home positions,
Euclidean path cost with the configured safety factor, nominal speed, task
durations, battery consumption model, landing energy, workload limit, task
capacity, per-UAV operating bounds, minimum-separation rule, and base/UGV
return policy. They are scored with one shared route evaluator. No mission is
executed and no measured flight metrics are included in the comparison files.

## Algorithms

- **Existing** repeatedly invokes the project's `AuctionAllocator` and updates
  the hypothetical UAV position and battery after each simulated task
  completion, rather than treating a one-cycle assignment limit as a
  mission-wide limit.
- **Greedy** adds one highest-priority feasible task at a time and chooses the
  candidate with the lowest projected makespan, then estimated resource use.
- **MILP** jointly chooses task assignments and visit order. Binary assignment,
  start, end, and inter-task arc variables form one route per active UAV.
  MTZ ordering constraints prevent subtours. Its objective maximizes safe task
  coverage first, then task priority, then minimizes mission makespan, distance,
  and energy.

Battery reserve, workload, task capacity, operating bounds, and return safety
remain hard constraints for every method. The benchmark does not tune battery
parameters or weaken the configured post-landing reserve.

## Shared constraints and estimates

All routes start at that UAV's configured initial position. Each task goal and
the selected return destination must be inside that UAV's initial-position-
relative operating bounds. The estimated straight-line segments connect
in-bounds endpoints in the axis-aligned region. Each visit adds the same
safety-scaled Euclidean leg distance and inspection duration. The route returns
to that UAV's original takeoff position if predicted battery after travel and
landing retains the configured reserve. Otherwise it may use only the
configured UGV point, if that route retains the reserve. A route with no safe
return is infeasible.

Each UAV is subject to the same availability check, `max_tasks_per_uav`,
`maximum_workload_sec`, battery reserve and landing-energy model. Tasks closer
than `minimum_separation_m` may be assigned to the same UAV, but cannot be
split across different UAVs, matching the live allocator's constraint. The
safety factor is applied once by `PathCostClient` to each leg.

The MILP is a route-ordering assignment model, not a low-level motion planner.
It does not model obstacles, wind, acceleration, inspection dwell variation,
or PX4 flight dynamics. The path-cost client uses a safety-scaled Euclidean
estimate rather than Nav2 obstacle-aware path lengths; mission-time validation
and PX4 execution are still required.

PuLP and CBC are required to solve the MILP. On Ubuntu, for example:

```bash
sudo apt install coinor-cbc python3-pulp
```

The runtime accepts `--milp-time-limit-sec`. If CBC is unavailable or does not
return a feasible incumbent, the result records that status and the Existing
and Greedy results are still written.
