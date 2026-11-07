from uav_allocator.auction_allocator import (
    AuctionAllocator,
    InspectionTask,
    UavSnapshot,
)
from uav_allocator.battery_feasibility import BatteryFeasibility
from uav_allocator.cost_function import CostFunction
from uav_allocator.path_cost_client import PathCostClient
from uav_allocator.workload_estimator import WorkloadEstimator
from uav_interfaces.msg import UAVState


def make_allocator():
    return AuctionAllocator(
        PathCostClient(),
        WorkloadEstimator(2.0),
        BatteryFeasibility(0.05, 0.25, 20.0),
        CostFunction(
            {'distance': 0.4, 'time': 0.25, 'energy': 0.25, 'priority': 0.1},
            100.0,
            300.0,
            10.0,
            1,
            5,
        ),
        3.0,
    )


def make_uav(uav_id, x, battery=100.0, status=UAVState.IDLE, age=0.1):
    return UavSnapshot(
        uav_id,
        (x, 0.0, 0.0),
        battery,
        True,
        True,
        status,
        age,
    )


def make_task(task_id='task', x=10.0, priority=1, duration=30.0):
    return InspectionTask(task_id, (x, 0.0, 0.0), priority, duration)


def test_two_uavs_compete_and_lowest_bid_wins():
    result = make_allocator().allocate(
        [make_task()],
        [make_uav('uav1', 0.0), make_uav('uav2', 8.0)],
        {'uav1': (0.0, 0.0, 0.0), 'uav2': (8.0, 0.0, 0.0)},
    )

    assert result[0].uav_id == 'uav2'
    assert result[0].outbound_distance_m == 2.0
    assert result[0].return_distance_m == 2.0


def test_shorter_path_wins_when_other_inputs_match():
    result = make_allocator().allocate(
        [make_task(x=10.0)],
        [make_uav('farther', 0.0), make_uav('closer', 8.0)],
        {'farther': (10.0, 0.0, 0.0), 'closer': (10.0, 0.0, 0.0)},
    )

    assert result[0].uav_id == 'closer'


def test_higher_priority_has_lower_priority_penalty():
    cost = make_allocator().cost_function

    assert cost.calculate(10.0, 10.0, 1.0, 5) < cost.calculate(
        10.0, 10.0, 1.0, 1
    )


def test_insufficient_battery_rejects_bid():
    result = make_allocator().allocate(
        [make_task(x=100.0)],
        [make_uav('uav1', 0.0, battery=25.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None
    assert 'battery' in result[0].reason


def test_return_to_base_energy_is_included_in_feasibility():
    feasibility = BatteryFeasibility(0.05, 0.0, 20.0)
    result = feasibility.evaluate(25.0, 1.0, 1000.0, 0.0)

    assert not result.feasible
    assert result.required_percentage > 25.0


def test_stale_uav_state_is_excluded():
    result = make_allocator().allocate(
        [make_task()],
        [make_uav('uav1', 0.0, age=4.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None
    assert 'stale' in result[0].reason


def test_executing_uav_is_excluded():
    result = make_allocator().allocate(
        [make_task()],
        [make_uav('uav1', 0.0, status=UAVState.EXECUTING)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None


def test_unassigned_when_no_candidate_is_feasible():
    result = make_allocator().allocate(
        [make_task(x=100.0)],
        [make_uav('uav1', 0.0, battery=20.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None


def test_task_is_not_assigned_twice_in_a_cycle():
    result = make_allocator().allocate(
        [make_task(), make_task()],
        [make_uav('uav1', 0.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id == 'uav1'
    assert result[1].uav_id is None


def test_missing_planner_uses_provisional_euclidean_estimator():
    path_cost = PathCostClient()

    assert path_cost.is_provisional
    assert path_cost.estimate_distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == 5.0


def test_same_inputs_produce_repeatable_allocations():
    allocator = make_allocator()
    tasks = [make_task()]
    uavs = [make_uav('uav1', 0.0), make_uav('uav2', 8.0)]
    homes = {'uav1': (0.0, 0.0, 0.0), 'uav2': (8.0, 0.0, 0.0)}

    first = allocator.allocate(tasks, uavs, homes)
    second = allocator.allocate(tasks, uavs, homes)

    assert first == second
