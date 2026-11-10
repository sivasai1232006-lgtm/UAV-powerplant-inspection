import pytest

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


def make_allocator(future_weight=0.25, planning_batch_size=4):
    return AuctionAllocator(
        PathCostClient(cruise_altitude_m=0.0),
        WorkloadEstimator(2.0),
        BatteryFeasibility(0.05, 0.25, 10.0),
        CostFunction(
            {
                'distance': 0.4,
                'time': 0.25,
                'energy': 0.25,
                'priority': 0.1,
                'battery': 0.2,
                'workload': 0.15,
                'future': future_weight,
            },
            100.0,
            300.0,
            10.0,
            1,
            5,
        ),
        3.0,
        planning_batch_size=planning_batch_size,
    )


def make_uav(
    uav_id,
    x,
    battery=100.0,
    status=UAVState.IDLE,
    age=0.1,
    current_task=None,
    queued_tasks=(),
    inspecting=False,
    remaining_inspection_sec=0.0,
):
    return UavSnapshot(
        uav_id,
        (x, 0.0, 0.0),
        battery,
        True,
        True,
        status,
        age,
        current_task=current_task,
        queued_tasks=tuple(queued_tasks),
        current_task_inspecting=inspecting,
        remaining_inspection_sec=remaining_inspection_sec,
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
    assert result[0].outbound_distance_m == 2.6
    assert result[0].return_distance_m == pytest.approx(2.86)


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


def test_configured_home_height_is_converted_to_landing_altitude():
    result = make_allocator().allocate(
        [make_task(x=10.0, duration=0.0)],
        [make_uav('uav1', 0.0, battery=30.0)],
        {'uav1': (0.0, 0.0, 3.2)},
    )

    assert result[0].uav_id == 'uav1'
    assert result[0].return_distance_m == pytest.approx(13.26)


def test_post_landing_reserve_and_landing_energy_are_both_required():
    feasibility = BatteryFeasibility(0.05, 0.0, 10.0, 2.0)

    assert feasibility.evaluate(12.0, 0.0, 0.0, 0.0).feasible
    result = feasibility.evaluate(11.9, 0.0, 0.0, 0.0)

    assert not result.feasible
    assert result.required_percentage == 12.0


def test_stale_uav_state_is_excluded():
    result = make_allocator().allocate(
        [make_task()],
        [make_uav('uav1', 0.0, age=4.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None
    assert 'stale' in result[0].reason


def test_travelling_uav_with_active_task_cannot_participate_in_bidding():
    result = make_allocator().allocate(
        [make_task()],
        [make_uav(
            'uav1', 0.0, status=UAVState.EXECUTING,
            current_task=make_task('current', x=5.0),
        )],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None
    assert 'not ready' in result[0].reason


def test_inspecting_uav_bids_from_its_projected_queue_endpoint():
    inspecting = make_uav(
        'inspecting',
        0.0,
        status=UAVState.INSPECTING,
        current_task=make_task('current', x=10.0),
        inspecting=True,
        remaining_inspection_sec=5.0,
    )
    idle = make_uav('idle', 0.0)
    result = make_allocator().allocate(
        [make_task('new', x=21.0)],
        [inspecting, idle],
        {'inspecting': (0.0, 0.0, 0.0), 'idle': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id == 'inspecting'
    assert result[0].outbound_distance_m == pytest.approx(14.3)


def test_inspection_queue_is_included_in_hard_battery_feasibility():
    result = make_allocator().allocate(
        [make_task('new', x=1.0)],
        [make_uav(
            'busy', 0.0, battery=15.0,
            status=UAVState.INSPECTING,
            current_task=make_task('current', x=10.0),
            inspecting=True,
            remaining_inspection_sec=1000.0,
        )],
        {'busy': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id is None
    assert 'battery' in result[0].reason or 'reachable' in result[0].reason


def test_falls_back_to_feasible_base_when_ugv_return_is_unsafe():
    allocator = make_allocator()
    allocator.recovery_position = (200.0, 0.0, 0.0)
    result = allocator.allocate(
        [make_task(x=10.0, duration=0.0)],
        [make_uav('uav1', 0.0, battery=25.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assert result[0].uav_id == 'uav1'
    assert result[0].return_distance_m == pytest.approx(13.26)


def test_ten_percent_reserve_returns_to_own_home_when_feasible():
    allocator = make_allocator()
    allocator.battery_feasibility = BatteryFeasibility(0.05, 0.25, 10.0)
    allocator.post_landing_reserve_percentage = 10.0
    allocator.recovery_position = (10.0, 0.0, 0.2)
    result = allocator.allocate(
        [make_task(x=10.0, duration=0.0)],
        [make_uav('uav1', 0.0, battery=30.0)],
        {'uav1': (0.0, 0.0, 0.2)},
    )

    assert result[0].uav_id == 'uav1'
    assert result[0].return_distance_m == pytest.approx(13.26, abs=0.01)


def test_home_reserve_violation_uses_configured_ugv_return_option():
    allocator = make_allocator()
    allocator.battery_feasibility = BatteryFeasibility(0.05, 0.25, 10.0)
    allocator.post_landing_reserve_percentage = 10.0
    allocator.recovery_position = (10.0, 0.0, 0.2)
    result = allocator.allocate(
        [make_task(x=10.0, duration=0.0)],
        [make_uav('uav1', 0.0, battery=12.7)],
        {'uav1': (0.0, 0.0, 0.2)},
    )

    assert result[0].uav_id == 'uav1'
    assert result[0].return_distance_m == pytest.approx(0.26)


def test_allocator_never_uses_task_endpoint_as_emergency_landing():
    allocator = make_allocator()
    allocator.battery_feasibility = BatteryFeasibility(0.05, 0.25, 5.0)
    allocator.recovery_position = (100.0, 0.0, 0.2)
    result = allocator.allocate(
        [make_task(x=10.0, duration=0.0)],
        [make_uav('uav1', 0.0, battery=8.0)],
        {'uav1': (0.0, 0.0, 0.2)},
    )

    assert result[0].uav_id is None


def test_returning_uav_cannot_bid():
    result = make_allocator().allocate(
        [make_task()],
        [make_uav('uav1', 0.0, status=UAVState.RETURNING)],
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
        [make_task('task-a'), make_task('task-b', x=20.0)],
        [make_uav('uav1', 0.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    assigned = [decision for decision in result if decision.uav_id]
    assert len(assigned) == 1
    assert assigned[0].uav_id == 'uav1'
    assert len({decision.task_id for decision in result}) == 2
    assert any(
        'already received one task in this allocation cycle'
        in decision.reason
        for decision in result if decision.uav_id is None
    )


def test_duplicate_task_ids_are_never_allocated_to_both_uavs():
    result = make_allocator().allocate(
        [make_task('same'), make_task('same')],
        [make_uav('uav1', 0.0), make_uav('uav2', 8.0)],
        {'uav1': (0.0, 0.0, 0.0), 'uav2': (8.0, 0.0, 0.0)},
    )

    assert sum(decision.uav_id is not None for decision in result) == 1
    assert sum(decision.task_id == 'same' for decision in result) == 2


def test_missing_planner_uses_provisional_cruise_route_estimator():
    path_cost = PathCostClient()

    assert path_cost.is_provisional
    assert path_cost.estimate_distance(
        (0.0, 0.0, 0.0), (3.0, 4.0, 0.0)
    ) == 24.7


def test_cruise_route_applies_configured_safety_factor_once():
    path_cost = PathCostClient(distance_safety_factor=1.3)

    assert path_cost.estimate_distance(
        (0.0, 0.0, 0.0), (3.0, 4.0, 0.0)
    ) == 24.7
    assert path_cost.estimate_distance(
        (0.0, 0.0, 7.0), (3.0, 4.0, 7.0)
    ) == 6.5


@pytest.mark.parametrize('factor', [1.24, 1.31])
def test_distance_safety_factor_rejects_values_outside_allowed_range(factor):
    with pytest.raises(ValueError, match='between 1.25 and 1.30'):
        PathCostClient(distance_safety_factor=factor)


def test_only_inspecting_uav_with_empty_queue_can_bid():
    allocator = make_allocator()
    assert allocator._eligible(make_uav(
        'inspecting', 0.0, status=UAVState.INSPECTING,
        current_task=make_task('active'), inspecting=True,
    ))[0]
    assert not allocator._eligible(make_uav(
        'inspecting-queued', 0.0, status=UAVState.INSPECTING,
        current_task=make_task('active'),
        queued_tasks=(make_task('queued'),), inspecting=True,
    ))[0]
    assert not allocator._eligible(make_uav(
        'travelling', 0.0, status=UAVState.EXECUTING,
        current_task=make_task('active'),
    ))[0]


def test_same_inputs_produce_repeatable_allocations():
    allocator = make_allocator()
    tasks = [make_task()]
    uavs = [make_uav('uav1', 0.0), make_uav('uav2', 8.0)]
    homes = {'uav1': (0.0, 0.0, 0.0), 'uav2': (8.0, 0.0, 0.0)}

    first = allocator.allocate(tasks, uavs, homes)
    second = allocator.allocate(tasks, uavs, homes)

    assert first == second


def test_batch_processes_four_highest_priority_tasks_in_order():
    tasks = [
        make_task('low', x=10.0, priority=1),
        make_task('p2', x=20.0, priority=2),
        make_task('p5', x=50.0, priority=5),
        make_task('p4', x=40.0, priority=4),
        make_task('p3', x=30.0, priority=3),
    ]
    result = make_allocator().allocate(
        tasks,
        [make_uav('uav1', 0.0), make_uav('uav2', 8.0)],
        {
            'uav1': (0.0, 0.0, 0.0),
            'uav2': (8.0, 0.0, 0.0),
        },
    )

    assigned = {decision.task_id for decision in result if decision.uav_id}
    assert assigned == {'p2', 'p3'}
    assert len(result) == 4


def test_free_uav_gets_next_task_without_waiting_for_busy_uav():
    result = make_allocator().allocate(
        [make_task('first', x=10.0), make_task('second', x=20.0)],
        [make_uav('uav1', 0.0), make_uav('uav2', 20.0)],
        {
            'uav1': (0.0, 0.0, 0.0),
            'uav2': (20.0, 0.0, 0.0),
        },
    )
    decisions = {decision.task_id: decision for decision in result}

    assert decisions['first'].uav_id == 'uav1'
    assert decisions['second'].uav_id == 'uav2'
    assert len({decision.uav_id for decision in result}) == 2


def test_travelling_uav_does_not_bid_against_free_peer():
    result = make_allocator().allocate(
        [make_task('task1'), make_task('task2', x=20.0)],
        [
            make_uav('uav1', 0.0, status=UAVState.EXECUTING),
            make_uav('uav2', 20.0),
        ],
        {
            'uav1': (0.0, 0.0, 0.0),
            'uav2': (20.0, 0.0, 0.0),
        },
    )

    assigned = [decision for decision in result if decision.uav_id]
    assert len(assigned) == 1
    assert assigned[0].uav_id == 'uav2'


@pytest.mark.parametrize(
    'busy_status',
    [
        UAVState.ASSIGNED,
        UAVState.EXECUTING,
        UAVState.RETURNING,
        UAVState.FAILED,
        UAVState.LANDING,
        UAVState.LANDED,
        UAVState.RECOVERING,
    ],
)
def test_free_peer_keeps_receiving_tasks_while_other_is_unavailable(
    busy_status,
):
    result = make_allocator().allocate(
        [make_task('task1'), make_task('task2', x=20.0)],
        [
            make_uav('uav1', 0.0, status=busy_status),
            make_uav('uav2', 20.0),
        ],
        {
            'uav1': (0.0, 0.0, 0.0),
            'uav2': (20.0, 0.0, 0.0),
        },
    )

    assigned = [decision for decision in result if decision.uav_id]
    assert len(assigned) == 1
    assert assigned[0].uav_id == 'uav2'


def test_returning_and_travelling_uavs_are_both_excluded():
    result = make_allocator().allocate(
        [make_task('task1')],
        [
            make_uav('uav1', 0.0, status=UAVState.EXECUTING),
            make_uav('uav2', 20.0, status=UAVState.RETURNING),
        ],
        {
            'uav1': (0.0, 0.0, 0.0),
            'uav2': (20.0, 0.0, 0.0),
        },
    )

    assert len(result) == 1
    assert result[0].uav_id is None


def test_task_specific_uav_allowlist_rejects_out_of_region_nearest_uav():
    task = make_task('bounded', x=10.0)
    result = make_allocator().allocate(
        [task],
        [make_uav('nearest', 0.0), make_uav('allowed', 8.0)],
        {
            'nearest': (0.0, 0.0, 0.0),
            'allowed': (8.0, 0.0, 0.0),
        },
        allowed_uav_ids_by_task={'bounded': {'allowed'}},
    )

    assert result[0].uav_id == 'allowed'
    assert result[0].reason == 'lowest feasible marginal bid'


def test_completed_uav_can_receive_work_while_peer_executes():
    result = make_allocator().allocate(
        [make_task('next-task')],
        [
            make_uav('uav1', 0.0, status=UAVState.COMPLETED),
            make_uav('uav2', 20.0, status=UAVState.EXECUTING),
        ],
        {
            'uav1': (0.0, 0.0, 0.0),
            'uav2': (20.0, 0.0, 0.0),
        },
    )

    assert len(result) == 1
    assert result[0].uav_id == 'uav1'


def test_future_lookahead_keeps_pairing_cost_based_and_deterministic():
    tasks = [
        make_task('first', x=200.0),
        make_task('future', x=100.0),
    ]
    uavs = [make_uav('uav_a', 0.0), make_uav('uav_b', 200.0)]
    homes = {
        'uav_a': (0.0, 0.0, 0.0),
        'uav_b': (200.0, 0.0, 0.0),
    }

    without_lookahead = make_allocator(future_weight=0.0).allocate(
        tasks, uavs, homes
    )
    with_lookahead = make_allocator(future_weight=20.0).allocate(
        tasks, uavs, homes
    )

    first_without = next(
        item for item in without_lookahead if item.task_id == 'first'
    )
    first_with = next(
        item for item in with_lookahead if item.task_id == 'first'
    )
    assert first_without.uav_id == 'uav_b'
    assert first_with.uav_id == 'uav_b'
    assert {
        item.task_id: item.uav_id for item in without_lookahead
    } == {
        item.task_id: item.uav_id for item in with_lookahead
    }


def test_infeasible_high_priority_task_does_not_block_next_feasible_task():
    result = make_allocator(future_weight=0.0).allocate(
        [
            make_task('infeasible', x=200.0, priority=5, duration=60.0),
            make_task('feasible', x=10.0, priority=4, duration=30.0),
        ],
        [make_uav('uav1', 0.0, battery=30.0)],
        {'uav1': (0.0, 0.0, 0.0)},
    )

    decisions = {decision.task_id: decision for decision in result}
    assert decisions['infeasible'].uav_id is None
    assert any(
        reason in decisions['infeasible'].reason
        for reason in ('battery', 'reachable', 'estimate is unavailable')
    )
    assert decisions['feasible'].uav_id == 'uav1'
