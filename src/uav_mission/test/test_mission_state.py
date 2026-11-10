import pytest

from uav_interfaces.msg import UAVState
from uav_mission.mission_state import (
    MissionState,
    RecoveryState,
    all_uavs_confirmed_landed,
    completion_return_decision,
    feasible_return_options,
)


def make_state(uav_id, mission_status):
    state = UAVState()
    state.uav_id = uav_id
    state.connected = True
    state.operational = True
    state.mission_status = mission_status
    return state


def test_idle_uav_remains_available_while_peer_is_executing():
    mission = MissionState(['uav1', 'uav2'])
    mission.update(make_state('uav1', UAVState.EXECUTING))
    mission.update(make_state('uav2', UAVState.IDLE))

    assert [state.uav_id for state in mission.available_uavs()] == ['uav2']


def test_completed_uav_is_available_while_peer_is_executing():
    mission = MissionState(['uav1', 'uav2'])
    mission.update(make_state('uav1', UAVState.COMPLETED))
    mission.update(make_state('uav2', UAVState.EXECUTING))

    assert [state.uav_id for state in mission.available_uavs()] == ['uav1']


def test_failed_uav_does_not_block_available_peer():
    mission = MissionState(['uav1', 'uav2'])
    mission.update(make_state('uav1', UAVState.FAILED))
    mission.update(make_state('uav2', UAVState.IDLE))

    assert [state.uav_id for state in mission.available_uavs()] == ['uav2']


def test_landed_and_recovering_uavs_are_not_available_for_assignment():
    mission = MissionState(['landed', 'recovering'])
    mission.update(make_state('landed', UAVState.LANDED))
    mission.update(make_state('recovering', UAVState.RECOVERING))

    assert mission.available_uavs() == []


def test_low_battery_starts_recovery_once_below_twenty_percent():
    recovery = RecoveryState(20.0)

    assert not recovery.begin('uav1', 20.0)
    assert recovery.begin('uav1', 19.9)
    assert not recovery.begin('uav1', 10.0)
    assert 'uav1' in recovery.returning


def test_recovery_requires_arrival_operational_state_and_recharged_battery():
    recovery = RecoveryState(20.0)
    recovery.begin('uav1', 15.0)

    assert not recovery.confirm_ready('uav1', 80.0, True)
    assert recovery.mark_arrived('uav1')
    assert not recovery.confirm_ready('uav1', 19.9, True)
    assert not recovery.confirm_ready('uav1', 80.0, False)
    assert recovery.confirm_ready('uav1', 80.0, True)
    assert 'uav1' not in recovery.returning


def test_normal_ugv_return_can_enter_recovery_state_after_landing():
    recovery = RecoveryState(30.0)

    assert recovery.begin_return('uav1')
    assert recovery.mark_arrived('uav1')
    assert recovery.confirm_ready('uav1', 40.0, True)


def test_recovery_target_uses_configured_ugv_coordinate_and_frame():
    recovery = RecoveryState(
        20.0, ugv_location=(-5.0, 0.0, 0.2), frame_id='map'
    )

    assert recovery.return_target() == {
        'position': (-5.0, 0.0, 0.2),
        'frame_id': 'map',
        'recovery': True,
    }


def test_low_battery_recovery_is_independent_per_uav():
    recovery = RecoveryState(20.0)

    assert recovery.begin('uav1', 19.0)
    assert not recovery.begin('uav2', 65.0)
    assert recovery.returning == {'uav1'}


def test_each_uav_has_independent_return_battery_feasibility():
    targets = [
        {'name': 'UGV', 'position': (0.0, 0.0, 0.2), 'recovery': True},
        {'name': 'base', 'position': (20.0, 0.0, 0.2), 'recovery': False},
    ]

    uav1 = feasible_return_options(
        (1.0, 0.0, 7.0), 80.0, targets, 0.05, 20.0
    )
    uav2 = feasible_return_options(
        (19.0, 0.0, 7.0), 8.0, targets, 0.05, 5.0
    )

    assert uav1[0]['name'] == 'UGV'
    assert [option['name'] for option in uav2] == ['base']


def test_return_options_keep_the_callers_destination_priority():
    options = feasible_return_options(
        (19.0, 0.0, 7.0),
        75.0,
        [
            {'name': 'UGV', 'position': (0.0, 0.0, 0.2)},
            {'name': 'base', 'position': (20.0, 0.0, 0.2)},
        ],
        0.05,
        20.0,
    )

    assert options[0]['name'] == 'UGV'
    assert options[0]['battery_after_landing_percent'] >= 20.0


def test_return_options_do_not_reorder_destinations_by_distance():
    options = feasible_return_options(
        (1.0, 0.0, 7.0),
        75.0,
        [
            {'name': 'base', 'position': (20.0, 0.0, 0.2)},
            {'name': 'UGV', 'position': (0.0, 0.0, 0.2)},
        ],
        0.05,
        20.0,
    )

    assert options[0]['name'] == 'base'
    assert options[0]['energy_required_percent'] > 2.0


def test_return_options_remove_only_battery_infeasible_destinations():
    options = feasible_return_options(
        (0.0, 0.0, 7.0),
        25.0,
        [
            {
                'name': 'UGV',
                'position': (1.0, 0.0, 0.2),
                'estimated_route_distance_m': 300.0,
            },
            {
                'name': 'base',
                'position': (80.0, 0.0, 0.2),
                'estimated_route_distance_m': 50.0,
            },
        ],
        0.05,
        20.0,
        distance_safety_factor=1.3,
        landing_energy_percentage=2.0,
    )

    assert [option['name'] for option in options] == ['base']


def test_no_return_option_is_selected_without_safety_reserve():
    options = feasible_return_options(
        (0.0, 0.0, 7.0),
        20.5,
        [
            {'name': 'UGV', 'position': (1.0, 0.0, 0.2)},
            {'name': 'base', 'position': (2.0, 0.0, 0.2)},
        ],
        0.05,
        20.0,
        landing_energy_percentage=2.0,
    )

    assert options == []


def _completion_decision(
    battery, ugv=(1.0, 0.0, 0.2), start=(100.0, 0.0, 0.0)
):
    return completion_return_decision(
        (0.0, 0.0, 7.0), battery, start,
        {'name': 'UGV', 'position': ugv, 'recovery': True},
        consumption_per_meter=0.05,
        distance_safety_factor=1.3,
        landing_energy_percentage=2.0,
        minimum_reserve_percentage=10.0,
        landing_altitude_m=0.2,
    )


@pytest.mark.parametrize('predicted_remaining', [85.0, 50.0, 10.0])
def test_completion_return_at_or_above_ten_percent_uses_home_position(
    predicted_remaining,
):
    normal_distance = (
        100.0 + abs(7.0 - 7.0) + abs(0.2 - 7.0)
    )
    normal_energy = normal_distance * 1.3 * 0.05 + 2.0
    result = _completion_decision(predicted_remaining + normal_energy)

    assert result['mode'] == 'normal'
    assert abs(
        result['predicted_normal_remaining_percent'] - predicted_remaining
    ) < 1e-9
    assert [option['name'] for option in result['options']] == [
        'home_position'
    ]
    assert result['options'][0]['position'] == (100.0, 0.0, 0.2)
    assert all(
        option['name'] != 'current_position'
        for option in result['options']
    )


def test_below_ten_percent_after_base_return_uses_configured_ugv():
    # The base decision includes the vertical cruise-to-landing segment.
    result = _completion_decision(
        16.0,
        ugv=(0.0, 0.0, 0.2),
        start=(100.0, 0.0, 0.0),
    )

    assert result['predicted_normal_remaining_percent'] < 10.0
    assert result['mode'] == 'emergency'
    assert [option['name'] for option in result['options']] == ['UGV']
    assert result['options'][0]['position'] == (0.0, 0.0, 0.2)


def test_low_battery_uses_only_reachable_configured_ugv():
    start = (100.0, 0.0, 0.0)
    normal_distance = 100.0 + abs(0.2 - 7.0)
    normal_energy = normal_distance * 1.3 * 0.05 + 2.0
    result = _completion_decision(
        normal_energy + 9.0, ugv=(1.0, 0.0, 0.2), start=start
    )

    assert result['mode'] == 'emergency'
    assert abs(result['predicted_normal_remaining_percent'] - 9.0) < 1e-9
    assert [option['name'] for option in result['options']] == ['UGV']
    assert result['options'][0]['position'] == (1.0, 0.0, 0.2)


def test_each_uav_uses_its_own_original_starting_position():
    uav_1_start = (-20.0, 3.0, 0.0)
    uav_2_start = (25.0, -4.0, 0.0)
    uav_1 = completion_return_decision(
        (0.0, 0.0, 7.0), 85.0, uav_1_start,
        {'name': 'UGV', 'position': (1.0, 0.0, 0.2)},
        0.05, 1.3, 2.0, 10.0, 0.2,
    )
    uav_2 = completion_return_decision(
        (0.0, 0.0, 7.0), 85.0, uav_2_start,
        {'name': 'UGV', 'position': (1.0, 0.0, 0.2)},
        0.05, 1.3, 2.0, 5.0, 0.2,
    )

    assert uav_1['options'][0]['position'] == (-20.0, 3.0, 0.2)
    assert uav_2['options'][0]['position'] == (25.0, -4.0, 0.2)


def test_emergency_options_require_ten_percent_after_landing():
    result = completion_return_decision(
        (0.0, 0.0, 7.0), 4.0, (100.0, 0.0, 0.0),
        {'name': 'UGV', 'position': (0.0, 0.0, 0.2), 'recovery': True},
        0.05, 1.3, 2.0, 10.0, 0.2,
    )

    assert result['mode'] == 'unsafe'
    assert result['options'] == []


def test_only_configured_validated_nearby_landing_is_last_option():
    result = completion_return_decision(
        (0.0, 0.0, 7.0), 13.1, (100.0, 0.0, 0.0),
        {'name': 'UGV', 'position': (1000.0, 0.0, 0.2), 'recovery': True},
        0.05, 1.3, 2.0, 10.0, 0.2,
        nearby_landing_targets=[{
            'name': 'validated_pad',
            'position': (0.0, 0.0, 0.2),
            'validated': True,
            'recovery': False,
        }],
    )

    assert result['mode'] == 'nearby'
    assert [target['name'] for target in result['options']] == [
        'validated_pad'
    ]
    assert result['options'][0]['position'] == (0.0, 0.0, 0.2)
    assert result['options'][0]['battery_after_landing_percent'] >= 10.0


def test_unconfigured_nearby_landing_is_not_used():
    result = completion_return_decision(
        (0.0, 0.0, 7.0), 3.0, (100.0, 0.0, 0.0),
        {'name': 'UGV', 'position': (1000.0, 0.0, 0.2), 'recovery': True},
        0.05, 1.3, 2.0, 10.0, 0.2,
    )

    assert result['mode'] == 'unsafe'
    assert result['options'] == []


def test_shutdown_requires_both_confirmed_successful_landed_states():
    states = {
        'uav_1': make_state('uav_1', UAVState.LANDED),
        'uav_2': make_state('uav_2', UAVState.RETURNING),
    }
    states['uav_1'].landed = True
    states['uav_2'].landed = False
    assert not all_uavs_confirmed_landed(
        ['uav_1', 'uav_2'], states, {'uav_1'}
    )

    states['uav_2'].mission_status = UAVState.LANDED
    states['uav_2'].landed = True
    assert all_uavs_confirmed_landed(
        ['uav_1', 'uav_2'], states, {'uav_1', 'uav_2'}
    )
    assert not all_uavs_confirmed_landed(
        ['uav_1', 'uav_2'], states, {'uav_1', 'uav_2'},
        recovering_uavs={'uav_2'},
    )

    states['uav_2'].mission_status = UAVState.FAILED
    assert not all_uavs_confirmed_landed(
        ['uav_1', 'uav_2'], states, {'uav_1', 'uav_2'}
    )
