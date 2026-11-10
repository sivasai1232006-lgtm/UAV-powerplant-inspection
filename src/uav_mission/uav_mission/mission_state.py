import time
import math

from uav_interfaces.msg import UAVState


def feasible_return_options(
    position,
    battery_percentage,
    targets,
    consumption_per_meter,
    minimum_reserve_percentage,
    distance_safety_factor=1.3,
    landing_energy_percentage=2.0,
    cruise_altitude_m=7.0,
):
    """Return battery-feasible destinations in the supplied priority order."""
    try:
        current = tuple(float(value) for value in position)
        battery = float(battery_percentage)
        consumption = float(consumption_per_meter)
        reserve = float(minimum_reserve_percentage)
        factor = float(distance_safety_factor)
        landing_energy = float(landing_energy_percentage)
        cruise_altitude = float(cruise_altitude_m)
    except (TypeError, ValueError):
        return []
    values = (
        battery, consumption, reserve, factor, landing_energy,
        cruise_altitude, *current,
    )
    if len(current) != 3 or not all(math.isfinite(value) for value in values):
        return []
    if (
        not 0.0 <= battery <= 100.0
        or consumption < 0.0
        or reserve < 0.0
        or not 1.25 <= factor <= 1.30
        or landing_energy < 0.0
        or cruise_altitude < 0.0
    ):
        return []

    options = []
    for target in targets:
        if not isinstance(target, dict):
            continue
        try:
            target_position = tuple(
                float(value) for value in target['position']
            )
        except (KeyError, TypeError, ValueError):
            continue
        if len(target_position) != 3 or not all(
            math.isfinite(value) for value in target_position
        ) or target_position[2] < 0.0:
            continue
        distance_m = math.dist(position, target_position)
        cruise_route_m = (
            math.hypot(
                current[0] - target_position[0],
                current[1] - target_position[1],
            )
            + abs(current[2] - cruise_altitude)
            + abs(target_position[2] - cruise_altitude)
        )
        try:
            estimated_route_m = float(target.get(
                'estimated_route_distance_m',
                cruise_route_m * factor,
            ))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(estimated_route_m) or estimated_route_m < 0.0:
            continue
        energy_required = (
            estimated_route_m * consumption
            + landing_energy
        )
        if battery - energy_required < reserve:
            continue
        option = dict(target)
        option.update({
            'position': target_position,
            'distance_m': distance_m,
            'estimated_route_m': estimated_route_m,
            'energy_required_percent': energy_required,
            'minimum_remaining_battery_percentage': (
                reserve
            ),
            'battery_after_landing_percent': (
                battery - energy_required
            ),
        })
        options.append(option)
    return options


def completion_return_decision(
    position,
    battery_percentage,
    home_position,
    ugv_target,
    consumption_per_meter,
    distance_safety_factor=1.3,
    landing_energy_percentage=2.0,
    minimum_reserve_percentage=10.0,
    landing_altitude_m=0.2,
    frame_id='map',
    nearby_landing_targets=(),
    cruise_altitude_m=7.0,
):
    """Choose home, configured UGV, then validated nearby pads by priority."""
    try:
        current = tuple(float(value) for value in position)
        home = tuple(float(value) for value in home_position)
        battery = float(battery_percentage)
        consumption = float(consumption_per_meter)
        factor = float(distance_safety_factor)
        landing_energy = float(landing_energy_percentage)
        threshold = float(minimum_reserve_percentage)
        landing_altitude = float(landing_altitude_m)
        cruise_altitude = float(cruise_altitude_m)
    except (TypeError, ValueError):
        return {'mode': 'unsafe', 'predicted_normal_remaining_percent': None,
                'options': []}
    values = (
        *current, *home, battery, consumption, factor, landing_energy,
        threshold, landing_altitude, cruise_altitude,
    )
    if (
        len(current) != 3 or len(home) != 3
        or not all(math.isfinite(value) for value in values)
        or not 0.0 <= battery <= 100.0
        or consumption < 0.0 or not 1.25 <= factor <= 1.30
        or landing_energy < 0.0 or not 0.0 <= threshold <= 100.0
        or landing_altitude < 0.0
        or cruise_altitude < 0.0 or not 0.0 <= threshold <= 100.0
    ):
        return {'mode': 'unsafe', 'predicted_normal_remaining_percent': None,
                'options': []}

    landing_position = (home[0], home[1], landing_altitude)
    normal_distance = math.dist(current, landing_position)
    normal_route_m = (
        math.hypot(
            current[0] - landing_position[0],
            current[1] - landing_position[1],
        )
        + abs(current[2] - cruise_altitude)
        + abs(landing_position[2] - cruise_altitude)
    ) * factor
    normal_energy = normal_route_m * consumption + landing_energy
    normal_remaining = battery - normal_energy
    normal_target = {
        'name': 'home_position',
        'position': landing_position,
        'frame_id': str(frame_id),
        'recovery': False,
        'distance_m': normal_distance,
        'estimated_route_m': normal_route_m,
        'energy_required_percent': normal_energy,
        'battery_after_landing_percent': normal_remaining,
        'minimum_remaining_battery_percentage': threshold,
    }

    recovery_options = feasible_return_options(
        current,
        battery,
        [dict(ugv_target)] if isinstance(ugv_target, dict) else [],
        consumption,
        threshold,
        factor,
        landing_energy,
        cruise_altitude,
    )
    nearby_options = feasible_return_options(
        current,
        battery,
        [
            dict(target) for target in nearby_landing_targets
            if isinstance(target, dict)
            and target.get('validated') is True
        ],
        consumption,
        threshold,
        factor,
        landing_energy,
        cruise_altitude,
    )
    if normal_remaining >= threshold:
        options = [normal_target]
        mode = 'normal'
    else:
        options = recovery_options
        mode = 'emergency'
        if not options:
            options = nearby_options
            mode = 'nearby'
    return {
        'mode': mode if options else 'unsafe',
        'predicted_normal_remaining_percent': normal_remaining,
        'options': options,
    }


def all_uavs_confirmed_landed(
    uav_ids,
    uav_states,
    successful_landing_uavs,
    failed_uavs=(),
    recovering_uavs=(),
):
    """Require a successful, confirmed LANDED state for every UAV."""
    expected = set(uav_ids)
    if not expected or not expected.issubset(successful_landing_uavs):
        return False
    if expected.intersection(failed_uavs) or expected.intersection(
        recovering_uavs
    ):
        return False
    for uav_id in expected:
        state = uav_states.get(uav_id)
        if (
            state is None
            or state.mission_status != UAVState.LANDED
            or not bool(state.landed)
        ):
            return False
    return True


class MissionState:
    def __init__(self, uav_ids, state_timeout_sec=3.0):
        if state_timeout_sec <= 0.0:
            raise ValueError('State timeout must be greater than zero')

        self.uav_ids = set(uav_ids)
        self.state_timeout_sec = float(state_timeout_sec)
        self.uav_states = {}
        self.last_state_update = {}

    def update(self, uav_state):
        if uav_state.uav_id not in self.uav_ids:
            return False

        self.uav_states[uav_state.uav_id] = uav_state
        self.last_state_update[uav_state.uav_id] = time.monotonic()
        return True

    def operational_uavs(self):
        now = time.monotonic()
        return [
            state
            for uav_id, state in self.uav_states.items()
            if now - self.last_state_update[uav_id] <= self.state_timeout_sec
            and state.connected
            and state.operational
        ]

    def available_uavs(self):
        ready_statuses = (UAVState.IDLE, UAVState.COMPLETED)
        return [
            state
            for state in self.operational_uavs()
            if state.mission_status in ready_statuses
        ]


class RecoveryState:
    def __init__(
        self,
        low_battery_threshold_percent=20.0,
        ugv_location=None,
        frame_id='map',
    ):
        self.low_battery_threshold_percent = float(
            low_battery_threshold_percent
        )
        self.returning = set()
        self.arrived = set()
        self.ugv_location = (
            tuple(float(value) for value in ugv_location)
            if ugv_location is not None else None
        )
        self.frame_id = str(frame_id)

    def return_target(self):
        if self.ugv_location is None or len(self.ugv_location) != 3:
            raise ValueError('UGV recovery location must have 3 coordinates')
        return {
            'position': self.ugv_location,
            'frame_id': self.frame_id,
            'recovery': True,
        }

    def begin(self, uav_id, battery_percentage):
        if (
            battery_percentage >= self.low_battery_threshold_percent
            or uav_id in self.returning
        ):
            return False
        return self.begin_return(uav_id)

    def begin_return(self, uav_id):
        if uav_id in self.returning:
            return False
        self.returning.add(uav_id)
        self.arrived.discard(uav_id)
        return True

    def mark_arrived(self, uav_id):
        if uav_id not in self.returning:
            return False
        self.arrived.add(uav_id)
        return True

    def confirm_ready(self, uav_id, battery_percentage, operational):
        if (
            uav_id not in self.arrived
            or battery_percentage < self.low_battery_threshold_percent
            or not operational
        ):
            return False
        self.returning.discard(uav_id)
        self.arrived.discard(uav_id)
        return True
