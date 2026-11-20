from types import SimpleNamespace

import pytest

from uav_info.battery_monitor import BatteryMonitor
from uav_info.px4_interface import PX4Interface
from uav_info.vehicle_state import VehicleState


@pytest.mark.parametrize(
    ('uav_id', 'initial_percentage'),
    [('uav_1', 30.0), ('uav_2', 100.0)],
)
def test_battery_telemetry_does_not_replace_initial_estimate(
    uav_id, initial_percentage
):
    interface = PX4Interface.__new__(PX4Interface)
    interface.state = VehicleState(uav_id)
    interface.battery_monitor = BatteryMonitor(initial_percentage, 0.05)
    message = SimpleNamespace(connected=True, remaining=0.95)

    interface.battery_callback(message)

    assert interface.state.get('battery_status') is message
    assert interface.battery_monitor.estimated_percentage() == pytest.approx(
        initial_percentage
    )
