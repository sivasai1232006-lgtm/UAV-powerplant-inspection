import pytest

from uav_info.battery_monitor import BatteryMonitor


def test_measured_battery_calibrates_distance_estimate():
    monitor = BatteryMonitor(100.0, 0.05)
    assert monitor.update_path_distance(100.0)
    assert monitor.update_measured_percentage(82.0)
    assert monitor.estimated_percentage() == pytest.approx(82.0)

    assert monitor.update_path_distance(110.0)
    assert monitor.estimated_percentage() == pytest.approx(81.5)


def test_battery_monitors_keep_each_uav_estimate_independent():
    uav_1 = BatteryMonitor(100.0, 0.05)
    uav_2 = BatteryMonitor(100.0, 0.05)
    uav_1.update_path_distance(20.0)
    uav_2.update_path_distance(50.0)
    uav_1.update_measured_percentage(92.0)
    uav_2.update_measured_percentage(71.0)

    uav_1.update_path_distance(30.0)

    assert uav_1.estimated_percentage() == pytest.approx(91.5)
    assert uav_2.estimated_percentage() == pytest.approx(71.0)


def test_invalid_measured_battery_does_not_replace_last_estimate():
    monitor = BatteryMonitor(100.0, 0.05)
    monitor.update_measured_percentage(80.0)
    monitor.update_path_distance(10.0)

    assert not monitor.update_measured_percentage(float('nan'))
    assert not monitor.update_measured_percentage(101.0)
    assert monitor.estimated_percentage() == pytest.approx(79.5)
