import math
from types import SimpleNamespace

from geometry_msgs.msg import Pose, TransformStamped
from tf2_geometry_msgs import do_transform_pose
from uav_control.controller import (
    OffboardController,
    calculate_velocity_setpoint,
    is_controlled_landing_request,
    is_emergency_landing_request,
)
from uav_info.coordinate_transform import enu_to_ned
from uav_interfaces.msg import UAVState


def test_velocity_setpoint_respects_speed_caps_and_preserves_direction():
    velocity = calculate_velocity_setpoint(
        (10.0, 10.0, 4.0), 1.0, 2.0, 5.0
    )
    assert math.isclose(math.hypot(*velocity[:2]), 2.0, rel_tol=1e-9)
    assert math.isclose(velocity[0], velocity[1], rel_tol=1e-9)
    assert velocity[2] == 2.0


def test_velocity_gain_applies_equally_to_horizontal_and_vertical():
    velocity = calculate_velocity_setpoint(
        (4.0, 3.0, 1.0), 0.5, 5.0, 5.0
    )
    assert velocity == (2.0, 1.5, 0.5)


def test_application_velocity_limit_caps_horizontal_and_vertical_commands():
    velocity = calculate_velocity_setpoint(
        (10.0, 0.0, -10.0), 1.0, 5.0, 5.0
    )
    assert velocity == (5.0, 0.0, -5.0)


def test_map_goal_uses_tf_translation_before_ned_conversion():
    transform = TransformStamped()
    transform.header.frame_id = 'uav_1/odom'
    transform.child_frame_id = 'map'
    transform.transform.translation.x = -5.0
    transform.transform.rotation.w = 1.0
    map_goal = Pose()
    map_goal.position.x = 10.0
    map_goal.position.y = 2.0
    map_goal.position.z = 5.0
    map_goal.orientation.w = 1.0

    local_goal = do_transform_pose(map_goal, transform)
    ned_goal = enu_to_ned((
        local_goal.position.x,
        local_goal.position.y,
        local_goal.position.z,
    ))

    assert all(math.isclose(actual, expected, abs_tol=1e-9)
               for actual, expected in zip(ned_goal, (2.0, 5.0, -5.0)))


def test_emergency_return_failure_requests_px4_emergency_landing():
    assert is_emergency_landing_request(
        'EMERGENCY_RETURN', UAVState.FAILED
    )
    assert not is_emergency_landing_request(
        'ordinary-task', UAVState.FAILED
    )
    assert not is_emergency_landing_request(
        'EMERGENCY_RETURN', UAVState.RETURNING
    )
    assert not is_emergency_landing_request(
        'RETURN_HOME', UAVState.LANDING
    )
    assert not is_emergency_landing_request(
        'EMERGENCY_LANDING', UAVState.RETURNING
    )


def test_selected_return_touchdown_requests_controlled_px4_landing():
    assert is_controlled_landing_request('LANDING', UAVState.LANDING)
    assert not is_controlled_landing_request('ordinary-task', UAVState.LANDING)
    assert not is_controlled_landing_request('LANDING', UAVState.RETURNING)


def test_controller_operating_bounds_include_edges_and_reject_outside():
    parameters = {
        'x_min': -30.0,
        'x_max': 10.0,
        'y_min': -70.0,
        'y_max': 70.0,
        'z_min': 0.0,
        'z_max': 10.0,
    }

    def get_parameter(name):
        bound_name = name.rsplit('.', maxsplit=1)[-1]
        return SimpleNamespace(
            get_parameter_value=lambda: SimpleNamespace(
                double_value=parameters[bound_name]
            )
        )

    controller = SimpleNamespace(get_parameter=get_parameter)
    for point in (
        (-30.0, 0.0, 5.0),
        (10.0, 0.0, 5.0),
        (0.0, -70.0, 5.0),
        (0.0, 70.0, 5.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 10.0),
    ):
        assert OffboardController._inside_operating_bounds(
            controller, *point
        )
    for point in (
        (-30.000001, 0.0, 5.0),
        (10.000001, 0.0, 5.0),
        (0.0, -70.000001, 5.0),
        (0.0, 70.000001, 5.0),
        (0.0, 0.0, -0.000001),
        (0.0, 0.0, 10.000001),
    ):
        assert not OffboardController._inside_operating_bounds(
            controller, *point
        )
