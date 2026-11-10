from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from uav_mission import nav2_shutdown


class _Client:
    def __init__(self, available=True, response=True):
        self.available = available
        self.response = response
        self.request = None

    def wait_for_service(self, timeout_sec):
        assert timeout_sec == nav2_shutdown.SERVICE_TIMEOUT_SEC
        return self.available

    def call_async(self, request):
        self.request = request
        return SimpleNamespace(done=lambda: True, result=lambda: (
            SimpleNamespace(success=self.response)
        ))


@pytest.mark.parametrize(
    ('available', 'response', 'expected'),
    [(True, True, True), (True, False, False), (False, True, False)],
)
def test_shutdown_requests_each_lifecycle_manager(
    monkeypatch, available, response, expected
):
    client = _Client(available, response)
    node = SimpleNamespace(
        create_client=Mock(return_value=client),
        destroy_client=Mock(),
        get_logger=Mock(return_value=Mock()),
    )
    spin = Mock()
    monkeypatch.setattr(
        nav2_shutdown.rclpy, 'spin_until_future_complete', spin
    )

    result = nav2_shutdown.shutdown_lifecycle_managers(
        node, ['/planner_3m/lifecycle_manager/manage_nodes']
    )

    assert result is expected
    if available:
        assert (
            client.request.command
            == nav2_shutdown.ManageLifecycleNodes.Request.SHUTDOWN
        )
        spin.assert_called_once()
    else:
        spin.assert_not_called()
    node.destroy_client.assert_called_once_with(client)
