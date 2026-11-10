import rclpy
from nav2_msgs.srv import ManageLifecycleNodes


SERVICE_TIMEOUT_SEC = 1.0
RESPONSE_TIMEOUT_SEC = 5.0


def shutdown_lifecycle_managers(node, manager_services):
    all_shutdown = True
    for service_name in manager_services:
        client = node.create_client(
            ManageLifecycleNodes, service_name
        )
        try:
            if not client.wait_for_service(
                timeout_sec=SERVICE_TIMEOUT_SEC
            ):
                node.get_logger().error(
                    f'Lifecycle manager service {service_name} is '
                    'unavailable; continuing shutdown.'
                )
                all_shutdown = False
                continue

            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.SHUTDOWN
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=RESPONSE_TIMEOUT_SEC
            )
            if not future.done():
                node.get_logger().error(
                    f'Lifecycle manager {service_name} did not respond '
                    'to its shutdown request.'
                )
                all_shutdown = False
                continue

            response = future.result()
            if response is None or not response.success:
                node.get_logger().error(
                    f'Lifecycle manager {service_name} failed to shut '
                    'down its managed nodes.'
                )
                all_shutdown = False
        except Exception as error:
            node.get_logger().error(
                f'Error shutting down lifecycle manager {service_name}: '
                f'{error}'
            )
            all_shutdown = False
        finally:
            node.destroy_client(client)

    return all_shutdown


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('nav2_shutdown_coordinator')
    node.declare_parameter('manager_services', [])
    try:
        manager_services_value = node.get_parameter(
            'manager_services'
        ).value
        if not isinstance(manager_services_value, list) or not all(
            isinstance(service_name, str)
            for service_name in manager_services_value
        ):
            raise ValueError(
                'manager_services must be a list of lifecycle manager '
                'service names.'
            )
        manager_services = manager_services_value
        success = shutdown_lifecycle_managers(node, manager_services)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1
