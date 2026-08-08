#!/usr/bin/env python3
"""
Command-line helper for the waypoint navigator.

Examples:
  ros2 run autoVehicle waypoint_cli.py add 3.0 0.0
  ros2 run autoVehicle waypoint_cli.py add 5.94 1.2 --yaw 90
  ros2 run autoVehicle waypoint_cli.py start
  ros2 run autoVehicle waypoint_cli.py status
"""
import argparse

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import Trigger


def _call_service(node, name):
    client = node.create_client(Trigger, name)
    if not client.wait_for_service(timeout_sec=3.0):
        print(f'Service {name} not available')
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done():
        result = future.result()
        print(f'{name}: success={result.success} message={result.message}')
        return result.success
    print(f'{name}: call timed out')
    return False


def _add_waypoint(node, args):
    client = node.create_client(SetParameters, '/waypoint/add')
    if not client.wait_for_service(timeout_sec=3.0):
        print('Service /waypoint/add not available')
        return False
    request = SetParameters.Request()
    request.parameters = [Parameter(
        name='goal',
        value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[args.x, args.y, args.z, args.yaw],
        ),
    )]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        print('/waypoint/add: call timed out')
        return False
    result = future.result()
    ok = all(item.successful for item in result.results)
    reason = '; '.join(item.reason for item in result.results)
    print(
        f'/waypoint/add: success={ok} reason={reason} '
        f'({args.x}, {args.y}, {args.z}, yaw={args.yaw}deg)')
    return ok


def main():
    parser = argparse.ArgumentParser(description='Manual waypoint CLI')
    sub = parser.add_subparsers(dest='command', required=True)

    add = sub.add_parser('add', help='add a waypoint')
    add.add_argument('x', type=float)
    add.add_argument('y', type=float)
    add.add_argument('z', type=float, nargs='?', default=0.0)
    add.add_argument('--yaw', type=float, default=0.0)
    add.set_defaults(func=_add_waypoint)

    for name in ('start', 'stop', 'clear', 'save', 'load', 'status', 'reload_map'):
        p = sub.add_parser(name, help=f'{name} waypoint navigation')
        p.set_defaults(func=lambda node, args: _call_service(
            node, f'/waypoint/{args.command}'))

    args = parser.parse_args()
    rclpy.init()
    node = Node('waypoint_cli')
    try:
        args.func(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
