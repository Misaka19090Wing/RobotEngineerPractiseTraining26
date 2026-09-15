#!/usr/bin/env python3
"""
Command-line helper for the distance measurement node.

Examples:
  ros2 run autoVehicle distance_cli.py status
  ros2 run autoVehicle distance_cli.py save
  ros2 run autoVehicle distance_cli.py reset
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


TOPICS = {
    'forward': '/distance/forward',
    'left': '/distance/left',
    'right': '/distance/right',
    'width': '/distance/width',
    'traveled': '/distance/traveled',
    'tray_length': '/distance/tray_length',
}


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


def _read_topic(node, name, timeout_sec=3.0):
    """Grab the latest value from a Float32 topic, or None on timeout."""
    holder = {}

    def callback(msg):
        holder['value'] = msg.data

    subscription = node.create_subscription(Float32, name, callback, 10)
    deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while 'value' not in holder:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            break
    node.destroy_subscription(subscription)
    return holder.get('value')


def _show(node, args):
    """Print every measurement topic as a table."""
    print(f'{"measurement":<14}{"value":>12}  {"topic"}')
    print('-' * 62)
    for label, topic in TOPICS.items():
        value = _read_topic(node, topic)
        shown = 'n/a' if value is None else f'{value:.3f} m'
        print(f'{label:<14}{shown:>12}  {topic}')


def _status_text(node, timeout_sec=3.0):
    """Grab the /distance/status String summary."""
    holder = {}

    def callback(msg):
        holder['text'] = msg.data

    subscription = node.create_subscription(String, '/distance/status', callback, 10)
    deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while 'text' not in holder:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            break
    node.destroy_subscription(subscription)
    return holder.get('text')


def _status(node, args):
    text = _status_text(node)
    if text is None:
        print('No /distance/status received — is distance_measure running?')
        return False
    print(text)
    return True


def main():
    parser = argparse.ArgumentParser(description='Distance measurement CLI')
    sub = parser.add_subparsers(dest='command', required=True)

    show = sub.add_parser('show', help='print every measurement topic')
    show.set_defaults(func=_show)

    status = sub.add_parser('status', help='print the node summary line')
    status.set_defaults(func=_status)

    for name in ('save', 'reset'):
        p = sub.add_parser(name, help=f'{name} distance measurement')
        p.set_defaults(func=lambda node, args: _call_service(
            node, f'/distance/{args.command}'))

    args = parser.parse_args()
    rclpy.init()
    node = Node('distance_cli')
    try:
        args.func(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
