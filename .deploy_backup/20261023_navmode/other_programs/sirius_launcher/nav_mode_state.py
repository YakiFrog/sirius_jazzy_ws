#!/usr/bin/env python3
"""Write the latest confirmed /sirius/navigation_mode to a small state file."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def write_state(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    os.replace(temporary, path)


class NavModeState(Node):
    def __init__(self, output, domain):
        super().__init__('nav_mode_state_writer')
        self.output = Path(output)
        self.domain = str(domain)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            String, '/sirius/navigation_mode', self.on_mode, qos)
        write_state(self.output, {
            'status': 'waiting',
            'mode': None,
            'domain': self.domain,
            'received_at': time.time(),
            'pid': os.getpid(),
        })

    def on_mode(self, message):
        mode = str(message.data or '').strip()
        if not mode:
            return
        write_state(self.output, {
            'status': 'ok',
            'mode': mode,
            'domain': self.domain,
            'received_at': time.time(),
            'pid': os.getpid(),
        })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--domain', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = NavModeState(args.output, args.domain)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
