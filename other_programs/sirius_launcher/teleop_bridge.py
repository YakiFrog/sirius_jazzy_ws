#!/usr/bin/env python3
"""テレオペブリッジ: stdinのキーコードを /controller へ流す。

SiriusLauncher Web (web_server.py) の十字キーパッドが遅延起動するヘルパー。
sirius_controller_node は /controller トピック(sirius_interfaces/ControllerInput)
からのリモート入力を processKeyboardInput と同じに処理する。

stdin: 1行=1キーイベント（整数）
  65=前進(+0.1m/s) 66=後退(-0.1m/s) 67=左旋回(+0.157rad/s) 68=右旋回(-0.157rad/s)
  44=並進停止    46=旋回停止
stdinが閉じると終了する。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sirius_interfaces.msg import ControllerInput


class TeleopBridge(Node):
    def __init__(self):
        super().__init__("sirius_launcher_teleop")
        self.pub = self.create_publisher(ControllerInput, "controller", QoSProfile(depth=10))

    def send(self, key):
        msg = ControllerInput()
        msg.pressed_key = key
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopBridge()
    print("teleop bridge ready", flush=True)
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                key = int(line)
            except ValueError:
                continue
            node.send(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
