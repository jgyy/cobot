from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = "/forward_position_controller/commands"
NUM_JOINTS = 6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously move the arm through a sine-wave joint motion"
    )
    parser.add_argument("--hz", type=float, default=20.0, help="publish rate")
    parser.add_argument(
        "--amplitude", type=float, default=0.5, help="oscillation amplitude, radians"
    )
    parser.add_argument(
        "--period", type=float, default=6.0, help="oscillation period, seconds"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Node("cobot_demo_move")
    pub = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)

    dt = 1.0 / args.hz
    t = 0.0
    try:
        while rclpy.ok():
            phase = 2.0 * math.pi * t / args.period
            positions = [
                args.amplitude * math.sin(phase + i * math.pi / NUM_JOINTS)
                for i in range(NUM_JOINTS)
            ]
            pub.publish(Float64MultiArray(data=positions))
            rclpy.spin_once(node, timeout_sec=0.0)
            t += dt
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
