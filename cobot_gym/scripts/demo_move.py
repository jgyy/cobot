from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = "/forward_position_controller/commands"

# A pick-and-place style cycle: rest -> approach a part on the left -> pick
# it up -> retreat -> rest -> approach a drop point on the right -> place it
# -> retreat -> rest. All poses verified to keep the end effector well clear
# of the ground plane on the UR5e. Joint order:
# shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
APPROACH_LEFT = [0.9, -1.2, 1.4, -1.9, -1.57, 0.0]
PICK_LEFT = [0.9, -1.0, 1.7, -2.3, -1.57, 0.0]
APPROACH_RIGHT = [-0.9, -1.2, 1.4, -1.9, -1.57, 0.0]
PLACE_RIGHT = [-0.9, -1.0, 1.7, -2.3, -1.57, 0.0]

# (target pose, move duration in seconds, dwell time at the target in
# seconds — dwell mimics a gripper actuating before the next move starts).
CYCLE = [
    (HOME, 2.0, 0.0),
    (APPROACH_LEFT, 2.0, 0.3),
    (PICK_LEFT, 1.2, 0.6),  # dwell = simulated grasp
    (APPROACH_LEFT, 1.2, 0.3),
    (HOME, 2.0, 0.0),
    (APPROACH_RIGHT, 2.0, 0.3),
    (PLACE_RIGHT, 1.2, 0.6),  # dwell = simulated release
    (APPROACH_RIGHT, 1.2, 0.3),
]


def smootherstep(x: float) -> float:
    """Ease-in-out, zero velocity and acceleration at both ends."""
    x = min(max(x, 0.0), 1.0)
    return 6 * x**5 - 15 * x**4 + 10 * x**3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Cycle the arm through a smooth pick-and-place motion"
    )
    parser.add_argument("--hz", type=float, default=50.0, help="publish rate")
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="multiplier on move/dwell durations; >1 is slower",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Node("cobot_demo_move")
    pub = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)

    dt = 1.0 / args.hz
    current = np.array(HOME, dtype=np.float64)

    try:
        while rclpy.ok():
            for target, move_s, dwell_s in CYCLE:
                target = np.array(target, dtype=np.float64)
                start = current.copy()
                move_s *= args.speed
                dwell_s *= args.speed
                steps = max(int(move_s / dt), 1)

                for step in range(1, steps + 1):
                    eased = smootherstep(step / steps)
                    pose = start + (target - start) * eased
                    pub.publish(Float64MultiArray(data=pose.tolist()))
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(dt)

                current = target
                if dwell_s > 0:
                    end_time = time.monotonic() + dwell_s
                    while time.monotonic() < end_time:
                        pub.publish(Float64MultiArray(data=current.tolist()))
                        rclpy.spin_once(node, timeout_sec=0.0)
                        time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
