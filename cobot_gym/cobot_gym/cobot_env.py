from __future__ import annotations

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from cobot_gym.trajectory import RandomSplineTrajectory

# Real UR5e joint names (ur_description / ur_simulation_gz), not a
# hand-rolled arm.
JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
BASE_FRAME = "world"
EE_FRAME = "tool0"
COMMAND_TOPIC = "/forward_position_controller/commands"

# Roughly the UR5e's reachable workspace (~0.85 m reach) around its base.
DEFAULT_BOUNDS = (
    np.array([-0.6, -0.6, 0.2]),
    np.array([0.6, 0.6, 0.9]),
)
MAX_JOINT_DELTA = 0.05  # rad per control step, applied per action in [-1, 1]

_TF_LOOKUP_ERRORS = (LookupException, ConnectivityException, ExtrapolationException)


class CobotTrackingEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        control_hz: float = 20.0,
        episode_steps: int = 500,
        effort_weight: float = 0.01,
        workspace_bounds: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> None:
        super().__init__()
        self.control_hz = control_hz
        self.episode_steps = episode_steps
        self.effort_weight = effort_weight
        self.workspace_bounds = workspace_bounds or DEFAULT_BOUNDS

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(15,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("cobot_tracking_env")
        self._cmd_pub = self._node.create_publisher(
            Float64MultiArray, COMMAND_TOPIC, 10
        )
        self._joint_state: JointState | None = None
        self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        # world -> tool0 end-effector pose comes from TF (robot_state_publisher
        # broadcasting forward kinematics off /joint_states), not a
        # per-robot Gazebo pose plugin.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._joint_pos = np.zeros(6, dtype=np.float64)
        self._traj: RandomSplineTrajectory | None = None
        self._t = 0.0
        self._step_count = 0

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _lookup_ee_position(self) -> np.ndarray | None:
        try:
            tf = self._tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, Time())
        except _TF_LOOKUP_ERRORS:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=np.float64)

    def _spin_until_fresh_state(self) -> np.ndarray:
        self._joint_state = None
        ee_position = None
        while self._joint_state is None or ee_position is None:
            rclpy.spin_once(self._node, timeout_sec=1.0)
            if self._joint_state is not None:
                ee_position = self._lookup_ee_position()
        return ee_position

    def _current_joint_vector(self, field: str) -> np.ndarray:
        msg = self._joint_state
        values = dict(zip(msg.name, getattr(msg, field)))
        return np.array([values[name] for name in JOINT_ORDER], dtype=np.float64)

    def _build_obs(self, ee_position: np.ndarray) -> np.ndarray:
        joint_pos = self._current_joint_vector("position")
        joint_vel = self._current_joint_vector("velocity")
        ref = self._traj.point_at(self._t)
        error = ref - ee_position
        return np.concatenate([joint_pos, joint_vel, error]).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._traj = RandomSplineTrajectory(
            self.workspace_bounds, num_waypoints=5,
            duration_s=self.episode_steps / self.control_hz, seed=seed,
        )
        self._t = 0.0
        self._step_count = 0
        self._joint_pos = np.zeros(6, dtype=np.float64)
        self._cmd_pub.publish(Float64MultiArray(data=self._joint_pos.tolist()))
        ee_position = self._spin_until_fresh_state()
        obs = self._build_obs(ee_position)
        return obs, {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0)
        self._joint_pos = self._joint_pos + action * MAX_JOINT_DELTA
        self._cmd_pub.publish(Float64MultiArray(data=self._joint_pos.tolist()))

        ee_position = self._spin_until_fresh_state()
        self._t += 1.0 / self.control_hz
        self._step_count += 1

        ref = self._traj.point_at(self._t)
        tracking_error = np.linalg.norm(ref - ee_position)
        effort_penalty = self.effort_weight * float(np.sum(np.square(action)))
        reward = -tracking_error - effort_penalty

        terminated = False
        truncated = self._step_count >= self.episode_steps
        obs = self._build_obs(ee_position)
        info = {"tracking_error": tracking_error}
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        self._node.destroy_node()
