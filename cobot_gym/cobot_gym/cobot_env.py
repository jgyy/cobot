from __future__ import annotations

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from cobot_gym.trajectory import RandomSplineTrajectory

JOINT_ORDER = [f"joint_{i}_joint" for i in range(1, 7)]
DEFAULT_BOUNDS = (
    np.array([-0.5, -0.5, 0.2]),
    np.array([0.5, 0.5, 0.8]),
)
MAX_JOINT_DELTA = 0.05  # rad per control step, applied per action in [-1, 1]


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
            Float64MultiArray, "/joint_group_position_controller/commands", 10
        )
        self._joint_state: JointState | None = None
        self._ee_pose: PoseStamped | None = None
        self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self._node.create_subscription(
            PoseStamped, "/cobot/ee_pose", self._on_ee_pose, 10
        )

        self._joint_pos = np.zeros(6, dtype=np.float64)
        self._traj: RandomSplineTrajectory | None = None
        self._t = 0.0
        self._step_count = 0

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        self._ee_pose = msg

    def _spin_until_fresh_state(self) -> None:
        self._joint_state = None
        self._ee_pose = None
        while self._joint_state is None or self._ee_pose is None:
            rclpy.spin_once(self._node, timeout_sec=1.0)

    def _current_joint_vector(self, field: str) -> np.ndarray:
        msg = self._joint_state
        values = dict(zip(msg.name, getattr(msg, field)))
        return np.array([values[name] for name in JOINT_ORDER], dtype=np.float64)

    def _ee_position(self) -> np.ndarray:
        p = self._ee_pose.pose.position
        return np.array([p.x, p.y, p.z], dtype=np.float64)

    def _build_obs(self) -> np.ndarray:
        joint_pos = self._current_joint_vector("position")
        joint_vel = self._current_joint_vector("velocity")
        ref = self._traj.point_at(self._t)
        error = ref - self._ee_position()
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
        self._spin_until_fresh_state()
        obs = self._build_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0)
        self._joint_pos = self._joint_pos + action * MAX_JOINT_DELTA
        self._cmd_pub.publish(Float64MultiArray(data=self._joint_pos.tolist()))

        self._spin_until_fresh_state()
        self._t += 1.0 / self.control_hz
        self._step_count += 1

        ref = self._traj.point_at(self._t)
        tracking_error = np.linalg.norm(ref - self._ee_position())
        effort_penalty = self.effort_weight * float(np.sum(np.square(action)))
        reward = -tracking_error - effort_penalty

        terminated = False
        truncated = self._step_count >= self.episode_steps
        obs = self._build_obs()
        info = {"tracking_error": tracking_error}
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        self._node.destroy_node()
