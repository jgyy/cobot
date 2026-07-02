# Cobot Arm Trajectory-Tracking RL — Design

Date: 2026-07-02

## Goal

Train a reinforcement-learning policy that makes a simulated 6-DOF cobot arm
track a randomized smooth 3D end-effector trajectory, using the existing
project stack (ROS 2 Jazzy, Gazebo Harmonic, Gymnasium, Stable-Baselines3 PPO).

## Scope

- Generic 6-DOF arm URDF/xacro (stand-in model, not vendor-specific)
- Gazebo simulation launch (ROS 2 + gz-harmonic)
- Gymnasium environment wrapping the sim via direct ROS 2 topics (rclpy) —
  no MoveIt in the RL control loop (MoveIt stays available for other uses,
  per the existing README, but per-step RL control bypasses its planning
  latency)
- Randomized smooth reference trajectory generator (spline through random
  waypoints), resampled every episode
- PPO training entrypoint (Stable-Baselines3)
- Evaluation script to run a trained checkpoint and report tracking error
- README section documenting how to run training and evaluation

Out of scope: pick-and-place, grasping, real-hardware deployment, multi-arm.

## Package layout

```
cobot_description/           # ROS 2 package: URDF/xacro + meshes + Gazebo model
  urdf/cobot.urdf.xacro
  package.xml, CMakeLists.txt

cobot_bringup/                # ROS 2 package: launch files
  launch/sim.launch.py        # starts Gazebo, spawns arm, starts ros2_control controllers
  package.xml, CMakeLists.txt

cobot_gym/                    # Python package (pip-installable, not a ROS package)
  cobot_gym/
    __init__.py
    trajectory.py             # RandomSplineTrajectory: samples waypoints, builds a
                               # smooth path, exposes point-at-time lookahead
    cobot_env.py               # CobotTrackingEnv(gymnasium.Env), an rclpy node:
                               #   - publishes joint velocity/position commands
                               #   - subscribes to /joint_states for obs
                               #   - subscribes to end-effector pose (via TF or a
                               #     Gazebo pose topic) for reward/obs
                               #   - reset() samples a new RandomSplineTrajectory
                               #   - step() advances sim, computes reward from
                               #     tracking error + control-effort penalty
    train.py                  # builds env, wraps with SB3 VecEnv, trains PPO,
                               # saves checkpoints + TensorBoard logs
  scripts/
    evaluate.py                # loads a checkpoint, runs deterministic rollout,
                               # prints mean tracking error, optionally plots it
  setup.py
  pyproject.toml (or setup.cfg)
```

## Environment details

**Observation space** (`Box`, float32):
- Current joint positions (6)
- Current joint velocities (6)
- Reference path lookahead point, end-effector frame relative offset (3)
- Current tracking error vector (3)

**Action space** (`Box`, float32): joint position or velocity deltas (6),
clipped to safe joint limits read from the URDF.

**Reward**: `-||ee_pos - ref_point||` (tracking error) `- effort_weight * ||action||^2`
(control-effort penalty), computed each step.

**Episode**: fixed horizon (default 500 steps at a fixed control rate, e.g.
20 Hz); `reset()` re-samples a new random spline trajectory through N random
waypoints within the arm's reachable workspace.

**Sim <-> Gym bridge**: `CobotTrackingEnv` is itself an `rclpy` node created
in `__init__`; `step()` publishes a command, spins the node briefly to pick
up the next `/joint_states` message, and returns obs/reward/done. Gazebo
runs as a separate process (started via `cobot_bringup`'s launch file, not
by the Gym env itself) so training scripts assume the sim is already up.

## Training

`train.py`:
- CLI args: total timesteps, checkpoint dir, log dir, seed
- Builds `CobotTrackingEnv`, wraps in SB3's `Monitor` + `PPO("MlpPolicy", ...)`
- Periodic checkpoint saving via SB3's `CheckpointCallback`
- TensorBoard logging enabled by default

`evaluate.py`:
- CLI args: checkpoint path, number of episodes
- Loads the policy, runs deterministic rollouts against the live sim,
  reports mean/final tracking error per episode

## README update

Add a "Running RL Training" section after the existing Setup section,
covering:
1. Launch the sim: `ros2 launch cobot_bringup sim.launch.py`
2. Install the `cobot_gym` package (`pip install -e cobot_gym/`)
3. Train: `python3 -m cobot_gym.train --timesteps 200000`
4. Evaluate: `python3 cobot_gym/scripts/evaluate.py --checkpoint <path>`
5. Where checkpoints/TensorBoard logs are written, and how to view logs
   (`tensorboard --logdir ...`)

## Testing

- Unit test `trajectory.py` (pure Python, no ROS/Gazebo needed): waypoint
  sampling produces a smooth path, lookahead-at-time returns points on the
  path, path stays within workspace bounds.
- Manual verification: launch sim, run a short training run (a few thousand
  timesteps) to confirm the env doesn't crash and reward is finite/improves,
  then run `evaluate.py` against the resulting checkpoint.
- Full RL convergence is not something we can verify programmatically in
  this session — the manual smoke test above is the acceptance bar.
