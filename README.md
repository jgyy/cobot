# cobot

Cobot arm simulation + reinforcement learning.

## Tech Stack

- **ROS 2 Jazzy Jalisco** — robot middleware
- **Gazebo Harmonic (gz-harmonic)** — physics simulation
- **MoveIt 2** — motion planning
- **Gymnasium** — RL environment API
- **Universal Robots UR5e** (`ur_description` + `ur_simulation_gz`) — the
  simulated arm, real meshes/kinematics instead of a placeholder model

## Setup (CachyOS / Arch, fish shell)

ROS 2 has no official Arch packages, and the AUR route (compiling `ros2-jazzy` via
`paru`) depends on `aur.archlinux.org` being reachable — it isn't always. This
project instead uses **RoboStack (conda/mamba)**: prebuilt ROS 2 binaries from
conda-forge, installed natively into a conda environment (no container, no
AUR compile). Requires `micromamba` (or `mamba`/`conda`):

```fish
curl -Ls https://micro.mamba.pm/install.sh | bash
```

If that install script itself is unreachable, download `micromamba` manually from
its GitHub releases page and put it on `PATH`.

```fish
micromamba create -n ros2 -c robostack-jazzy -c conda-forge \
  ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros-gz \
  ros-jazzy-ur-simulation-gz \
  python-colcon-common-extensions
```

`ros-jazzy-ur-simulation-gz` pulls in `ur_description` (the real UR5e
meshes/kinematics), `ros2_control`/`ros2_controllers`, and `gz_ros2_control`
transitively — no need to list them separately.

RoboStack's ROS activation hook (`ros-jazzy-ros-workspace_activate.sh`) is a
bash-only script, so plain `micromamba activate ros2` does **not** set
`AMENT_PREFIX_PATH` and friends under fish — `ros2`/`gz` commands will fail
with "package not found" errors. Fix this with
[`bass`](https://github.com/edc/bass), which lets fish source bash scripts:

```fish
fisher install edc/bass
```

RoboStack's `python-colcon-common-extensions` also doesn't reliably pull in
the fish overlay generator for your own workspace builds, so install that
too:

```fish
python3 -m pip install colcon-fish
```

From here on, activate the ROS env with this two-step sequence (not just
`micromamba activate ros2`):

```fish
micromamba activate ros2
bass source $CONDA_PREFIX/setup.bash
```

Gymnasium into the same environment:

```fish
python3 -m pip install gymnasium
```

Sanity check:

```fish
micromamba activate ros2
bass source $CONDA_PREFIX/setup.bash
ros2 doctor
gz sim --version
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## Running RL Training

Build the ROS 2 packages once (from the repo root, with the ROS env active
per the two-step activation above):

```fish
micromamba activate ros2
bass source $CONDA_PREFIX/setup.bash
colcon build --packages-select cobot_bringup
source install/setup.fish
```

Install the `cobot_gym` Python package into the same environment:

```fish
python3 -m pip install -e cobot_gym/
```

**1. Launch the simulation** (keep this running in its own terminal):

```fish
micromamba activate ros2
bass source $CONDA_PREFIX/setup.bash
source install/setup.fish
ros2 launch cobot_bringup sim.launch.py
```

`sim.launch.py` sets `GZ_SIM_SYSTEM_PLUGIN_PATH` (so gz sim finds
`gz_ros2_control`, which RoboStack installs to `$CONDA_PREFIX/lib` but
doesn't add to gz's plugin search path) and `GZ_IP=127.0.0.1` (so gz-transport
discovery doesn't get confused by other network interfaces — VPNs, Docker
bridges, etc.) before including `ur_simulation_gz`'s own launch file
(`ur_type:=ur5e`) — no extra env vars needed. Wait for both
`Configured and activated joint_state_broadcaster` and
`Configured and activated forward_position_controller` in the log before
moving on to training/evaluation. You should see a real UR5e model in the
Gazebo window, not a placeholder shape.

**2. (Optional) Continuous motion demo** — repeatedly publishes a sine-wave
joint motion so you can see the arm actually moving, rather than holding one
pose:

```fish
micromamba activate ros2
bass source $CONDA_PREFIX/setup.bash
source install/setup.fish
python3 cobot_gym/scripts/demo_move.py
```

Ctrl-C to stop. `--amplitude` (radians, default `0.5`) and `--period`
(seconds per oscillation, default `6`) control how it moves; `--hz`
(default `20`) controls the publish rate. This is just a fixed-motion demo —
training (next step) and the trained policy are what actually drive the arm
purposefully.

**3. Train a policy** (same terminal used for the checks above, or a fresh
one with the ROS env active):

```fish
python3 -m cobot_gym.train --timesteps 200000
```

Checkpoints are written to `checkpoints/` every `--save-freq` steps (default
10k), with the final model at `checkpoints/cobot_ppo_final.zip`. TensorBoard
logs go to `logs/`; view them with:

```fish
tensorboard --logdir logs/
```

**4. Evaluate a trained policy** — runs deterministic rollouts against the
live sim and reports mean end-effector tracking error per episode:

```fish
python3 cobot_gym/scripts/evaluate.py --checkpoint checkpoints/cobot_ppo_final.zip --episodes 5
```
