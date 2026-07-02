# cobot

Cobot arm simulation + reinforcement learning.

## Tech Stack

- **ROS 2 Jazzy Jalisco** — robot middleware
- **Gazebo Harmonic (gz-harmonic)** — physics simulation
- **MoveIt 2** — motion planning
- **Gymnasium** — RL environment API

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
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  python-colcon-common-extensions
```

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
colcon build --packages-select cobot_description cobot_bringup
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
bridges, etc.) — no extra env vars needed. Wait for both
`Configured and activated joint_state_broadcaster` and
`Configured and activated joint_group_position_controller` in the log before
moving on to training/evaluation.

**2. (Optional) Sanity-check joint control** — in a second terminal, send a
one-shot position command to all 6 joints and confirm the arm moves in the
Gazebo window:

```fish
micromamba activate ros2
bass source $CONDA_PREFIX/setup.bash
source install/setup.fish
ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.8, 0.5, -0.5, 0.3, 0.4, 0.2]}"
```

The arm's rest pose (all joints at 0) is a straight vertical pole — that's
expected, not a bug, since nothing is publishing commands yet. The command
above bends it into a zig-zag; each of the 6 numbers is a joint angle in
radians (roughly `-3.14` to `3.14`). Send `{data: [0, 0, 0, 0, 0, 0]}` to
reset it straight. Confirm the joints actually reached those positions with:

```fish
ros2 topic echo /joint_states --once
```

This only holds a single pose — it doesn't move continuously, since nothing
is publishing repeatedly on that topic. Continuous motion is what training
(next step) and the trained policy actually drive.

**3. Train a policy** (same terminal used for the sanity check above, or a
fresh one with the ROS env active):

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
