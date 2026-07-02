# cobot

Cobot arm simulation + reinforcement learning.

## Tech Stack

- **ROS 2 Jazzy Jalisco** — robot middleware
- **Gazebo Harmonic (gz-harmonic)** — physics simulation
- **MoveIt 2** — motion planning
- **Gymnasium** — RL environment API

## Setup (CachyOS / Arch)

ROS 2 has no official Arch packages, and the AUR route (compiling `ros2-jazzy` via
`paru`) depends on `aur.archlinux.org` being reachable — it isn't always. This
project instead uses **RoboStack (conda/mamba)**: prebuilt ROS 2 binaries from
conda-forge, installed natively into a conda environment (no container, no
AUR compile). Requires `micromamba` (or `mamba`/`conda`):

```bash
curl -Ls https://micro.mamba.pm/install.sh | bash
```

If that install script itself is unreachable, download `micromamba` manually from
its GitHub releases page and put it on `PATH`.

```bash
micromamba create -n ros2 -c robostack-jazzy -c conda-forge \
  ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros-gz \
  python-colcon-common-extensions
micromamba activate ros2
```

Gymnasium into the same environment:

```bash
python3 -m pip install gymnasium
```

Sanity check:

```bash
micromamba activate ros2
ros2 doctor
gz sim --version
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## Running RL Training

Build the ROS 2 packages once (from the repo root, with the `ros2` env active):

```bash
micromamba activate ros2
colcon build --packages-select cobot_description cobot_bringup
source install/setup.bash   # fish: source install/setup.fish
```

If you're on `fish` and `install/setup.fish` doesn't exist after building,
RoboStack's `python-colcon-common-extensions` didn't pull in the fish
overlay generator — install it and rebuild:

```fish
python3 -m pip install colcon-fish
colcon build --packages-select cobot_description cobot_bringup
source install/setup.fish
```

Install the `cobot_gym` Python package into the same environment:

```bash
python3 -m pip install -e cobot_gym/
```

**1. Launch the simulation** (keep this running in its own terminal):

```bash
micromamba activate ros2
source install/setup.bash   # fish: source install/setup.fish
ros2 launch cobot_bringup sim.launch.py
```

**2. Train a policy** (in a second terminal, same `ros2` env active):

```bash
python3 -m cobot_gym.train --timesteps 200000
```

Checkpoints are written to `checkpoints/` every `--save-freq` steps (default
10k), with the final model at `checkpoints/cobot_ppo_final.zip`. TensorBoard
logs go to `logs/`; view them with:

```bash
tensorboard --logdir logs/
```

**3. Evaluate a trained policy** — runs deterministic rollouts against the
live sim and reports mean end-effector tracking error per episode:

```bash
python3 cobot_gym/scripts/evaluate.py --checkpoint checkpoints/cobot_ppo_final.zip --episodes 5
```
