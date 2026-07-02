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
