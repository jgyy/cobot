# cobot

Cobot arm simulation + reinforcement learning.

## Tech Stack

- **ROS 2 Jazzy Jalisco** — robot middleware
- **Gazebo Harmonic (gz-harmonic)** — physics simulation
- **MoveIt 2** — motion planning
- **Gymnasium** — RL environment API

## Setup (CachyOS / Arch)

ROS 2 has no official Arch packages, so this project uses the AUR route via `paru`.

### 1. Base ROS 2 Jazzy

```bash
paru -S ros2-jazzy
```

Pulls the full desktop meta-package. This compiles ~300 packages from source —
expect 1-3 hours and 15-20GB disk on first install.

### 2. Environment setup

Add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
```

or `~/.config/fish/config.fish`:

```fish
source /opt/ros/jazzy/setup.fish
```

### 3. Gazebo (Harmonic, not Classic — Classic is EOL)

```bash
paru -S gz-harmonic
paru -S ros-jazzy-ros-gz
```

`ros-jazzy-ros-gz` is the ROS 2 <-> Gazebo bridge, required to connect topics/services
between the two.

### 4. MoveIt 2

```bash
paru -S ros-jazzy-moveit
```

### 5. Build tooling

```bash
paru -S python-colcon-common-extensions ros-jazzy-ros2-controllers ros-jazzy-ros2-control
sudo rosdep init
rosdep update
```

### 6. Gymnasium (Python, pip not AUR)

```bash
python3 -m pip install --user gymnasium
```

Note: if `gymnasium` or its deps lack wheels for your Python version, fall back to a
`venv`/`pyenv` with Python 3.11 or 3.12.

### Sanity check

```bash
source /opt/ros/jazzy/setup.bash
ros2 doctor
gz sim --version
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
