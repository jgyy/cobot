from __future__ import annotations

import argparse

import numpy as np
from stable_baselines3 import PPO

from cobot_gym.cobot_env import CobotTrackingEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a trained CobotTrackingEnv policy")
    parser.add_argument("--checkpoint", type=str, required=True)
    parser.add_argument("--episodes", type=int, default=5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = CobotTrackingEnv()
    model = PPO.load(args.checkpoint, env=env)

    episode_means = []
    for ep in range(args.episodes):
        obs, info = env.reset(seed=ep)
        errors = []
        terminated = truncated = False
        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            errors.append(info["tracking_error"])
        mean_error = float(np.mean(errors))
        episode_means.append(mean_error)
        print(f"episode {ep}: mean tracking error = {mean_error:.4f} m")

    print(f"overall mean tracking error = {np.mean(episode_means):.4f} m")
    env.close()


if __name__ == "__main__":
    main()
