from __future__ import annotations

import argparse

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback

from cobot_gym.cobot_env import CobotTrackingEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train PPO on CobotTrackingEnv")
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--checkpoint-dir", type=str, default="checkpoints")
    parser.add_argument("--log-dir", type=str, default="logs")
    parser.add_argument("--save-freq", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = Monitor(CobotTrackingEnv())

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        seed=args.seed,
        tensorboard_log=args.log_dir,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq,
        save_path=args.checkpoint_dir,
        name_prefix="cobot_ppo",
    )

    model.learn(total_timesteps=args.timesteps, callback=checkpoint_callback)
    model.save(f"{args.checkpoint_dir}/cobot_ppo_final")
    env.close()


if __name__ == "__main__":
    main()
