"""Train the Stage 1 navigation policy with PPO in Python/Gymnasium.

Run from the repository root, after building ``aerofleet_cpp`` and installing
``requirements-rl.txt``::

    python3 python/train_nav.py --total-timesteps 100000 --seed 1
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/simulation.json")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--resume", type=Path, help="continue from an existing PPO .zip checkpoint")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--n-envs", type=int, default=1)
    parser.add_argument("--max-episode-steps", type=int, default=2_000)
    parser.add_argument("--rollout-steps", type=int, default=1_024)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--gamma", type=float, default=0.999)
    parser.add_argument("--gae-lambda", type=float, default=0.99)
    parser.add_argument("--eval-freq", type=int, default=5_000)
    parser.add_argument("--eval-episodes", type=int, default=20)
    parser.add_argument("--checkpoint-freq", type=int, default=10_000)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--tensorboard", action="store_true")
    args = parser.parse_args()

    if args.total_timesteps <= 0 or args.n_envs <= 0:
        parser.error("total-timesteps and n-envs must be positive")
    if args.max_episode_steps <= 0 or args.rollout_steps <= 0:
        parser.error("max-episode-steps and rollout-steps must be positive")
    if args.batch_size <= 0 or args.eval_freq <= 0 or args.eval_episodes <= 0:
        parser.error("batch-size, eval-freq, and eval-episodes must be positive")
    if not 0.0 < args.gamma <= 1.0 or not 0.0 < args.gae_lambda <= 1.0:
        parser.error("gamma and gae-lambda must be in (0, 1]")
    if args.checkpoint_freq <= 0:
        parser.error("checkpoint-freq must be positive")
    if args.batch_size > args.rollout_steps * args.n_envs:
        parser.error("batch-size cannot exceed rollout-steps multiplied by n-envs")
    if not args.config.is_file():
        parser.error(f"configuration file does not exist: {args.config}")

    if args.output_dir is None:
        args.output_dir = ROOT / "runs/navigation" / f"seed-{args.seed}"
    return args


def _metadata(args: argparse.Namespace) -> dict[str, Any]:
    # Import after CLI parsing so --help works even if optional RL packages are
    # not installed in the current Python environment.
    from aerofleet.nav_env import (  # noqa: PLC0415
        OBSERVATION_FEATURE_ORDER,
        OBSERVATION_SCHEMA_VERSION,
        NAVIGATION_TASK_VERSION,
    )

    return {
        "algorithm": "PPO",
        "training_runtime": "Python",
        "environment": "AeroFleetNavEnv",
        "scenario": "open_waypoint",
        "randomize_waypoint_per_episode": True,
        "randomize_initial_yaw_per_episode": True,
        "navigation_task_version": NAVIGATION_TASK_VERSION,
        "action_frame": "body",
        "seed": args.seed,
        "requested_additional_timesteps": args.total_timesteps,
        "hyperparameters": {
            "learning_rate": 3e-4,
            "rollout_steps_per_env": args.rollout_steps,
            "batch_size": args.batch_size,
            "epochs_per_update": 10,
            "gamma_per_policy_step": args.gamma,
            "gae_lambda": args.gae_lambda,
            "clip_range": 0.2,
        },
        "resumed_from": str(args.resume.resolve()) if args.resume else None,
        "config": str(args.config.resolve()),
        "observation_schema_version": OBSERVATION_SCHEMA_VERSION,
        "observation_feature_order": list(OBSERVATION_FEATURE_ORDER),
        "action": {
            "meaning": "normalized desired velocity [forward, left, up]",
            "shape": [3],
            "low": -1.0,
            "high": 1.0,
        },
    }


def _check_resume_contract(
    resume_path: Path, schema_version: int, task_version: int, args: argparse.Namespace
) -> None:
    """Reject a checkpoint whose saved observation/action contract is stale."""
    candidates = (resume_path.parent / "training_config.json", resume_path.parent.parent / "training_config.json")
    config_path = next((path for path in candidates if path.is_file()), None)
    if config_path is None:
        raise ValueError(
            f"cannot verify checkpoint contract: no training_config.json adjacent to {resume_path}"
        )
    saved = json.loads(config_path.read_text(encoding="utf-8"))
    if saved.get("observation_schema_version") != schema_version:
        raise ValueError(
            "checkpoint observation schema does not match this environment "
            f"({saved.get('observation_schema_version')} != {schema_version})"
        )
    if (saved.get("action_frame") != "body" or saved.get("scenario") != "open_waypoint"
            or saved.get("navigation_task_version") != task_version):
        raise ValueError("checkpoint action/scenario contract is incompatible with this trainer")
    if saved.get("randomize_initial_yaw_per_episode") is not True:
        raise ValueError("checkpoint was trained without randomized initial heading")
    saved_hyperparameters = saved.get("hyperparameters", {})
    for key, requested in (("gamma_per_policy_step", args.gamma), ("gae_lambda", args.gae_lambda)):
        if key in saved_hyperparameters and float(saved_hyperparameters[key]) != requested:
            raise ValueError(
                f"resume checkpoint uses {key}={saved_hyperparameters[key]}, "
                f"but requested {requested}; PPO.load preserves the checkpoint value"
            )


def _make_seeded_eval_callback(
    eval_env: Any, output_dir: Path, eval_freq: int, n_episodes: int, seed_start: int
) -> Any:
    """Evaluate on a fixed route set and rank success before shaping return."""
    from stable_baselines3.common.callbacks import BaseCallback

    class SeededEvaluationCallback(BaseCallback):
        def __init__(self) -> None:
            super().__init__(verbose=1)
            self.best_key: tuple[float, float, float, float] | None = None
            self.eval_seeds = [seed_start + index for index in range(n_episodes)]
            self.history_path = output_dir / "evaluations" / "seeded_eval.jsonl"
            self.last_eval_timesteps = -1

        def _on_step(self) -> bool:
            if self.num_timesteps > 0 and self.num_timesteps % eval_freq == 0:
                self._evaluate_and_rank()
            return True

        def _on_training_end(self) -> None:
            if self.num_timesteps > 0 and self.last_eval_timesteps != self.num_timesteps:
                self._evaluate_and_rank()

        def _evaluate_and_rank(self) -> None:
            returns: list[float] = []
            lengths: list[int] = []
            success: list[bool] = []
            final_distances: list[float] = []
            collision_episodes: list[bool] = []
            distance_scale = float(eval_env.unwrapped.observation_schema["normalization"]["target_distance_m"])
            for route_seed in self.eval_seeds:
                observation, _ = eval_env.reset(seed=route_seed)
                episode_return = 0.0
                length = 0
                terminated = truncated = False
                info: dict[str, Any] = {}
                while not (terminated or truncated):
                    action, _ = self.model.predict(observation, deterministic=True)
                    observation, reward, terminated, truncated, info = eval_env.step(action)
                    episode_return += float(reward)
                    length += 1
                returns.append(episode_return)
                lengths.append(length)
                success.append(bool(info.get("is_success", False)))
                final_distances.append(float(observation[9]) * distance_scale)
                collision_episodes.append(int(info.get("metrics", {}).get("collision_count", 0)) > 0)
            row = {
                "timesteps": int(self.num_timesteps),
                "seed_start": seed_start,
                "seed_end": seed_start + n_episodes - 1,
                "episodes": n_episodes,
                "mean_return": float(sum(returns) / n_episodes),
                "success_rate": float(sum(success) / n_episodes),
                "mean_episode_steps": float(sum(lengths) / n_episodes),
                "mean_final_distance_m": float(sum(final_distances) / n_episodes),
                "collision_episode_rate": float(sum(collision_episodes) / n_episodes),
            }
            with self.history_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row) + "\n")
            self.last_eval_timesteps = int(self.num_timesteps)
            # Fixed routes make this comparison paired across checkpoints.
            # Success is primary, then remaining distance, then shaped return.
            key = (
                row["success_rate"],
                -row["collision_episode_rate"],
                -row["mean_final_distance_m"],
                row["mean_return"],
            )
            if self.best_key is None or key > self.best_key:
                self.best_key = key
                self.model.save(str(output_dir / "best" / "best_model"))
                (output_dir / "best" / "best_metrics.json").write_text(
                    json.dumps(row, indent=2) + "\n", encoding="utf-8"
                )
            print(json.dumps(row))

    return SeededEvaluationCallback()


def main() -> None:
    args = _parse_args()
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import (
            CallbackList,
            CheckpointCallback,
        )
        from stable_baselines3.common.env_checker import check_env as sb3_check_env
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.vec_env import DummyVecEnv
    except ImportError as exc:
        raise SystemExit(
            "PPO training requires Stable-Baselines3 and PyTorch. Install the "
            "Python learning dependencies with `python3 -m pip install -r "
            "requirements-rl.txt`."
        ) from exc

    from aerofleet.nav_env import AeroFleetNavEnv  # noqa: PLC0415
    from aerofleet.nav_env import (  # noqa: PLC0415
        NAVIGATION_TASK_VERSION,
        OBSERVATION_SCHEMA_VERSION,
    )

    if args.resume:
        _check_resume_contract(
            args.resume, OBSERVATION_SCHEMA_VERSION, NAVIGATION_TASK_VERSION, args
        )

    if args.tensorboard:
        try:
            import tensorboard  # noqa: F401, PLC0415
        except ImportError as exc:
            raise SystemExit(
                "TensorBoard logging was requested; install it with "
                "`python3 -m pip install tensorboard`."
            ) from exc

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "checkpoints").mkdir(exist_ok=True)
    (args.output_dir / "best").mkdir(exist_ok=True)
    (args.output_dir / "evaluations").mkdir(exist_ok=True)
    (args.output_dir / "monitor").mkdir(exist_ok=True)
    def make_training_env(rank: int):
        def create():
            env = AeroFleetNavEnv(
                config_path=args.config,
                max_episode_steps=args.max_episode_steps,
                randomize_waypoint=True,
                action_frame="body",
            )
            monitor_file = args.output_dir / "monitor" / f"train-{rank}"
            return Monitor(env, filename=str(monitor_file), info_keywords=("is_success",))

        return create

    validation_env = AeroFleetNavEnv(
        config_path=args.config,
        max_episode_steps=args.max_episode_steps,
        randomize_waypoint=True,
        action_frame="body",
    )
    try:
        sb3_check_env(validation_env, warn=True)
    finally:
        validation_env.close()

    train_env = DummyVecEnv([make_training_env(i) for i in range(args.n_envs)])
    train_env.seed(args.seed)

    eval_env = Monitor(AeroFleetNavEnv(
            config_path=args.config,
            max_episode_steps=args.max_episode_steps,
            randomize_waypoint=True,
            action_frame="body",
        ), filename=str(args.output_dir / "monitor" / "evaluation"), info_keywords=("is_success",))
    eval_callback = _make_seeded_eval_callback(
        eval_env,
        args.output_dir,
        max(args.eval_freq // args.n_envs, 1) * args.n_envs,
        args.eval_episodes,
        seed_start=args.seed + 1_000_000,
    )
    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.checkpoint_freq // args.n_envs, 1),
        save_path=str(args.output_dir / "checkpoints"),
        name_prefix="nav_ppo",
        verbose=1,
    )
    callbacks = CallbackList([eval_callback, checkpoint_callback])
    tensorboard_log = str(args.output_dir / "tensorboard") if args.tensorboard else None

    if args.resume:
        model = PPO.load(
            str(args.resume),
            env=train_env,
            device=args.device,
            tensorboard_log=tensorboard_log,
        )
    else:
        model = PPO(
            "MlpPolicy",
            train_env,
            learning_rate=3e-4,
            n_steps=args.rollout_steps,
            batch_size=args.batch_size,
            n_epochs=10,
            gamma=args.gamma,
            gae_lambda=args.gae_lambda,
            clip_range=0.2,
            policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}},
            tensorboard_log=tensorboard_log,
            seed=args.seed,
            device=args.device,
            verbose=1,
        )
    initial_timesteps = int(model.num_timesteps)
    metadata = _metadata(args)
    metadata.update({
        "initial_model_timesteps": initial_timesteps,
        "requested_final_model_timesteps": initial_timesteps + args.total_timesteps,
        "evaluation": {
            "seed_start": args.seed + 1_000_000,
            "episodes": args.eval_episodes,
            "checkpoint_ranking": [
                "success_rate", "lower_collision_episode_rate",
                "lower_mean_final_distance_m", "mean_return",
            ],
        },
        "observation_schema": train_env.envs[0].unwrapped.observation_schema,
    })
    metadata["hyperparameters"]["gamma_per_policy_step"] = float(model.gamma)
    metadata["hyperparameters"]["gae_lambda"] = float(model.gae_lambda)
    (args.output_dir / "training_config.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callbacks,
            tb_log_name="nav_ppo",
            progress_bar=False,
            reset_num_timesteps=args.resume is None,
        )
        model.save(str(args.output_dir / "final_model"))
        metadata["actual_final_model_timesteps"] = int(model.num_timesteps)
        (args.output_dir / "training_config.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
    finally:
        train_env.close()
        eval_env.close()

    print(f"Training finished. Model and logs: {args.output_dir}")
    print(
        "Evaluate the policy with: python3 python/evaluate_nav.py "
        f"--model {args.output_dir / 'best' / 'best_model.zip'}"
    )


if __name__ == "__main__":
    main()
