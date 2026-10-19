"""Evaluate a trained Stage 1 navigation policy on seeded episodes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BATTERY_OBSERVATION_INDEX = 10


def _training_metadata_path(model_path: Path) -> Path | None:
    """Find the run manifest for final_model.zip or best/best_model.zip."""
    for candidate in (
        model_path.parent / "training_config.json",
        model_path.parent.parent / "training_config.json",
    ):
        if candidate.is_file():
            return candidate
    return None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True, help="SB3 PPO model (.zip optional)")
    parser.add_argument("--config", type=Path, default=ROOT / "config/simulation.json")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=10_000)
    parser.add_argument("--max-episode-steps", type=int, default=2_000)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--action-frame", choices=("body", "world"))
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error("episodes must be positive")
    if args.max_episode_steps <= 0:
        parser.error("max-episode-steps must be positive")
    if not args.config.is_file():
        parser.error(f"configuration file does not exist: {args.config}")
    if not args.model.is_file() and not Path(f"{args.model}.zip").is_file():
        parser.error(f"model file does not exist: {args.model} (or {args.model}.zip)")
    return args


def _run_policy(model: Any, env: Any, episodes: int, seed: int) -> dict[str, Any]:
    returns: list[float] = []
    lengths: list[int] = []
    successes: list[bool] = []
    collisions: list[int] = []
    battery_used: list[float] = []
    final_distances: list[float] = []
    distance_scale = float(env.observation_schema["normalization"]["target_distance_m"])

    for episode in range(episodes):
        observation, _ = env.reset(seed=seed + episode)
        initial_battery = float(observation[BATTERY_OBSERVATION_INDEX])
        episode_return = 0.0
        steps = 0
        terminated = truncated = False
        info: dict[str, Any] = {}
        while not (terminated or truncated):
            action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = env.step(action)
            episode_return += float(reward)
            steps += 1

        returns.append(episode_return)
        lengths.append(steps)
        successes.append(bool(info.get("is_success", info.get("success", False))))
        metrics = info.get("metrics", {})
        collisions.append(int(metrics.get("collision_count", 0)))
        final_battery = float(observation[BATTERY_OBSERVATION_INDEX])
        battery_used.append(max(0.0, initial_battery - final_battery))
        final_distances.append(float(observation[9]) * distance_scale)

    return {
        "episodes": episodes,
        "mean_return": float(np.mean(returns)),
        "std_return": float(np.std(returns)),
        "success_rate": float(np.mean(successes)),
        "mean_episode_steps": float(np.mean(lengths)),
        "collision_episode_rate": float(np.mean(np.asarray(collisions) > 0)),
        "mean_collision_events": float(np.mean(collisions)),
        "mean_battery_fraction_used": float(np.mean(battery_used)),
        "mean_final_distance_m": float(np.mean(final_distances)),
        "median_final_distance_m": float(np.median(final_distances)),
        "per_episode": [
            {
                "episode": index,
                "seed": seed + index,
                "return": returns[index],
                "steps": lengths[index],
                "success": successes[index],
                "collision_events": collisions[index],
                "battery_fraction_used": battery_used[index],
                "final_distance_m": final_distances[index],
            }
            for index in range(episodes)
        ],
    }


def main() -> None:
    args = _parse_args()
    try:
        from stable_baselines3 import PPO
    except ImportError as exc:
        raise SystemExit(
            "Policy evaluation requires Stable-Baselines3 and PyTorch. Install "
            "the Python learning dependencies with `python3 -m pip install -r "
            "requirements-rl.txt`."
        ) from exc

    from aerofleet.nav_env import AeroFleetNavEnv  # noqa: PLC0415
    from aerofleet.nav_env import (  # noqa: PLC0415
        NAVIGATION_TASK_VERSION,
        OBSERVATION_SCHEMA_VERSION,
    )

    metadata_path = _training_metadata_path(args.model)
    if metadata_path is None:
        raise SystemExit(
            "Cannot verify the model's observation/action contract: training_config.json "
            "was not found beside the model. Evaluate only with a matching run manifest."
        )
    training_metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    saved_schema = training_metadata.get("observation_schema_version")
    if saved_schema != OBSERVATION_SCHEMA_VERSION:
        raise SystemExit(
            f"Model observation schema v{saved_schema} does not match environment "
            f"schema v{OBSERVATION_SCHEMA_VERSION}; retrain or restore the matching environment."
        )
    if training_metadata.get("scenario") != "open_waypoint":
        raise SystemExit("This evaluator supports only the open_waypoint training scenario")
    if training_metadata.get("navigation_task_version") != NAVIGATION_TASK_VERSION:
        raise SystemExit("Model navigation task version does not match the current reset distribution")
    if training_metadata.get("randomize_initial_yaw_per_episode") is not True:
        raise SystemExit("Model manifest does not declare randomized initial heading")
    manifest_frame = training_metadata.get("action_frame")
    if args.action_frame and manifest_frame and args.action_frame != manifest_frame:
        raise SystemExit("--action-frame conflicts with the saved training manifest")
    action_frame = args.action_frame or manifest_frame
    if action_frame not in ("body", "world"):
        raise SystemExit("training manifest is missing a valid action_frame")

    env = AeroFleetNavEnv(
        config_path=args.config,
        max_episode_steps=args.max_episode_steps,
        randomize_waypoint=True,
        action_frame=action_frame,
    )
    try:
        saved_schema = training_metadata.get("observation_schema")
        current_schema = json.loads(json.dumps(env.observation_schema))
        if saved_schema != current_schema:
            raise SystemExit("Model observation normalization/features do not match the current environment")
        model = PPO.load(str(args.model), env=env, device=args.device)
        results = _run_policy(model, env, args.episodes, args.seed)
    finally:
        env.close()

    report = {
        "model": str(args.model.resolve()),
        "training_manifest": str(metadata_path.resolve()),
        "scenario": "open_waypoint_seeded_random_routes",
        "action_frame": action_frame,
        "observation_schema_version": OBSERVATION_SCHEMA_VERSION,
        "seed_start": args.seed,
        "seed_end": args.seed + args.episodes - 1,
        **results,
    }
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
