"""Train the Central Fleet Coordinator policy with MaskablePPO in Python/Gymnasium.

Trains a high-level scheduling policy to dispatch delivery requests across the fleet,
maximizing delivery throughput while minimizing wait times, tardiness, and energy.

Usage:
    python3 python/train_coordinator.py --total-timesteps 50000 --seed 42
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "python"))

try:
    from sb3_contrib import MaskablePPO
    from sb3_contrib.common.maskable.evaluation import evaluate_policy
    from sb3_contrib.common.wrappers import ActionMasker
    from aerofleet import AeroFleetCoordinatorEnv
except ImportError as exc:
    raise ImportError(
        "train_coordinator.py requires sb3_contrib and aerofleet. "
        "Please install requirements."
    ) from exc


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/simulation.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "models/coordinator")
    parser.add_argument("--total-timesteps", type=int, default=50_000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--rollout-steps", type=int, default=512)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--ent-coef", type=float, default=0.01)
    parser.add_argument("--max-episode-steps", type=int, default=300)
    parser.add_argument("--use-onnx-nav", action="store_true", default=True)
    return parser.parse_args()


def export_coordinator_onnx(model: MaskablePPO, output_path: Path) -> None:
    """Extract deterministic actor MLP and export to ONNX format."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    class CoordinatorActor(nn.Module):
        def __init__(self, policy: Any):
            super().__init__()
            self.features_extractor = policy.features_extractor
            self.mlp_extractor = policy.mlp_extractor.policy_net
            self.action_net = policy.action_net

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            features = self.features_extractor(obs)
            latent = self.mlp_extractor(features)
            logits = self.action_net(latent)
            return logits

    actor = CoordinatorActor(model.policy).to("cpu")
    actor.eval()

    dummy_input = torch.zeros((1, 164), dtype=torch.float32, device="cpu")

    try:
        torch.onnx.export(
            actor,
            dummy_input,
            str(output_path),
            input_names=["observation"],
            output_names=["action_logits"],
            dynamic_axes={"observation": {0: "batch_size"}, "action_logits": {0: "batch_size"}},
            opset_version=17,
            dynamo=False,
        )
    except TypeError:
        torch.onnx.export(
            actor,
            dummy_input,
            str(output_path),
            input_names=["observation"],
            output_names=["action_logits"],
            dynamic_axes={"observation": {0: "batch_size"}, "action_logits": {0: "batch_size"}},
            opset_version=17,
        )
    print(f"[EXPORT] Coordinator policy exported successfully to {output_path}")


def main() -> None:
    args = _parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    print("=================================================================")
    print("      AeroFleet Central Fleet Coordinator Training (MaskablePPO) ")
    print("=================================================================")
    print(f"Config:          {args.config}")
    print(f"Total Timesteps: {args.total_timesteps}")
    print(f"Seed:            {args.seed}")
    print(f"Output Dir:      {args.output_dir}")
    print("=================================================================")

    def make_env():
        raw_env = AeroFleetCoordinatorEnv(
            config_path=str(args.config),
            use_onnx_nav=args.use_onnx_nav,
            max_steps=args.max_episode_steps,
        )
        return ActionMasker(raw_env, lambda e: e.action_masks())

    env = make_env()

    policy_kwargs = dict(
        net_arch=dict(pi=[256, 256], vf=[256, 256]),
        activation_fn=nn.ReLU,
    )

    model = MaskablePPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=args.learning_rate,
        n_steps=args.rollout_steps,
        batch_size=args.batch_size,
        gamma=args.gamma,
        ent_coef=args.ent_coef,
        policy_kwargs=policy_kwargs,
        seed=args.seed,
        verbose=1,
    )

    print("\n[TRAIN] Beginning MaskablePPO rollout collection...")
    model.learn(total_timesteps=args.total_timesteps)

    # Save SB3 zip model
    zip_path = args.output_dir / "coord_policy.zip"
    model.save(str(zip_path))
    print(f"[SAVE] Checkpoint saved to {zip_path}")

    # Export ONNX model
    onnx_path = args.output_dir / "coord_policy.onnx"
    export_coordinator_onnx(model, onnx_path)

    # Write metadata manifest
    manifest = {
        "model_name": "AeroFleetCoordinatorPolicy",
        "algorithm": "MaskablePPO",
        "action_space": 101,
        "observation_dim": 164,
        "total_timesteps": args.total_timesteps,
        "seed": args.seed,
    }
    with open(args.output_dir / "manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)

    print("\n[COMPLETE] Coordinator training and ONNX export successfully finished!")


if __name__ == "__main__":
    main()
