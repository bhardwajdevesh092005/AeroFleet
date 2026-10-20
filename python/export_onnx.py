"""Export trained navigation policy actor from PyTorch/SB3 to ONNX format.

Extracts the deterministic actor network from a Stable-Baselines3 PPO checkpoint
and exports it using torch.onnx.export with dynamic batch axis support.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
import onnx
import torch
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]


class NavigationActor(torch.nn.Module):
    """Deterministic actor network extracted from SB3 MlpPolicy.

    Maps normalized observation [batch_size, 87] to normalized desired
    velocity [batch_size, 3] in [-1.0, 1.0].
    """

    def __init__(self, policy: Any) -> None:
        super().__init__()
        self.policy_net = policy.mlp_extractor.policy_net
        self.action_net = policy.action_net

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        features = self.policy_net(observation)
        action = self.action_net(features)
        return torch.clamp(action, -1.0, 1.0)


def _get_git_commit() -> str:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL, text=True
        ).strip()
        return commit
    except Exception:
        return "unknown"


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while chunk := f.read(65536):
            h.update(chunk)
    return h.hexdigest()


def export_actor_to_onnx(
    model_path: Path,
    output_onnx: Path,
    opset_version: int = 17,
) -> Path:
    """Load SB3 PPO model, extract actor, and export to ONNX."""
    if not model_path.is_file():
        raise FileNotFoundError(f"Model checkpoint not found: {model_path}")

    model = PPO.load(str(model_path), device="cpu")
    actor = NavigationActor(model.policy)
    actor.eval()

    dummy_input = torch.zeros(1, 87, dtype=torch.float32)
    output_onnx.parent.mkdir(parents=True, exist_ok=True)

    # Export using TorchScript-based exporter for maximum C++ runtime compatibility
    try:
        torch.onnx.export(
            actor,
            dummy_input,
            str(output_onnx),
            input_names=["observation"],
            output_names=["action"],
            dynamic_axes={
                "observation": {0: "batch_size"},
                "action": {0: "batch_size"},
            },
            opset_version=opset_version,
            dynamo=False,
        )
    except TypeError:
        torch.onnx.export(
            actor,
            dummy_input,
            str(output_onnx),
            input_names=["observation"],
            output_names=["action"],
            dynamic_axes={
                "observation": {0: "batch_size"},
                "action": {0: "batch_size"},
            },
            opset_version=opset_version,
        )

    # Validate ONNX model with native onnx validator
    onnx_model = onnx.load(str(output_onnx))
    onnx.checker.check_model(onnx_model)
    return output_onnx


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "models/navigation/nav_policy.zip",
        help="Path to SB3 PPO checkpoint .zip",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "models/navigation/nav_policy.onnx",
        help="Destination path for exported ONNX model",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=ROOT / "models/navigation/manifest.json",
        help="Path to model manifest JSON",
    )
    parser.add_argument(
        "--normalization",
        type=Path,
        default=ROOT / "models/navigation/normalization.json",
        help="Path to observation normalization JSON",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version (default: 17)",
    )
    args = parser.parse_args()

    print(f"Loading policy from: {args.model}")
    exported_path = export_actor_to_onnx(args.model, args.output, opset_version=args.opset)
    onnx_sha256 = _sha256_file(exported_path)
    file_size_kb = exported_path.stat().st_size / 1024.0

    print(f"Exported ONNX model: {exported_path} ({file_size_kb:.1f} KB)")
    print(f"ONNX SHA-256: {onnx_sha256}")

    # Update or create manifest
    manifest_data: dict[str, Any] = {}
    if args.manifest.is_file():
        manifest_data = json.loads(args.manifest.read_text(encoding="utf-8"))

    manifest_data.update({
        "task": "AeroFleet Navigation Policy",
        "version": 2,
        "model_type": "PPO_MlpPolicy",
        "observation_schema_version": 3,
        "observation_dim": 87,
        "action_dim": 3,
        "action_frame": "body",
        "action_limits": {
            "forward_mps": 15.0,
            "left_mps": 15.0,
            "up_mps": 5.0,
        },
        "onnx_file": args.output.name,
        "onnx_sha256": onnx_sha256,
        "onnx_opset_version": args.opset,
        "normalization_file": args.normalization.name,
        "git_commit": _get_git_commit(),
    })
    args.manifest.write_text(json.dumps(manifest_data, indent=2) + "\n", encoding="utf-8")
    print(f"Updated manifest at: {args.manifest}")


if __name__ == "__main__":
    main()
