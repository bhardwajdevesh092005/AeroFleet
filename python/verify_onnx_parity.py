"""Verify numerical parity between PyTorch and ONNX Runtime navigation policy inference.

Runs synthetic random observations through both PyTorch and ONNX Runtime sessions,
verifying that the maximum absolute error across 10,000 samples satisfies epsilon < 1e-5.
Also benchmarks dynamic batching and execution latency.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]


class NavigationActor(torch.nn.Module):
    """Deterministic actor network extracted from SB3 MlpPolicy."""

    def __init__(self, policy):
        super().__init__()
        self.policy_net = policy.mlp_extractor.policy_net
        self.action_net = policy.action_net

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        features = self.policy_net(observation)
        action = self.action_net(features)
        return torch.clamp(action, -1.0, 1.0)


def verify_dual_parity(
    model_path: Path,
    onnx_path: Path,
    num_samples: int = 10_000,
    seed: int = 42,
    tolerance: float = 1e-5,
) -> dict[str, float]:
    """Run dual-inference parity test between PyTorch and ONNX Runtime."""
    if not model_path.is_file():
        raise FileNotFoundError(f"PyTorch model not found: {model_path}")
    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

    # Load PyTorch actor
    model = PPO.load(str(model_path), device="cpu")
    actor = NavigationActor(model.policy)
    actor.eval()

    # Load ONNX Runtime session
    opts = ort.SessionOptions()
    opts.inter_op_num_threads = 1
    opts.intra_op_num_threads = 1
    session = ort.InferenceSession(str(onnx_path), sess_options=opts, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    # Generate synthetic random inputs across normalized domain [-10.0, 10.0]
    torch.manual_seed(seed)
    np.random.seed(seed)
    synthetic_inputs = torch.randn(num_samples, 87, dtype=torch.float32) * 2.0
    synthetic_inputs = torch.clamp(synthetic_inputs, -10.0, 10.0)

    # 1. Full batch inference
    with torch.no_grad():
        pytorch_actions = actor(synthetic_inputs).numpy()

    onnx_inputs = {input_name: synthetic_inputs.numpy()}
    onnx_actions = session.run(None, onnx_inputs)[0]

    max_abs_error = float(np.max(np.abs(pytorch_actions - onnx_actions)))
    mean_abs_error = float(np.mean(np.abs(pytorch_actions - onnx_actions)))

    # 2. Dynamic batch size test: test [1, 87], [8, 87], and [64, 87]
    dynamic_batch_max_err = 0.0
    for bs in (1, 8, 32, 64):
        sub_input = synthetic_inputs[:bs]
        with torch.no_grad():
            sub_pt = actor(sub_input).numpy()
        sub_onnx = session.run(None, {input_name: sub_input.numpy()})[0]
        err = float(np.max(np.abs(sub_pt - sub_onnx)))
        dynamic_batch_max_err = max(dynamic_batch_max_err, err)

    # 3. Single-step latency benchmark (1,000 runs of batch_size=1)
    single_input = {input_name: synthetic_inputs[:1].numpy()}
    # Warmup
    for _ in range(50):
        session.run(None, single_input)
    t0 = time.perf_counter()
    n_latency_steps = 1000
    for _ in range(n_latency_steps):
        session.run(None, single_input)
    t1 = time.perf_counter()
    latency_ms_per_step = ((t1 - t0) / n_latency_steps) * 1000.0

    metrics = {
        "num_samples": num_samples,
        "max_abs_error": max_abs_error,
        "mean_abs_error": mean_abs_error,
        "dynamic_batch_max_error": dynamic_batch_max_err,
        "latency_ms_per_step": latency_ms_per_step,
        "tolerance": tolerance,
        "passed": float(max_abs_error < tolerance),
    }

    if max_abs_error >= tolerance:
        raise AssertionError(
            f"Dual parity assertion failed: max error {max_abs_error:.6e} >= tolerance {tolerance:.6e}"
        )

    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "models/navigation/nav_policy.zip",
        help="Path to PyTorch/SB3 model checkpoint",
    )
    parser.add_argument(
        "--onnx",
        type=Path,
        default=ROOT / "models/navigation/nav_policy.onnx",
        help="Path to exported ONNX model",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=10_000,
        help="Number of synthetic observation test samples (default: 10,000)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for synthetic test generation",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-5,
        help="Maximum allowable absolute error (default: 1e-5)",
    )
    args = parser.parse_args()

    print("=================================================================")
    print("       AeroFleet Dual-Inference Parity Verification Report       ")
    print("=================================================================")
    print(f"PyTorch Model: {args.model}")
    print(f"ONNX Model:    {args.onnx}")
    print(f"Samples:       {args.samples:,}")
    print(f"Tolerance:     {args.tolerance:.1e}")

    try:
        metrics = verify_dual_parity(
            args.model,
            args.onnx,
            num_samples=args.samples,
            seed=args.seed,
            tolerance=args.tolerance,
        )
    except AssertionError as err:
        print(f"\n[FAIL] {err}", file=sys.stderr)
        sys.exit(1)

    print("\nResults:")
    print(f"  - Max Absolute Error:     {metrics['max_abs_error']:.6e}")
    print(f"  - Mean Absolute Error:    {metrics['mean_abs_error']:.6e}")
    print(f"  - Dynamic Batch Max Err:  {metrics['dynamic_batch_max_error']:.6e}")
    print(f"  - Single-Step Latency:    {metrics['latency_ms_per_step']:.4f} ms ({1000.0 / metrics['latency_ms_per_step']:.0f} Hz)")
    print(f"  - Threshold Check:        {metrics['max_abs_error']:.6e} < {metrics['tolerance']:.1e} [OK]")
    print("\n[PASS] Dual-Inference parity verified successfully.")


if __name__ == "__main__":
    main()
