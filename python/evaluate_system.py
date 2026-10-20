"""End-to-End System Integration Benchmark Suite for AeroFleet.

Evaluates and compares 4 configurations under identical seeded Poisson demand:
  Config 1: Baseline Heuristic Nav + Baseline Heuristic Coordinator (FCFS)
  Config 2: Trained ONNX Nav + Baseline Heuristic Coordinator (FCFS)
  Config 3: Baseline Heuristic Nav + Trained MaskablePPO Coordinator
  Config 4: Trained ONNX Nav + Trained MaskablePPO Coordinator (Full AeroFleet)

Usage:
    python python/evaluate_system.py --seeds 5 --max-steps 300
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "python"))

try:
    import aerofleet_cpp
    from aerofleet_cpp import RequestStatus, Simulator, UAVMode, Vec3
    from aerofleet import AeroFleetCoordinatorEnv
    from aerofleet.coordinator_env import FLEET_SIZE, MAX_OBSERVED_REQUESTS
    from sb3_contrib import MaskablePPO
    from sb3_contrib.common.wrappers import ActionMasker
except ImportError as exc:
    raise ImportError("evaluate_system.py requires aerofleet_cpp and sb3_contrib.") from exc


def run_fcfs_episode(
    config_path: str,
    seed: int,
    use_onnx_nav: bool,
    max_steps: int = 300,
    model_path: str = "models/navigation/nav_policy.onnx",
    norm_path: str = "models/navigation/normalization.json",
) -> Dict[str, Any]:
    """Run episode with First-Come First-Served coordinator and specified navigation."""
    env = AeroFleetCoordinatorEnv(
        config_path=config_path,
        use_onnx_nav=use_onnx_nav,
        model_path=model_path,
        norm_path=norm_path,
        max_steps=max_steps,
    )
    obs, info = env.reset(seed=seed)
    sim = env.simulator

    for _ in range(max_steps):
        pending = sim.get_pending_requests()
        snapshot = sim.get_snapshot()
        uavs = snapshot.uavs
        sim_time = sim.simulated_time

        action = 0  # Default WAIT
        for req_idx, req in enumerate(pending[:MAX_OBSERVED_REQUESTS]):
            if req.status != RequestStatus.PENDING or sim_time >= req.deadline:
                continue
            if req.payload_mass > sim.max_payload_mass:
                continue

            for uav_idx, u in enumerate(uavs[:FLEET_SIZE]):
                if u.mode == UAVMode.IDLE and u.battery_percent > 20.0:
                    action = 1 + req_idx * FLEET_SIZE + uav_idx
                    break
            if action > 0:
                break

        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break

    snap = sim.get_snapshot()
    m = snap.metrics
    return {
        "delivered": float(m.total_delivered),
        "expired": float(m.total_expired),
        "created": float(m.total_created),
        "success_rate": float(m.delivery_success_rate),
        "latency": float(m.average_delivery_duration),
        "collisions": int(m.collision_count),
    }


def run_learned_episode(
    model: MaskablePPO,
    config_path: str,
    seed: int,
    use_onnx_nav: bool,
    max_steps: int = 300,
    model_path: str = "models/navigation/nav_policy.onnx",
    norm_path: str = "models/navigation/normalization.json",
) -> Dict[str, Any]:
    """Run episode with learned MaskablePPO coordinator and specified navigation."""
    raw_env = AeroFleetCoordinatorEnv(
        config_path=config_path,
        use_onnx_nav=use_onnx_nav,
        model_path=model_path,
        norm_path=norm_path,
        max_steps=max_steps,
    )
    env = ActionMasker(raw_env, lambda e: e.action_masks())
    obs, info = env.reset(seed=seed)

    for _ in range(max_steps):
        action_masks = env.action_masks()
        action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(int(action))
        if terminated or truncated:
            break

    snap = raw_env.simulator.get_snapshot()
    m = snap.metrics
    return {
        "delivered": float(m.total_delivered),
        "expired": float(m.total_expired),
        "created": float(m.total_created),
        "success_rate": float(m.delivery_success_rate),
        "latency": float(m.average_delivery_duration),
        "collisions": int(m.collision_count),
    }


def aggregate_stats(episodes: List[Dict[str, Any]]) -> Dict[str, float]:
    """Compute mean and std across episodes."""
    if not episodes:
        return {}
    keys = ["delivered", "expired", "created", "success_rate", "latency", "collisions"]
    res = {}
    for k in keys:
        vals = [float(ep[k]) for ep in episodes]
        res[f"{k}_mean"] = float(np.mean(vals))
        res[f"{k}_std"] = float(np.std(vals))
    return res


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/simulation.json")
    parser.add_argument("--nav-model", type=Path, default=ROOT / "models/navigation/nav_policy.onnx")
    parser.add_argument("--nav-norm", type=Path, default=ROOT / "models/navigation/normalization.json")
    parser.add_argument("--coord-model", type=Path, default=ROOT / "models/coordinator/coord_policy.zip")
    parser.add_argument("--seeds", type=int, default=5, help="Number of random seeds to evaluate")
    parser.add_argument("--base-seed", type=int, default=1000)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--output", type=Path, default=ROOT / "logs/system_benchmark.json")
    args = parser.parse_args()

    print("=================================================================")
    print("      AeroFleet End-to-End System 4-Way Comparative Benchmark    ")
    print("=================================================================")
    print(f"Seeds:          {args.seeds} (base: {args.base_seed})")
    print(f"Max Steps:      {args.max_steps} per episode")
    print(f"Config:         {args.config}")
    print(f"Nav Model:      {args.nav_model}")
    print(f"Coord Model:    {args.coord_model}")
    print("=================================================================\n")

    if not args.coord_model.is_file():
        raise FileNotFoundError(f"Coordinator model checkpoint not found: {args.coord_model}")
    if not args.nav_model.is_file():
        raise FileNotFoundError(f"Navigation model not found: {args.nav_model}")

    print("[1/5] Loading trained MaskablePPO Coordinator checkpoint...")
    coord_model = MaskablePPO.load(str(args.coord_model))
    print("[1/5] Coordinator loaded successfully.\n")

    config_names = [
        "Config 1: Heuristic Nav + FCFS Coord",
        "Config 2: ONNX Nav + FCFS Coord",
        "Config 3: Heuristic Nav + Learned Coord",
        "Config 4: ONNX Nav + Learned Coord (Full AeroFleet)",
    ]

    results: Dict[str, List[Dict[str, Any]]] = {name: [] for name in config_names}

    start_time = time.time()
    for idx in range(args.seeds):
        seed = args.base_seed + idx
        print(f"--- Running Seed {idx + 1}/{args.seeds} (seed={seed}) ---", flush=True)

        # Config 1: Heuristic Nav + FCFS
        res1 = run_fcfs_episode(
            str(args.config), seed, use_onnx_nav=False, max_steps=args.max_steps,
            model_path=str(args.nav_model), norm_path=str(args.nav_norm),
        )
        results[config_names[0]].append(res1)

        # Config 2: ONNX Nav + FCFS
        res2 = run_fcfs_episode(
            str(args.config), seed, use_onnx_nav=True, max_steps=args.max_steps,
            model_path=str(args.nav_model), norm_path=str(args.nav_norm),
        )
        results[config_names[1]].append(res2)

        # Config 3: Heuristic Nav + Learned Coord
        res3 = run_learned_episode(
            coord_model, str(args.config), seed, use_onnx_nav=False, max_steps=args.max_steps,
            model_path=str(args.nav_model), norm_path=str(args.nav_norm),
        )
        results[config_names[2]].append(res3)

        # Config 4: ONNX Nav + Learned Coord (Full AeroFleet)
        res4 = run_learned_episode(
            coord_model, str(args.config), seed, use_onnx_nav=True, max_steps=args.max_steps,
            model_path=str(args.nav_model), norm_path=str(args.nav_norm),
        )
        results[config_names[3]].append(res4)

        print(f"    C1 (Heur/FCFS): Del={res1['delivered']}, Exp={res1['expired']}, Coll={res1['collisions']}")
        print(f"    C2 (ONNX/FCFS): Del={res2['delivered']}, Exp={res2['expired']}, Coll={res2['collisions']}")
        print(f"    C3 (Heur/RL):   Del={res3['delivered']}, Exp={res3['expired']}, Coll={res3['collisions']}")
        print(f"    C4 (ONNX/RL):   Del={res4['delivered']}, Exp={res4['expired']}, Coll={res4['collisions']}")

    elapsed = time.time() - start_time
    print(f"\nCompleted {args.seeds} seeds in {elapsed:.2f} seconds.\n")

    summary = {name: aggregate_stats(results[name]) for name in config_names}

    print("=========================================================================================")
    print("                           4-WAY COMPARATIVE BENCHMARK SUMMARY                           ")
    print("=========================================================================================")
    header = f"{'Configuration':<45} | {'Mean Deliv':<10} | {'Mean Expired':<12} | {'Success Rate':<12} | {'Collisions'}"
    print(header)
    print("-" * len(header))
    for name in config_names:
        st = summary[name]
        del_str = f"{st['delivered_mean']:.2f} ± {st['delivered_std']:.2f}"
        exp_str = f"{st['expired_mean']:.2f} ± {st['expired_std']:.2f}"
        succ_str = f"{st['success_rate_mean']:.1f}%"
        coll_str = f"{st['collisions_mean']:.2f}"
        print(f"{name:<45} | {del_str:<10} | {exp_str:<12} | {succ_str:<12} | {coll_str}")
    print("=========================================================================================\n")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump({"summary": summary, "raw": results, "seeds": args.seeds, "elapsed_seconds": elapsed}, f, indent=2)
    print(f"[EXPORT] Full benchmark telemetry saved to {args.output}")


if __name__ == "__main__":
    main()
