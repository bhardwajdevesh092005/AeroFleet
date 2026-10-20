"""Evaluate the learned Coordinator policy against FCFS, Nearest-Neighbor, and C++ Heuristic baselines.

Executes comparative benchmarking across 50 seeded demand scenarios under identical
demands. Evaluates delivery success rate, average latency, and fleet throughput.

Usage:
    python3 python/evaluate_coordinator.py --episodes 50 --model-path models/coordinator/coord_policy.zip
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

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
    raise ImportError("evaluate_coordinator.py requires aerofleet_cpp and sb3_contrib.") from exc


def run_fcfs_episode(config_path: str, seed: int, max_steps: int = 300) -> Dict[str, float]:
    """First-Come First-Served (FCFS) dispatcher."""
    env = AeroFleetCoordinatorEnv(config_path=config_path, max_steps=max_steps)
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
    }


def run_nearest_neighbor_episode(config_path: str, seed: int, max_steps: int = 300) -> Dict[str, float]:
    """Nearest Eligible Neighbor dispatcher."""
    env = AeroFleetCoordinatorEnv(config_path=config_path, max_steps=max_steps)
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

            best_uav = -1
            best_dist = float("inf")
            for uav_idx, u in enumerate(uavs[:FLEET_SIZE]):
                if u.mode == UAVMode.IDLE and u.battery_percent > 20.0:
                    dist = u.position.distance(req.pickup_location)
                    if dist < best_dist:
                        best_dist = dist
                        best_uav = uav_idx

            if best_uav >= 0:
                action = 1 + req_idx * FLEET_SIZE + best_uav
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
    }


def run_learned_policy_episode(
    model: MaskablePPO,
    config_path: str,
    seed: int,
    max_steps: int = 300,
) -> Dict[str, float]:
    """Learned MaskablePPO Coordinator."""
    raw_env = AeroFleetCoordinatorEnv(config_path=config_path, max_steps=max_steps)
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
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/simulation.json")
    parser.add_argument("--model-path", type=Path, default=ROOT / "models/coordinator/coord_policy.zip")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--base-seed", type=int, default=1000)
    args = parser.parse_args()

    print("=================================================================")
    print("      AeroFleet Central Fleet Coordinator Benchmark Suite        ")
    print("=================================================================")
    print(f"Episodes:    {args.episodes}")
    print(f"Model:       {args.model_path}")
    print(f"Config:      {args.config}")
    print("=================================================================\n")

    has_model = args.model_path.is_file()
    model = None
    if has_model:
        model = MaskablePPO.load(str(args.model_path))
        print(f"[INFO] Loaded trained MaskablePPO coordinator from {args.model_path}")
    else:
        print(f"[WARN] Trained model {args.model_path} not found. Running baseline comparisons.")

    fcfs_results = []
    nn_results = []
    learned_results = []

    for ep in range(args.episodes):
        seed = args.base_seed + ep
        res_fcfs = run_fcfs_episode(str(args.config), seed, args.max_steps)
        fcfs_results.append(res_fcfs)

        res_nn = run_nearest_neighbor_episode(str(args.config), seed, args.max_steps)
        nn_results.append(res_nn)

        if model is not None:
            res_learned = run_learned_policy_episode(model, str(args.config), seed, args.max_steps)
            learned_results.append(res_learned)

        if (ep + 1) % 5 == 0 or ep + 1 == args.episodes:
            print(f"  Completed {ep + 1}/{args.episodes} episodes...", flush=True)

    def summarize(data: List[Dict[str, float]], name: str) -> None:
        success = [d["success_rate"] for d in data]
        latency = [d["latency"] for d in data]
        delivered = [d["delivered"] for d in data]
        expired = [d["expired"] for d in data]
        print(f"\n--- {name} Results ({len(data)} episodes) ---", flush=True)
        print(f"  Delivery Success Rate: {np.mean(success):.2f}% +/- {np.std(success):.2f}%", flush=True)
        print(f"  Mean Latency:          {np.mean(latency):.2f} s +/- {np.std(latency):.2f} s", flush=True)
        print(f"  Mean Delivered Count:  {np.mean(delivered):.2f}", flush=True)
        print(f"  Mean Expired Count:    {np.mean(expired):.2f}", flush=True)

    summarize(fcfs_results, "First-Come First-Served (FCFS)")
    summarize(nn_results, "Nearest Eligible Neighbor")

    if learned_results:
        summarize(learned_results, "Learned Coordinator (MaskablePPO)")

        fcfs_sr = np.mean([d["success_rate"] for d in fcfs_results])
        learned_sr = np.mean([d["success_rate"] for d in learned_results])
        relative_sr_gain = ((learned_sr - fcfs_sr) / max(fcfs_sr, 1e-6)) * 100.0

        fcfs_del = np.mean([d["delivered"] for d in fcfs_results])
        learned_del = np.mean([d["delivered"] for d in learned_results])
        del_gain = ((learned_del - fcfs_del) / max(fcfs_del, 1e-6)) * 100.0

        fcfs_lat = np.mean([d["latency"] for d in fcfs_results])
        learned_lat = np.mean([d["latency"] for d in learned_results])
        lat_diff = learned_lat - fcfs_lat

        print("\n=================================================================", flush=True)
        print("                   Comparative Benchmark Verdict                 ", flush=True)
        print("=================================================================", flush=True)
        print(f"FCFS Mean Delivered:     {fcfs_del:.2f}", flush=True)
        print(f"Learned Mean Delivered:  {learned_del:.2f} (Gain: +{del_gain:.2f}%)", flush=True)
        print(f"FCFS Success Rate:       {fcfs_sr:.2f}%", flush=True)
        print(f"Learned Success Rate:    {learned_sr:.2f}% (Gain: +{relative_sr_gain:.2f}%)", flush=True)
        print(f"FCFS Mean Latency:       {fcfs_lat:.2f} s", flush=True)
        print(f"Learned Mean Latency:    {learned_lat:.2f} s (Diff: {lat_diff:+.2f} s)", flush=True)

        if del_gain >= 15.0 or relative_sr_gain >= 15.0 or (learned_del > fcfs_del and learned_lat <= fcfs_lat):
            print("[PASS] Learned coordinator achieved superior performance over FCFS baseline!", flush=True)
        else:
            print(f"[STATUS] Comparative throughput gain: +{del_gain:.2f}%", flush=True)


if __name__ == "__main__":
    main()
