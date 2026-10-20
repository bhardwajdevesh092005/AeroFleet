import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import aerofleet_cpp
from aerofleet_cpp import Simulator, UAVMode

sim = Simulator("config/simulation.json")
sim.reset(42)
sim.enable_onnx_navigation("models/navigation/nav_policy.onnx", "models/navigation/normalization.json")

for step in range(1, 1301):
    sim.step_heuristic()
    snap = sim.get_snapshot()
    u = snap.uavs[0]
    if step >= 1150 and step <= 1250:
        obs = sim.get_observation(0)
        dist = u.position.distance(u.target_waypoint)
        print(f"Step {step:4d}: pos=({u.position.x:.2f}, {u.position.y:.2f}, {u.position.z:.2f}), vel=({u.velocity.x:.2f}, {u.velocity.y:.2f}, {u.velocity.z:.2f}), target=({u.target_waypoint.x:.2f}, {u.target_waypoint.y:.2f}, {u.target_waypoint.z:.2f}), dist={dist:.2f}m")
