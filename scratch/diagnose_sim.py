import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import aerofleet_cpp
from aerofleet_cpp import Simulator, UAVMode, Vec3

print("--- HEURISTIC NAVIGATION SIMULATION ---")
sim_heur = Simulator("config/simulation.json")
sim_heur.reset(42)

for step in range(1, 1501):
    sim_heur.step_heuristic()
    snap = sim_heur.get_snapshot()
    u = snap.uavs[0]
    if step % 150 == 0 or u.mode == UAVMode.PICKING_UP or u.mode == UAVMode.DROPPING_OFF:
        print(f"Step {step:4d} (t={sim_heur.simulated_time:5.1f}s): mode={u.mode}, pos=({u.position.x:6.1f}, {u.position.y:6.1f}, {u.position.z:4.1f}), target=({u.target_waypoint.x:6.1f}, {u.target_waypoint.y:6.1f}, {u.target_waypoint.z:4.1f}), batt={u.battery_percent:5.1f}%, activeReq={u.active_request_id}")

snap_heur = sim_heur.get_snapshot()
print(f"Heuristic Total Delivered: {snap_heur.metrics.total_delivered}, Expired: {snap_heur.metrics.total_expired}, Collisions: {snap_heur.metrics.collision_count}")

print("\n--- ONNX RL NAVIGATION SIMULATION ---")
sim_onnx = Simulator("config/simulation.json")
sim_onnx.reset(42)
sim_onnx.enable_onnx_navigation("models/navigation/nav_policy.onnx", "models/navigation/normalization.json")

for step in range(1, 1501):
    sim_onnx.step_heuristic()
    snap = sim_onnx.get_snapshot()
    u = snap.uavs[0]
    if step % 150 == 0 or u.mode == UAVMode.PICKING_UP or u.mode == UAVMode.DROPPING_OFF:
        print(f"Step {step:4d} (t={sim_onnx.simulated_time:5.1f}s): mode={u.mode}, pos=({u.position.x:6.1f}, {u.position.y:6.1f}, {u.position.z:4.1f}), target=({u.target_waypoint.x:6.1f}, {u.target_waypoint.y:6.1f}, {u.target_waypoint.z:4.1f}), batt={u.battery_percent:5.1f}%, activeReq={u.active_request_id}")

snap_onnx = sim_onnx.get_snapshot()
print(f"ONNX Total Delivered: {snap_onnx.metrics.total_delivered}, Expired: {snap_onnx.metrics.total_expired}, Collisions: {snap_onnx.metrics.collision_count}")
