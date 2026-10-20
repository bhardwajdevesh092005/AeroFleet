import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "python"))

from aerofleet import AeroFleetCoordinatorEnv
from aerofleet_cpp import UAVMode

env = AeroFleetCoordinatorEnv("config/simulation.json", use_onnx_nav=True, max_steps=600)
obs, info = env.reset(seed=42)
sim = env.simulator
print("Initial pending:", len(sim.get_pending_requests()))

delivered_total = 0
reported_emergencies = set()
for step in range(600):
    masks = env.action_masks()
    valid_actions = [a for a in range(len(masks)) if masks[a]]
    action = 0
    if len(valid_actions) > 1:
        action = valid_actions[1]
    obs, r, term, trunc, info = env.step(action)
    snap = sim.get_snapshot()
    if snap.metrics.total_delivered > delivered_total:
        delivered_total = snap.metrics.total_delivered
        print(f"DELIVERY COMPLETED at step {step} (t={sim.simulated_time:.1f}s)! Total delivered = {delivered_total}")
    if step % 50 == 0 or step == 299:
        modes = [u.mode.name for u in snap.uavs]
        print(f"Step {step:3d} (t={sim.simulated_time:5.1f}s): delivered={snap.metrics.total_delivered} expired={snap.metrics.total_expired} pending={len(sim.get_pending_requests())} active_modes={dict((m, modes.count(m)) for m in set(modes))}")

    for u in snap.uavs:
        if u.mode == UAVMode.EMERGENCY and u.id not in reported_emergencies:
            reported_emergencies.add(u.id)
            print(f"EMERGENCY DETECTED at step {step} (t={sim.simulated_time:.1f}s) for UAV {u.id}: pos=({u.position.x:.2f},{u.position.y:.2f},{u.position.z:.2f})")
    if snap.metrics.total_delivered > delivered_total:
        delivered_total = snap.metrics.total_delivered
        print(f"DELIVERY COMPLETED at step {step} (t={sim.simulated_time:.1f}s)! Total delivered = {delivered_total}")
    if step % 50 == 0 or step == 299:
        modes = [u.mode.name for u in snap.uavs]
        batt = [f"{u.battery_percent:.1f}%" for u in snap.uavs]
        print(f"Step {step:3d} (t={sim.simulated_time:5.1f}s): delivered={snap.metrics.total_delivered} expired={snap.metrics.total_expired} pending={len(sim.get_pending_requests())} active_modes={dict((m, modes.count(m)) for m in set(modes))}")
        print(f"   Batteries: {batt[:5]}")

