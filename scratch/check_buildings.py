import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import aerofleet_cpp
from aerofleet_cpp import Simulator, Vec3

sim = Simulator("config/simulation.json")
map = sim.get_map()
target = Vec3(167.45, -175.06, 15.0)
drone = Vec3(158.0, -172.0, 16.2)

print(f"Target: {target}")
print(f"Drone pos: {drone}")
for i, b in enumerate(map.buildings):
    dist_target = b.bounds.distance(target)
    dist_drone = b.bounds.distance(drone)
    if dist_target < 30.0 or dist_drone < 30.0:
        print(f"Building {i}: min=({b.bounds.min_point.x:.1f}, {b.bounds.min_point.y:.1f}, {b.bounds.min_point.z:.1f}) max=({b.bounds.max_point.x:.1f}, {b.bounds.max_point.y:.1f}, {b.bounds.max_point.z:.1f}) -> dist to target: {dist_target:.2f}m, dist to drone: {dist_drone:.2f}m")
