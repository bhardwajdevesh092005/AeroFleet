import numpy as np
from stable_baselines3 import PPO
from aerofleet.nav_env import AeroFleetNavEnv

model = PPO.load("models/navigation/train_run_ent0/best/best_model.zip", device="cpu")
env = AeroFleetNavEnv(randomize_waypoint=True, action_frame="body", max_episode_steps=1000)

for ep in range(5):
    obs, _ = env.reset(seed=1000042 + ep)
    init_dist = env.sim.get_observation(0).distance_to_target
    print(f"\n--- Episode {ep}: initial distance = {init_dist:.1f} m ---")
    min_dist = init_dist
    for step in range(1000):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, term, trunc, info = env.step(action)
        dist = env.sim.get_observation(0).distance_to_target
        min_dist = min(min_dist, dist)
        if step % 100 == 0 or term or trunc:
            vel = env.sim.get_snapshot().uavs[0].velocity
            print(f"Step {step:3d}: dist={dist:6.2f}m, act=[{action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f}], vel=[{vel.x:.1f}, {vel.y:.1f}, {vel.z:.1f}]")
        if term or trunc:
            print(f"Outcome: step={step}, success={info.get('success', False)}, final_dist={dist:.2f}m, min_dist={min_dist:.2f}m")
            break

env.close()
