import numpy as np
from stable_baselines3 import PPO
from aerofleet.nav_env import AeroFleetNavEnv

model = PPO.load("models/navigation/train_run_ent0/best/best_model.zip", device="cpu")
env = AeroFleetNavEnv(action_frame="body", max_episode_steps=60)

obs, _ = env.reset(seed=0, options={"start": [0.0, 0.0, 15.0], "target": [10.0, 0.0, 15.0], "initial_yaw": 0.0})

for step in range(60):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, term, trunc, info = env.step(action)
    dist = env.sim.get_observation(0).distance_to_target
    vel = env.sim.get_snapshot().uavs[0].velocity
    pos = env.sim.get_snapshot().uavs[0].position
    print(f"Step {step:2d}: pos=[{pos.x:5.2f}, {pos.y:5.2f}, {pos.z:5.2f}], dist={dist:5.2f}m, act=[{action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f}]")
    if term or trunc:
        print("Done:", info)
        break

env.close()
