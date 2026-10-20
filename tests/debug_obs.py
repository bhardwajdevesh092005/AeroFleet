import numpy as np
from stable_baselines3 import PPO
from aerofleet.nav_env import AeroFleetNavEnv, OBSERVATION_FEATURE_ORDER

model = PPO.load("models/navigation/train_run_ent0/best/best_model.zip", device="cpu")
env = AeroFleetNavEnv(randomize_waypoint=True, action_frame="body", max_episode_steps=1000)

obs, _ = env.reset(seed=1000042 + 4)
for step in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, term, trunc, info = env.step(action)
    dist = env.sim.get_observation(0).distance_to_target
    if dist < 5.0 or step == 700:
        print(f"\nStep {step}, dist={dist:.2f}m:")
        raw = env.sim.get_observation_numpy(0)
        print("Raw features [0:10]:", raw[0:10])
        print("Normalized obs [0:10]:", obs[0:10])
        print("Heading sin/cos:", raw[75:77])
        print("Action:", action)
        break

env.close()
