"""Unit and integration coverage for the Gymnasium navigation wrapper.

Run with: PYTHONPATH=python python -m pytest tests/test_nav_env.py
Requires requirements-rl.txt and an importable aerofleet_cpp extension.
"""

import json

import numpy as np
import pytest

from aerofleet.nav_env import (
    OBSERVATION_FEATURE_ORDER,
    OBSERVATION_SCHEMA_VERSION,
    AeroFleetNavEnv,
    _normalized_velocity,
)


def test_normalized_action_scales_horizontal_and_vertical_limits_independently():
    velocity = _normalized_velocity(np.array([1.0, -0.5, 0.25]), 12.0, 4.0)

    np.testing.assert_allclose(velocity, [12.0, -6.0, 1.0])
    assert velocity.dtype == np.float64


@pytest.mark.parametrize(
    "action",
    [
        np.array([0.0, 1.0]),
        np.array([0.0, np.nan, 0.0]),
        np.array([0.0, 0.0, np.inf]),
        np.array([1.01, 0.0, 0.0]),
        np.array([0.0, -1.01, 0.0]),
    ],
)
def test_invalid_normalized_actions_are_rejected(action):
    with pytest.raises(ValueError):
        _normalized_velocity(action, 15.0, 5.0)


def test_reset_seed_reproduces_observation_including_seed_zero():
    env = AeroFleetNavEnv()
    try:
        first, info_first = env.reset(seed=0)
        second, info_second = env.reset(seed=0)

        assert first.shape == (87,)
        assert first.dtype == np.float32
        assert env.observation_space.contains(first)
        np.testing.assert_array_equal(first, second)
        assert info_first["seed"] == info_second["seed"] == 0
        assert info_first["scenario"] == "open_waypoint"
        assert info_first["observation_schema_version"] == OBSERVATION_SCHEMA_VERSION
        assert len(OBSERVATION_FEATURE_ORDER) == 87
        assert env.observation_schema["version"] == OBSERVATION_SCHEMA_VERSION
        assert env.sim.fleet_size == 1
        assert len(env.sim.get_snapshot().active_requests) == 0
    finally:
        env.close()


def test_step_returns_gymnasium_transition_and_truncates_at_episode_limit():
    env = AeroFleetNavEnv(max_episode_steps=1)
    try:
        env.reset(seed=123)
        observation, reward, terminated, truncated, info = env.step(
            np.zeros(3, dtype=np.float32)
        )

        assert observation.shape == (87,)
        assert observation.dtype == np.float32
        assert env.observation_space.contains(observation)
        assert np.isfinite(reward)
        assert isinstance(terminated, bool)
        assert truncated is (not terminated)
        assert info["episode_steps"] == 1
        assert info["uav_id"] == 0
        assert info["simulated_time"] > 0.0
    finally:
        env.close()


def test_environment_passes_gymnasium_api_checker():
    from gymnasium.utils.env_checker import check_env

    env = AeroFleetNavEnv(max_episode_steps=20)
    try:
        check_env(env, skip_render_check=True)
    finally:
        env.close()


def test_default_waypoint_task_is_reachable_and_terminates_successfully():
    env = AeroFleetNavEnv(max_episode_steps=1_000)
    try:
        observation, _ = env.reset(seed=7)
        initial_distance = float(env.sim.get_observation(0).distance_to_target)
        assert initial_distance > 2.0
        assert np.all(observation >= env.observation_space.low)
        assert np.all(observation <= env.observation_space.high)

        previous_distance = initial_distance
        for _ in range(env.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.array([1.0, 0.0, 0.0], dtype=np.float32)
            )
            current_distance = float(env.sim.get_observation(0).distance_to_target)
            assert env.observation_space.contains(observation)
            if terminated or truncated:
                break
            previous_distance = current_distance

        assert terminated
        assert not truncated
        assert reward > 9.0
        assert info["success"]
        assert env.sim.get_observation(0).distance_to_target <= 2.0
        dt_policy = 0.1
        max_progress = (env.sim.max_horizontal_speed + env.sim.max_vertical_speed) * dt_policy
        expected_reward = -0.01 + 0.1 * np.clip(
            previous_distance - current_distance, -max_progress, max_progress
        ) + 10.0
        assert reward == pytest.approx(expected_reward, abs=1e-5)
    finally:
        env.close()


def test_waypoint_reset_options_are_validated():
    env = AeroFleetNavEnv()
    try:
        with pytest.raises(ValueError, match="three coordinates"):
            env.reset(seed=1, options={"start": [0.0, 0.0], "target": [10.0, 0.0, 15.0]})
        with pytest.raises(ValueError, match="unsupported scenario"):
            env.reset(seed=1, options={"scenario": "unknown"})
    finally:
        env.close()


def test_randomized_routes_are_seeded_and_vary_across_seeds():
    env = AeroFleetNavEnv(randomize_waypoint=True, action_frame="body")
    try:
        first, _ = env.reset(seed=501)
        first_raw = env.sim.get_observation_numpy(0)
        repeat, _ = env.reset(seed=501)
        repeat_raw = env.sim.get_observation_numpy(0)
        different, _ = env.reset(seed=502)
        different_raw = env.sim.get_observation_numpy(0)

        np.testing.assert_array_equal(first, repeat)
        np.testing.assert_array_equal(first_raw, repeat_raw)
        assert not np.array_equal(first_raw[6:10], different_raw[6:10])
        assert not np.array_equal(first_raw[75:77], different_raw[75:77])
        assert env.observation_space.contains(different)
    finally:
        env.close()


def test_body_action_matches_direct_cpp_transition():
    from aerofleet_cpp import Simulator, Vec3, VelocityCommand

    config = "config/simulation.json"
    start = [-120.0, -80.0, 15.0]
    target = [140.0, 210.0, 15.0]
    initial_yaw = -1.1
    action = np.asarray([0.35, -0.2, 0.1], dtype=np.float32)
    env = AeroFleetNavEnv(config_path=config, action_frame="body")
    direct = Simulator(config)
    try:
        env.reset(seed=71, options={
            "start": start, "target": target, "initial_yaw": initial_yaw,
        })
        direct.reset_navigation_task(71, Vec3(*start), Vec3(*target), initial_yaw)
        heading_sin, heading_cos = env.sim.get_observation_numpy(0)[75:77]
        forward, left, up = _normalized_velocity(
            action, env.sim.max_horizontal_speed, env.sim.max_vertical_speed
        )
        command = VelocityCommand(
            float(heading_cos * forward - heading_sin * left),
            float(heading_sin * forward + heading_cos * left),
            float(up),
        )
        direct_result = direct.step([command])
        _, reward, terminated, truncated, _ = env.step(action)

        assert reward == pytest.approx(direct_result.rewards[0], abs=1e-6)
        assert terminated is bool(direct_result.terminated[0])
        assert truncated is False
        np.testing.assert_allclose(
            env.sim.get_observation_numpy(0), direct_result.observations[0].to_numpy(), atol=1e-6
        )
    finally:
        env.close()


def test_progress_reward_bound_uses_actual_rounded_physics_duration(tmp_path):
    with open("config/simulation.json", encoding="utf-8") as config_file:
        config = json.load(config_file)
    config["simulation"]["dt_physics"] = 0.02
    config["simulation"]["dt_policy"] = 0.03  # C++ rounds 1.5 substeps to 2 (= 0.04s).
    config_path = tmp_path / "rounded_timestep.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    env = AeroFleetNavEnv(config_path=config_path, action_frame="world")
    try:
        env.reset(seed=813, options={"start": [0.0, 0.0, 15.0], "target": [300.0, 0.0, 90.0]})
        action = np.asarray([1.0, 0.0, 1.0], dtype=np.float32)
        for _ in range(300):
            observation, _, terminated, truncated, _ = env.step(action)
            assert not terminated and not truncated

        previous_distance = float(env.sim.get_observation(0).distance_to_target)
        observation, reward, terminated, truncated, _ = env.step(action)
        current_distance = float(env.sim.get_observation(0).distance_to_target)
        effective_dt = 0.04
        max_progress = (env.sim.max_horizontal_speed + env.sim.max_vertical_speed) * effective_dt
        expected = -0.01 + 0.1 * np.clip(
            previous_distance - current_distance, -max_progress, max_progress
        )

        assert env.sim.simulated_time == pytest.approx(12.04)
        assert reward == pytest.approx(expected, abs=1e-5)
        assert not terminated and not truncated
        assert env.observation_space.contains(observation)
    finally:
        env.close()


def test_full_fleet_compatibility_scenario_observations_remain_bounded():
    env = AeroFleetNavEnv(max_episode_steps=10)
    try:
        observation, info = env.reset(seed=99, options={"scenario": "full_fleet"})
        assert info["scenario"] == "full_fleet"
        assert env.sim.fleet_size == 10
        assert env.observation_space.contains(observation)
        for _ in range(10):
            observation, _, terminated, truncated, _ = env.step(np.zeros(3, dtype=np.float32))
            assert env.observation_space.contains(observation)
            if terminated or truncated:
                break
    finally:
        env.close()


def test_random_policy_completes_one_hundred_short_episodes():
    env = AeroFleetNavEnv(max_episode_steps=25)
    try:
        for episode in range(100):
            observation, _ = env.reset(seed=episode)
            assert env.observation_space.contains(observation)

            for _ in range(env.max_episode_steps):
                action = env.action_space.sample()
                observation, reward, terminated, truncated, _ = env.step(action)
                assert env.observation_space.contains(observation)
                assert np.isfinite(reward)
                if terminated or truncated:
                    break
            else:
                pytest.fail(f"episode {episode} exceeded its configured step limit")

            assert terminated or truncated
    finally:
        env.close()


def test_max_route_distance_validation_and_bounds():
    with pytest.raises(ValueError, match="max_route_distance must be positive"):
        AeroFleetNavEnv(max_route_distance=-5.0)

    with pytest.raises(ValueError, match="min_route_distance cannot exceed max_route_distance"):
        AeroFleetNavEnv(min_route_distance=50.0, max_route_distance=20.0)

    env = AeroFleetNavEnv(
        randomize_waypoint=True,
        min_route_distance=10.0,
        max_route_distance=30.0,
    )
    try:
        for s in range(50):
            env.reset(seed=100 + s)
            distance = float(env.sim.get_observation(0).distance_to_target)
            assert 9.9 <= distance <= 30.1, f"Seed {s} distance {distance} out of bounds"
    finally:
        env.close()

