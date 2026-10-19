"""Unit and integration coverage for the Gymnasium navigation wrapper.

Run with: PYTHONPATH=python python -m pytest tests/test_nav_env.py
Requires requirements-rl.txt and an importable aerofleet_cpp extension.
"""

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
        assert info_first["observation_schema_version"] == 2
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

        for _ in range(env.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.array([1.0, 0.0, 0.0], dtype=np.float32)
            )
            assert env.observation_space.contains(observation)
            if terminated or truncated:
                break

        assert terminated
        assert not truncated
        assert reward > 9.0
        assert info["success"]
        assert env.sim.get_observation(0).distance_to_target <= 2.0
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
