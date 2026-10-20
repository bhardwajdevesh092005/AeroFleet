"""Unit and integration tests for AeroFleetCoordinatorEnv."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Ensure python path has python/ directory
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT / "python") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "python"))

try:
    import aerofleet_cpp
    from aerofleet import AeroFleetCoordinatorEnv
    from aerofleet.coordinator_env import ACTION_SPACE_SIZE, FLEET_SIZE, MAX_OBSERVED_REQUESTS, OBSERVATION_DIM
    from sb3_contrib.common.wrappers import ActionMasker
    HAS_DEPS = True
except ImportError:
    HAS_DEPS = False

pytestmark = pytest.mark.skipif(not HAS_DEPS, reason="aerofleet_cpp or sb3_contrib not available")


def test_coordinator_env_initialization():
    env = AeroFleetCoordinatorEnv(max_steps=50)
    assert env.action_space.n == ACTION_SPACE_SIZE
    assert env.observation_space.shape == (OBSERVATION_DIM,)
    assert env.observation_space.dtype == np.float32

    obs, info = env.reset(seed=123)
    assert obs.shape == (OBSERVATION_DIM,)
    assert np.all(np.isfinite(obs))
    assert np.all(obs >= -1.0) and np.all(obs <= 1.0)
    assert info["step"] == 0
    assert info["sim_time"] == 0.0


def test_action_masking_contract():
    env = AeroFleetCoordinatorEnv(max_steps=50)
    env.reset(seed=42)

    mask = env.action_masks()
    assert mask.shape == (ACTION_SPACE_SIZE,)
    assert mask.dtype == bool
    # Contract: WAIT (action 0) must ALWAYS be valid
    assert mask[0] is True or mask[0] == 1


def test_step_wait_action():
    env = AeroFleetCoordinatorEnv(decision_interval=1.0, max_steps=10)
    env.reset(seed=42)

    initial_time = env.simulator.simulated_time
    obs, reward, terminated, truncated, info = env.step(0)  # WAIT

    assert obs.shape == (OBSERVATION_DIM,)
    assert np.all(np.isfinite(obs))
    assert env.simulator.simulated_time == pytest.approx(initial_time + 1.0, abs=1e-3)
    assert info["step"] == 1
    assert not terminated
    assert not truncated


def test_dispatch_lifecycle_and_mask_update():
    env = AeroFleetCoordinatorEnv(decision_interval=1.0, max_steps=30)
    env.reset(seed=100)

    # Step until some requests arrive
    for _ in range(5):
        env.step(0)

    pending = env.simulator.get_pending_requests()
    if len(pending) > 0:
        mask = env.action_masks()
        valid_indices = np.where(mask)[0]
        # Must have at least WAIT valid
        assert 0 in valid_indices

        # If there are valid pairing actions, pick the first one
        dispatch_actions = [a for a in valid_indices if a > 0]
        if dispatch_actions:
            chosen = dispatch_actions[0]
            obs, reward, terminated, truncated, info = env.step(chosen)
            assert np.all(np.isfinite(obs))
            assert info["assignment_success"] is True


def test_action_masker_wrapper_integration():
    env = AeroFleetCoordinatorEnv(decision_interval=1.0, max_steps=20)
    masked_env = ActionMasker(env, lambda e: e.action_masks())
    obs, info = masked_env.reset(seed=777)

    assert obs.shape == (OBSERVATION_DIM,)
    masks = masked_env.action_masks()
    assert len(masks) == ACTION_SPACE_SIZE
    assert masks[0] is True or masks[0] == 1

    # Execute a step through the wrapper
    next_obs, reward, terminated, truncated, info = masked_env.step(0)
    assert next_obs.shape == (OBSERVATION_DIM,)
    assert not terminated
