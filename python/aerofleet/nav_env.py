"""Gymnasium navigation environment backed by the authoritative C++ simulator.

The learning agent controls UAV 0. Other UAVs continue to use the simulator's
heuristic controller when they are assigned to a mission.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

try:
    from aerofleet_cpp import Simulator, VelocityCommand
except ImportError as exc:  # pragma: no cover - depends on optional native build
    raise ImportError(
        "AeroFleetNavEnv requires the aerofleet_cpp extension. Build the "
        "aerofleet_cpp CMake target and make it importable from Python."
    ) from exc


def _normalized_velocity(action: np.ndarray, max_horizontal: float, max_vertical: float) -> np.ndarray:
    """Validate and scale a normalized action to physical desired velocity."""
    action_array = np.asarray(action, dtype=np.float32)
    if action_array.shape != (3,):
        raise ValueError(f"action must have shape (3,), got {action_array.shape}")
    if not np.all(np.isfinite(action_array)):
        raise ValueError("action values must be finite")
    if np.any(action_array < -1.0) or np.any(action_array > 1.0):
        raise ValueError("action values must be in [-1, 1]")
    return np.asarray(
        [
            action_array[0] * max_horizontal,
            action_array[1] * max_horizontal,
            action_array[2] * max_vertical,
        ],
        dtype=np.float64,
    )


class AeroFleetNavEnv(gym.Env):
    """Single-agent navigation task using normalized velocity commands.

    Actions are ``[vx, vy, vz]`` in ``[-1, 1]``. The first two components
    scale by the configured maximum horizontal speed, and the last by maximum
    vertical speed. C++ kinematics remain authoritative and enforce limits.

    The fixed 72-element observation is supplied by the C++ simulator. An
    episode ends when the controlled UAV enters emergency mode and is
    truncated at ``max_episode_steps``.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        config_path: str | Path = "config/simulation.json",
        max_episode_steps: int = 2_000,
    ) -> None:
        super().__init__()
        if max_episode_steps <= 0:
            raise ValueError("max_episode_steps must be positive")

        self.config_path = str(config_path)
        self.max_episode_steps = int(max_episode_steps)
        self.sim = Simulator(self.config_path)
        self._uav_id = 0
        self._episode_steps = 0

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(72,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),
            dtype=np.float32,
        )

        self._max_horizontal_speed = float(self.sim.max_horizontal_speed)
        self._max_vertical_speed = float(self.sim.max_vertical_speed)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options  # Reserved for future scenario selection.
        actual_seed = int(seed) if seed is not None else int(
            self.np_random.integers(1, np.iinfo(np.uint64).max, dtype=np.uint64)
        )
        # C++ reserves 0 as "reuse the current seed", while Gymnasium treats
        # seed=0 as an ordinary explicit seed. Map that one value to a stable
        # nonzero seed so repeated reset(seed=0) calls are reproducible.
        self.sim.reset(actual_seed if actual_seed != 0 else np.iinfo(np.uint64).max)
        self._episode_steps = 0
        return self._observation(), {"seed": actual_seed, "uav_id": self._uav_id}

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        desired_velocity = _normalized_velocity(
            action, self._max_horizontal_speed, self._max_vertical_speed
        )
        command = VelocityCommand(
            float(desired_velocity[0]),
            float(desired_velocity[1]),
            float(desired_velocity[2]),
        )
        result = self.sim.step([command])
        self._episode_steps += 1

        terminated = bool(result.terminated[self._uav_id])
        truncated = self._episode_steps >= self.max_episode_steps and not terminated
        metrics = result.metrics
        info = {
            "uav_id": self._uav_id,
            "simulated_time": float(self.sim.simulated_time),
            "episode_steps": self._episode_steps,
            "metrics": {
                "total_created": int(metrics.total_created),
                "total_delivered": int(metrics.total_delivered),
                "total_expired": int(metrics.total_expired),
                "total_active": int(metrics.total_active),
                "total_pending": int(metrics.total_pending),
                "collision_count": int(metrics.collision_count),
                "delivery_success_rate": float(metrics.delivery_success_rate),
                "average_delivery_duration": float(metrics.average_delivery_duration),
                "real_fps": float(metrics.real_fps),
            },
        }
        return (
            self._observation(),
            float(result.rewards[self._uav_id]),
            terminated,
            truncated,
            info,
        )

    def _observation(self) -> np.ndarray:
        observation = np.asarray(
            self.sim.get_observation_numpy(self._uav_id), dtype=np.float32
        )
        if observation.shape != self.observation_space.shape:
            raise RuntimeError(
                f"C++ observation has shape {observation.shape}; expected "
                f"{self.observation_space.shape}"
            )
        if not np.all(np.isfinite(observation)):
            raise RuntimeError("C++ simulator returned a non-finite observation")
        return observation

