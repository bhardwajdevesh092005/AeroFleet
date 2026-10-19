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
    from aerofleet_cpp import Simulator, Vec3, VelocityCommand
except ImportError as exc:  # pragma: no cover - depends on optional native build
    raise ImportError(
        "AeroFleetNavEnv requires the aerofleet_cpp extension. Build the "
        "aerofleet_cpp CMake target and make it importable from Python."
    ) from exc


OBSERVATION_SCHEMA_VERSION = 2
OBSERVATION_SIZE = 87
OBSERVATION_FEATURE_ORDER = (
    "velocity_forward", "velocity_left", "velocity_up",
    "acceleration_forward", "acceleration_left", "acceleration_up",
    "target_forward", "target_left", "target_up", "target_distance",
    "battery_fraction", "payload_fraction", "speed_fraction",
    *(f"lidar_horizontal_{i:02d}" for i in range(36)),
    *(f"lidar_elevation_{i:02d}" for i in range(8)),
    *(f"neighbor_{n}_{field}" for n in range(3) for field in (
        "forward", "left", "up", "velocity_forward", "velocity_left", "velocity_up"
    )),
    "heading_sin", "heading_cos",
    *(f"mode_{mode}" for mode in (
        "idle", "assigned", "flying_to_pickup", "picking_up",
        "flying_to_destination", "dropping_off", "returning_to_charge",
        "charging", "emergency", "offline"
    )),
)


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
    """Single-agent navigation environment using normalized velocity commands.

    Actions are ``[vx, vy, vz]`` in ``[-1, 1]``. The first two components
    scale by the configured maximum horizontal speed, and the last by maximum
    vertical speed. C++ kinematics remain authoritative and enforce limits.

    The default reset creates a deterministic, obstacle-free waypoint task.
    Pass ``options={"scenario": "full_fleet"}`` to run the stochastic delivery
    simulation. Observations use versioned, bounded body-frame features.
    Navigation episodes terminate on waypoint arrival, collision, or battery
    depletion and truncate at ``max_episode_steps``.
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
        self._scenario = "open_waypoint"

        low = np.full(OBSERVATION_SIZE, -1.0, dtype=np.float32)
        high = np.full(OBSERVATION_SIZE, 1.0, dtype=np.float32)
        for start, stop in ((9, 10), (10, 12), (12, 13), (13, 57), (77, 87)):
            low[start:stop] = 0.0
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),
            dtype=np.float32,
        )

        self._max_horizontal_speed = float(self.sim.max_horizontal_speed)
        self._max_vertical_speed = float(self.sim.max_vertical_speed)

    @property
    def observation_schema(self) -> dict[str, Any]:
        """Return the versioned feature order and config-derived scales."""
        return {
            "version": OBSERVATION_SCHEMA_VERSION,
            "feature_order": OBSERVATION_FEATURE_ORDER,
            "coordinate_frame": "UAV body frame (forward, left, up); heading is world yaw",
            "normalization": {
                "velocity_xy_mps": self._max_horizontal_speed,
                "velocity_z_mps": self._max_vertical_speed,
                "acceleration_mps2": float(self.sim.max_acceleration),
                "relative_position_m": tuple(
                    float(v) for v in (
                        self.sim.world_extent.x,
                        self.sim.world_extent.y,
                        self.sim.world_extent.z,
                    )
                ),
                "target_distance_m": float(np.linalg.norm([
                    self.sim.world_extent.x,
                    self.sim.world_extent.y,
                    self.sim.world_extent.z,
                ])),
                "payload_mass_kg": float(self.sim.max_payload_mass),
                "speed_mps": float(self.sim.max_observation_speed),
                "neighbor_relative_velocity_mps": (
                    2.0 * self._max_horizontal_speed,
                    2.0 * self._max_horizontal_speed,
                    2.0 * self._max_vertical_speed,
                ),
                "lidar": "already normalized to [0, 1] by C++",
                "battery": "fraction in [0, 1]",
                "mode": "10-value one-hot",
            },
        }

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        actual_seed = int(seed) if seed is not None else int(
            self.np_random.integers(1, np.iinfo(np.uint64).max, dtype=np.uint64)
        )
        # C++ reserves 0 as "reuse the current seed", while Gymnasium treats
        # seed=0 as an ordinary explicit seed. Map that one value to a stable
        # nonzero seed so repeated reset(seed=0) calls are reproducible.
        cpp_seed = actual_seed if actual_seed != 0 else np.iinfo(np.uint64).max
        options = options or {}
        self._scenario = str(options.get("scenario", "open_waypoint"))
        if self._scenario == "open_waypoint":
            start, target = self._waypoint_positions(options)
            self.sim.reset_navigation_task(cpp_seed, start, target)
        elif self._scenario == "full_fleet":
            self.sim.reset(cpp_seed)
        else:
            raise ValueError(
                f"unsupported scenario {self._scenario!r}; choose 'open_waypoint' or 'full_fleet'"
            )
        self._episode_steps = 0
        return self._observation(), {
            "seed": actual_seed,
            "uav_id": self._uav_id,
            "scenario": self._scenario,
            "observation_schema_version": OBSERVATION_SCHEMA_VERSION,
        }

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
            "scenario": self._scenario,
            "success": bool(self.sim.navigation_task_succeeded),
            "observation_schema_version": OBSERVATION_SCHEMA_VERSION,
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
        raw = np.asarray(
            self.sim.get_observation_numpy(self._uav_id), dtype=np.float32
        )
        if raw.shape != (OBSERVATION_SIZE,):
            raise RuntimeError(
                f"C++ observation has shape {raw.shape}; expected {(OBSERVATION_SIZE,)}"
            )
        if not np.all(np.isfinite(raw)):
            raise RuntimeError("C++ simulator returned a non-finite observation")
        observation = self._normalize_observation(raw)
        if not self.observation_space.contains(observation):
            raise RuntimeError("normalized observation falls outside the declared Box")
        return observation

    def _waypoint_positions(self, options: dict[str, Any]) -> tuple[Vec3, Vec3]:
        extent = self.sim.world_extent
        lower = self.sim.world_min
        start_default = [
            lower.x + 0.25 * extent.x,
            lower.y + 0.50 * extent.y,
            self.sim.default_spawn_altitude,
        ]
        target_default = [
            lower.x + 0.75 * extent.x,
            lower.y + 0.50 * extent.y,
            self.sim.default_spawn_altitude,
        ]
        start = np.asarray(options.get("start", start_default), dtype=np.float64)
        target = np.asarray(options.get("target", target_default), dtype=np.float64)
        if start.shape != (3,) or target.shape != (3,):
            raise ValueError("scenario start and target must each contain three coordinates")
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(target)):
            raise ValueError("scenario start and target coordinates must be finite")
        return Vec3(*map(float, start)), Vec3(*map(float, target))

    def _normalize_observation(self, raw: np.ndarray) -> np.ndarray:
        extent = self.sim.world_extent
        max_horizontal = self._max_horizontal_speed
        max_vertical = self._max_vertical_speed
        max_relative_speed = np.array(
            [2.0 * max_horizontal, 2.0 * max_horizontal, 2.0 * max_vertical],
            dtype=np.float64,
        )
        scales = np.ones(OBSERVATION_SIZE, dtype=np.float64)
        scales[0:3] = [max_horizontal, max_horizontal, max_vertical]
        scales[3:6] = self.sim.max_acceleration
        scales[6:9] = [extent.x, extent.y, extent.z]
        scales[9] = float(np.linalg.norm([extent.x, extent.y, extent.z]))
        scales[11] = max(self.sim.max_payload_mass, 1e-6)
        scales[12] = self.sim.max_observation_speed
        scales[57:75] = np.tile(
            [extent.x, extent.y, extent.z, *max_relative_speed], 3
        )
        observation = raw.astype(np.float64) / scales
        observation[10] = raw[10]  # battery fraction
        observation[13:57] = raw[13:57]  # normalized lidar distances
        observation[75:77] = raw[75:77]  # heading sine and cosine
        observation[77:87] = raw[77:87]  # operational/task-phase one-hot
        return observation.astype(np.float32)

