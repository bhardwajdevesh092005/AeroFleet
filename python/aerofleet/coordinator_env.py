"""Gymnasium coordinator environment for central multi-UAV delivery fleet dispatch.

The central coordinator observes fleet availability and pending delivery requests,
and executes discrete dispatch decisions (matching request i to UAV j, or WAIT)
every decision interval (1.0 second). Subordinate UAVs navigate using the
authoritative embedded C++ ONNX navigation policy (or baseline heuristic navigation).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

try:
    import aerofleet_cpp
    from aerofleet_cpp import DeliveryRequest, RequestStatus, Simulator, UAVMode, UAVState, Vec3
except ImportError as exc:
    raise ImportError(
        "AeroFleetCoordinatorEnv requires the aerofleet_cpp extension. "
        "Build the aerofleet_cpp CMake target before running."
    ) from exc


MAX_OBSERVED_REQUESTS = 10
FLEET_SIZE = 10
OBSERVATION_DIM = FLEET_SIZE * 8 + MAX_OBSERVED_REQUESTS * 8 + 4  # 80 + 80 + 4 = 164
ACTION_SPACE_SIZE = MAX_OBSERVED_REQUESTS * FLEET_SIZE + 1       # 101 (0 is WAIT)


class AeroFleetCoordinatorEnv(gym.Env):
    """Central fleet coordinator dispatch environment with action masking."""

    metadata = {"render_modes": ["human"], "render_fps": 1}

    def __init__(
        self,
        config_path: str = "config/simulation.json",
        use_onnx_nav: bool = False,
        decision_interval: float = 1.0,
        max_steps: int = 300,
        model_path: str = "models/navigation/nav_policy.onnx",
        norm_path: str = "models/navigation/normalization.json",
        w_delivered: float = 10.0,
        w_expired: float = 10.0,
        w_wait: float = 0.001,
        w_queue: float = 0.05,
        w_assign_bonus: float = 0.5,
        min_battery_dispatch_percent: float = 20.0,
    ) -> None:
        super().__init__()

        self._config_path = config_path
        self._use_onnx_nav = use_onnx_nav
        self._decision_interval = decision_interval
        self._max_steps = max_steps
        self._model_path = model_path
        self._norm_path = norm_path

        self._w_delivered = w_delivered
        self._w_expired = w_expired
        self._w_wait = w_wait
        self._w_queue = w_queue
        self._w_assign_bonus = w_assign_bonus
        self._min_battery_dispatch_percent = min_battery_dispatch_percent

        self._sim = Simulator(self._config_path)
        self._sim.set_auto_coordinator_dispatch(False)
        if self._use_onnx_nav:
            self._sim.enable_onnx_navigation(self._model_path, self._norm_path)

        self._world_min = self._sim.world_min
        self._world_extent = self._sim.world_extent
        self._half_x = max(abs(self._world_min.x), abs(self._world_min.x + self._world_extent.x))
        self._half_y = max(abs(self._world_min.y), abs(self._world_min.y + self._world_extent.y))
        self._max_z = max(self._world_min.z + self._world_extent.z, 150.0)
        self._max_h_speed = self._sim.max_horizontal_speed
        self._max_v_speed = self._sim.max_vertical_speed
        self._max_payload_kg = self._sim.max_payload_mass

        self.action_space = spaces.Discrete(ACTION_SPACE_SIZE)
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(OBSERVATION_DIM,),
            dtype=np.float32,
        )

        self._step_count = 0

    @property
    def simulator(self) -> Simulator:
        return self._sim

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        seed_value = 42 if seed is None else int(seed)

        self._sim.reset(seed_value)
        self._sim.set_auto_coordinator_dispatch(False)
        if self._use_onnx_nav:
            self._sim.enable_onnx_navigation(self._model_path, self._norm_path)

        self._step_count = 0
        obs = self._build_observation()
        info = {
            "step": 0,
            "sim_time": self._sim.simulated_time,
            "pending_count": len(self._sim.get_pending_requests()),
            "total_delivered": 0,
            "total_expired": 0,
        }
        return obs, info

    def action_masks(self) -> np.ndarray:
        """Returns boolean mask of valid actions.

        Index 0 (WAIT) is always True.
        Action 1 + i * FLEET_SIZE + j corresponds to matching request i with UAV j.
        """
        mask = np.zeros(ACTION_SPACE_SIZE, dtype=bool)
        mask[0] = True  # WAIT is unconditionally valid

        pending = self._sim.get_pending_requests()
        snapshot = self._sim.get_snapshot()
        uavs = snapshot.uavs
        sim_time = self._sim.simulated_time

        num_reqs = min(len(pending), MAX_OBSERVED_REQUESTS)
        num_uavs = min(len(uavs), FLEET_SIZE)

        for req_idx in range(num_reqs):
            req = pending[req_idx]
            if req.status != RequestStatus.PENDING or sim_time >= req.deadline:
                continue
            if req.payload_mass > self._max_payload_kg:
                continue

            for uav_idx in range(num_uavs):
                uav = uavs[uav_idx]
                if uav.mode == UAVMode.IDLE and uav.battery_percent > self._min_battery_dispatch_percent:
                    if hasattr(self._sim, "is_energy_feasible") and not self._sim.is_energy_feasible(uav_idx, req.id):
                        continue
                    action_idx = 1 + req_idx * FLEET_SIZE + uav_idx
                    mask[action_idx] = True

        return mask

    def _build_observation(self) -> np.ndarray:
        obs = np.zeros(OBSERVATION_DIM, dtype=np.float32)
        snapshot = self._sim.get_snapshot()
        uavs = snapshot.uavs
        pending = self._sim.get_pending_requests()
        sim_time = self._sim.simulated_time

        offset = 0
        # 1. Fleet features (FLEET_SIZE x 8 features)
        for j in range(FLEET_SIZE):
            if j < len(uavs):
                u = uavs[j]
                obs[offset + 0] = np.clip(u.position.x / self._half_x, -1.0, 1.0)
                obs[offset + 1] = np.clip(u.position.y / self._half_y, -1.0, 1.0)
                obs[offset + 2] = np.clip(u.position.z / self._max_z, 0.0, 1.0)
                obs[offset + 3] = np.clip(u.velocity.x / self._max_h_speed, -1.0, 1.0)
                obs[offset + 4] = np.clip(u.velocity.y / self._max_h_speed, -1.0, 1.0)
                obs[offset + 5] = np.clip(u.velocity.z / self._max_v_speed, -1.0, 1.0)
                obs[offset + 6] = np.clip(u.battery_percent / 100.0, 0.0, 1.0)
                obs[offset + 7] = (
                    1.0
                    if (u.mode == UAVMode.IDLE and u.battery_percent > self._min_battery_dispatch_percent)
                    else 0.0
                )
            offset += 8

        # 2. Pending Request features (MAX_OBSERVED_REQUESTS x 8 features)
        for i in range(MAX_OBSERVED_REQUESTS):
            if i < len(pending):
                r = pending[i]
                obs[offset + 0] = np.clip(r.pickup_location.x / self._half_x, -1.0, 1.0)
                obs[offset + 1] = np.clip(r.pickup_location.y / self._half_y, -1.0, 1.0)
                obs[offset + 2] = np.clip(r.pickup_location.z / self._max_z, 0.0, 1.0)
                obs[offset + 3] = np.clip(r.dropoff_location.x / self._half_x, -1.0, 1.0)
                obs[offset + 4] = np.clip(r.dropoff_location.y / self._half_y, -1.0, 1.0)
                obs[offset + 5] = np.clip(r.dropoff_location.z / self._max_z, 0.0, 1.0)
                obs[offset + 6] = np.clip(r.payload_mass / self._max_payload_kg, 0.0, 1.0)
                slack = (r.deadline - sim_time) / 300.0
                obs[offset + 7] = np.clip(slack, -1.0, 1.0)
            offset += 8

        # 3. Global context features (4 features)
        obs[offset + 0] = (sim_time % 600.0) / 600.0
        obs[offset + 1] = min(len(pending) / 20.0, 1.0)
        idle_count = sum(1 for u in uavs if u.mode == UAVMode.IDLE)
        obs[offset + 2] = idle_count / max(len(uavs), 1)
        charging_count = sum(1 for u in uavs if u.mode == UAVMode.CHARGING)
        obs[offset + 3] = charging_count / max(len(uavs), 1)

        return obs

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self._step_count += 1
        reward = 0.0
        assignment_success = False

        if action > 0:
            pair_idx = action - 1
            req_idx = pair_idx // FLEET_SIZE
            uav_idx = pair_idx % FLEET_SIZE

            pending = self._sim.get_pending_requests()
            if req_idx < len(pending):
                target_req = pending[req_idx]
                if self._sim.dispatch_request(uav_idx, target_req.id):
                    assignment_success = True
                    snap_uavs = self._sim.get_snapshot().uavs
                    if uav_idx < len(snap_uavs):
                        dist = snap_uavs[uav_idx].position.distance(target_req.pickup_location)
                        proximity_bonus = max(0.0, 1.0 - dist / 500.0)
                    else:
                        proximity_bonus = 0.0

                    slack = max(0.0, (target_req.deadline - self._sim.simulated_time) / 300.0)
                    urgency_bonus = max(0.0, 1.0 - slack)
                    soc = snap_uavs[uav_idx].battery_percent if uav_idx < len(snap_uavs) else 100.0
                    battery_comfort = max(0.0, (soc - 30.0) / 70.0)
                    reward += self._w_assign_bonus + proximity_bonus + 0.5 * urgency_bonus + 0.25 * battery_comfort
                else:
                    reward -= 1.0  # Invalid dispatch
            else:
                reward -= 1.0

        # Snapshot before physics step to compute deltas
        snap_before = self._sim.get_snapshot()
        delivered_before = snap_before.metrics.total_delivered
        expired_before = snap_before.metrics.total_expired

        # Advance simulation by decision interval
        self._sim.step_coordinator(self._decision_interval)

        # Snapshot after physics step
        snap_after = self._sim.get_snapshot()
        delta_delivered = snap_after.metrics.total_delivered - delivered_before
        delta_expired = snap_after.metrics.total_expired - expired_before

        pending_after = self._sim.get_pending_requests()
        total_wait_time = sum(self._sim.simulated_time - r.creation_time for r in pending_after)

        # Weighted reward composition
        reward += self._w_delivered * delta_delivered
        reward -= self._w_expired * delta_expired
        reward -= self._w_wait * total_wait_time
        reward -= self._w_queue * len(pending_after)

        terminated = False
        truncated = self._step_count >= self._max_steps

        info = {
            "step": self._step_count,
            "sim_time": self._sim.simulated_time,
            "assignment_success": assignment_success,
            "delta_delivered": delta_delivered,
            "delta_expired": delta_expired,
            "total_delivered": snap_after.metrics.total_delivered,
            "total_expired": snap_after.metrics.total_expired,
            "pending_count": len(pending_after),
            "success_rate": snap_after.metrics.delivery_success_rate,
            "mean_latency": snap_after.metrics.average_delivery_duration,
        }

        obs = self._build_observation()
        return obs, float(reward), terminated, truncated, info
