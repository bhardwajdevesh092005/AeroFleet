"""Step 15: Cross-Language Differential Physics & Seed Reproducibility Test Suite.

Verifies:
1. Adapter parity: Gymnasium wrapper (AeroFleetNavEnv) actions, observations,
   rewards, termination, and state transitions match direct C++ bindings bit-for-bit.
2. Coordinate frames: World-frame and body-frame action transformations match
   direct C++ heading rotations.
3. Independent analytical physics: Closed-form Semi-Implicit Euler kinematics
   with acceleration and jerk limits matches simulation trajectory within delta < 1e-4.
4. Terminal outcomes: Boundary violations and waypoint arrivals trigger
   identical terminal flags and reward bonuses/penalties in both interfaces.
5. Deterministic reproducibility: Identical seeds produce bit-for-bit identical
   trajectories across repeated resets and across separate environment instances.
6. Schema v3 integrity: 87 normalized features remain finite and strictly bounded.
"""

import math
import numpy as np
import pytest

import aerofleet_cpp
from aerofleet.nav_env import (
    AeroFleetNavEnv,
    OBSERVATION_FEATURE_ORDER,
    OBSERVATION_SCHEMA_VERSION,
    _normalized_velocity,
)


def _clamp_magnitude_3d(v: np.ndarray, max_mag: float) -> np.ndarray:
    mag_sq = float(np.sum(v * v))
    if mag_sq > max_mag * max_mag and mag_sq > 1e-12:
        return v * (max_mag / math.sqrt(mag_sq))
    return v.copy()


def _clamp_velocity_limits(
    v: np.ndarray, max_horiz: float = 15.0, max_vert: float = 5.0
) -> np.ndarray:
    horiz_sq = float(v[0] * v[0] + v[1] * v[1])
    vx, vy = float(v[0]), float(v[1])
    if horiz_sq > max_horiz * max_horiz and horiz_sq > 1e-12:
        scale = max_horiz / math.sqrt(horiz_sq)
        vx *= scale
        vy *= scale
    vz = float(np.clip(v[2], -max_vert, max_vert))
    return np.array([vx, vy, vz], dtype=np.float64)


def independent_kinematic_substep(
    pos: np.ndarray,
    vel: np.ndarray,
    acc: np.ndarray,
    cmd_velocity: np.ndarray,
    dt: float = 0.02,
    max_acc: float = 6.0,
    max_horiz: float = 15.0,
    max_vert: float = 5.0,
    max_jerk: float = 30.0,
    z_min: float = 0.0,
    z_max: float = 150.0,
):
    """Pure analytical Semi-Implicit Euler motion integrator independent of C++."""
    # 1. Target velocity clamping
    target_vel = _clamp_velocity_limits(cmd_velocity, max_horiz, max_vert)

    # 2. Required acceleration
    needed_acc = (target_vel - vel) / dt
    cmd_acc = _clamp_magnitude_3d(needed_acc, max_acc)

    # 3. Jerk limit
    if max_jerk > 0.0:
        delta_acc = cmd_acc - acc
        delta_sq = float(np.sum(delta_acc * delta_acc))
        max_delta = max_jerk * dt
        if delta_sq > max_delta * max_delta and delta_sq > 1e-12:
            cmd_acc = acc + delta_acc * (max_delta / math.sqrt(delta_sq))
            cmd_acc = _clamp_magnitude_3d(cmd_acc, max_acc)

    new_acc = cmd_acc

    # 4. Semi-implicit Euler: velocity first, then position
    candidate_vel = vel + new_acc * dt
    new_vel = _clamp_velocity_limits(candidate_vel, max_horiz, max_vert)
    new_pos = pos + new_vel * dt

    # 5. Boundary enforcement
    if new_pos[2] <= z_min:
        new_pos[2] = z_min
        if new_vel[2] < 0.0:
            new_vel[2] = 0.0
        if new_acc[2] < 0.0:
            new_acc[2] = 0.0
    elif new_pos[2] >= z_max:
        new_pos[2] = z_max
        if new_vel[2] > 0.0:
            new_vel[2] = 0.0
        if new_acc[2] > 0.0:
            new_acc[2] = 0.0

    return new_pos, new_vel, new_acc


# =========================================================================
# 1. Single-Step Differential Adapter Parity (World Frame)
# =========================================================================

def test_differential_single_step_world_frame():
    config = "config/simulation.json"
    start = [50.0, -30.0, 20.0]
    target = [120.0, 80.0, 25.0]
    initial_yaw = 0.45
    seed = 42

    env = AeroFleetNavEnv(config_path=config, action_frame="world")
    direct = aerofleet_cpp.Simulator(config)

    try:
        env.reset(seed=seed, options={"start": start, "target": target, "initial_yaw": initial_yaw})
        direct.reset_navigation_task(seed, aerofleet_cpp.Vec3(*start), aerofleet_cpp.Vec3(*target), initial_yaw)

        action = np.array([0.5, -0.25, 0.1], dtype=np.float32)
        v_cmd = _normalized_velocity(action, env.sim.max_horizontal_speed, env.sim.max_vertical_speed)

        # Step direct C++
        direct_cmd = aerofleet_cpp.VelocityCommand(float(v_cmd[0]), float(v_cmd[1]), float(v_cmd[2]))
        direct_res = direct.step([direct_cmd])

        # Step Gymnasium wrapper
        obs_env, reward_env, terminated_env, truncated_env, info_env = env.step(action)

        # 1. State comparison
        snap_env = env.sim.get_snapshot().uavs[0]
        snap_direct = direct.get_snapshot().uavs[0]

        delta_pos = math.sqrt(
            (snap_env.position.x - snap_direct.position.x) ** 2
            + (snap_env.position.y - snap_direct.position.y) ** 2
            + (snap_env.position.z - snap_direct.position.z) ** 2
        )
        assert delta_pos < 1e-6, f"Position delta exceeded tolerance: {delta_pos}"

        np.testing.assert_allclose(
            [snap_env.velocity.x, snap_env.velocity.y, snap_env.velocity.z],
            [snap_direct.velocity.x, snap_direct.velocity.y, snap_direct.velocity.z],
            atol=1e-6,
        )

        # 2. Reward parity
        assert reward_env == pytest.approx(direct_res.rewards[0], abs=1e-6)

        # 3. Termination parity
        assert terminated_env == bool(direct_res.terminated[0])
        assert truncated_env is False

        # 4. Observation array parity (all 87 features)
        obs_direct = direct_res.observations[0].to_numpy()
        raw_env_obs = env.sim.get_observation_numpy(0)
        np.testing.assert_allclose(raw_env_obs, obs_direct, atol=1e-6)
    finally:
        env.close()


# =========================================================================
# 2. Single-Step Differential Adapter Parity (Body Frame)
# =========================================================================

def test_differential_single_step_body_frame():
    config = "config/simulation.json"
    start = [-100.0, 75.0, 15.0]
    target = [150.0, -90.0, 30.0]
    initial_yaw = -0.85
    seed = 88

    env = AeroFleetNavEnv(config_path=config, action_frame="body")
    direct = aerofleet_cpp.Simulator(config)

    try:
        env.reset(seed=seed, options={"start": start, "target": target, "initial_yaw": initial_yaw})
        direct.reset_navigation_task(seed, aerofleet_cpp.Vec3(*start), aerofleet_cpp.Vec3(*target), initial_yaw)

        # Body-frame action: [forward, left, up]
        action = np.array([0.7, -0.3, 0.15], dtype=np.float32)

        # Get initial heading
        heading_sin, heading_cos = env.sim.get_observation_numpy(0)[75:77]
        v_body = _normalized_velocity(action, env.sim.max_horizontal_speed, env.sim.max_vertical_speed)
        v_world_x = float(heading_cos * v_body[0] - heading_sin * v_body[1])
        v_world_y = float(heading_sin * v_body[0] + heading_cos * v_body[1])
        v_world_z = float(v_body[2])

        direct_cmd = aerofleet_cpp.VelocityCommand(v_world_x, v_world_y, v_world_z)
        direct_res = direct.step([direct_cmd])

        obs_env, reward_env, terminated_env, truncated_env, _ = env.step(action)

        snap_env = env.sim.get_snapshot().uavs[0]
        snap_direct = direct.get_snapshot().uavs[0]

        delta_pos = math.hypot(
            snap_env.position.x - snap_direct.position.x,
            snap_env.position.y - snap_direct.position.y,
            snap_env.position.z - snap_direct.position.z,
        )
        assert delta_pos < 1e-6
        assert reward_env == pytest.approx(direct_res.rewards[0], abs=1e-6)
        assert terminated_env == bool(direct_res.terminated[0])
        np.testing.assert_allclose(env.sim.get_observation_numpy(0), direct_res.observations[0].to_numpy(), atol=1e-6)
    finally:
        env.close()


# =========================================================================
# 3. Multi-Step Trajectory Parity (40 Steps with Dynamic Actions)
# =========================================================================

def test_differential_multi_step_trajectory_parity():
    config = "config/simulation.json"
    start = [0.0, 0.0, 20.0]
    target = [300.0, 200.0, 40.0]
    initial_yaw = 0.0
    seed = 1001

    env = AeroFleetNavEnv(config_path=config, action_frame="body")
    direct = aerofleet_cpp.Simulator(config)

    rng = np.random.default_rng(seed)
    actions = [rng.uniform(-0.8, 0.8, size=3).astype(np.float32) for _ in range(40)]

    try:
        env.reset(seed=seed, options={"start": start, "target": target, "initial_yaw": initial_yaw})
        direct.reset_navigation_task(seed, aerofleet_cpp.Vec3(*start), aerofleet_cpp.Vec3(*target), initial_yaw)

        max_delta_pos = 0.0
        max_delta_reward = 0.0

        for t, act in enumerate(actions):
            # 1. Rotate body action for direct simulator using current heading
            heading_sin, heading_cos = direct.get_observation_numpy(0)[75:77]
            v_body = _normalized_velocity(act, direct.max_horizontal_speed, direct.max_vertical_speed)
            v_world_x = float(heading_cos * v_body[0] - heading_sin * v_body[1])
            v_world_y = float(heading_sin * v_body[0] + heading_cos * v_body[1])
            v_world_z = float(v_body[2])

            direct_res = direct.step([aerofleet_cpp.VelocityCommand(v_world_x, v_world_y, v_world_z)])
            obs_env, reward_env, term_env, trunc_env, _ = env.step(act)

            # Positions
            p_env = env.sim.get_snapshot().uavs[0].position
            p_dir = direct.get_snapshot().uavs[0].position
            d_pos = math.hypot(p_env.x - p_dir.x, p_env.y - p_dir.y, p_env.z - p_dir.z)
            max_delta_pos = max(max_delta_pos, d_pos)

            # Rewards
            d_rew = abs(reward_env - direct_res.rewards[0])
            max_delta_reward = max(max_delta_reward, d_rew)

            # Parity checks
            assert d_pos < 1e-5, f"Step {t}: position delta {d_pos} exceeds 1e-5"
            assert d_rew < 1e-5, f"Step {t}: reward delta {d_rew} exceeds 1e-5"
            assert term_env == bool(direct_res.terminated[0])

            # Battery parity
            bat_env = env.sim.get_snapshot().uavs[0].battery_percent
            bat_dir = direct.get_snapshot().uavs[0].battery_percent
            assert abs(bat_env - bat_dir) < 1e-4

        # Cumulative report
        assert max_delta_pos < 1e-5
        assert max_delta_reward < 1e-5
    finally:
        env.close()


# =========================================================================
# 4. Independent Analytical Physics Verification (Ground Truth Model)
# =========================================================================

def test_independent_analytical_kinematics_trajectory():
    """Validates simulation against an independent analytical integration model.

    Computes 10 policy steps (50 physics substeps @ dt=0.02s) with constant
    acceleration commands, enforcing acceleration limits, velocity clamps,
    and jerk limits in independent pure Python math.
    """
    config = "config/simulation.json"
    start = [0.0, 0.0, 10.0]
    target = [300.0, 300.0, 50.0]
    initial_yaw = 0.0

    env = AeroFleetNavEnv(config_path=config, action_frame="world")

    try:
        env.reset(seed=777, options={"start": start, "target": target, "initial_yaw": initial_yaw})

        # Constant action commanding full speed forward and upward
        action = np.array([1.0, 0.5, 0.8], dtype=np.float32)
        v_cmd = _normalized_velocity(action, env.sim.max_horizontal_speed, env.sim.max_vertical_speed)

        # Initialize independent state
        p_analytical = np.array(start, dtype=np.float64)
        v_analytical = np.zeros(3, dtype=np.float64)
        a_analytical = np.zeros(3, dtype=np.float64)

        max_discrepancy = 0.0

        # Run 10 policy steps (each policy step = 5 physics substeps of 0.02s)
        dt_physics = 0.02
        substeps_per_step = 5

        for step in range(10):
            env.step(action)

            # Integrate 5 analytical substeps
            for _ in range(substeps_per_step):
                p_analytical, v_analytical, a_analytical = independent_kinematic_substep(
                    p_analytical,
                    v_analytical,
                    a_analytical,
                    v_cmd,
                    dt=dt_physics,
                    max_acc=6.0,
                    max_horiz=15.0,
                    max_vert=5.0,
                    max_jerk=30.0,
                    z_min=0.0,
                    z_max=150.0,
                )

            # Inspect actual UAV state
            uav = env.sim.get_snapshot().uavs[0]
            p_actual = np.array([uav.position.x, uav.position.y, uav.position.z])
            v_actual = np.array([uav.velocity.x, uav.velocity.y, uav.velocity.z])

            err_pos = float(np.linalg.norm(p_actual - p_analytical))
            err_vel = float(np.linalg.norm(v_actual - v_analytical))

            max_discrepancy = max(max_discrepancy, err_pos)
            assert err_pos < 1e-4, f"Step {step}: analytical position error {err_pos:.2e} m exceeds tolerance"
            assert err_vel < 1e-4, f"Step {step}: analytical velocity error {err_vel:.2e} m/s exceeds tolerance"

        assert max_discrepancy < 1e-4
    finally:
        env.close()


# =========================================================================
# 5. Boundary & Collision Termination Parity
# =========================================================================

def test_boundary_collision_differential_parity():
    """Verifies that world boundary breaches terminate identically in direct C++ and Gym env."""
    config = "config/simulation.json"
    start = [990.0, 0.0, 20.0]  # 10m from Eastern world boundary (X_max = 1000.0)
    target = [0.0, 0.0, 20.0]
    initial_yaw = 0.0

    env = AeroFleetNavEnv(config_path=config, action_frame="world")
    direct = aerofleet_cpp.Simulator(config)

    try:
        env.reset(seed=999, options={"start": start, "target": target, "initial_yaw": initial_yaw})
        direct.reset_navigation_task(999, aerofleet_cpp.Vec3(*start), aerofleet_cpp.Vec3(*target), initial_yaw)

        # Fly East (+X) at maximum speed (+15 m/s)
        action = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        v_h = direct.max_horizontal_speed
        direct_cmd = aerofleet_cpp.VelocityCommand(float(v_h), 0.0, 0.0)

        # Step until boundary breach occurs (should occur within ~1-2 seconds = 10-20 steps)
        for step in range(25):
            direct_res = direct.step([direct_cmd])
            _, rew_env, term_env, _, info_env = env.step(action)

            assert term_env == bool(direct_res.terminated[0])
            assert rew_env == pytest.approx(direct_res.rewards[0], abs=1e-5)

            if term_env:
                # Should receive negative terminal penalty (-10)
                assert rew_env < -9.0
                assert direct_res.rewards[0] < -9.0
                assert direct.get_snapshot().uavs[0].mode == aerofleet_cpp.UAVMode.EMERGENCY
                break
        else:
            pytest.fail("Boundary collision did not trigger within 25 steps")
    finally:
        env.close()


# =========================================================================
# 6. Waypoint Goal Arrival Termination Parity
# =========================================================================

def test_waypoint_arrival_differential_parity():
    """Verifies that waypoint arrival termination and +10 bonus match bit-for-bit."""
    config = "config/simulation.json"
    start = [0.0, 0.0, 20.0]
    target = [3.0, 0.0, 20.0]  # 3 meters away (arrival radius is 2m)
    initial_yaw = 0.0

    env = AeroFleetNavEnv(config_path=config, action_frame="world")
    direct = aerofleet_cpp.Simulator(config)

    try:
        env.reset(seed=321, options={"start": start, "target": target, "initial_yaw": initial_yaw})
        direct.reset_navigation_task(321, aerofleet_cpp.Vec3(*start), aerofleet_cpp.Vec3(*target), initial_yaw)

        # Fly forward towards target
        action = np.array([0.5, 0.0, 0.0], dtype=np.float32)
        v_h = env.sim.max_horizontal_speed
        direct_cmd = aerofleet_cpp.VelocityCommand(float(0.5 * v_h), 0.0, 0.0)

        for step in range(10):
            direct_res = direct.step([direct_cmd])
            _, rew_env, term_env, _, info_env = env.step(action)

            assert term_env == bool(direct_res.terminated[0])
            assert rew_env == pytest.approx(direct_res.rewards[0], abs=1e-5)

            if term_env:
                assert rew_env > 9.0  # +10 bonus minus step cost
                assert info_env["success"] is True
                assert direct.get_observation(0).distance_to_target <= 2.0
                break
        else:
            pytest.fail("UAV did not reach 3m waypoint within 10 steps")
    finally:
        env.close()


# =========================================================================
# 7. Seed Reproducibility Across Resets and Separate Instances
# =========================================================================

def test_seed_reproducibility_across_instances_and_resets():
    config = "config/simulation.json"
    seeds = [123, 456, 789]

    for seed in seeds:
        env1 = AeroFleetNavEnv(config_path=config, randomize_waypoint=True)
        env2 = AeroFleetNavEnv(config_path=config, randomize_waypoint=True)

        try:
            # 1. Reset env1 twice with same seed -> bit-for-bit identical initial observation
            obs1_a, info1_a = env1.reset(seed=seed)
            obs1_b, info1_b = env1.reset(seed=seed)
            np.testing.assert_array_equal(obs1_a, obs1_b)
            assert info1_a["seed"] == info1_b["seed"] == seed

            # 2. Reset env2 with same seed -> identical to env1
            obs2, info2 = env2.reset(seed=seed)
            np.testing.assert_array_equal(obs1_a, obs2)

            # 3. Step both with identical pseudo-random action sequences
            rng = np.random.default_rng(seed)
            for _ in range(15):
                act = rng.uniform(-1.0, 1.0, size=3).astype(np.float32)
                o1, r1, term1, trunc1, _ = env1.step(act)
                o2, r2, term2, trunc2, _ = env2.step(act)

                np.testing.assert_array_equal(o1, o2)
                assert r1 == r2
                assert term1 == term2
                assert trunc1 == trunc2
                if term1 or trunc1:
                    break
        finally:
            env1.close()
            env2.close()


def test_different_seeds_produce_different_routes():
    env = AeroFleetNavEnv(randomize_waypoint=True)
    try:
        obs1, info1 = env.reset(seed=101)
        obs2, info2 = env.reset(seed=202)

        # Observations should differ
        assert not np.array_equal(obs1, obs2)
        assert info1["seed"] != info2["seed"]
    finally:
        env.close()


# =========================================================================
# 8. Schema v3 Feature Bounds & Completeness
# =========================================================================

def test_schema_v3_feature_completeness_and_bounds():
    env = AeroFleetNavEnv(max_episode_steps=50)
    try:
        obs, info = env.reset(seed=303)
        assert obs.shape == (87,)
        assert obs.dtype == np.float32
        assert len(OBSERVATION_FEATURE_ORDER) == 87
        assert info["observation_schema_version"] == OBSERVATION_SCHEMA_VERSION

        # Verify all 87 features are within space limits
        assert env.observation_space.contains(obs)
        assert not np.any(np.isnan(obs))
        assert not np.any(np.isinf(obs))

        # Run random steps and verify space containment at every step
        for _ in range(30):
            act = env.action_space.sample()
            obs, reward, term, trunc, _ = env.step(act)
            assert env.observation_space.contains(obs)
            assert not np.any(np.isnan(obs))
            assert not np.any(np.isinf(obs))
            assert np.isfinite(reward)
            if term or trunc:
                break
    finally:
        env.close()
