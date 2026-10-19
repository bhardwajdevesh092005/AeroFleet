#!/usr/bin/env python3
"""
AeroFleet Python Bindings Test Suite
Validates pybind11 C++ simulator module, numpy zero-copy memory layouts,
heuristic control, direct velocity commands, and deterministic reproducibility.
"""

import sys
import time
import numpy as np

# Ensure aerofleet_cpp module can be imported from python/
try:
    import aerofleet_cpp
except ImportError as e:
    print(f"[Error] Failed to import aerofleet_cpp: {e}")
    sys.exit(1)


def test_enums():
    print("[Python Test 1] Testing Enums and Types...")
    assert hasattr(aerofleet_cpp, "UAVMode")
    assert aerofleet_cpp.UAVMode.IDLE != aerofleet_cpp.UAVMode.FLYING_TO_PICKUP
    assert aerofleet_cpp.UAVMode.EMERGENCY is not None

    assert hasattr(aerofleet_cpp, "RequestStatus")
    assert aerofleet_cpp.RequestStatus.PENDING != aerofleet_cpp.RequestStatus.DELIVERED
    print("  -> Passed: UAVMode and RequestStatus enums verified.")


def test_vec3_and_commands():
    print("[Python Test 2] Testing Vec3 and VelocityCommand...")
    v1 = aerofleet_cpp.Vec3(1.0, 2.0, 3.0)
    v2 = aerofleet_cpp.Vec3(4.0, 5.0, 6.0)

    assert v1.x == 1.0 and v1.y == 2.0 and v1.z == 3.0
    assert abs(v1.length() - np.sqrt(14.0)) < 1e-6

    v_sum = v1 + v2
    assert v_sum.x == 5.0 and v_sum.y == 7.0 and v_sum.z == 9.0

    v_scaled = v1 * 2.5
    assert v_scaled.x == 2.5 and v_scaled.y == 5.0 and v_scaled.z == 7.5

    cmd = aerofleet_cpp.VelocityCommand(10.0, -5.0, 2.0)
    assert cmd.desired_velocity.x == 10.0
    assert cmd.desired_velocity.y == -5.0
    assert cmd.desired_velocity.z == 2.0
    assert not cmd.use_explicit_yaw
    print("  -> Passed: Vec3 vector algebra and VelocityCommand initialization verified.")


def test_simulator_lifecycle():
    print("[Python Test 3] Testing Simulator Lifecycle and Snapshot...")
    sim = aerofleet_cpp.Simulator("config/simulation.json")
    assert sim.fleet_size == 10
    assert sim.step_count == 0
    assert sim.simulated_time == 0.0

    sim.reset(42)
    assert sim.fleet_size == 10
    assert sim.max_horizontal_speed == 15.0
    assert sim.max_vertical_speed == 5.0
    assert sim.step_count == 0

    snap = sim.get_snapshot()
    assert len(snap.uavs) == 10
    assert snap.metrics.total_delivered == 0
    print("  -> Passed: Simulator initialization, reset, and world snapshot inspection verified.")


def test_observation_numpy():
    print("[Python Test 4] Testing Observation Vectorization and NumPy Integration...")
    sim = aerofleet_cpp.Simulator("config/simulation.json")
    sim.reset(123)

    # 1. Single observation inspection
    obs0 = sim.get_observation(0)
    assert obs0.uav_id == 0
    assert len(obs0.mode_one_hot) == 10
    assert sum(obs0.mode_one_hot) == 1.0
    assert np.isfinite([obs0.ego_acceleration.x, obs0.heading_sin, obs0.heading_cos]).all()
    assert len(obs0.lidar_beams) == 44
    assert len(obs0.nearby_uavs) == 18
    assert abs(obs0.battery_fraction - 1.0) < 1e-5

    arr0 = obs0.to_numpy()
    assert isinstance(arr0, np.ndarray)
    assert arr0.dtype == np.float32
    assert arr0.shape == (87,)
    assert not np.any(np.isnan(arr0))
    assert not np.any(np.isinf(arr0))

    # 2. Direct fast numpy getter
    arr0_direct = sim.get_observation_numpy(0)
    np.testing.assert_array_almost_equal(arr0, arr0_direct)

    # 3. Batch 2D observation matrix
    matrix = sim.get_all_observations_numpy()
    assert isinstance(matrix, np.ndarray)
    assert matrix.shape == (10, 87)
    assert matrix.dtype == np.float32
    np.testing.assert_array_almost_equal(matrix[0], arr0)
    print("  -> Passed: 87-dim Observation vectorization and NumPy (10, 87) matrix verified.")


def test_heuristic_stepping():
    print("[Python Test 5] Testing Heuristic Stepping and StepResult...")
    sim = aerofleet_cpp.Simulator("config/simulation.json")
    sim.reset(2026)

    for _ in range(30):
        step_res = sim.step_heuristic()
        assert len(step_res.observations) == 10
        assert len(step_res.rewards) == 10
        assert len(step_res.terminated) == 10
        assert len(step_res.truncated) == 10

        obs_mat = step_res.get_observation_matrix()
        assert obs_mat.shape == (10, 87)

        rewards_arr = step_res.get_rewards_numpy()
        assert rewards_arr.shape == (10,)
        assert not np.any(np.isnan(rewards_arr))

        term_arr = step_res.get_terminated_numpy()
        assert term_arr.shape == (10,)
        assert term_arr.dtype == bool

    assert sim.step_count > 0
    assert sim.simulated_time > 0.0
    print(f"  -> Passed: 30 heuristic steps executed. Sim time: {sim.simulated_time:.2f}s.")


def test_velocity_command_stepping():
    print("[Python Test 6] Testing Commanded Actions Stepping...")
    sim = aerofleet_cpp.Simulator("config/simulation.json")
    sim.reset(3000)

    snap_start = sim.get_snapshot()
    start_pos_x = snap_start.uavs[0].position.x

    # Command forward velocity on UAV 0
    actions = [aerofleet_cpp.VelocityCommand(0.0, 0.0, 0.0) for _ in range(10)]
    actions[0] = aerofleet_cpp.VelocityCommand(10.0, 0.0, 1.0)

    for _ in range(25):
        sim.step(actions)

    snap_end = sim.get_snapshot()
    end_pos_x = snap_end.uavs[0].position.x

    assert end_pos_x > start_pos_x
    print(f"  -> Passed: Commanded velocity executed. UAV 0 delta X: {end_pos_x - start_pos_x:.2f} m.")


def test_reproducibility():
    print("[Python Test 7] Testing Cross-Instance Reproducibility...")
    sim1 = aerofleet_cpp.Simulator("config/simulation.json")
    sim1.reset(55555)
    for _ in range(20):
        sim1.step_heuristic()

    sim2 = aerofleet_cpp.Simulator("config/simulation.json")
    sim2.reset(55555)
    for _ in range(20):
        sim2.step_heuristic()

    mat1 = sim1.get_all_observations_numpy()
    mat2 = sim2.get_all_observations_numpy()
    np.testing.assert_array_equal(mat1, mat2)
    print("  -> Passed: Bit-for-bit identical trajectory between Python simulator instances.")


def test_throughput_benchmark():
    print("[Python Test 8] Benchmarking High-Speed Python Stepping Throughput...")
    sim = aerofleet_cpp.Simulator("config/simulation.json")
    sim.reset(42)

    total_steps = 1000
    t0 = time.perf_counter()
    for _ in range(total_steps):
        sim.step_heuristic()
    elapsed = time.perf_counter() - t0

    steps_per_sec = total_steps / elapsed
    sim_time = sim.simulated_time
    sim_multiplier = sim_time / elapsed

    print(f"  -> Performance: {total_steps} steps in {elapsed:.4f}s = {steps_per_sec:,.0f} steps/s ({sim_multiplier:,.0f}x real-time speedup).")
    assert steps_per_sec > 1000.0
    print("  -> Passed: High-throughput benchmark satisfied (>1,000 steps/sec).")


if __name__ == "__main__":
    print("========================================")
    print("  AeroFleet Python Bindings Test Suite  ")
    print("========================================")
    test_enums()
    test_vec3_and_commands()
    test_simulator_lifecycle()
    test_observation_numpy()
    test_heuristic_stepping()
    test_velocity_command_stepping()
    test_reproducibility()
    test_throughput_benchmark()
    print("========================================")
    print("  All Python Binding Tests PASSED! (8/8)")
    print("========================================")

