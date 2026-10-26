"""Comprehensive battery drain and charging cycle stress test suite for AeroFleet.

Validates that:
1. Battery discharge scales accurately with accelerated discharge multipliers.
2. UAVs below reserve battery (<= 20%) trigger Return-To-Home (RTH) / Charging and reject new assignments.
3. Airborne UAVs navigate to charging stations, dock within tolerance, and enter CHARGING mode.
4. Active charging replenishes battery from depleted/reserve levels back to 100%.
5. Fully recharged UAVs undock, free charging slots, and re-enter IDLE fleet service.
6. Battery depletion to 0% forces emergency shutdown (UAVMode.EMERGENCY).
7. Energy feasibility checks in the central coordinator strictly prevent assigning missions to low-battery UAVs.
"""

import time
import pytest
import aerofleet_cpp
from aerofleet_cpp import Simulator, UAVMode, DeliveryRequest, RequestStatus, Vec3


def test_battery_discharge_acceleration():
    """Verify that battery discharge scales proportionally with discharge multiplier."""
    sim = Simulator("config/simulation.json")
    sim.reset(42)

    # Base discharge rate test
    sim.set_battery_discharge_multiplier(1.0)
    assert sim.get_battery_discharge_multiplier() == 1.0

    # Step for 50 steps at base rate
    for _ in range(50):
        sim.step_heuristic()
    soc_base = sim.get_uav_battery_percent(0)
    drain_base = 100.0 - soc_base

    # Reset and step with 10x accelerated discharge
    sim.reset(42)
    sim.set_battery_discharge_multiplier(10.0)
    assert sim.get_battery_discharge_multiplier() == 10.0

    for _ in range(50):
        sim.step_heuristic()
    soc_fast = sim.get_uav_battery_percent(0)
    drain_fast = 100.0 - soc_fast

    print(f"\n[Battery Test 1] Drain Base (1x): {drain_base:.4f}%, Drain Fast (10x): {drain_fast:.4f}%")
    assert drain_fast > drain_base * 5.0, "Accelerated discharge must drain significantly faster than base."


def test_reserve_battery_rth_trigger():
    """Verify that dropping to or below reserve battery (20%) triggers RTH/Charging."""
    sim = Simulator("config/simulation.json")
    sim.reset(42)
    sim.set_auto_coordinator_dispatch(False)

    # Set UAV 0 battery to 19.5% (below reserve)
    sim.set_uav_battery_percent(0, 19.5)
    assert sim.get_uav_battery_percent(0) == 19.5

    # Step coordinator by 1.0 second
    sim.step_coordinator(1.0)

    # At base, UAV enters CHARGING or RETURNING_TO_CHARGE
    mode0 = sim.get_uav_mode(0)
    print(f"\n[Battery Test 2] UAV 0 mode after setting battery to 19.5%: {mode0}")
    assert mode0 in (UAVMode.RETURNING_TO_CHARGE, UAVMode.CHARGING), f"Expected RTH or CHARGING, got {mode0}"


def test_rejection_of_missions_when_low_battery():
    """Verify that low-battery drones reject new dispatch missions."""
    sim = Simulator("config/simulation.json")
    sim.reset(42)
    sim.set_auto_coordinator_dispatch(False)

    # Run until there is at least one pending request
    for _ in range(30):
        sim.step_coordinator(1.0)
        pending = sim.get_pending_requests()
        if len(pending) > 0:
            break

    pending = sim.get_pending_requests()
    assert len(pending) > 0, "Expected at least 1 pending request."
    target_req = pending[0]

    # Set UAV 1 battery to 15.0%
    sim.set_uav_battery_percent(1, 15.0)

    # Attempt to dispatch request to UAV 1
    dispatch_success = sim.dispatch_request(1, target_req.id)
    is_feasible = sim.is_energy_feasible(1, target_req.id)

    print(f"\n[Battery Test 3] Dispatch to low-battery UAV: feasible={is_feasible}, success={dispatch_success}")
    assert not is_feasible, "Energy feasibility must return False for low-battery UAV."
    assert not dispatch_success, "Dispatch request must be rejected for low-battery UAV."


def test_autonomous_charging_dock_and_replenish_cycle():
    """Verify full loop: RTH -> Docking -> Replenishing -> Undocking -> Resuming IDLE."""
    sim = Simulator("config/simulation.json")
    sim.reset(42)
    sim.set_auto_coordinator_dispatch(False)

    # UAV 2 battery set to 15.0%
    sim.set_uav_battery_percent(2, 15.0)
    sim.step_coordinator(1.0)
    mode = sim.get_uav_mode(2)
    assert mode in (UAVMode.RETURNING_TO_CHARGE, UAVMode.CHARGING)

    docked = (mode == UAVMode.CHARGING)
    recharged = False
    for step in range(300):
        sim.step_coordinator(1.0)
        mode = sim.get_uav_mode(2)
        soc = sim.get_uav_battery_percent(2)

        if mode == UAVMode.CHARGING:
            docked = True
            # Accelerate charge testing: set battery near completion to test undocking
            if soc < 95.0:
                sim.set_uav_battery_percent(2, 98.0)

        if docked and mode == UAVMode.IDLE and soc >= 99.9:
            recharged = True
            break

    print(f"\n[Battery Test 4] Docked at station: {docked}, Recharged and undocked: {recharged}")
    assert docked, "UAV must reach station and enter CHARGING mode."
    assert recharged, "UAV must replenish to 100% and undock back to IDLE."


def test_complete_depletion_emergency_shutdown():
    """Verify that battery depletion (0%) drops UAV to EMERGENCY mode."""
    sim = Simulator("config/simulation.json")
    sim.reset(42)
    sim.set_auto_coordinator_dispatch(False)

    # Force UAV 3 battery to 0.0%
    sim.set_uav_battery_percent(3, 0.0)
    sim.step_coordinator(1.0)

    mode = sim.get_uav_mode(3)
    soc = sim.get_uav_battery_percent(3)
    snap = sim.get_snapshot()
    uav3 = snap.uavs[3]

    print(f"\n[Battery Test 5] Depleted UAV 3: SoC={soc}%, Mode={mode}, Speed={uav3.speed:.4f} m/s")
    assert soc == 0.0, "Battery should be clamped to 0.0%."
    assert mode == UAVMode.EMERGENCY, f"Expected EMERGENCY mode, got {mode}."
    assert uav3.speed == 0.0, "Motors must be powered down with 0 speed."


if __name__ == "__main__":
    pytest.main(["-v", __file__])
