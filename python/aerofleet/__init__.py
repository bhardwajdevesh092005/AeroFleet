"""
AeroFleet - Autonomous Multi-UAV Urban Delivery Simulator
Python API & Gymnasium Environments
"""

try:
    import aerofleet_cpp
    from aerofleet_cpp import (
        Simulator,
        Observation,
        StepResult,
        WorldSnapshot,
        SnapshotMetrics,
        VelocityCommand,
        UAVState,
        DeliveryRequest,
        UAVMode,
        RequestStatus,
        Vec3,
    )
    __all__ = [
        "aerofleet_cpp",
        "Simulator",
        "Observation",
        "StepResult",
        "WorldSnapshot",
        "SnapshotMetrics",
        "VelocityCommand",
        "UAVState",
        "DeliveryRequest",
        "UAVMode",
        "RequestStatus",
        "Vec3",
    ]
except ImportError:
    pass

