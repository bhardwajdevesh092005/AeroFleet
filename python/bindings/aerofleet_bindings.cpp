#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <sstream>
#include <cstring>
#include <cmath>

#include "core/Simulator.hpp"
#include "core/Types.hpp"
#include "core/WorldState.hpp"
#include "fleet/UAVState.hpp"
#include "requests/DeliveryRequest.hpp"
#include "physics/Kinematics.hpp"
#include "math/Vec3.hpp"

namespace py = pybind11;
using namespace aerofleet;

PYBIND11_MODULE(aerofleet_cpp, m) {
    m.doc() = "AeroFleet C++ Simulation Engine Python Bindings";

    // -------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------
    py::enum_<UAVMode>(m, "UAVMode")
        .value("IDLE", UAVMode::IDLE)
        .value("ASSIGNED", UAVMode::ASSIGNED)
        .value("FLYING_TO_PICKUP", UAVMode::FLYING_TO_PICKUP)
        .value("PICKING_UP", UAVMode::PICKING_UP)
        .value("FLYING_TO_DESTINATION", UAVMode::FLYING_TO_DESTINATION)
        .value("DROPPING_OFF", UAVMode::DROPPING_OFF)
        .value("RETURNING_TO_CHARGE", UAVMode::RETURNING_TO_CHARGE)
        .value("CHARGING", UAVMode::CHARGING)
        .value("EMERGENCY", UAVMode::EMERGENCY)
        .value("OFFLINE", UAVMode::OFFLINE)
        .export_values();

    py::enum_<RequestStatus>(m, "RequestStatus")
        .value("PENDING", RequestStatus::PENDING)
        .value("ASSIGNED", RequestStatus::ASSIGNED)
        .value("PICKING_UP", RequestStatus::PICKING_UP)
        .value("IN_TRANSIT", RequestStatus::IN_TRANSIT)
        .value("DELIVERED", RequestStatus::DELIVERED)
        .value("EXPIRED", RequestStatus::EXPIRED)
        .value("CANCELLED", RequestStatus::CANCELLED)
        .value("REJECTED", RequestStatus::REJECTED)
        .export_values();

    // -------------------------------------------------------------
    // Math Primitives: Vec3
    // -------------------------------------------------------------
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<>())
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("length", &Vec3::length)
        .def("length_squared", &Vec3::lengthSquared)
        .def("distance", &Vec3::distance, py::arg("other"))
        .def("distance_squared", &Vec3::distanceSquared, py::arg("other"))
        .def("dot", &Vec3::dot, py::arg("other"))
        .def("cross", &Vec3::cross, py::arg("other"))
        .def("normalized", &Vec3::normalized)
        .def("__add__", [](const Vec3& a, const Vec3& b) { return a + b; })
        .def("__sub__", [](const Vec3& a, const Vec3& b) { return a - b; })
        .def("__mul__", [](const Vec3& a, double s) { return a * s; })
        .def("__rmul__", [](const Vec3& a, double s) { return a * s; })
        .def("__truediv__", [](const Vec3& a, double s) { return a / s; })
        .def("__eq__", [](const Vec3& a, const Vec3& b) { return a == b; })
        .def("__repr__", [](const Vec3& v) {
            std::ostringstream ss;
            ss << "Vec3(x=" << v.x << ", y=" << v.y << ", z=" << v.z << ")";
            return ss.str();
        });

    // -------------------------------------------------------------
    // Control Commands: VelocityCommand
    // -------------------------------------------------------------
    py::class_<VelocityCommand>(m, "VelocityCommand")
        .def(py::init<>())
        .def(py::init<Vec3, double, bool>(),
             py::arg("desired_velocity"), py::arg("desired_yaw") = 0.0, py::arg("use_explicit_yaw") = false)
        .def(py::init([](double vx, double vy, double vz) {
            return VelocityCommand{Vec3(vx, vy, vz), 0.0, false};
        }), py::arg("vx"), py::arg("vy"), py::arg("vz"))
        .def_readwrite("desired_velocity", &VelocityCommand::desiredVelocity)
        .def_readwrite("desired_yaw", &VelocityCommand::desiredYaw)
        .def_readwrite("use_explicit_yaw", &VelocityCommand::useExplicitYaw)
        .def("__repr__", [](const VelocityCommand& cmd) {
            std::ostringstream ss;
            ss << "VelocityCommand(desired_velocity=" << cmd.desiredVelocity.toString()
               << ", desired_yaw=" << cmd.desiredYaw
               << ", use_explicit_yaw=" << (cmd.useExplicitYaw ? "True" : "False") << ")";
            return ss.str();
        });

    // -------------------------------------------------------------
    // Entities: UAVState & DeliveryRequest
    // -------------------------------------------------------------
    py::class_<UAVState>(m, "UAVState")
        .def_readonly("id", &UAVState::id)
        .def_readonly("position", &UAVState::position)
        .def_readonly("velocity", &UAVState::velocity)
        .def_readonly("acceleration", &UAVState::acceleration)
        .def_readonly("yaw", &UAVState::yaw)
        .def_readonly("yaw_rate", &UAVState::yawRate)
        .def_readonly("battery_percent", &UAVState::batteryPercent)
        .def_readonly("payload_mass", &UAVState::payloadMass)
        .def_readonly("mode", &UAVState::mode)
        .def_readonly("active_request_id", &UAVState::activeRequestId)
        .def_readonly("target_waypoint", &UAVState::targetWaypoint)
        .def_readonly("is_grounded", &UAVState::isGrounded)
        .def_property_readonly("speed", &UAVState::speed)
        .def_property_readonly("horizontal_speed", &UAVState::horizontalSpeed)
        .def_property_readonly("vertical_speed", &UAVState::verticalSpeed)
        .def("__repr__", [](const UAVState& u) {
            std::ostringstream ss;
            ss << "UAVState(id=" << u.id << ", pos=" << u.position.toString()
               << ", mode=" << uavModeToString(u.mode)
               << ", battery=" << u.batteryPercent << "%)";
            return ss.str();
        });

    py::class_<DeliveryRequest>(m, "DeliveryRequest")
        .def_readonly("id", &DeliveryRequest::id)
        .def_readonly("store_id", &DeliveryRequest::storeId)
        .def_readonly("customer_id", &DeliveryRequest::customerId)
        .def_readonly("pickup_location", &DeliveryRequest::pickupLocation)
        .def_readonly("pickup_position", &DeliveryRequest::pickupLocation)
        .def_readonly("dropoff_location", &DeliveryRequest::dropoffLocation)
        .def_readonly("delivery_position", &DeliveryRequest::dropoffLocation)
        .def_readonly("payload_mass", &DeliveryRequest::payloadMass)
        .def_readonly("creation_time", &DeliveryRequest::creationTime)
        .def_readonly("deadline", &DeliveryRequest::deadline)
        .def_readonly("deadline_time", &DeliveryRequest::deadline)
        .def_readonly("pickup_time", &DeliveryRequest::pickupTime)
        .def_readonly("delivery_time", &DeliveryRequest::deliveryTime)
        .def_readonly("status", &DeliveryRequest::status)
        .def("__repr__", [](const DeliveryRequest& r) {
            std::ostringstream ss;
            ss << "DeliveryRequest(id=" << r.id << ", store=" << r.storeId
               << ", customer=" << r.customerId << ", mass=" << r.payloadMass
               << "kg, status=" << requestStatusToString(r.status) << ")";
            return ss.str();
        });

    // -------------------------------------------------------------
    // Telemetry & Snapshots
    // -------------------------------------------------------------
    py::class_<SnapshotMetrics>(m, "SnapshotMetrics")
        .def_readonly("total_created", &SnapshotMetrics::totalCreated)
        .def_readonly("total_delivered", &SnapshotMetrics::totalDelivered)
        .def_readonly("total_expired", &SnapshotMetrics::totalExpired)
        .def_readonly("total_active", &SnapshotMetrics::totalActive)
        .def_readonly("total_pending", &SnapshotMetrics::totalPending)
        .def_readonly("collision_count", &SnapshotMetrics::collisionCount)
        .def_readonly("delivery_success_rate", &SnapshotMetrics::deliverySuccessRate)
        .def_readonly("average_delivery_duration", &SnapshotMetrics::averageDeliveryDuration)
        .def_readonly("real_fps", &SnapshotMetrics::realFPS)
        .def("__repr__", [](const SnapshotMetrics& sm) {
            std::ostringstream ss;
            ss << "SnapshotMetrics(delivered=" << sm.totalDelivered
               << ", active=" << sm.totalActive
               << ", pending=" << sm.totalPending
               << ", collisions=" << sm.collisionCount
               << ", success_rate=" << sm.deliverySuccessRate << "%)";
            return ss.str();
        });

    py::class_<WorldSnapshot>(m, "WorldSnapshot")
        .def_readonly("simulation_time", &WorldSnapshot::simulationTime)
        .def_readonly("wall_time", &WorldSnapshot::wallTime)
        .def_readonly("step_count", &WorldSnapshot::stepCount)
        .def_readonly("uavs", &WorldSnapshot::uavs)
        .def_readonly("active_requests", &WorldSnapshot::activeRequests)
        .def_readonly("metrics", &WorldSnapshot::metrics);

    // -------------------------------------------------------------
    // RL Observation & StepResult
    // -------------------------------------------------------------
    py::class_<Observation>(m, "Observation")
        .def_readonly("uav_id", &Observation::uavId)
        .def_readonly("ego_velocity", &Observation::egoVelocity)
        .def_readonly("ego_acceleration", &Observation::egoAcceleration)
        .def_readonly("relative_target", &Observation::relativeTarget)
        .def_readonly("distance_to_target", &Observation::distanceToTarget)
        .def_readonly("battery_fraction", &Observation::batteryFraction)
        .def_readonly("payload_mass", &Observation::payloadMass)
        .def_readonly("speed", &Observation::speed)
        .def_readonly("lidar_beams", &Observation::lidarBeams)
        .def_readonly("nearby_uavs", &Observation::nearbyUavs)
        .def_readonly("heading_sin", &Observation::headingSin)
        .def_readonly("heading_cos", &Observation::headingCos)
        .def_readonly("mode_one_hot", &Observation::modeOneHot)
        .def("to_list", &Observation::toVector)
        .def("to_numpy", [](const Observation& obs) {
            auto vec = obs.toVector();
            py::array_t<float> arr(vec.size());
            auto buf = arr.request();
            std::memcpy(buf.ptr, vec.data(), vec.size() * sizeof(float));
            return arr;
        })
        .def_static("vector_size", &Observation::vectorSize)
        .def("__len__", [](const Observation&) { return Observation::vectorSize(); })
        .def("__repr__", [](const Observation& o) {
            std::ostringstream ss;
            ss << "Observation(uav_id=" << o.uavId
               << ", dist_target=" << o.distanceToTarget
               << ", battery=" << (o.batteryFraction * 100.0) << "%"
               << ", speed=" << o.speed << "m/s"
               << ", dim=" << Observation::vectorSize() << ")";
            return ss.str();
        });

    py::class_<StepResult>(m, "StepResult")
        .def_readonly("observations", &StepResult::observations)
        .def_readonly("rewards", &StepResult::rewards)
        .def_readonly("terminated", &StepResult::terminated)
        .def_readonly("truncated", &StepResult::truncated)
        .def_readonly("all_done", &StepResult::allDone)
        .def_readonly("metrics", &StepResult::metrics)
        .def("get_observation_matrix", [](const StepResult& res) {
            const size_t n = res.observations.size();
            const size_t d = Observation::vectorSize();
            py::array_t<float> arr({n, d});
            auto buf = arr.request();
            float* ptr = static_cast<float*>(buf.ptr);
            for (size_t i = 0; i < n; ++i) {
                auto vec = res.observations[i].toVector();
                std::memcpy(ptr + i * d, vec.data(), d * sizeof(float));
            }
            return arr;
        })
        .def("get_rewards_numpy", [](const StepResult& res) {
            py::array_t<float> arr(res.rewards.size());
            auto buf = arr.request();
            std::memcpy(buf.ptr, res.rewards.data(), res.rewards.size() * sizeof(float));
            return arr;
        })
        .def("get_terminated_numpy", [](const StepResult& res) {
            py::array_t<bool> arr(res.terminated.size());
            auto buf = arr.request();
            bool* ptr = static_cast<bool*>(buf.ptr);
            for (size_t i = 0; i < res.terminated.size(); ++i) {
                ptr[i] = res.terminated[i];
            }
            return arr;
        })
        .def("get_truncated_numpy", [](const StepResult& res) {
            py::array_t<bool> arr(res.truncated.size());
            auto buf = arr.request();
            bool* ptr = static_cast<bool*>(buf.ptr);
            for (size_t i = 0; i < res.truncated.size(); ++i) {
                ptr[i] = res.truncated[i];
            }
            return arr;
        });

    // -------------------------------------------------------------
    // Core Simulator Engine
    // -------------------------------------------------------------
    py::class_<Simulator>(m, "Simulator")
        .def(py::init<const std::string&>(), py::arg("config_path") = "config/simulation.json")
        .def("reset", [](Simulator& self, uint64_t seed) {
            py::gil_scoped_release release;
            self.reset(seed);
        }, py::arg("seed") = 0)
        .def("reset_navigation_task", [](Simulator& self, uint64_t seed, const Vec3& start, const Vec3& target) {
            py::gil_scoped_release release;
            self.resetNavigationTask(seed, start, target);
        }, py::arg("seed"), py::arg("start"), py::arg("target"))
        .def("step", [](Simulator& self, const std::vector<VelocityCommand>& actions) {
            py::gil_scoped_release release;
            return self.step(actions);
        }, py::arg("actions"))
        .def("step_heuristic", [](Simulator& self) {
            py::gil_scoped_release release;
            return self.stepHeuristic();
        })
        .def("get_observation", &Simulator::getObservation, py::arg("uav_id"))
        .def("get_observation_numpy", [](const Simulator& self, UAVId uavId) {
            auto obs = self.getObservation(uavId);
            auto vec = obs.toVector();
            py::array_t<float> arr(vec.size());
            auto buf = arr.request();
            std::memcpy(buf.ptr, vec.data(), vec.size() * sizeof(float));
            return arr;
        }, py::arg("uav_id"))
        .def("get_all_observations", &Simulator::getAllObservations)
        .def("get_all_observations_numpy", [](const Simulator& self) {
            auto allObs = self.getAllObservations();
            const size_t n = allObs.size();
            const size_t d = Observation::vectorSize();
            py::array_t<float> arr({n, d});
            auto buf = arr.request();
            float* ptr = static_cast<float*>(buf.ptr);
            for (size_t i = 0; i < n; ++i) {
                auto vec = allObs[i].toVector();
                std::memcpy(ptr + i * d, vec.data(), d * sizeof(float));
            }
            return arr;
        })
        .def("get_snapshot", &Simulator::getSnapshot)
        .def("set_seed", &Simulator::setSeed, py::arg("seed"))
        .def_property_readonly("simulated_time", &Simulator::getSimulatedTime)
        .def_property_readonly("step_count", &Simulator::getStepCount)
        .def_property_readonly("fleet_size", &Simulator::getFleetSize)
        .def_property_readonly("max_horizontal_speed", [](const Simulator& self) {
            return self.getConfig().fleet.maxHorizontalSpeed;
        })
        .def_property_readonly("max_vertical_speed", [](const Simulator& self) {
            return self.getConfig().fleet.maxVerticalSpeed;
        })
        .def_property_readonly("max_acceleration", [](const Simulator& self) {
            return self.getConfig().fleet.maxAcceleration;
        })
        .def_property_readonly("max_payload_mass", [](const Simulator& self) {
            return self.getConfig().requests.maxPayloadKg;
        })
        .def_property_readonly("world_extent", [](const Simulator& self) {
            return self.getConfig().world.getDimensions();
        })
        .def_property_readonly("world_min", [](const Simulator& self) {
            const auto& w = self.getConfig().world;
            return Vec3(w.xMin, w.yMin, w.zMin);
        })
        .def_property_readonly("default_spawn_altitude", [](const Simulator& self) {
            return self.getConfig().world.defaultSpawnAltitude;
        })
        .def_property_readonly("navigation_task_succeeded", &Simulator::navigationTaskSucceeded)
        .def_property_readonly("max_observation_speed", [](const Simulator& self) {
            const auto& f = self.getConfig().fleet;
            return std::sqrt(2.0 * f.maxHorizontalSpeed * f.maxHorizontalSpeed +
                             f.maxVerticalSpeed * f.maxVerticalSpeed);
        });
}

