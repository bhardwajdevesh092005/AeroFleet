#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <mutex>
#include <iomanip>
#include "core/Types.hpp"

namespace aerofleet {

enum class SimulationEventType : uint8_t {
    REQUEST_CREATED = 0,
    REQUEST_ASSIGNED,
    REQUEST_PICKED_UP,
    REQUEST_DELIVERED,
    REQUEST_EXPIRED,
    REQUEST_REQUEUED,
    COLLISION_BUILDING,
    COLLISION_UAV,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    CHARGING_STARTED,
    CHARGING_COMPLETED
};

inline std::string eventTypeToString(SimulationEventType type) {
    switch (type) {
        case SimulationEventType::REQUEST_CREATED: return "REQUEST_CREATED";
        case SimulationEventType::REQUEST_ASSIGNED: return "REQUEST_ASSIGNED";
        case SimulationEventType::REQUEST_PICKED_UP: return "REQUEST_PICKED_UP";
        case SimulationEventType::REQUEST_DELIVERED: return "REQUEST_DELIVERED";
        case SimulationEventType::REQUEST_EXPIRED: return "REQUEST_EXPIRED";
        case SimulationEventType::REQUEST_REQUEUED: return "REQUEST_REQUEUED";
        case SimulationEventType::COLLISION_BUILDING: return "COLLISION_BUILDING";
        case SimulationEventType::COLLISION_UAV: return "COLLISION_UAV";
        case SimulationEventType::BATTERY_LOW: return "BATTERY_LOW";
        case SimulationEventType::BATTERY_CRITICAL: return "BATTERY_CRITICAL";
        case SimulationEventType::CHARGING_STARTED: return "CHARGING_STARTED";
        case SimulationEventType::CHARGING_COMPLETED: return "CHARGING_COMPLETED";
        default: return "UNKNOWN";
    }
}

struct SimulationEvent {
    double timestamp{0.0};
    SimulationEventType type{SimulationEventType::REQUEST_CREATED};
    int64_t entityIdA{-1};
    int64_t entityIdB{-1};
    double value{0.0};
    std::string detail{};

    [[nodiscard]] std::string toJson() const {
        std::ostringstream ss;
        ss << "{\"time\":" << std::fixed << std::setprecision(4) << timestamp
           << ",\"event\":\"" << eventTypeToString(type) << "\""
           << ",\"entityA\":" << entityIdA
           << ",\"entityB\":" << entityIdB
           << ",\"value\":" << value;
        if (!detail.empty()) {
            ss << ",\"detail\":\"" << detail << "\"";
        }
        ss << "}";
        return ss.str();
    }
};

/**
 * @brief High-speed in-memory ring-buffer event logger writing structured JSONL telemetry.
 * Pre-allocates buffer in memory to ensure zero I/O jitter during simulation loops.
 */
class EventLogger {
public:
    explicit EventLogger(size_t capacity = 65536)
        : m_capacity(capacity > 0 ? capacity : 65536) {
        m_buffer.reserve(m_capacity);
    }

    void log(
        double timestamp,
        SimulationEventType type,
        int64_t entityIdA = -1,
        int64_t entityIdB = -1,
        double value = 0.0,
        const std::string& detail = ""
    ) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_buffer.size() < m_capacity) {
            m_buffer.push_back(SimulationEvent{timestamp, type, entityIdA, entityIdB, value, detail});
        } else {
            m_buffer[m_head] = SimulationEvent{timestamp, type, entityIdA, entityIdB, value, detail};
            m_head = (m_head + 1) % m_capacity;
            m_wrapped = true;
        }
        m_totalLogged++;
    }

    [[nodiscard]] size_t size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_buffer.size();
    }

    [[nodiscard]] size_t totalLogged() const noexcept {
        return m_totalLogged;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_buffer.clear();
        m_head = 0;
        m_wrapped = false;
        m_totalLogged = 0;
    }

    [[nodiscard]] std::vector<SimulationEvent> getEvents() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_wrapped) {
            return m_buffer;
        }
        std::vector<SimulationEvent> result;
        result.reserve(m_buffer.size());
        for (size_t i = m_head; i < m_buffer.size(); ++i) {
            result.push_back(m_buffer[i]);
        }
        for (size_t i = 0; i < m_head; ++i) {
            result.push_back(m_buffer[i]);
        }
        return result;
    }

    bool flushToFile(const std::string& filepath) {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::ofstream file(filepath, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            return false;
        }

        if (m_wrapped) {
            for (size_t i = m_head; i < m_buffer.size(); ++i) {
                file << m_buffer[i].toJson() << "\n";
            }
            for (size_t i = 0; i < m_head; ++i) {
                file << m_buffer[i].toJson() << "\n";
            }
        } else {
            for (const auto& ev : m_buffer) {
                file << ev.toJson() << "\n";
            }
        }

        return true;
    }

private:
    size_t m_capacity;
    std::vector<SimulationEvent> m_buffer;
    size_t m_head{0};
    bool m_wrapped{false};
    size_t m_totalLogged{0};
    mutable std::mutex m_mutex;
};

} // namespace aerofleet

