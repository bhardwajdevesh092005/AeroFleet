#pragma once

#include <algorithm>
#include <cmath>

namespace aerofleet {

/**
 * @brief High-precision authoritative fixed-step simulation clock with accumulator.
 * Decouples fixed physics integration (e.g. 50 Hz, dt = 0.02s) from variable
 * rendering frame rates (e.g. 30, 60, 144 FPS) with sub-step interpolation.
 */
class SimulationClock {
public:
    explicit SimulationClock(double dtPhysics = 0.02, double timeScale = 1.0)
        : m_dtPhysics(dtPhysics > 0.0 ? dtPhysics : 0.02),
          m_timeScale(timeScale >= 0.0 ? timeScale : 1.0) {}

    /**
     * @brief Feed real-world wall clock elapsed seconds into the accumulator.
     * Clamps maximum accumulated time to maxAccumulator (0.2s) to prevent the
     * "spiral of death" if rendering or CPU execution experiences stutter.
     */
    void update(double wallDeltaTime) noexcept {
        if (!std::isfinite(wallDeltaTime) || wallDeltaTime <= 0.0) return;
        m_wallTime += wallDeltaTime;
        m_accumulator += wallDeltaTime * m_timeScale;
        if (m_accumulator > m_maxAccumulator) {
            m_accumulator = m_maxAccumulator;
        }
    }

    /**
     * @brief Checks if enough accumulated time is available for a physics integration step.
     */
    [[nodiscard]] bool hasStep() const noexcept {
        return m_accumulator >= (m_dtPhysics - 1e-9);
    }

    /**
     * @brief Consumes one fixed physics time slice and advances simulation time.
     */
    void advance() noexcept {
        m_accumulator -= m_dtPhysics;
        if (m_accumulator < 0.0) {
            m_accumulator = 0.0;
        }
        m_simulationTime += m_dtPhysics;
        m_stepCount++;
    }

    /**
     * @brief Current authoritative continuous simulation time in seconds.
     */
    [[nodiscard]] double now() const noexcept {
        return m_simulationTime;
    }

    /**
     * @brief Sub-step fractional interpolation parameter alpha in [0.0, 1.0) for visual smoothing.
     * alpha = accumulator / dtPhysics
     */
    [[nodiscard]] double getAlpha() const noexcept {
        if (m_dtPhysics <= 0.0) return 0.0;
        return std::clamp(m_accumulator / m_dtPhysics, 0.0, 1.0);
    }

    [[nodiscard]] double getDtPhysics() const noexcept { return m_dtPhysics; }
    void setDtPhysics(double dt) noexcept { if (dt > 0.0) m_dtPhysics = dt; }

    [[nodiscard]] double getTimeScale() const noexcept { return m_timeScale; }
    void setTimeScale(double scale) noexcept { if (scale >= 0.0) m_timeScale = scale; }

    [[nodiscard]] uint64_t getStepCount() const noexcept { return m_stepCount; }
    [[nodiscard]] double getWallTime() const noexcept { return m_wallTime; }

    void reset(double initialSimTime = 0.0) noexcept {
        m_simulationTime = initialSimTime;
        m_accumulator = 0.0;
        m_wallTime = 0.0;
        m_stepCount = 0;
    }

private:
    double m_dtPhysics{0.02};
    double m_timeScale{1.0};
    double m_simulationTime{0.0};
    double m_accumulator{0.0};
    double m_wallTime{0.0};
    double m_maxAccumulator{0.20}; // Max 200ms lag protection
    uint64_t m_stepCount{0};
};

} // namespace aerofleet

