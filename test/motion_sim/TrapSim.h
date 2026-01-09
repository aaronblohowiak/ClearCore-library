#pragma once
// TrapSim: Concrete subclass of StepGenerator for simulation
// Uses existing trapezoidal motion profile code unchanged

#include "StepGenerator.h"
#include "SysTiming.h"

namespace ClearCore {

// Forward declaration
class TrapSim;

// TestIO is declared as friend in StepGenerator.h (line 60)
// This gives us access to private members for simulation
class TestIO {
public:
    // Initialize step rate (required before use)
    static void Init(StepGenerator* gen, uint32_t stepsPerSampleMax = 100) {
        gen->StepsPerSampleMaxSet(stepsPerSampleMax);
    }

    // Access private state for simulation output
    static int32_t GetVelCurrentQx(StepGenerator* gen) {
        return gen->m_velCurrentQx;
    }

    static int32_t GetAccelCurrentQx(StepGenerator* gen) {
        return gen->m_accelCurrentQx;
    }

    static int64_t GetPosnCurrentQx(StepGenerator* gen) {
        return gen->m_posnCurrentQx;
    }

    static int32_t GetVelLimitQx(StepGenerator* gen) {
        return gen->m_velLimitQx;
    }

    static int32_t GetAccelLimitQx(StepGenerator* gen) {
        return gen->m_accelLimitQx;
    }

    // Apply pending limits (normally done inside Move())
    static void ApplyPendingLimits(StepGenerator* gen) {
        gen->m_velLimitQx = gen->m_velLimitPendingQx;
        gen->m_accelLimitQx = gen->m_accelLimitPendingQx;
    }
};

// Thin wrapper to make StepGenerator instantiable for simulation
class TrapSim : public StepGenerator {
public:
    TrapSim(uint32_t stepsPerSampleMax = 100) {
        TestIO::Init(this, stepsPerSampleMax);
    }

    // Simulation interface - call Step() to simulate one ISR cycle (200us)
    void Step() {
        StepsCalculated();
    }

    // Public accessors
    uint32_t GetStepsPrevious() const {
        return m_stepsPrevious;
    }

    int32_t GetPosition() const {
        return m_posnAbsolute;
    }

    // Velocity in pulses/sec (uses public method)
    int32_t GetVelocity() {
        return VelocityRefCommanded();
    }

    // Velocity in Qx format (internal units)
    int32_t GetVelocityQx() {
        return TestIO::GetVelCurrentQx(this);
    }

    // Acceleration in Qx format (internal units)
    int32_t GetAccelQx() {
        return TestIO::GetAccelCurrentQx(this);
    }

    // Acceleration converted to pulses/sec^2
    int32_t GetAcceleration() {
        int32_t accelQx = TestIO::GetAccelCurrentQx(this);
        // Convert from Qx (pulses/sample^2) to pulses/sec^2
        // accel_real = accelQx * SampleRateHz^2 / 2^FRACT_BITS
        int64_t accel = (static_cast<int64_t>(accelQx) * SampleRateHz * SampleRateHz) >> FRACT_BITS;
        return static_cast<int32_t>(accel);
    }

    bool IsIdle() const {
        return m_moveState == MS_IDLE;
    }

    MoveStates GetState() const {
        return m_moveState;
    }

    bool GetDirection() const {
        return m_direction;
    }

protected:
    void OutputDirection() override {
        // No-op for simulation
    }
};

} // namespace ClearCore
