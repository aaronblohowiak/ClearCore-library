#pragma once
// SCurveSim: Test wrapper for SCurveStepGenerator
// Provides simulation interface for testing the library implementation

#include "SCurveStepGenerator.h"
#include "TrapSim.h"  // For TestIO helper class

namespace ClearCore {

/**
    Test simulation wrapper for SCurveStepGenerator.
    Adds simulation-specific methods for testing and verification.
**/
class SCurveSim : public SCurveStepGenerator {
public:
    SCurveSim(uint32_t stepsPerSampleMax = 100);

    // Simulation interface - call Step() each sample period
    void Step();

    // Test accessors
    uint32_t GetStepsPrevious() const { return m_stepsPrevious; }
    int32_t GetPosition() const { return m_posnAbsolute; }
    int32_t GetVelocity() const;      // pulses/sec
    int32_t GetAcceleration() const;  // pulses/sec^2
    int32_t GetJerk() const;          // pulses/sec^3
    bool IsIdle() const { return m_scPhase == SC_IDLE; }
    int GetPhase() const { return static_cast<int>(m_scPhase); }
    int32_t GetPhaseSamplesLeft() const { return m_phaseSamplesLeft; }

    // Debug/test access to plan details
    int32_t GetPhaseSamples(int phase) const {
        return (phase >= 0 && phase < 7) ? m_plan.phaseSamples[phase] : 0;
    }
    int32_t GetPhaseJerkQ24(int phase) const {
        return (phase >= 0 && phase < 7) ? m_plan.phaseJerkQ24[phase] : 0;
    }
    int32_t GetPlanPeakVel() const { return m_plan.peakVelQx; }
    int32_t GetPlanPeakAccel() const { return m_plan.peakAccelQx; }
    int32_t GetTargetPos() const { return m_plan.targetPos; }
    int32_t GetFinalCorrectionSteps() const { return m_lastFinalCorrection; }

    // Access internal limits via TestIO (StepGenerator members are private)
    int32_t GetVelLimitQx() const { return TestIO::GetVelLimitQx(const_cast<SCurveSim*>(this)); }
    int32_t GetAccelLimitQx() const { return TestIO::GetAccelLimitQx(const_cast<SCurveSim*>(this)); }

protected:
    void OutputDirection() override { /* No hardware in simulation */ }

private:
    // Unit conversion helpers for test output
    static int32_t QxToVel(int32_t velQx);
    static int32_t QxToAccel(int32_t accelQx);
};

} // namespace ClearCore
