// SCurveSim: Test wrapper for SCurveStepGenerator
// Thin wrapper that adds simulation interface for testing

#include "SCurveSim.h"
#include "SysTiming.h"

namespace ClearCore {

// Unit conversion helpers
int32_t SCurveSim::QxToVel(int32_t velQx) {
    int64_t vel = (static_cast<int64_t>(velQx) * SampleRateHz) >> FRACT_BITS;
    return static_cast<int32_t>(vel);
}

int32_t SCurveSim::QxToAccel(int32_t accelQx) {
    int64_t accel = (static_cast<int64_t>(accelQx) * SampleRateHz * SampleRateHz) >> FRACT_BITS;
    return static_cast<int32_t>(accel);
}

SCurveSim::SCurveSim(uint32_t stepsPerSampleMax)
    : SCurveStepGenerator() {
    TestIO::Init(this, stepsPerSampleMax);
}

void SCurveSim::Step() {
    // Call the library implementation
    // Final correction tracking is handled in SCurveStepGenerator::AdvancePhase()
    SCurveStepsCalculated();
}

int32_t SCurveSim::GetVelocity() const {
    return QxToVel(m_scVelQx);
}

int32_t SCurveSim::GetAcceleration() const {
    return QxToAccel(m_scAccelQx);
}

int32_t SCurveSim::GetJerk() const {
    return Q24ToJerk(m_jerkCurrentQ24);
}

} // namespace ClearCore
