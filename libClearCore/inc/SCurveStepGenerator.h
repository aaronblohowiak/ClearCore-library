/*
 * Copyright (c) 2020 Teknic, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
    \file
    ClearCore S-Curve Step & Direction motion profile generator

    Provides jerk-limited S-curve motion profiles as an alternative to the
    trapezoidal profiles in the base StepGenerator class. S-curves provide
    smoother acceleration/deceleration with reduced mechanical stress.
**/

#ifndef __SCURVESTEPGENERATOR_H__
#define __SCURVESTEPGENERATOR_H__

#include "StepGenerator.h"
#include <stdint.h>

namespace ClearCore {

/**
    \class SCurveStepGenerator
    \brief ClearCore S-Curve Step and Direction generator class

    This class provides 7-phase jerk-limited S-curve motion profiles:
    1. Jerk up (accel ramps 0 → a_max)
    2. Constant acceleration
    3. Jerk down (accel ramps a_max → 0)
    4. Cruise (constant velocity)
    5. Jerk decel up (accel ramps 0 → -a_max)
    6. Constant deceleration
    7. Jerk decel down (accel ramps -a_max → 0)

    Uses Q24 fractional jerk accumulator for fine resolution, supporting
    jerk values from ~7,500 to 200,000,000+ pulses/sec³.

    For more detailed information on the ClearCore Motion Connector interface,
    check out the \ref MotorDriverMain informational page.
**/
class SCurveStepGenerator : public StepGenerator {
    friend class MotorManager;
    friend class TestIO;

public:
    SCurveStepGenerator();

    /**
        \brief Sets the maximum jerk limit for S-curve moves.

        Jerk is the rate of change of acceleration. Lower jerk values
        produce smoother motion with less mechanical stress.

        \code{.cpp}
        // Set the jerk limit to 500000 pulses/sec³
        ConnectorM0.JerkMax(500000);
        \endcode

        \param[in] jerkMax The jerk limit in step pulses per second³

        \note Minimum effective jerk is ~7,500 pulses/sec³ due to
        fixed-point resolution at 5kHz sample rate.

        <div class="sd-disclaimer">For use with Step and Direction mode.</div>
    **/
    void JerkMax(uint32_t jerkMax);

    /**
        \brief Accessor for the current jerk limit.

        \code{.cpp}
        // Get the current jerk limit
        uint32_t currentJerk = ConnectorM0.JerkMax();
        \endcode

        \return The current jerk limit in step pulses per second³

        <div class="sd-disclaimer">For use with Step and Direction mode.</div>
    **/
    uint32_t JerkMax() const {
        return m_jerkMaxPending;
    }

    /**
        \brief Issues a positional move using S-curve profile.

        Overrides the base class Move() to use jerk-limited S-curve
        motion instead of trapezoidal motion.

        \code{.cpp}
        // Move 5000 steps using S-curve profile
        ConnectorM0.Move(5000);
        \endcode

        \param[in] dist The distance of the move in step pulses
        \param[in] moveTarget Absolute or relative positioning
        \return true if move was accepted

        <div class="sd-disclaimer">For use with Step and Direction mode.</div>
    **/
    bool Move(int32_t dist,
              MoveTarget moveTarget = MOVE_TARGET_REL_END_POSN) override;

protected:
    // S-curve specific phases
    typedef enum {
        SC_IDLE = 0,
        SC_JERK_ACCEL_UP,    // Phase 1: accel ramps 0 → a_max
        SC_ACCEL_CONST,      // Phase 2: constant acceleration
        SC_JERK_ACCEL_DN,    // Phase 3: accel ramps a_max → 0
        SC_CRUISE,           // Phase 4: constant velocity
        SC_JERK_DECEL_UP,    // Phase 5: accel ramps 0 → -a_max
        SC_DECEL_CONST,      // Phase 6: constant deceleration
        SC_JERK_DECEL_DN,    // Phase 7: accel ramps -a_max → 0
        SC_END
    } SCPhase;

    // Pre-computed motion plan
    typedef struct {
        int32_t phaseSamples[7];   // Number of samples for each phase
        int32_t phaseJerkQ24[7];   // Jerk value in Q24 format
        int32_t peakVelQx;         // Peak velocity achieved
        int32_t peakAccelQx;       // Peak acceleration used
        int32_t targetPos;         // Target position (pulses)
        bool valid;
    } SCPlan;

    // Called from ISR to calculate steps for current sample
    void SCurveStepsCalculated();

    // Planning function
    void PlanSCurve(int32_t distance);

    // Phase transition
    void AdvancePhase();

    // Unit conversion helpers
    static constexpr int JERK_FRACT_BITS = 24;
    static int32_t JerkToQ24(uint32_t pulsesPerSecCubed);
    static int32_t Q24ToJerk(int32_t jerkQ24);

    // Configuration (pending values - latched on Move())
    uint32_t m_jerkMaxPending;   // User units: pulses/sec³ (pending)
    int32_t m_jerkMaxQ24;        // Active jerk limit in Q24 format

    // Runtime state
    SCPhase m_scPhase;
    SCPlan m_plan;
    int32_t m_jerkCurrentQ24;    // Current jerk in Q24 format
    int32_t m_accelFracQ24;      // Fractional accumulator for sub-LSB jerk
    int32_t m_phaseSamplesLeft;  // Samples remaining in current phase

    // Motion state
    int64_t m_scPosnQx;          // Current position in Qx
    int32_t m_scVelQx;           // Current velocity in Qx
    int32_t m_scAccelQx;         // Current acceleration in Qx
    int32_t m_scStepsSent;       // Accumulated integer steps
    int32_t m_lastFinalCorrection; // Steps emitted as final correction (for diagnostics)
};

} // namespace ClearCore

#endif // __SCURVESTEPGENERATOR_H__
