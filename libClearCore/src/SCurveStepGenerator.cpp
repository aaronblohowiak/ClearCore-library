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

#include "SCurveStepGenerator.h"
#include "SysTiming.h"
#include <algorithm>

namespace ClearCore {

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

// Unit conversion: user jerk (pulses/sec³) to Q24 per-sample format
int32_t SCurveStepGenerator::JerkToQ24(uint32_t pulsesPerSecCubed) {
    int64_t jerkQ24 = (static_cast<int64_t>(pulsesPerSecCubed) << JERK_FRACT_BITS) /
                      (static_cast<int64_t>(SampleRateHz) * SampleRateHz * SampleRateHz);
    return static_cast<int32_t>(min(jerkQ24, static_cast<int64_t>(INT32_MAX)));
}

// Unit conversion: Q24 per-sample format to user jerk (pulses/sec³)
int32_t SCurveStepGenerator::Q24ToJerk(int32_t jerkQ24) {
    int64_t jerk = (static_cast<int64_t>(jerkQ24) * SampleRateHz * SampleRateHz * SampleRateHz)
                   >> JERK_FRACT_BITS;
    return static_cast<int32_t>(jerk);
}

SCurveStepGenerator::SCurveStepGenerator()
    : StepGenerator(),
      m_jerkMaxPending(500000),
      m_jerkMaxQ24(0),
      m_scPhase(SC_IDLE),
      m_plan{},
      m_jerkCurrentQ24(0),
      m_accelFracQ24(0),
      m_phaseSamplesLeft(0),
      m_scPosnQx(0),
      m_scVelQx(0),
      m_scAccelQx(0),
      m_scStepsSent(0),
      m_lastFinalCorrection(0) {
}

void SCurveStepGenerator::JerkMax(uint32_t jerkMax) {
    m_jerkMaxPending = jerkMax;
}

bool SCurveStepGenerator::Move(int32_t dist, MoveTarget moveTarget) {
    int32_t distance = dist;
    if (moveTarget == MOVE_TARGET_ABSOLUTE) {
        distance = dist - m_posnAbsolute;
    }

    bool negative = distance < 0;
    if (negative) {
        distance = -distance;
    }
    m_direction = negative;

    // Latch pending limits (same pattern as base class)
    UpdatePendingMoveLimits();
    m_jerkMaxQ24 = JerkToQ24(m_jerkMaxPending);

    // Reset S-curve state
    m_scPosnQx = 0;
    m_scVelQx = 0;
    m_scAccelQx = 0;
    m_scStepsSent = 0;
    m_jerkCurrentQ24 = 0;
    m_accelFracQ24 = 0;
    m_phaseSamplesLeft = 0;
    m_lastFinalCorrection = 0;

    // Plan the S-curve
    PlanSCurve(distance);

    // Start execution
    if (m_plan.valid && distance > 0) {
        m_scPhase = SC_JERK_ACCEL_UP;
        m_jerkCurrentQ24 = m_plan.phaseJerkQ24[0];
        m_phaseSamplesLeft = m_plan.phaseSamples[0];

        // Skip empty phases at the start
        while (m_phaseSamplesLeft == 0 && m_scPhase != SC_IDLE) {
            AdvancePhase();
        }

        // Update base class state for compatibility
        m_moveState = MS_ACCEL;
    }
    else {
        m_scPhase = SC_IDLE;
        m_moveState = MS_IDLE;
    }

    return true;
}

void SCurveStepGenerator::PlanSCurve(int32_t distance) {
    m_plan = {};

    if (distance <= 0) {
        m_plan.valid = false;
        return;
    }

    m_plan.targetPos = distance;

    // Get limits - use active (latched) values
    int32_t velLimitQx = m_velLimitQx;
    int32_t accelLimitQx = m_accelLimitQx;

    if (velLimitQx <= 0) velLimitQx = 1;
    if (accelLimitQx <= 0) accelLimitQx = 2;

    // Use active jerk limit (already converted to Q24 in Move())
    int32_t jerkQ24 = m_jerkMaxQ24;
    if (jerkQ24 < 1) jerkQ24 = 1;

    // Calculate number of jerk samples to reach accel limit
    int32_t nJerk = static_cast<int32_t>((static_cast<int64_t>(accelLimitQx) << 9) / jerkQ24);
    if (nJerk < 1) nJerk = 1;

    int64_t totalDistQx = static_cast<int64_t>(distance) << FRACT_BITS;

    // Simulation lambda for planning - matches Step() behavior
    auto simulate = [jerkQ24](int32_t nJ, int32_t nCA) {
        int64_t pos = 0, vel = 0;
        int32_t accel = 0;
        int32_t accelFrac = 0;

        // Phase 0: Jerk up
        for (int i = 0; i < nJ; i++) {
            accelFrac += jerkQ24;
            int32_t accelDelta = accelFrac >> 9;
            accelFrac -= accelDelta << 9;
            accel += accelDelta;
            vel += accel;
            pos += vel;
        }
        int32_t actualAccel = accel;

        // Phase 1: Const accel
        for (int i = 0; i < nCA; i++) {
            vel += accel;
            pos += vel;
        }

        // Phase 2: Jerk down
        for (int i = 0; i < nJ; i++) {
            accelFrac -= jerkQ24;
            int32_t accelDelta = accelFrac >> 9;
            accelFrac -= accelDelta << 9;
            accel += accelDelta;
            vel += accel;
            pos += vel;
        }

        struct Result { int64_t dist; int64_t vel; int32_t accel; };
        return Result{pos, vel, actualAccel};
    };

    // Simulate decel phase
    auto simulateDecel = [jerkQ24](int32_t nJ, int32_t nCA, int64_t startVel) {
        int64_t pos = 0, vel = startVel;
        int32_t accel = 0;
        int32_t accelFrac = 0;

        // Phase 4: Jerk decel up
        for (int i = 0; i < nJ; i++) {
            accelFrac -= jerkQ24;
            int32_t accelDelta = accelFrac >> 9;
            accelFrac -= accelDelta << 9;
            accel += accelDelta;
            vel += accel;
            pos += vel;
        }

        // Phase 5: Const decel
        for (int i = 0; i < nCA; i++) {
            vel += accel;
            pos += vel;
        }

        // Phase 6: Jerk decel down
        for (int i = 0; i < nJ; i++) {
            accelFrac += jerkQ24;
            int32_t accelDelta = accelFrac >> 9;
            accelFrac -= accelDelta << 9;
            accel += accelDelta;
            vel += accel;
            pos += vel;
        }

        return pos;
    };

    // Check if jerk phases alone exceed velocity limit
    auto testResult = simulate(nJerk, 0);
    int64_t peakVelTest = testResult.vel;
    int32_t actualAccelTest = testResult.accel;

    int32_t nConstAccel = 0;
    if (peakVelTest > velLimitQx) {
        // Binary search for correct nJerk
        int32_t lo = 1, hi = nJerk;
        while (lo < hi) {
            int32_t mid = (lo + hi + 1) / 2;
            auto r = simulate(mid, 0);
            if (r.vel <= velLimitQx) {
                lo = mid;
            }
            else {
                hi = mid - 1;
            }
        }
        nJerk = lo;
        nConstAccel = 0;
    }
    else if (peakVelTest < velLimitQx) {
        // Binary search for nConstAccel
        int32_t actualAccelQx = actualAccelTest;
        if (actualAccelQx < 1) actualAccelQx = 1;
        int32_t maxNConstAccel = static_cast<int32_t>((velLimitQx - peakVelTest) / actualAccelQx) + 1;
        int32_t lo = 0, hi = maxNConstAccel;
        while (lo < hi) {
            int32_t mid = (lo + hi + 1) / 2;
            auto r = simulate(nJerk, mid);
            if (r.vel <= velLimitQx) {
                lo = mid;
            }
            else {
                hi = mid - 1;
            }
        }
        nConstAccel = lo;
    }

    // Compute final distances
    auto accelResult = simulate(nJerk, nConstAccel);
    int64_t distAccel = accelResult.dist;
    int64_t peakVelQx = accelResult.vel;
    int32_t actualAccelQx = accelResult.accel;

    int64_t distDecel = simulateDecel(nJerk, nConstAccel, peakVelQx);
    int64_t distAccelDecel = distAccel + distDecel;

    // Calculate cruise samples
    int32_t nCruise = 0;
    if (totalDistQx > distAccelDecel && peakVelQx > 0) {
        int64_t distCruise = totalDistQx - distAccelDecel;
        nCruise = static_cast<int32_t>(distCruise / peakVelQx);

        // Check if one more cruise sample hits target exactly
        int64_t actualDist = distAccelDecel + nCruise * peakVelQx;
        int64_t distWithOneMore = actualDist + peakVelQx;
        int32_t currentPulses = static_cast<int32_t>(actualDist >> FRACT_BITS);
        int32_t pulsesWithOneMore = static_cast<int32_t>(distWithOneMore >> FRACT_BITS);

        if (currentPulses < distance && pulsesWithOneMore == distance) {
            nCruise++;
        }
    }
    else if (totalDistQx < distAccelDecel) {
        // Short move - scale down
        double lo = 0.0, hi = 1.0;
        for (int iter = 0; iter < 50; iter++) {
            double mid = (lo + hi) / 2.0;
            int32_t testNJerk = max(1, static_cast<int32_t>(nJerk * mid));
            int32_t testNConstAccel = static_cast<int32_t>(nConstAccel * mid);

            auto r = simulate(testNJerk, testNConstAccel);
            int64_t dD = simulateDecel(testNJerk, testNConstAccel, r.vel);
            int64_t totalD = r.dist + dD;

            if (totalD <= totalDistQx) {
                lo = mid;
            }
            else {
                hi = mid;
            }
        }

        nJerk = max(1, static_cast<int32_t>(nJerk * lo));
        nConstAccel = static_cast<int32_t>(nConstAccel * lo);

        // Recompute
        auto r = simulate(nJerk, nConstAccel);
        distAccel = r.dist;
        peakVelQx = r.vel;
        actualAccelQx = r.accel;
        distDecel = simulateDecel(nJerk, nConstAccel, peakVelQx);
        distAccelDecel = distAccel + distDecel;

        if (totalDistQx > distAccelDecel && peakVelQx > 0) {
            int64_t distCruise = totalDistQx - distAccelDecel;
            nCruise = static_cast<int32_t>(distCruise / peakVelQx);

            int64_t actualDist = distAccelDecel + nCruise * peakVelQx;
            int64_t distWithOneMore = actualDist + peakVelQx;
            int32_t currentPulses = static_cast<int32_t>(actualDist >> FRACT_BITS);
            int32_t pulsesWithOneMore = static_cast<int32_t>(distWithOneMore >> FRACT_BITS);

            if (currentPulses < distance && pulsesWithOneMore == distance) {
                nCruise++;
            }
        }
    }

    // Store plan
    m_plan.phaseSamples[0] = nJerk;
    m_plan.phaseSamples[1] = nConstAccel;
    m_plan.phaseSamples[2] = nJerk;
    m_plan.phaseSamples[3] = nCruise;
    m_plan.phaseSamples[4] = nJerk;
    m_plan.phaseSamples[5] = nConstAccel;
    m_plan.phaseSamples[6] = nJerk;

    m_plan.phaseJerkQ24[0] = jerkQ24;
    m_plan.phaseJerkQ24[1] = 0;
    m_plan.phaseJerkQ24[2] = -jerkQ24;
    m_plan.phaseJerkQ24[3] = 0;
    m_plan.phaseJerkQ24[4] = -jerkQ24;
    m_plan.phaseJerkQ24[5] = 0;
    m_plan.phaseJerkQ24[6] = jerkQ24;

    m_plan.peakVelQx = static_cast<int32_t>(min(peakVelQx, static_cast<int64_t>(INT32_MAX)));
    m_plan.peakAccelQx = actualAccelQx;
    m_plan.valid = true;
}

void SCurveStepGenerator::SCurveStepsCalculated() {
    if (m_scPhase == SC_IDLE) {
        m_stepsPrevious = 0;
        m_moveState = MS_IDLE;
        return;
    }

    int phaseIdx = static_cast<int>(m_scPhase) - 1;

    // Apply jerk using Q24 fractional accumulator
    m_jerkCurrentQ24 = m_plan.phaseJerkQ24[phaseIdx];
    m_accelFracQ24 += m_jerkCurrentQ24;

    // Extract Q15 portion
    int32_t accelDelta = m_accelFracQ24 >> 9;
    m_accelFracQ24 -= accelDelta << 9;
    m_scAccelQx += accelDelta;

    // Force zero accel in cruise
    if (m_scPhase == SC_CRUISE) {
        m_scAccelQx = 0;
        m_accelFracQ24 = 0;
    }

    // Update velocity and position
    m_scVelQx += m_scAccelQx;
    m_scPosnQx += m_scVelQx;

    // Extract integer steps
    int32_t currentIntSteps = static_cast<int32_t>(m_scPosnQx >> FRACT_BITS);
    m_stepsPrevious = currentIntSteps - m_scStepsSent;
    m_scStepsSent = currentIntSteps;

    // Update absolute position
    m_posnAbsolute += m_direction ? -static_cast<int32_t>(m_stepsPrevious)
                                  : static_cast<int32_t>(m_stepsPrevious);

    // Update base class move state for compatibility
    if (m_scPhase <= SC_JERK_ACCEL_DN) {
        m_moveState = MS_ACCEL;
    }
    else if (m_scPhase == SC_CRUISE) {
        m_moveState = MS_CRUISE;
    }
    else {
        m_moveState = MS_DECEL;
    }

    // Phase transition
    m_phaseSamplesLeft--;
    if (m_phaseSamplesLeft <= 0) {
        AdvancePhase();
    }
}

void SCurveStepGenerator::AdvancePhase() {
    int nextPhase = static_cast<int>(m_scPhase) + 1;

    // Skip empty phases
    while (nextPhase <= SC_JERK_DECEL_DN) {
        int phaseIndex = nextPhase - 1;
        if (m_plan.phaseSamples[phaseIndex] > 0) {
            break;
        }
        nextPhase++;
    }

    if (nextPhase > SC_JERK_DECEL_DN) {
        // Move complete - emit any remaining steps
        int32_t stepsRemaining = m_plan.targetPos - m_scStepsSent;

        if (stepsRemaining > 0) {
            m_stepsPrevious = stepsRemaining;
            m_posnAbsolute += m_direction ? -stepsRemaining : stepsRemaining;
            m_scStepsSent = m_plan.targetPos;
            m_lastFinalCorrection = stepsRemaining;  // Track for diagnostics
        }

        m_scPhase = SC_IDLE;
        m_jerkCurrentQ24 = 0;
        m_accelFracQ24 = 0;
        m_scAccelQx = 0;
        m_scVelQx = 0;
        m_moveState = MS_IDLE;
        return;
    }

    m_scPhase = static_cast<SCPhase>(nextPhase);
    int phaseIdx = nextPhase - 1;
    m_jerkCurrentQ24 = m_plan.phaseJerkQ24[phaseIdx];
    m_phaseSamplesLeft = m_plan.phaseSamples[phaseIdx];
}

} // namespace ClearCore
