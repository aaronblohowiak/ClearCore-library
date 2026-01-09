#pragma once
// Motion profile comparator
// Runs trapezoidal and S-curve simulations and compares results

#include "TrapSim.h"
#include "SCurveSim.h"
#include <vector>
#include <cstdint>

// Sample point data for profiling
struct SamplePoint {
    uint32_t sample;          // Sample number (time = sample * 200us)
    int32_t position;         // Absolute position (pulses)
    int32_t velocity;         // Velocity (pulses/sec)
    int32_t acceleration;     // Acceleration (pulses/sec^2)
    int32_t jerk;             // Jerk (pulses/sec^3) - 0 for trapezoidal
    uint32_t stepsOutput;     // Steps output this sample
    int state;                // State machine state
};

// Complete move profile
struct MoveProfile {
    std::vector<SamplePoint> samples;
    uint32_t totalSamples;    // Total samples to complete move
    double totalTimeMs;       // Total time in milliseconds
    int32_t finalPosition;    // Final position (should match target)
    int32_t peakVelocity;     // Peak velocity reached (pulses/sec)
    int32_t peakAcceleration; // Peak acceleration (pulses/sec^2)
    int32_t peakJerk;         // Peak jerk (pulses/sec^3)
};

// Comparison results
struct ComparisonResult {
    MoveProfile trap;
    MoveProfile scurve;

    // Delta analysis
    int32_t timeDeltaSamples;   // Positive = S-curve slower
    double timeDeltaMs;
    double timeDeltaPercent;
    bool positionMatch;          // Both reached exact target
};

class Comparator {
public:
    Comparator();

    // Configuration
    void SetVelMax(uint32_t velMax);
    void SetAccelMax(uint32_t accelMax);
    void SetJerkMax(uint32_t jerkMax);  // S-curve only

    // Run trapezoidal profile only
    MoveProfile RunTrapezoidal(int32_t distance);

    // Run S-curve profile only
    MoveProfile RunSCurve(int32_t distance);

    // Run full comparison (trap vs s-curve)
    ComparisonResult Compare(int32_t distance);

    // Output
    void PrintProfile(const MoveProfile& profile, const char* name);
    void PrintComparison(const ComparisonResult& result);
    void ExportCSV(const MoveProfile& profile, const char* filename);
    void ExportComparisonCSV(const ComparisonResult& result, const char* filename);

private:
    uint32_t m_velMax;
    uint32_t m_accelMax;
    uint32_t m_jerkMax;
};
