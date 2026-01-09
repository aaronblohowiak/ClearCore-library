#include "TrapSim.h"
#include "SCurveSim.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

using namespace ClearCore;

struct TestCase {
    int id;
    const char* name;
    int32_t distance;       // pulses (this IS the expected position)
    uint32_t velMax;        // pulses/sec
    uint32_t accelMax;      // pulses/sec^2
    uint32_t jerkMax;       // pulses/sec^3
    const char* regime;     // expected motion regime
};

struct TestResult {
    int32_t finalPosition;
    int32_t positionError;  // final - expected
    uint32_t totalSamples;
    int32_t peakVelocity;
    int32_t peakAccel;
    int32_t peakJerk;
    int32_t finalCorrectionSteps;  // steps emitted in final correction
    bool completed;         // did it finish (reach idle)?
};

// Test case table - expected position is always equal to distance
const std::vector<TestCase> testCases = {
    // Baseline
    {1,  "Baseline",              3000,   10000,  100000,    500000, "Full 7-phase + cruise"},

    // Distance variations
    {2,  "Micro-move",              50,   10000,  100000,    500000, "May abort mid-phase"},
    {3,  "Short triangular",       200,   10000,  100000,    500000, "Never hits velMax"},
    {4,  "Medium triangular",     1000,   10000,  100000,    500000, "Triangular, no cruise"},
    {5,  "Long cruise",          10000,   10000,  100000,    500000, "Cruise dominates"},
    {6,  "Very long",           100000,   10000,  100000,    500000, "Position accuracy at scale"},

    // Jerk variations
    {7,  "Very smooth (low jerk)", 3000,  10000,  100000,     50000, "Long jerk phases"},
    {8,  "Sharp jerk",            3000,   10000,  100000,   5000000, "Short jerk, has const accel"},
    {9,  "Near-trapezoidal",      3000,   10000,  100000,  50000000, "Jerk phases negligible"},

    // Accel variations
    {10, "Accel-limited",         3000,   10000,   10000,    500000, "Long const accel phases"},
    {11, "Jerk = Accel limited",  3000,   10000,  500000,    500000, "Jerk phases = accel time"},
    {12, "Pure jerk (high accel)", 3000,  10000, 1000000,    500000, "No const accel phase"},

    // Velocity variations
    {13, "Low vel limit",         3000,    1000,  100000,    500000, "Quick to hit velMax"},
    {14, "High vel limit",        3000,   50000,  100000,    500000, "Never reaches velMax"},

    // Edge cases
    {15, "Single pulse",             1,   10000,  100000,    500000, "Minimum move"},
    {16, "Few pulses",              10,   10000,  100000,    500000, "Very constrained"},
};

TestResult runTrapezoidal(const TestCase& tc) {
    TestResult result = {0, 0, 0, 0, 0, 0, 0, false};

    TrapSim sim(100);  // 100 steps per sample max
    sim.VelMax(tc.velMax);
    sim.AccelMax(tc.accelMax);
    sim.Move(tc.distance);

    const uint32_t maxSamples = 10000000;  // 2000 seconds max
    int32_t peakVel = 0;
    int32_t peakAccel = 0;

    while (!sim.IsIdle() && result.totalSamples < maxSamples) {
        sim.Step();
        result.totalSamples++;

        int32_t vel = std::abs(sim.GetVelocity());
        int32_t accel = std::abs(sim.GetAcceleration());
        if (vel > peakVel) peakVel = vel;
        if (accel > peakAccel) peakAccel = accel;
    }

    result.finalPosition = sim.GetPosition();
    result.positionError = result.finalPosition - tc.distance;
    result.peakVelocity = peakVel;
    result.peakAccel = peakAccel;
    result.peakJerk = 0;  // Trapezoidal doesn't track jerk
    result.completed = sim.IsIdle();

    return result;
}

TestResult runSCurve(const TestCase& tc) {
    TestResult result = {0, 0, 0, 0, 0, 0, 0, false};

    SCurveSim sim(100);  // 100 steps per sample max
    sim.VelMax(tc.velMax);
    sim.AccelMax(tc.accelMax);
    sim.JerkMax(tc.jerkMax);
    sim.Move(tc.distance);

    const uint32_t maxSamples = 10000000;  // 2000 seconds max
    int32_t peakVel = 0;
    int32_t peakAccel = 0;
    int32_t peakJerk = 0;

    while (!sim.IsIdle() && result.totalSamples < maxSamples) {
        sim.Step();
        result.totalSamples++;

        int32_t vel = std::abs(sim.GetVelocity());
        int32_t accel = std::abs(sim.GetAcceleration());
        int32_t jerk = std::abs(sim.GetJerk());
        if (vel > peakVel) peakVel = vel;
        if (accel > peakAccel) peakAccel = accel;
        if (jerk > peakJerk) peakJerk = jerk;
    }

    result.finalPosition = sim.GetPosition();
    result.positionError = result.finalPosition - tc.distance;
    result.peakVelocity = peakVel;
    result.peakAccel = peakAccel;
    result.peakJerk = peakJerk;
    result.finalCorrectionSteps = sim.GetFinalCorrectionSteps();
    result.completed = sim.IsIdle();

    return result;
}

void printHeader() {
    std::cout << "\n";
    std::cout << std::setw(4) << "ID"
              << std::setw(24) << "Name"
              << std::setw(10) << "Expected"
              << " | "
              << std::setw(10) << "Trap Pos"
              << std::setw(8) << "Err"
              << std::setw(10) << "Samples"
              << " | "
              << std::setw(10) << "SCurve Pos"
              << std::setw(8) << "Err"
              << std::setw(10) << "Samples"
              << std::endl;
    std::cout << std::string(106, '-') << std::endl;
}

void printResult(const TestCase& tc, const TestResult& trap, const TestResult& scurve) {
    std::cout << std::setw(4) << tc.id
              << std::setw(24) << tc.name
              << std::setw(10) << tc.distance
              << " | "
              << std::setw(10) << trap.finalPosition
              << std::setw(8) << trap.positionError
              << std::setw(10) << trap.totalSamples
              << " | "
              << std::setw(10) << scurve.finalPosition
              << std::setw(8) << scurve.positionError
              << std::setw(10) << scurve.totalSamples;

    // Flag errors
    if (trap.positionError != 0) std::cout << " TRAP_ERR!";
    if (scurve.positionError != 0) std::cout << " SCURVE_ERR!";
    if (!trap.completed) std::cout << " TRAP_TIMEOUT!";
    if (!scurve.completed) std::cout << " SCURVE_TIMEOUT!";

    std::cout << std::endl;
}

int main() {
    std::cout << "==================================================================" << std::endl;
    std::cout << "MOTION PROFILE POSITION ACCURACY TEST" << std::endl;
    std::cout << "==================================================================" << std::endl;
    std::cout << "\nTest verifies that final position equals commanded distance." << std::endl;
    std::cout << "Expected position = Distance (pulses commanded)" << std::endl;
    std::cout << "Error = Final Position - Expected" << std::endl;

    printHeader();

    int trapErrors = 0;
    int scurveErrors = 0;
    int trapTimeouts = 0;
    int scurveTimeouts = 0;

    for (const auto& tc : testCases) {
        TestResult trap = runTrapezoidal(tc);
        TestResult scurve = runSCurve(tc);

        printResult(tc, trap, scurve);

        if (trap.positionError != 0) trapErrors++;
        if (scurve.positionError != 0) scurveErrors++;
        if (!trap.completed) trapTimeouts++;
        if (!scurve.completed) scurveTimeouts++;
    }

    std::cout << std::string(106, '-') << std::endl;

    // Summary
    std::cout << "\n=== SUMMARY ===" << std::endl;
    std::cout << "Total test cases: " << testCases.size() << std::endl;
    std::cout << "\nTrapezoidal:" << std::endl;
    std::cout << "  Position errors: " << trapErrors << std::endl;
    std::cout << "  Timeouts: " << trapTimeouts << std::endl;
    std::cout << "\nS-Curve:" << std::endl;
    std::cout << "  Position errors: " << scurveErrors << std::endl;
    std::cout << "  Timeouts: " << scurveTimeouts << std::endl;

    // Detailed error report for S-curve
    std::cout << "\n=== S-CURVE POSITION ACCURACY ===" << std::endl;
    std::cout << std::setw(4) << "ID"
              << std::setw(24) << "Name"
              << std::setw(10) << "Expected"
              << std::setw(10) << "Actual"
              << std::setw(10) << "Error"
              << std::setw(12) << "Error %"
              << std::endl;
    std::cout << std::string(70, '-') << std::endl;

    for (const auto& tc : testCases) {
        TestResult scurve = runSCurve(tc);
        double errorPct = (tc.distance > 0) ?
            (100.0 * scurve.positionError / tc.distance) : 0.0;

        std::cout << std::setw(4) << tc.id
                  << std::setw(24) << tc.name
                  << std::setw(10) << tc.distance
                  << std::setw(10) << scurve.finalPosition
                  << std::setw(10) << scurve.positionError
                  << std::setw(11) << std::fixed << std::setprecision(2) << errorPct << "%"
                  << std::endl;
    }

    // Limit violation report for S-curve
    std::cout << "\n=== S-CURVE LIMIT VIOLATIONS ===" << std::endl;
    std::cout << std::setw(4) << "ID"
              << std::setw(24) << "Name"
              << std::setw(12) << "AccelLimit"
              << std::setw(12) << "PeakAccel"
              << std::setw(12) << "Accel %"
              << std::setw(12) << "JerkLimit"
              << std::setw(12) << "PeakJerk"
              << std::setw(12) << "Jerk %"
              << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    int accelViolations = 0;
    int jerkViolations = 0;

    for (const auto& tc : testCases) {
        TestResult scurve = runSCurve(tc);

        double accelPct = (tc.accelMax > 0) ?
            (100.0 * scurve.peakAccel / tc.accelMax) : 0.0;
        double jerkPct = (tc.jerkMax > 0) ?
            (100.0 * scurve.peakJerk / tc.jerkMax) : 0.0;

        bool accelViolation = scurve.peakAccel > static_cast<int32_t>(tc.accelMax);
        bool jerkViolation = scurve.peakJerk > static_cast<int32_t>(tc.jerkMax);

        if (accelViolation) accelViolations++;
        if (jerkViolation) jerkViolations++;

        std::cout << std::setw(4) << tc.id
                  << std::setw(24) << tc.name
                  << std::setw(12) << tc.accelMax
                  << std::setw(12) << scurve.peakAccel
                  << std::setw(11) << std::fixed << std::setprecision(1) << accelPct << "%"
                  << std::setw(12) << tc.jerkMax
                  << std::setw(12) << scurve.peakJerk
                  << std::setw(11) << std::fixed << std::setprecision(1) << jerkPct << "%";

        if (accelViolation) std::cout << " ACCEL!";
        if (jerkViolation) std::cout << " JERK!";
        std::cout << std::endl;
    }

    std::cout << std::string(100, '-') << std::endl;
    std::cout << "\nLimit Violation Summary:" << std::endl;
    std::cout << "  Acceleration limit violations: " << accelViolations << "/" << testCases.size() << std::endl;
    std::cout << "  Jerk limit violations: " << jerkViolations << "/" << testCases.size() << std::endl;

    // Final correction steps report
    std::cout << "\n=== S-CURVE FINAL CORRECTION STEPS ===" << std::endl;
    std::cout << "Steps emitted in final correction to reach exact target position:" << std::endl;
    std::cout << std::setw(4) << "ID"
              << std::setw(24) << "Name"
              << std::setw(10) << "Distance"
              << std::setw(12) << "Correction"
              << std::endl;
    std::cout << std::string(50, '-') << std::endl;

    int nonZeroCorrections = 0;
    for (const auto& tc : testCases) {
        TestResult scurve = runSCurve(tc);
        if (scurve.finalCorrectionSteps != 0) {
            nonZeroCorrections++;
            std::cout << std::setw(4) << tc.id
                      << std::setw(24) << tc.name
                      << std::setw(10) << tc.distance
                      << std::setw(12) << scurve.finalCorrectionSteps
                      << std::endl;
        }
    }

    if (nonZeroCorrections == 0) {
        std::cout << "(No final corrections needed - all moves hit exact target)" << std::endl;
    }

    std::cout << std::string(50, '-') << std::endl;
    std::cout << "Tests with nonzero final correction: " << nonZeroCorrections << "/" << testCases.size() << std::endl;

    return (trapErrors > 0 || scurveErrors > 0) ? 1 : 0;
}
