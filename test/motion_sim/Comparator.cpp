#include "Comparator.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <algorithm>

Comparator::Comparator()
    : m_velMax(10000),
      m_accelMax(100000),
      m_jerkMax(500000) {
}

void Comparator::SetVelMax(uint32_t velMax) {
    m_velMax = velMax;
}

void Comparator::SetAccelMax(uint32_t accelMax) {
    m_accelMax = accelMax;
}

void Comparator::SetJerkMax(uint32_t jerkMax) {
    m_jerkMax = jerkMax;
}

MoveProfile Comparator::RunTrapezoidal(int32_t distance) {
    MoveProfile profile;
    profile.totalSamples = 0;
    profile.peakVelocity = 0;
    profile.peakAcceleration = 0;
    profile.peakJerk = 0;

    // Create and configure simulator
    ClearCore::TrapSim sim(100);  // 100 steps/sample max (500kHz / 5kHz)
    sim.VelMax(m_velMax);
    sim.AccelMax(m_accelMax);

    // Issue move
    sim.Move(distance);

    // Run simulation
    uint32_t sampleNum = 0;
    const uint32_t maxSamples = 10000000;  // Safety limit
    int32_t lastAccel = 0;

    while (!sim.IsIdle() && sampleNum < maxSamples) {
        // Record state before step
        SamplePoint pt;
        pt.sample = sampleNum;
        pt.position = sim.GetPosition();
        pt.velocity = sim.GetVelocity();
        pt.acceleration = sim.GetAcceleration();
        pt.state = static_cast<int>(sim.GetState());

        // Calculate jerk (change in acceleration per sample, scaled to per-second)
        pt.jerk = (pt.acceleration - lastAccel) * ClearCore::SampleRateHz;
        lastAccel = pt.acceleration;

        // Execute one sample
        sim.Step();

        // Record steps output
        pt.stepsOutput = sim.GetStepsPrevious();
        profile.samples.push_back(pt);

        // Track peaks
        profile.peakVelocity = std::max(profile.peakVelocity,
                                        std::abs(pt.velocity));
        profile.peakAcceleration = std::max(profile.peakAcceleration,
                                            std::abs(pt.acceleration));
        profile.peakJerk = std::max(profile.peakJerk, std::abs(pt.jerk));

        sampleNum++;
    }

    // Final statistics
    profile.totalSamples = sampleNum;
    profile.totalTimeMs = sampleNum * 0.2;  // 200us per sample
    profile.finalPosition = sim.GetPosition();

    return profile;
}

MoveProfile Comparator::RunSCurve(int32_t distance) {
    MoveProfile profile;
    profile.totalSamples = 0;
    profile.peakVelocity = 0;
    profile.peakAcceleration = 0;
    profile.peakJerk = 0;

    // Create and configure simulator
    ClearCore::SCurveSim sim(100);  // 100 steps/sample max
    sim.VelMax(m_velMax);
    sim.AccelMax(m_accelMax);
    sim.JerkMax(m_jerkMax);

    // Issue move
    sim.Move(distance);

    // Run simulation
    uint32_t sampleNum = 0;
    const uint32_t maxSamples = 10000000;  // Safety limit

    while (!sim.IsIdle() && sampleNum < maxSamples) {
        // Record state before step
        SamplePoint pt;
        pt.sample = sampleNum;
        pt.position = sim.GetPosition();
        pt.velocity = sim.GetVelocity();
        pt.acceleration = sim.GetAcceleration();
        pt.jerk = sim.GetJerk();
        pt.state = sim.GetPhase();

        // Execute one sample
        sim.Step();

        // Record steps output
        pt.stepsOutput = sim.GetStepsPrevious();
        profile.samples.push_back(pt);

        // Track peaks
        profile.peakVelocity = std::max(profile.peakVelocity,
                                        std::abs(pt.velocity));
        profile.peakAcceleration = std::max(profile.peakAcceleration,
                                            std::abs(pt.acceleration));
        profile.peakJerk = std::max(profile.peakJerk, std::abs(pt.jerk));

        sampleNum++;
    }

    // Final statistics
    profile.totalSamples = sampleNum;
    profile.totalTimeMs = sampleNum * 0.2;  // 200us per sample
    profile.finalPosition = sim.GetPosition();

    return profile;
}

ComparisonResult Comparator::Compare(int32_t distance) {
    ComparisonResult result;

    // Run both profiles
    result.trap = RunTrapezoidal(distance);
    result.scurve = RunSCurve(distance);

    // Calculate deltas
    result.timeDeltaSamples = result.scurve.totalSamples - result.trap.totalSamples;
    result.timeDeltaMs = result.timeDeltaSamples * 0.2;
    result.timeDeltaPercent = (result.trap.totalSamples > 0)
        ? (100.0 * result.timeDeltaSamples / result.trap.totalSamples)
        : 0.0;
    result.positionMatch = (result.trap.finalPosition == distance) &&
                           (result.scurve.finalPosition == distance);

    return result;
}

void Comparator::PrintProfile(const MoveProfile& profile, const char* name) {
    std::cout << "\n=== " << name << " ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Total samples:     " << profile.totalSamples << std::endl;
    std::cout << "  Total time:        " << profile.totalTimeMs << " ms" << std::endl;
    std::cout << "  Final position:    " << profile.finalPosition << " pulses" << std::endl;
    std::cout << "  Peak velocity:     " << profile.peakVelocity << " pulses/sec" << std::endl;
    std::cout << "  Peak acceleration: " << profile.peakAcceleration << " pulses/sec^2" << std::endl;
    std::cout << "  Peak jerk:         " << profile.peakJerk << " pulses/sec^3" << std::endl;
}

void Comparator::PrintComparison(const ComparisonResult& result) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "COMPARISON RESULTS" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    PrintProfile(result.trap, "Trapezoidal");
    PrintProfile(result.scurve, "S-Curve");

    std::cout << "\n--- Delta ---" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Time delta:     " << result.timeDeltaSamples << " samples ("
              << result.timeDeltaMs << " ms, "
              << (result.timeDeltaPercent >= 0 ? "+" : "")
              << result.timeDeltaPercent << "%)" << std::endl;
    std::cout << "  Position match: " << (result.positionMatch ? "YES" : "NO") << std::endl;
}

void Comparator::ExportCSV(const MoveProfile& profile, const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    file << "sample,time_ms,position,velocity,acceleration,steps,state\n";

    for (const auto& pt : profile.samples) {
        file << pt.sample << ","
             << (pt.sample * 0.2) << ","
             << pt.position << ","
             << pt.velocity << ","
             << pt.acceleration << ","
             << pt.stepsOutput << ","
             << pt.state << "\n";
    }

    file.close();
    std::cout << "Exported " << profile.samples.size()
              << " samples to " << filename << std::endl;
}

void Comparator::ExportComparisonCSV(const ComparisonResult& result,
                                     const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    file << "sample,time_ms,"
         << "trap_pos,trap_vel,trap_accel,trap_steps,"
         << "scurve_pos,scurve_vel,scurve_accel,scurve_steps\n";

    size_t maxSamples = std::max(result.trap.samples.size(),
                                  result.scurve.samples.size());

    for (size_t i = 0; i < maxSamples; i++) {
        file << i << "," << (i * 0.2) << ",";

        if (i < result.trap.samples.size()) {
            const auto& pt = result.trap.samples[i];
            file << pt.position << "," << pt.velocity << ","
                 << pt.acceleration << "," << pt.stepsOutput;
        } else {
            file << ",,,,";
        }

        file << ",";

        if (i < result.scurve.samples.size()) {
            const auto& pt = result.scurve.samples[i];
            file << pt.position << "," << pt.velocity << ","
                 << pt.acceleration << "," << pt.stepsOutput;
        } else {
            file << ",,,";
        }

        file << "\n";
    }

    file.close();
    std::cout << "Exported comparison to " << filename << std::endl;
}
