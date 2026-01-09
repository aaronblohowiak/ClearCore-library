// Motion Profile Simulation Test
//
// Compares trapezoidal vs S-curve motion profiles
// Build: make
// Run:   ./motion_sim

#include "Comparator.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>

int main(int argc, char* argv[]) {
    std::cout << "ClearCore Motion Profile Simulator" << std::endl;
    std::cout << "===================================" << std::endl;
    std::cout << "Sample rate: 5000 Hz (200us per sample)" << std::endl;

    Comparator cmp;

    // Default configuration
    uint32_t velMax = 10000;       // pulses/sec
    uint32_t accelMax = 100000;    // pulses/sec^2
    uint32_t jerkMax = 500000;     // pulses/sec^3
    int32_t distance = 3000;       // ~3 inches at 1000 pulses/inch

    // Parse command line args
    if (argc >= 2) {
        distance = std::atoi(argv[1]);
    }
    if (argc >= 3) {
        velMax = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        accelMax = std::atoi(argv[3]);
    }
    if (argc >= 5) {
        jerkMax = std::atoi(argv[4]);
    }

    std::cout << "\nConfiguration:" << std::endl;
    std::cout << "  Distance:  " << distance << " pulses" << std::endl;
    std::cout << "  VelMax:    " << velMax << " pulses/sec" << std::endl;
    std::cout << "  AccelMax:  " << accelMax << " pulses/sec^2" << std::endl;
    std::cout << "  JerkMax:   " << jerkMax << " pulses/sec^3" << std::endl;

    cmp.SetVelMax(velMax);
    cmp.SetAccelMax(accelMax);
    cmp.SetJerkMax(jerkMax);

    // Run comparison
    std::cout << "\n--- Running Comparison ---" << std::endl;
    auto result = cmp.Compare(distance);

    // Print results
    cmp.PrintComparison(result);

    // Verify position accuracy
    std::cout << "\n--- Position Accuracy ---" << std::endl;
    int32_t trapError = result.trap.finalPosition - distance;
    int32_t scurveError = result.scurve.finalPosition - distance;

    std::cout << "  Trapezoidal: " << result.trap.finalPosition
              << " (error: " << trapError << ")" << std::endl;
    std::cout << "  S-Curve:     " << result.scurve.finalPosition
              << " (error: " << scurveError << ")" << std::endl;

    // Export data
    cmp.ExportCSV(result.trap, "trap_profile.csv");
    cmp.ExportCSV(result.scurve, "scurve_profile.csv");
    cmp.ExportComparisonCSV(result, "comparison.csv");

    // Show jerk comparison
    std::cout << "\n--- Jerk Comparison ---" << std::endl;
    std::cout << "  Trapezoidal peak jerk: " << result.trap.peakJerk
              << " pulses/sec^3" << std::endl;
    std::cout << "  S-Curve peak jerk:     " << result.scurve.peakJerk
              << " pulses/sec^3" << std::endl;
    std::cout << "  S-Curve jerk limit:    " << jerkMax
              << " pulses/sec^3" << std::endl;

    bool jerkLimited = (result.scurve.peakJerk <= static_cast<int32_t>(jerkMax * 1.1)); // 10% tolerance
    std::cout << "  Jerk limiting:         "
              << (jerkLimited ? "OK" : "EXCEEDED") << std::endl;

    std::cout << "\nSimulation complete." << std::endl;

    // Return success if both positions match and jerk is limited
    bool success = result.positionMatch && jerkLimited;
    return success ? 0 : 1;
}
