/**
 * @file test_main.cpp
 * @brief Google Test entry point for Cutter tests
 */

#include <gtest/gtest.h>
#include "FakeHal.h"

/**
 * @brief Test environment that resets HAL state before each test
 */
class CutterTestEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        // Global setup if needed
    }

    void TearDown() override {
        // Global teardown if needed
    }
};

/**
 * @brief Main entry point for tests
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new CutterTestEnvironment);
    return RUN_ALL_TESTS();
}
