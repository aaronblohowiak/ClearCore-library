# libCutter Makefile
#
# Targets:
#   test      - Build and run tests on host PC
#   coverage  - Tests with coverage reporting
#   clearcore - Cross-compile for ClearCore (requires SDK)
#   clean     - Remove build artifacts

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I inc -I test

# Source files (excluding platform-specific HAL)
CUTTER_SRCS = \
	src/Cutter.cpp \
	src/CutterState.cpp \
	src/CommandParser.cpp \
	src/CommandDispatcher.cpp \
	src/ResponseWriter.cpp \
	src/PinCommands.cpp \
	src/MotorCommands.cpp

# Test source files
TEST_SRCS = \
	test/test_main.cpp \
	test/test_parser.cpp \
	test/test_response.cpp \
	test/test_state_machine.cpp \
	test/test_pins.cpp \
	test/test_motors.cpp \
	test/test_homing.cpp \
	test/test_move_until.cpp \
	test/test_errors.cpp \
	test/test_integration.cpp

# Build directory
BUILD_DIR = build

#
# Test target (runs on host PC with fake HAL)
#
.PHONY: test
test: CXXFLAGS += -DCUTTER_PLATFORM_TEST -g
test: $(BUILD_DIR)/run_tests
	./$(BUILD_DIR)/run_tests

$(BUILD_DIR)/run_tests: $(CUTTER_SRCS) test/CutterHal_Fake.cpp $(TEST_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lgtest -pthread

#
# Coverage target
#
.PHONY: coverage
coverage: CXXFLAGS += -DCUTTER_PLATFORM_TEST --coverage -g
coverage: $(BUILD_DIR)/run_tests_cov
	./$(BUILD_DIR)/run_tests_cov
	gcov -o $(BUILD_DIR) $(CUTTER_SRCS)
	@echo "Coverage files generated (*.gcov)"

$(BUILD_DIR)/run_tests_cov: $(CUTTER_SRCS) test/CutterHal_Fake.cpp $(TEST_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lgtest -pthread

#
# ClearCore target (cross-compile)
# Note: Requires ClearCore SDK and toolchain to be installed
#
.PHONY: clearcore
clearcore:
	@echo "ClearCore cross-compilation requires the Teknic SDK."
	@echo "This target is a placeholder - integrate with your ClearCore build system."

#
# Build directory
#
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

#
# Clean
#
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -f *.gcov *.gcda *.gcno

#
# Help
#
.PHONY: help
help:
	@echo "libCutter Build System"
	@echo ""
	@echo "Targets:"
	@echo "  test      - Build and run tests on host PC"
	@echo "  coverage  - Tests with coverage reporting"
	@echo "  clearcore - Cross-compile for ClearCore (placeholder)"
	@echo "  clean     - Remove build artifacts"
	@echo "  help      - Show this message"
