# Motion Profile Simulator Makefile
#
# Build:  make
# Run:    make run
# Clean:  make clean

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra

# Include paths: mocks first (to override hardware headers), then library
INCLUDES = -I./mocks \
           -I../../libClearCore/inc

# Source files
CLEARCORE_SRC = ../../libClearCore/src/StepGenerator.cpp \
                ../../libClearCore/src/SCurveStepGenerator.cpp
SIM_SRC = Comparator.cpp SCurveSim.cpp main.cpp
ACCURACY_SRC = SCurveSim.cpp accuracy_test.cpp

TARGET = motion_sim
ACCURACY_TARGET = accuracy_test

all: $(TARGET) $(ACCURACY_TARGET)

$(TARGET): $(CLEARCORE_SRC) $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ -lm

$(ACCURACY_TARGET): $(CLEARCORE_SRC) $(ACCURACY_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ -lm

# Run comparison
run: $(TARGET)
	./$(TARGET)

# Run accuracy test suite
accuracy: $(ACCURACY_TARGET)
	./$(ACCURACY_TARGET)

# Test various move distances
test: $(TARGET)
	@echo "=== Testing various distances ==="
	./$(TARGET) 100
	./$(TARGET) 500
	./$(TARGET) 1000
	./$(TARGET) 3000
	./$(TARGET) 10000

clean:
	rm -f $(TARGET) $(ACCURACY_TARGET) *.csv

.PHONY: all run accuracy clean
