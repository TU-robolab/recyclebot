#!/bin/bash
################################################################################
# Vision Workflow Automated Test Script
################################################################################
#
# This script runs end-to-end tests for the RecycleBot vision detection system.
#
# What it does:
#   1. Starts fake RGBD camera publisher (simulates RealSense camera)
#   2. Starts vision detector node (YOLO-based object detection)
#   3. Runs pytest integration tests
#   4. Generates detailed report with performance metrics
#   5. Cleans up all processes automatically
#
# Usage:
#   bash scripts/run_vision_test.sh
#
# Requirements:
#   - ROS2 environment sourced
#   - recycle_bot and test_suite packages built
#   - Inside Docker container or ROS2 environment
#
# Output:
#   - Console: Real-time test progress and summary
#   - File: /tmp/vision_workflow_test_report.txt
#
# Exit codes:
#   0 - All tests passed
#   1 - One or more tests failed
#
# See README_VISION_TESTS.md for detailed documentation
#
################################################################################

set -e

echo "Starting vision workflow automated test..."
echo "=========================================="

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up..."
    if [ ! -z "$FAKE_PUB_PID" ]; then
        kill $FAKE_PUB_PID 2>/dev/null || true
    fi
    if [ ! -z "$VISION_PID" ]; then
        kill $VISION_PID 2>/dev/null || true
    fi
}

# Set trap to cleanup on exit
trap cleanup EXIT INT TERM

# Start fake RGBD publisher
echo "1. Starting fake RGBD publisher..."
ros2 run test_suite fake_rgbd_publisher &
FAKE_PUB_PID=$!
sleep 3

# Start vision detector
echo "2. Starting vision detector node..."
ros2 run recycle_bot rec_bot_vision &
VISION_PID=$!
sleep 10  # Wait for YOLO model to load

# Run the tests
echo "3. Running integration tests..."
python3 -m pytest "$(dirname "$0")/../test/test_vision_workflow.py" -v --tb=short

# Check test result
TEST_RESULT=$?

echo ""
if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
else
    echo -e "${RED}✗ Tests failed!${NC}"
fi

exit $TEST_RESULT
