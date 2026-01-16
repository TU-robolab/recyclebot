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

# Clean up old visualization files to avoid accumulation
echo "Cleaning up old visualization files..."
rm -f /tmp/rgbd_frame_*.png
OUTPUT_DIR="$(dirname "$0")/../test_output"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

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

# Copy visualization images to test_suite output directory (accessible from host)
echo ""
echo "4. Copying visualization images..."

if [ -f "/tmp/rgbd_frame_0_combined.png" ]; then
    cp /tmp/rgbd_frame_0_combined.png "$OUTPUT_DIR/rgbd_combined.png"
    cp /tmp/rgbd_frame_0_rgb.png "$OUTPUT_DIR/rgbd_rgb.png" 2>/dev/null || true
    cp /tmp/rgbd_frame_0_depth.png "$OUTPUT_DIR/rgbd_depth.png" 2>/dev/null || true

    echo "Visualizations saved to: $OUTPUT_DIR/"
    echo "  - rgbd_combined.png (RGB + Depth side-by-side)"
    echo "  - rgbd_rgb.png (RGB only)"
    echo "  - rgbd_depth.png (Depth with legend)"

    # Try to open the combined image (works on host, ignored in Docker)
    # This will work if running directly on host, or when mounted volume is accessible
    COMBINED_IMG="$OUTPUT_DIR/rgbd_combined.png"

    # Detect platform and open image
    if command -v open &> /dev/null; then
        # macOS
        echo ""
        echo "Opening visualization (macOS)..."
        open "$COMBINED_IMG" 2>/dev/null || echo "Note: Run 'open test_suite/test_output/rgbd_combined.png' from host to view"
    elif command -v xdg-open &> /dev/null; then
        # Linux with X11
        echo ""
        echo "Opening visualization (Linux)..."
        xdg-open "$COMBINED_IMG" 2>/dev/null || echo "Note: Run 'xdg-open test_suite/test_output/rgbd_combined.png' from host to view"
    else
        echo ""
        echo "To view the visualization:"
        echo "  From host: open test_suite/test_output/rgbd_combined.png"
        echo "  Or check: $COMBINED_IMG"
    fi
else
    echo "No visualization images found in /tmp/"
fi

exit $TEST_RESULT
