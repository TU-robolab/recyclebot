#!/bin/bash
# Hardware-free test runner for RecycleBot
# Usage: ./run_tests.sh [vision|robot|gripper|full]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default test mode
TEST_MODE="${1:-vision}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}RecycleBot Hardware-Free Test Suite${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to cleanup background processes
cleanup() {
    echo -e "\n${YELLOW}Cleaning up background processes...${NC}"
    pkill -f fake_rgbd_publisher.py || true
    pkill -f fake_joint_state_publisher.py || true
    pkill -f rec_bot_vision || true
    pkill -f gripper_node || true
    sleep 2
}

# Set trap to cleanup on exit
trap cleanup EXIT INT TERM

# Check if we're in a ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS2 environment not sourced!${NC}"
    echo "Please run: source /ros_entrypoint.sh"
    exit 1
fi

echo -e "${GREEN}ROS2 ${ROS_DISTRO} detected${NC}"
echo -e "Test Mode: ${YELLOW}${TEST_MODE}${NC}"
echo ""

# Run tests based on mode
case "$TEST_MODE" in
    vision)
        echo -e "${BLUE}=== Running Vision System Test ===${NC}"
        echo ""

        # Start fake camera
        echo -e "${YELLOW}Starting fake RGBD publisher...${NC}"
        python3 /home/joao/fake_rgbd_publisher.py &
        CAMERA_PID=$!
        sleep 3

        # Start vision node
        echo -e "${YELLOW}Starting vision detector...${NC}"
        ros2 run recycle_bot rec_bot_vision &
        VISION_PID=$!
        sleep 5

        # Run vision test
        echo -e "${YELLOW}Running automated vision test...${NC}"
        python3 /home/joao/test_suite/scripts/test_vision.py
        RESULT=$?

        if [ $RESULT -eq 0 ]; then
            echo -e "\n${GREEN}✓ Vision test PASSED${NC}"
        else
            echo -e "\n${RED}✗ Vision test FAILED${NC}"
        fi

        ;;

    robot)
        echo -e "${BLUE}=== Running Robot System Test ===${NC}"
        echo ""

        # Start fake joint state publisher
        echo -e "${YELLOW}Starting fake joint state publisher...${NC}"
        python3 /home/joao/test_suite/scripts/fake_joint_state_publisher.py &
        JOINT_PID=$!
        sleep 3

        # Check joint states are being published
        echo -e "${YELLOW}Checking joint states...${NC}"
        timeout 5 ros2 topic echo /joint_states --once

        if [ $? -eq 0 ]; then
            echo -e "\n${GREEN}✓ Robot joint states PASSED${NC}"
            RESULT=0
        else
            echo -e "\n${RED}✗ Robot joint states FAILED${NC}"
            RESULT=1
        fi

        ;;

    gripper)
        echo -e "${BLUE}=== Running Gripper System Test ===${NC}"
        echo ""

        # Start gripper node
        echo -e "${YELLOW}Starting gripper node...${NC}"
        ros2 run grip_command_package gripper_node --ros-args -p debug:=true &
        GRIPPER_PID=$!
        sleep 5

        # Test gripper service
        echo -e "${YELLOW}Testing gripper grip action...${NC}"
        ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'grip'}"

        if [ $? -eq 0 ]; then
            echo -e "\n${GREEN}✓ Gripper test PASSED${NC}"
            RESULT=0
        else
            echo -e "\n${RED}✗ Gripper test FAILED${NC}"
            RESULT=1
        fi

        ;;

    full)
        echo -e "${BLUE}=== Running Full Integration Test ===${NC}"
        echo ""

        # Run all tests sequentially
        $0 vision
        VISION_RESULT=$?

        $0 robot
        ROBOT_RESULT=$?

        $0 gripper
        GRIPPER_RESULT=$?

        # Summary
        echo ""
        echo -e "${BLUE}=== Test Summary ===${NC}"
        [ $VISION_RESULT -eq 0 ] && echo -e "${GREEN}✓ Vision test PASSED${NC}" || echo -e "${RED}✗ Vision test FAILED${NC}"
        [ $ROBOT_RESULT -eq 0 ] && echo -e "${GREEN}✓ Robot test PASSED${NC}" || echo -e "${RED}✗ Robot test FAILED${NC}"
        [ $GRIPPER_RESULT -eq 0 ] && echo -e "${GREEN}✓ Gripper test PASSED${NC}" || echo -e "${RED}✗ Gripper test FAILED${NC}"

        if [ $VISION_RESULT -eq 0 ] && [ $ROBOT_RESULT -eq 0 ] && [ $GRIPPER_RESULT -eq 0 ]; then
            echo -e "\n${GREEN}All tests PASSED!${NC}"
            RESULT=0
        else
            echo -e "\n${RED}Some tests FAILED${NC}"
            RESULT=1
        fi

        ;;

    *)
        echo -e "${RED}Unknown test mode: $TEST_MODE${NC}"
        echo "Usage: $0 [vision|robot|gripper|full]"
        exit 1
        ;;
esac

echo ""
echo -e "${BLUE}========================================${NC}"
exit $RESULT
