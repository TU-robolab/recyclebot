# RecycleBot Hardware-Free Test Suite

Automated test suite for verifying RecycleBot functionality without requiring physical hardware.

## Overview

This test suite allows you to verify different subsystems of the RecycleBot project without needing:
- UR robot hardware
- RealSense camera
- Robotiq gripper
- Serial connections

## Quick Start

### From Inside the Container

```bash
# Source ROS environment
source /ros_entrypoint.sh

# Run specific test
cd /home/joao/test_suite
./run_tests.sh vision    # Test vision system only
./run_tests.sh robot     # Test robot system only
./run_tests.sh gripper   # Test gripper system only
./run_tests.sh full      # Run all tests
```

### From Host (Docker)

```bash
# Run vision test
docker exec recyclebot-dev-1 bash -c "source /ros_entrypoint.sh && /home/joao/test_suite/run_tests.sh vision"

# Run all tests
docker exec recyclebot-dev-1 bash -c "source /ros_entrypoint.sh && /home/joao/test_suite/run_tests.sh full"
```

## Test Components

### 1. Vision System Test (`vision`)

**What it tests:**
- YOLO object detection model loading
- RGBD image processing
- Detection service (`/capture_detections`)
- Detection publishing to `/object_detections` topic

**Fake components:**
- `fake_rgbd_publisher.py` - Simulates RealSense D435 camera with synthetic images

**What gets validated:**
- Vision node starts successfully
- YOLO model loads correctly
- Detection service is callable
- Objects are detected in fake images
- Detections are published to ROS topic

**Expected output:**
```
✓ Vision test PASSED
Total objects detected: 2-3
```

---

### 2. Robot System Test (`robot`)

**What it tests:**
- Joint state publishing
- MoveIt configuration
- Robot model loading

**Fake components:**
- `fake_joint_state_publisher.py` - Simulates UR16e robot joint states

**What gets validated:**
- Joint states are published at 10Hz
- All 6 joints are present
- Joint names match UR16e configuration

**Expected output:**
```
✓ Robot joint states PASSED
```

---

### 3. Gripper System Test (`gripper`)

**What it tests:**
- Gripper node initialization
- Service interface (`/gripper_action`)
- Command processing

**What gets validated:**
- Gripper node starts
- Service is available
- Grip/release commands are processed

**Expected output:**
```
✓ Gripper test PASSED
```

---

### 4. Full Integration Test (`full`)

Runs all three tests sequentially and provides a summary.

## Test Scripts

### Core Scripts

- `run_tests.sh` - Main test runner script
- `scripts/test_vision.py` - Automated vision system tester
- `scripts/fake_joint_state_publisher.py` - Fake robot joint states
- `fake_rgbd_publisher.py` - Fake camera data publisher

### Launch Files

- `launch/hardware_free_test.launch.py` - Launch fake nodes for testing

## Customizing Tests

### Modify Fake Camera Images

Edit `/home/joao/fake_rgbd_publisher.py` in the `create_fake_image()` method to change:
- Image size
- Shapes/colors
- Number of objects

### Adjust Detection Confidence

Modify confidence threshold in `rec_bot_vision.py`:
```python
inf_results = self.model(cv_image, conf=0.5)  # Default 50%
```

### Change Robot Joint Positions

Edit `scripts/fake_joint_state_publisher.py` in the `publish_joint_states()` method.

## Troubleshooting

### Test Fails with "Service not available"

The node may not have started yet. Increase sleep time in `run_tests.sh`:
```bash
sleep 5  # Increase from 3 to 5
```

### Vision test finds no objects

This is normal if YOLO doesn't detect anything in the fake images. The test will pass with a warning.

### ROS2 environment not sourced

Make sure to run:
```bash
source /ros_entrypoint.sh
```

## Adding New Tests

1. Create test script in `scripts/`
2. Add fake hardware simulator if needed
3. Update `run_tests.sh` with new test case
4. Document expected behavior in this README

## Exit Codes

- `0` - All tests passed
- `1` - One or more tests failed

## Example Session

```bash
$ ./run_tests.sh vision

========================================
RecycleBot Hardware-Free Test Suite
========================================

ROS2 jazzy detected
Test Mode: vision

=== Running Vision System Test ===

Starting fake RGBD publisher...
Starting vision detector...
Running automated vision test...

[INFO] Vision Tester initialized
[INFO] Waiting for /capture_detections service...
[INFO] Service available!
[INFO] Triggering detection...
[INFO] Detection result: True - Added 2 potential new detections
[INFO] Received 2 detections
[INFO]   Detection 1: flower_pot (confidence: 93.07%) at (575, 274)
[INFO]   Detection 2: box_pp (confidence: 62.51%) at (401, 599)
[INFO] Total objects detected: 2

============================================================
TEST PASSED: Vision system working correctly!
============================================================

✓ Vision test PASSED

========================================
```

## CI/CD Integration

This test suite can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions
- name: Run RecycleBot Tests
  run: |
    docker exec recyclebot-dev-1 bash -c "source /ros_entrypoint.sh && /home/joao/test_suite/run_tests.sh full"
```

## Performance Benchmarks

Typical test execution times (on AMD64):
- Vision test: ~15-20 seconds
- Robot test: ~8-10 seconds
- Gripper test: ~8-10 seconds
- Full test suite: ~40-50 seconds

Note: ARM64 (Apple Silicon) may be 2-3x slower due to emulation.
