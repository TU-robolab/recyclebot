# RecycleBot Test Suite

Automated end-to-end integration tests for RecycleBot vision detection system without requiring hardware.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Overview](#overview)
- [Running Tests](#running-tests)
- [Test Reports](#test-reports)
- [Troubleshooting](#troubleshooting)
- [Package Structure](#package-structure)
- [Automation & CI/CD](#automation--cicd)

---

## Quick Start

### Run Tests (Automated)

```bash
# Inside Docker container
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
cd ~/ros2_ws/src/test_suite
bash scripts/run_vision_test.sh
```

### From Host Machine

```bash
docker exec recyclebot-mac-1 /ros_entrypoint.sh bash -c \
  "cd ~/ros2_ws/src/test_suite && bash scripts/run_vision_test.sh"
```

### View Report

```bash
# Inside container
cat /tmp/vision_workflow_test_report.txt

# From host
docker exec recyclebot-mac-1 cat /tmp/vision_workflow_test_report.txt

# Copy to host
docker cp recyclebot-mac-1:/tmp/vision_workflow_test_report.txt ./
```

---

## Overview

### What Gets Tested

The vision workflow tests validate the complete object detection pipeline:

```
Fake RGBD Camera → Vision Detector Node → Object Detections
     (simulator)        (YOLO model)         (ROS2 topics)
```

**Test Coverage:**
1. **Service Availability** — `/capture_detections` service is accessible
2. **Detection Triggering** — Service calls successfully trigger detections
3. **Topic Publishing** — Detections published to `/object_detections` topic
4. **Detection Format** — Output has valid structure, bounding boxes, and confidence scores
5. **RGBD Data Available** — RGBD frames publishing with correct resolution and encoding
6. **Depth Channel Validation** — Depth data matches D415 specs and quality standards

### Test Components

- **Fake RGBD Publisher** (`test_suite/fake_rgbd_publisher.py`) — Simulates RealSense D415 output (1280x720, bgr8 + 16UC1, depth 300–3000 mm with noise). See module docstring for full specs.
- **Vision Detector Node** (from `recycle_bot` package) — YOLO-based detection via `/capture_detections` service, publishes to `/object_detections`.
- **Test Suite** (`test/test_vision_workflow.py`) — pytest-based integration tests with report generation. See test docstrings for per-test details and performance metrics.

---

## Running Tests

### Method 1: Automated Script (Recommended)

```bash
# Inside Docker container
cd ~/ros2_ws/src/test_suite
bash scripts/run_vision_test.sh
```

### Method 2: Direct pytest

For more control or debugging:

```bash
# With verbose output
python3 -m pytest test/test_vision_workflow.py -v

# With live console output (see test progress)
python3 -m pytest test/test_vision_workflow.py -v -s

# Run specific test
python3 -m pytest test/test_vision_workflow.py::TestVisionWorkflow::test_04_detection_format -v

# Stop on first failure
python3 -m pytest test/test_vision_workflow.py -v -x
```

**Note:** When running manually, you must start the fake publisher and vision node separately:

```bash
# Terminal 1
ros2 run test_suite fake_rgbd_publisher

# Terminal 2
ros2 run recycle_bot rec_bot_vision

# Terminal 3
python3 -m pytest test/test_vision_workflow.py -v -s
```

### Method 3: From Host Machine

```bash
# Run complete test
docker exec recyclebot-mac-1 /ros_entrypoint.sh bash -c \
  "cd ~/ros2_ws/src/test_suite && bash scripts/run_vision_test.sh"

# View report
docker exec recyclebot-mac-1 cat /tmp/vision_workflow_test_report.txt
```

### Method 4: End-to-End Robot Motion Tests (NEW)

Complete pipeline testing from camera to robot motion using UR's virtual robot:

```bash
# Build first
colcon build --packages-select test_suite --symlink-install
source install/setup.bash

# Run full pipeline test
ros2 launch test_suite test_real_control_robot_motion.launch.py
```

**What it tests:**
```
fake_rgbd → rec_bot_vision (YOLO) → rec_bot_core (3D projection) → rec_bot_control (MoveIt) → UR Virtual Robot
```

**Test Coverage (4 tests):**
1. Single pick motion (detection → robot moves to pick position)
2. Full pick-place sequence (neutral → pick → grip → neutral → place → release → neutral)
3. Multiple detections queued sequentially
4. Joint states update during execution

**Key Features:**
- Uses UR's **virtual robot** (`use_mock_hardware:=true`) - no custom mocks needed
- Tests complete control logic with MoveIt planning
- Validates task queueing and sequential execution
- **Duration**: ~5 minutes (30s warmup + 4min tests)

**Components:**
- `fake_rgbd_publisher` - Synthetic camera
- `rec_bot_vision` - YOLO detection
- `rec_bot_core` - 3D projection
- `ur_robot_driver` - Virtual robot (use_mock_hardware=true)
- `rec_bot_control` - MoveIt + control logic
- `mock_gripper_service` - Gripper simulation

**Test file**: `test/test_real_control_robot_motion.py`
**Launch file**: `launch/test_real_control_robot_motion.launch.py`

---

## Test Reports

Reports are saved to `/tmp/` inside Docker. See [EXAMPLE_REPORT.md](EXAMPLE_REPORT.md) for sample output.

**Saved report file:** `/tmp/vision_workflow_test_report.txt` (quick summary + detailed detection info).

---

## Troubleshooting

### Quick Reference Table

| Issue | Quick Fix |
|-------|-----------|
| Tests not found | `colcon build --packages-select test_suite` |
| Service timeout | Check if vision node started: `ros2 node list` |
| No detections | Check fake publisher: `ros2 topic list` |
| Import errors | Source workspace: `source ~/ros2_ws/install/setup.bash` |
| Slow performance | Normal on CPU; use GPU for faster inference |

---

## Package Structure

```
test_suite/
├── README.md                        # This documentation
├── EXAMPLE_REPORT.md                # Sample test output
├── package.xml                      # ROS2 package definition
├── setup.py                         # Python package setup
├── scripts/
│   ├── run_vision_test.sh          # Automated test runner script
│   ├── fake_rgbd_publisher.py      # Fake camera simulator
│   └── fake_joint_state_publisher.py # Fake joint state publisher
├── test/
│   ├── test_vision_workflow.py          # Vision workflow tests (7 tests)
│   ├── test_e2e_pipeline.py             # E2E pipeline tests (13 tests)
│   └── test_real_control_robot_motion.py # Robot motion tests (4 tests)
├── test_suite/                           # Python package
│   ├── __init__.py
│   ├── fake_rgbd_publisher.py
│   └── fake_joint_state_publisher.py
└── launch/
    ├── test_vision_workflow.launch.py
    ├── test_e2e_pipeline.launch.py
    ├── test_real_control_robot_motion.launch.py
    └── test_vision_real_camera.launch.py
```

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select test_suite
source install/setup.bash
```

---

## Automation & CI/CD

### GitHub Actions (Automatic)

Tests automatically run on GitHub when:

**Triggers:**
- Push to `main` or `dev/**` branches
- Pull requests to `main` or `dev/**`
- Changes to vision-related files:
  - `packages/recycle_bot/recycle_bot/rec_bot_vision.py`
  - `packages/recycle_bot/**/*.py`
  - `test_suite/` files
  - Docker configs

**Workflow:** `.github/workflows/vision-tests.yml`

**Steps:**
1. Checkout code → Build Docker → Start container
2. Build ROS2 workspace
3. Run vision tests
4. Upload test report artifact
5. Display results in PR summary

**View Results:**
- Check "Actions" tab in GitHub
- View summary in PR checks
- Download test report artifact

**Manual Trigger:**
- Actions → Vision Tests → Run workflow

### Pre-commit Hooks (Local)

Run tests automatically before pushing changes.

**Setup:**
```bash
# Install pre-commit
pip install pre-commit

# Install hooks
cd /path/to/recyclebot
pre-commit install --hook-type pre-commit --hook-type pre-push
```

**What Runs:**

*On Commit:*
- Docker Compose validation (when modifying docker-compose*.yml or export_env.sh)
- Trailing whitespace removal
- YAML syntax validation
- Python syntax check (AST)
- Large file detection (>100MB)
- Merge conflict detection

*On Push (vision files):*
- Vision workflow tests (when modifying vision node or tests)

**Manual Run:**
```bash
# All hooks
pre-commit run --all-files

# Vision tests only
pre-commit run vision-tests --all-files

# Skip hooks (not recommended)
git push --no-verify
```

**Update Hooks:**
```bash
pre-commit autoupdate
```

### Testing Stages Summary

| Stage | When | What Runs | Time | Can Skip |
|-------|------|-----------|------|----------|
| **On Commit** | `git commit` | Docker config validation, syntax checks | <1s | `--no-verify` |
| **On Push** | `git push` | Vision workflow tests (if vision files changed) | ~15s | `--no-verify` |
| **GitHub Actions** | Push/PR to GitHub | Complete CI pipeline with Docker build | ~15min | No |
