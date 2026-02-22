# RecycleBot

CV based pick-and-place system for trash sorting using ROS2 Jazzy inside a containerized environment, developed and maintained by Elvis Borges @**Triku Studio**.

---

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Setup & Run](#setup--run)
- [Run Modes](#run-modes)
- [macOS Docker Quickstart](#macos-docker-quickstart)
- [Subsystems](#subsystems)
  - [Robot (UR)](#robot-ur)
  - [Camera (RealSense)](#camera-realsense)
  - [Gripper (Robotiq E-Pick)](#gripper-robotiq-e-pick)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Design Notes](#design-notes)
- [License](#license)

---

## Overview

**RecycleBot** provides a portable ROS2 Jazzy workspace configured for simulation, vision, and robotics hardware operation, integrating:
- Motion control of **UR robots (UR16e / UR10e)**
- **Serial-controlled gripper** interface (Robotiq E-Pick)
- **Intel RealSense D415** vision for perception
- **YOLO-based object detection** for recyclable classification

It uses **Docker Compose** for version-consistent, portable deployments.

---

## System Requirements

- **Ubuntu 24.04 LTS** (or compatible)
- **Docker Engine ≥ 24** and **docker-compose plugin**
- **git-lfs** for large files
- (optional) **Real-time kernel** for UR control

> **macOS users:** See [macOS Docker Quickstart](#macos-docker-quickstart) below.

---

## Setup & Run

### 1. Configure Base Linux System

```bash
# Ubuntu 24.04 LTS
sudo apt update && sudo apt full-upgrade

# Real-time kernel (needed for UR control)
sudo pro attach
sudo apt install ubuntu-advantage-tools
sudo pro enable realtime-kernel
sudo reboot
```

### 2. Install Docker

Install using the [official Docker installation guide](https://docs.docker.com/engine/install/ubuntu/).

```bash
# Add user to docker group
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker

# Verify
docker run hello-world

# Allow GUI access
xhost +si:localuser:$USER
```

### 3. Clone & Configure

```bash
cd ~
git clone https://github.com/TU-robolab/recyclebot.git
cd recyclebot
git lfs pull

# Generate .env file (user/group IDs, display, robot IP) and export BuildKit vars
source ./export_env.sh

# Allow Docker display access
xhost +si:localuser:root
xhost +local:root
```

### 4. Docker Build & Launch

```bash
# Build (with BuildKit for faster cached builds)
source ./export_env.sh  # gives BuildKit and Compose Bake
docker compose --env-file .env -f docker-compose.base.yml -f docker-compose.dev.yml build

# Launch
docker compose --env-file .env -f docker-compose.base.yml -f docker-compose.dev.yml up -d
```

**Note:** BuildKit caching significantly speeds up rebuilds by caching apt downloads. First build may take ~30 minutes, subsequent builds are much faster.

### 5. Access Container & Build Workspace

```bash
# Enter container
docker exec -it recyclebot-dev-1 bash
source /ros_entrypoint.sh

# Build ROS workspace
colcon build --cmake-clean-first
source install/setup.bash
```

### 6. Real Gripper Serial Device (Linux, Hardware Mode)

`grip_command_package` expects a serial device at `/tmp/ttyUR`.

1. Create a stable host symlink for the USB-serial adapter (example udev rule):
```bash
sudo tee /etc/udev/rules.d/99-ttyUR.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyUR", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyUR
```
2. Bind the device into Docker by enabling this line in `docker-compose.dev.yml`:
```yaml
- /dev/ttyUR:/tmp/ttyUR
```
3. Recreate the container:
```bash
docker compose --env-file .env -f docker-compose.base.yml -f docker-compose.dev.yml up -d --force-recreate
```

---

## Run Modes

After workspace build (`colcon build` + `source install/setup.bash`), choose one mode:

### Full System (Real UR + RealSense + Gripper)

```bash
ros2 launch recycle_bot rec_bot.launch.py
```

`rec_bot.launch.py` starts a launch gate while the operator enables External Control on the teach pendant. Continue immediately with:

```bash
ros2 service call /launch_gate std_srvs/srv/Trigger "{}"
```

Or wait for the launch timeout (default: 30s).

### Full System (Mock UR + Fake Camera + Mock Gripper)

```bash
ros2 launch recycle_bot rec_bot_fake.launch.py
```

### Smoke Test (Real UR, no camera)

```bash
ros2 launch recycle_bot rec_bot_smoke.launch.py
```

### Smoke Test (Mock UR, no camera)

```bash
ros2 launch recycle_bot rec_bot_smoke_fake.launch.py
```

---

## macOS Docker Quickstart

Build and run on macOS using x86_64 platform emulation.

**Prerequisites:** Docker Desktop for Mac with Rosetta emulation enabled (recommended).

### Build & Launch

```bash
cd ~/.../recyclebot
source ./export_env.sh  # generates .env + exports BuildKit vars
docker compose --env-file .env -f docker-compose.mac.yml build
docker compose --env-file .env -f docker-compose.mac.yml up -d
```

### Access Container & Build

```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash

# Build ROS2 packages
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Stop Container

```bash
docker compose --env-file .env -f docker-compose.mac.yml down
```

### macOS Troubleshooting

| Issue | Fix |
|-------|-----|
| I/O errors during build | Retry or reduce parallelism: `docker compose ... build --parallel 1` |
| Permission denied on build artifacts | Inside container: `chown -R $(whoami):$(whoami) ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log` |

---

## Subsystems

### Robot (UR)

1. Start External Control on teach pendant
2. Launch driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur16e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="$(ros2 pkg prefix recycle_bot)/share/recycle_bot/config/my_robot_calibration.yaml" \
  launch_rviz:=false
```
3. Test with smoke demo:
```bash
ros2 launch recycle_bot rec_bot_smoke.launch.py
```

**Configuration notes:**
- Robot IP (`192.168.1.102`) is set in `export_env.sh` and mapped via Docker's `extra_hosts` as hostname `ur`
- `my_robot_calibration.yaml` — UR kinematics calibration exported from the teach pendant (unique per robot)
- `calibration.yaml` — camera-to-base TF measured with the UR tool tip

### Calibration Workflow (Required for Real Hardware)

1. Export UR kinematic calibration from the teach pendant and save it as:
`packages/recycle_bot/config/my_robot_calibration.yaml`
2. Measure camera-to-base transform and update:
`packages/recycle_bot/config/calibration.yaml`
3. Verify TF chain:
```bash
ros2 run tf2_tools view_frames
```
4. Rebuild package after config changes:
```bash
cd ~/ros2_ws
colcon build --packages-select recycle_bot
source install/setup.bash
```

---

### Camera (RealSense)

Launch camera with RGBD enabled:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true
```

Verify with:
```bash
ros2 run rviz2 rviz2
# or
realsense-viewer
```

---

### Gripper (Robotiq E-Pick)

**Launch:**
```bash
ros2 launch grip_command_package master.launch.py debug:=true
```

This starts the UR driver, serial interface, and gripper node.

**Commands:**
```bash
# Grip
ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'grip'}"

# Release
ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'release'}"

# Check status
ros2 topic echo /object_detection/status
```

**Service interface:**
```
string action      # "grip" or "release"
---
bool success
string message
```

---

## Testing

The test suite provides automated validation of the vision and control pipeline.

See [test_suite/README.md](test_suite/README.md) for full documentation.

### Available Tests

| Test Suite | Command | Description |
|------------|---------|-------------|
| Vision Workflow | `ros2 launch test_suite test_vision_workflow.launch.py` | 7 tests with fake camera |
| E2E Pipeline | `ros2 launch test_suite test_e2e_pipeline.launch.py` | 13 tests: vision → core → gripper → MoveIt |
| **Real Robot Motion** | `ros2 launch test_suite test_real_control_robot_motion.launch.py` | **4 tests: pick-place with real UR virtual robot** |
| Real Camera | `ros2 launch test_suite test_vision_real_camera.launch.py` | Vision tests with physical RealSense D415 (requires connected camera) |

### Quick Test

```bash
# Build test packages
colcon build --packages-select test_suite recycle_bot

# Run E2E pipeline tests (vision → core → gripper → MoveIt)
ros2 launch test_suite test_e2e_pipeline.launch.py

# Run real robot motion tests (pick-place with UR virtual robot)
ros2 launch test_suite test_real_control_robot_motion.launch.py
```

**Test outputs:**
- Console report with pass/fail status
- `/tmp/vision_workflow_test_report.txt`
- `/tmp/e2e_pipeline_test_report.txt`
- `/tmp/rgbd_frame_*_combined.png` (RGB + depth visualization)

**Extracting reports from Docker:**
```bash
docker exec <container> cat /tmp/e2e_pipeline_test_report.txt
docker cp <container>:/tmp/e2e_pipeline_test_report.txt ./
```

### E2E Pipeline Test Details

The `test_e2e_pipeline` suite validates the complete system end-to-end: fake_rgbd → YOLO detection → 3D projection → MoveIt planning → mock gripper. 13 tests covering RGBD publishing, vision service, detection format, TF, joint states, gripper commands, full pipeline flow, and depth validation.

### Pick-Place Sequence

The control node executes a **10-step pick-place cycle** using **Pilz PTP/LIN planners** with collision object management:

1. Add table collision object
2. Move to approach pose (PTP)
3. Move to pick pose (LIN)
4. Close gripper
5. Retreat from pick (LIN)
6. Move to neutral (PTP)
7. Move to bin approach (PTP)
8. Move to bin place (LIN)
9. Open gripper
10. Return to neutral (PTP)

### Motion Planners

The control node uses MoveIt planners configured in `packages/ur16e_moveit_config/config/moveit_cpp.yaml`:

| Planner | Use in pick-place | Behavior |
|---------|-------------------|----------|
| **Pilz PTP** | Free-space moves (to approach, neutral, bin) | Point-to-point joint interpolation, deterministic |
| **Pilz LIN** | Cartesian moves (approach→pick, retreat, approach→place) | Straight-line Cartesian path |
| **OMPL RRTConnect** | Fallback / default | Sampling-based, non-deterministic |
| **CHOMP** | Available, not used in default sequence | Gradient-based trajectory optimization |

Velocity/acceleration scaling defaults to 0.3 for all planners.

---

## Troubleshooting

### Docker Permissions

```bash
sudo systemctl start docker
sudo usermod -aG docker $USER && newgrp docker
ls -l /var/run/docker.sock  # Should show group 'docker'
```

### Build Cache Issues

```bash
docker volume rm build_cache
```

### Wayland GUI Access

```bash
xhost +si:localuser:$USER
xhost +local:root
```

### Maintenance

```bash
# Safe cleanup
docker system prune -f
docker builder prune -f

# Full reset (destructive)
docker rmi $(docker images -q) --force
docker rm -f $(docker ps -aq)
```

### Serial Device Checks (Gripper)

```bash
# On host (Linux)
ls -l /dev/ttyUR

# In container
ls -l /tmp/ttyUR
ros2 topic list | grep /serial/com1
```

---

## Design Notes

### Container Architecture

Three Docker Compose layers stack via `extends`:

| File | Target | Purpose |
|------|--------|---------|
| `docker-compose.base.yml` | `ros2_base` | ROS 2 Jazzy desktop, display/GPU env vars |
| `docker-compose.dev.yml` | `ros2_dev` | User mapping, host networking, device access, bind-mounts for `packages/` and `test_suite/` |
| `docker-compose.mac.yml` | `ros2_mac` | macOS: `platform: linux/amd64`, SSH mount, simplified volumes |

Linux uses `base + dev`; macOS uses `mac` (which inherits both). `export_env.sh` generates the `.env` file consumed by all three (user/group IDs, display vars, robot IP `192.168.1.102`) and exports `DOCKER_BUILDKIT=1` / `COMPOSE_BAKE=true`.

### Development vs Deployment

| Aspect | Development | Deployment |
|--------|-------------|------------|
| Source code | Bind-mounted | Built into image |
| Build cache | Persistent volume | None |
| Flexibility | Edit without rebuild | Immutable |

### MoveIt Configuration

All MoveIt config lives in `packages/ur16e_moveit_config/config/`:

| File | Purpose |
|------|---------|
| `moveit_cpp.yaml` | Planner pipelines (OMPL, Pilz, CHOMP), velocity/acceleration limits, plan request presets |
| `joint_limits.yaml` | Per-joint position, velocity, and acceleration limits |
| `kinematics.yaml` | IK solver plugin and search parameters |
| `pilz_cartesian_limits.yaml` | Max Cartesian velocity/acceleration for Pilz LIN/CIRC |
| `ros2_controllers.yaml` | Joint trajectory controller configuration |
| `moveit_controllers.yaml` | MoveIt controller manager mapping |
| `ompl_planning.yaml` | OMPL planner algorithm configs (RRTConnect) |
| `initial_positions.yaml` | Default joint positions for startup |

Application config lives in `packages/recycle_bot/config/`:

| File | Purpose |
|------|---------|
| `calibration.yaml` | Camera-to-base static TF, detection filter thresholds (confidence, depth range) |
| `sorting_sequence.yaml` | Neutral pose, bin target poses, approach height, grasped object size |
| `my_robot_calibration.yaml` | UR kinematics calibration from teach pendant (unique per robot) |

### Key Files

- `Dockerfile` — Two-stage build (`ros2_base` → `ros2_dev`)
- `docker-compose*.yml` — Service configuration (see [Container Architecture](#container-architecture))
- `export_env.sh` — Generates `.env` and exports BuildKit vars
- `ros_entrypoint.sh` — Sources ROS 2, third-party deps (`/opt/third_party`), and workspace
- `devcontainer.json` — VS Code integration
- `apt-*-packages` — Package lists per build phase

---

## License

This repository is distributed under the MIT License unless otherwise stated.

---

Maintained by Triku Studio
© 2026 Triku Studio — All Rights Reserved.
