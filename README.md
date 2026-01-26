# RecycleBot

CV based pick-and-place system for trash sorting using ROS2 Jazzy inside a containerized environment, developed and maintained by Elvis Borges @**Triku Studio**.

---

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Setup & Run](#setup--run)
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

# Configure environment (source to export DOCKER_BUILDKIT to current shell)
source ./export_env.sh

# Allow Docker display access
xhost +si:localuser:root
xhost +local:root
```

### 4. Docker Build & Launch

```bash
# Build (with BuildKit for faster cached builds)
DOCKER_BUILDKIT=1 docker compose --env-file .env -f docker-compose.base.yml -f docker-compose.dev.yml build

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

**Expected output after build:**

![Build output](resources/image-20250119211628872.png)

**Container running:**

![Container running](resources/image-20250119211726811.png)

---

## Subsystems

### Robot (UR)

1. Start External Control on teach pendant
2. Launch driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur16e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/ur16e/ros2_ws/src/recycle_bot/my_robot_calibration.yaml" \
  launch_rviz:=false
```
3. Test with smoke demo:
```bash
ros2 launch recycle_bot rec_bot_smoke.launch.py
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
| Vision Workflow | `ros2 launch test_suite test_vision_workflow.launch.py` | 6 tests with fake camera |
| E2E Pipeline | `ros2 launch test_suite test_e2e_pipeline.launch.py` | 8 tests: vision → core → gripper |
| Real Camera | `ros2 launch test_suite test_vision_real_camera.launch.py` | Vision tests with RealSense D415 |

### Quick Test

```bash
# Build test packages
colcon build --packages-select test_suite recycle_bot

# Run E2E tests
ros2 launch test_suite test_e2e_pipeline.launch.py
```

**Test outputs:**
- Console report with pass/fail status
- `/tmp/vision_workflow_test_report.txt`
- `/tmp/rgbd_frame_*_combined.png` (RGB + depth visualization)

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

---

## Design Notes

### Container Architecture

- **Base image:** ROS2 Jazzy packages
- **Dev image:** RecycleBot packages for vision, sim, and control

### Development vs Deployment

| Aspect | Development | Deployment |
|--------|-------------|------------|
| Source code | Bind-mounted | Built into image |
| Build cache | Persistent volume | None |
| Flexibility | Edit without rebuild | Immutable |

### Key Files

- `Dockerfile` - Container build instructions
- `docker-compose*.yml` - Service configuration
- `devcontainer.json` - VS Code integration
- `apt-*-packages` - Package lists per build phase

---

## License

This repository is distributed under the MIT License unless otherwise stated.

---

Maintained by Triku Studio
© 2026 Triku Studio — All Rights Reserved.
