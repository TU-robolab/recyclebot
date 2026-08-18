# RecycleBot

CV based pick-and-place system for trash sorting using ROS2 Jazzy inside a containerized environment, developed and maintained by Elvis Borges @**Triku Studio**.

---

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Setup & Run](#setup--run)
- [Run Modes](#run-modes)
- [Robot Arms (UR16e / UR3e)](#robot-arms-ur16e--ur3e)
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
- Motion control of **UR robots** — UR16e (default) and UR3e, selected with `ur_type:=`
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

---

## Run Modes

After workspace build (`colcon build` + `source install/setup.bash`), choose one mode:

All launch files below accept `ur_type:=ur16e` (default) or `ur_type:=ur3e`.
See [Robot Arms](#robot-arms-ur16e--ur3e) before running anything on a UR3e.

### Full System (Real UR + RealSense + Gripper)

```bash
ros2 launch recycle_bot rec_bot.launch.py
ros2 launch recycle_bot rec_bot.launch.py ur_type:=ur3e
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

## Robot Arms (UR16e / UR3e)

The stack supports more than one UR arm. A single `ur_type` launch argument
selects the MoveIt configuration, the application config, and the reach envelope.

```bash
ros2 launch recycle_bot rec_bot.launch.py            # UR16e (default)
ros2 launch recycle_bot rec_bot.launch.py ur_type:=ur3e
ros2 launch recycle_bot rec_bot_fake.launch.py ur_type:=ur3e   # simulation
```

`UR_TYPE` in the environment sets the default if you do not pass the argument.

| | UR16e | UR3e |
|---|---|---|
| Reach | 900 mm | 500 mm |
| Payload | 16 kg | 3 kg |
| Status | **Production** — measured and running | **Bring-up** — config is placeholder |

### What is per-arm

```
packages/recycle_bot/config/<ur_type>/
  calibration.yaml        camera->base TF, detection depth/confidence filters
  sorting_sequence.yaml   neutral pose, bin poses, label->bin routing
  cell.yaml               work-cell collision geometry (table, stand, camera)
  my_robot_calibration.yaml   teach-pendant kinematics (UR16e only; per robot)

packages/recycle_bot_moveit_config/config/<ur_type>/
  <ur_type>.urdf.xacro    flattened description + E-Pick tool offset
  <ur_type>.srdf          planning group "ur_arm" + self-collision pairs
  joint_limits.yaml       per-arm velocity limits
  pilz_cartesian_limits.yaml
  initial_positions.yaml
```

Anything not listed there (`moveit_cpp.yaml`, `ompl_planning.yaml`,
`kinematics.yaml`, `ros2_controllers.yaml`, `moveit_controllers.yaml`) is shared.

### Reach checking

`rec_bot_control` refuses to start if any configured pose lies outside the arm's
usable envelope (datasheet reach x 0.90), naming each offending pose:

```
RuntimeError: 3 configured pose(s) are outside the ur3e's reach envelope of 0.450 m:
    neutral_pose: 0.784 m  (over by 0.334 m)
    bins.general_waste: 0.755 m  (over by 0.305 m)
    ...
```

Detections outside the envelope are dropped at `vision_callback` rather than
queued. To bypass the check while jogging a partly measured cell:

```bash
ros2 launch recycle_bot rec_bot_fake.launch.py ur_type:=ur3e \
  --ros-args -p enforce_reach_check:=false
```

### Before running a UR3e on hardware

Every pose in `config/ur3e/` is a placeholder marked `TODO(ur3e-cell)`. They are
internally consistent and safe in simulation, but they are not a real cell.

1. **Export the UR3e's kinematic calibration.** See
   [Exporting a kinematic calibration](#exporting-a-kinematic-calibration) below.
   Until the file exists the launch files print a warning and fall back to
   nominal `ur_description` kinematics.
2. Measure the camera-to-base transform into `config/ur3e/calibration.yaml`, and
   retune `detection_filter.max_depth_m` for the new camera height.
3. Measure the table, camera stand, and camera into `config/ur3e/cell.yaml`.
4. Teach the neutral, under-camera, and bin poses into
   `config/ur3e/sorting_sequence.yaml`. Joint-space poses are preferred on a
   small arm — they bypass IK entirely.
5. Rebuild and verify:
   ```bash
   colcon build --packages-select recycle_bot recycle_bot_moveit_config test_suite
   source install/setup.bash
   python3 -m pytest src/test_suite/test/test_robot_profiles.py -v
   ros2 launch recycle_bot rec_bot_fake.launch.py ur_type:=ur3e   # simulate first
   ```

### Exporting a kinematic calibration

Every physical UR arm leaves the factory with per-unit deviations of up to about
a millimetre. `ur_robot_driver` needs those measured values or its forward
kinematics is wrong by a fixed offset — which shows up as picks that miss by a
consistent amount and reads like a camera calibration fault, not a kinematics
one. The file is unique to one serial number and must never be copied between
arms.

Run this with the UR3e powered on and reachable over the network. It only reads
from the controller; no program needs to be running and External Control does not
need to be enabled.

```bash
# Inside the container
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=<UR3E_IP> \
  target_filename:="$HOME/ur3e_calibration.yaml"
```

Then move it into the package and verify:

```bash
cp "$HOME/ur3e_calibration.yaml" \
   ~/ros2_ws/src/recycle_bot/config/ur3e/my_robot_calibration.yaml

colcon build --packages-select recycle_bot
source install/setup.bash
ros2 run recycle_bot check_calibration --ur-type ur3e
```

`check_calibration` confirms the file is well-formed and that its link lengths
actually belong to a UR3e. Cross-filing another arm's export is the easy mistake
here, and it is caught by name:

```
FAILED — 1 problem(s):
  - calibration deviates from ur3e nominal kinematics by 235.0 mm, well beyond
    the 5.0 mm tolerance.
  The link lengths match 'ur16e' instead (within 0.93 mm) — this looks like a
  ur16e calibration filed under ur3e
```

For reference, the measured UR16e in this repo sits within **0.93 mm** of
nominal, so anything past a few millimetres is a wrong or corrupt file rather
than a badly calibrated arm.

`test_robot_profiles.py` runs the same check on every arm that has a calibration
file, and skips the ones that do not.

### Adding another arm

Add a `RobotProfile` entry in
`packages/recycle_bot/recycle_bot/robot_profile.py`, then create the two
`config/<ur_type>/` directories above. No code changes are needed. Generate the
URDF with:

```bash
xacro /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro \
  ur_type:=<arm> name:=<arm> > <arm>.urdf.xacro
```

then re-apply the E-Pick tool offset — set the `flange-tool0` joint origin to
`xyz="0.150 0 0"`. This is a hand edit that URDF regeneration always drops, and
losing it shifts every pick by 150 mm.

---

## Subsystems

### Robot (UR)

1. Start External Control on teach pendant
2. Launch driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur16e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="$(ros2 pkg prefix recycle_bot)/share/recycle_bot/config/ur16e/my_robot_calibration.yaml" \
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

Config is per-arm; substitute your `ur_type` for `<ur_type>` below (see
[Robot Arms](#robot-arms-ur16e--ur3e)).

1. Export UR kinematic calibration from the teach pendant and save it as:
`packages/recycle_bot/config/<ur_type>/my_robot_calibration.yaml`
2. Measure camera-to-base transform and update:
`packages/recycle_bot/config/<ur_type>/calibration.yaml`
3. Verify TF chain:
```bash
ros2 run tf2_tools view_frames
```
4. Rebuild package after config changes:
```bash
cd ~/ros2_ws
colcon build --packages-select recycle_bot recycle_bot_moveit_config
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
| Robot Profiles | `python3 -m pytest test/test_robot_profiles.py` | 14 fast config checks, no hardware or ROS graph |
| Vision Workflow | `ros2 launch test_suite test_vision_workflow.launch.py` | 7 tests with fake camera |
| E2E Pipeline | `ros2 launch test_suite test_e2e_pipeline.launch.py` | 13 tests: vision → core → gripper → MoveIt |
| **Real Robot Motion** | `ros2 launch test_suite test_real_control_robot_motion.launch.py` | **4 tests: pick-place with real UR virtual robot** |
| Real Camera | `ros2 launch test_suite test_vision_real_camera.launch.py` | Vision tests with physical RealSense D415 (requires connected camera) |

### Quick Test

```bash
# Build test packages
colcon build --packages-select test_suite recycle_bot recycle_bot_moveit_config

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

The control node uses MoveIt planners configured in `packages/recycle_bot_moveit_config/config/moveit_cpp.yaml`:

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

All MoveIt config lives in `packages/recycle_bot_moveit_config/config/`. Files
marked *(per-arm)* live in a `<ur_type>/` subdirectory; the rest are shared:

| File | Purpose |
|------|---------|
| `moveit_cpp.yaml` | Planner pipelines (OMPL, Pilz, CHOMP), velocity/acceleration limits, plan request presets |
| `<ur_type>/joint_limits.yaml` | *(per-arm)* Per-joint position, velocity, and acceleration limits |
| `kinematics.yaml` | IK solver plugin and search parameters |
| `<ur_type>/pilz_cartesian_limits.yaml` | *(per-arm)* Max Cartesian velocity/acceleration for Pilz LIN/CIRC |
| `ros2_controllers.yaml` | Joint trajectory controller configuration |
| `moveit_controllers.yaml` | MoveIt controller manager mapping |
| `ompl_planning.yaml` | OMPL planner algorithm configs (RRTConnect) |
| `<ur_type>/initial_positions.yaml` | *(per-arm)* Default joint positions for startup |
| `<ur_type>/<ur_type>.urdf.xacro` | *(per-arm)* Flattened robot description + E-Pick tool offset |
| `<ur_type>/<ur_type>.srdf` | *(per-arm)* Planning group `ur_arm`, self-collision pairs |

Application config lives in `packages/recycle_bot/config/<ur_type>/`:

| File | Purpose |
|------|---------|
| `<ur_type>/calibration.yaml` | Camera-to-base static TF, detection filter thresholds (confidence, depth range) |
| `<ur_type>/sorting_sequence.yaml` | Neutral pose, bin target poses, label→bin routing, approach height, grasped object size |
| `<ur_type>/cell.yaml` | Work-cell collision geometry (table, camera stand, camera box) |
| `<ur_type>/my_robot_calibration.yaml` | UR kinematics calibration from teach pendant (unique per robot) |

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
