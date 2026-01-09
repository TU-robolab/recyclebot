# Docker Quickstart (macOS with x86_64 Emulation)

This guide covers building and running the RecycleBot container on macOS using platform emulation.

## Prerequisites

- Docker Desktop for Mac
- Enable Rosetta emulation in Docker Desktop settings (recommended for better performance)

## Build

```bash
cd ~/.../recyclebot

# Configure environment
./export_env.sh

# Build using docker-compose.mac.yml
docker compose --env-file .env -f docker-compose.mac.yml build
```

## Start Container

**Option 1: Using docker-compose (Recommended)**
```bash
docker compose --env-file .env -f docker-compose.mac.yml up -d
```

**Option 2: Manual docker run (Backup method)**
```bash
docker run -d \
  --name recyclebot-mac-1 \
  --platform linux/amd64 \
  -v "$(pwd)/packages:/home/${USER}/ros2_ws/src:rw" \
  -v "$(pwd)/test_suite:/home/${USER}/test_suite:rw" \
  ros2_mac \
  bash -c "while true; do sleep 3600; done"
```

## Access Container

Use the entrypoint script to automatically source ROS2 and the workspace:

```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
```

This runs the entrypoint which sources:
- `/opt/ros/${ROS_DISTRO}/setup.bash`
- `${ROS2_WS}/install/setup.bash`

## Build ROS2 Packages (inside container)

Build both the recycle_bot packages and test_suite:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Testing with Fake Publishers

The mac configuration includes the test_suite directory for testing without hardware.

### Testing Vision Node with Fake Camera

**Terminal 1 - Start fake RGBD publisher:**

Option A: Run as ROS2 node (recommended):
```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
ros2 run test_suite fake_rgbd_publisher
```

Option B: Run as bare Python script:
```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
cd ~/ros2_ws/src/test_suite/scripts
python3 fake_rgbd_publisher.py
```

**Terminal 2 - Run vision node:**
```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
ros2 run recycle_bot rec_bot_vision
```

**Terminal 3 - Trigger detection:**
```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
ros2 service call /capture_detections std_srvs/srv/Trigger
```

**Monitor detections:**
```bash
ros2 topic echo /object_detections
```

### Testing Robot Control with Fake Joint States

**Terminal 1 - Start fake joint state publisher:**

Option A: Run as ROS2 node (recommended):
```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
ros2 run test_suite fake_joint_state_publisher
```

Option B: Run as bare Python script:
```bash
docker exec -it recyclebot-mac-1 /ros_entrypoint.sh bash
cd ~/ros2_ws/src/test_suite/scripts
python3 fake_joint_state_publisher.py
```

**Monitor joint states:**
```bash
ros2 topic echo /joint_states
```

## Stop and Remove Container

**Using docker-compose:**
```bash
docker compose -f docker-compose.mac.yml down
```

**Manual method:**
```bash
docker stop recyclebot-mac-1
docker rm recyclebot-mac-1
```

## Troubleshooting

### I/O Errors during build
Common with macOS emulation. Retry the build or reduce parallel jobs:
```bash
DOCKER_DEFAULT_PLATFORM=linux/amd64 docker compose \
  --env-file .env \
  -f docker-compose.base.yml \
  -f docker-compose.dev.yml \
  build --parallel 1
```

### Permission denied on build artifacts
Inside container:
```bash
chown -R $(whoami):$(whoami) ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log
```
