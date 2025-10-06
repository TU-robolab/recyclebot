# RecycleBot

CV based pick-and-place system for trash sorting using ROS2 Jazzy inside a containerized environment, developed and maintained by Elvis Borges @**Triku Studio**.  

---

## Table of Contents 

- [RecycleBot](#recyclebot)
  * [Table of Contents](#table-of-contents)
  * [Overview](#overview)
  * [System Requirements](#system-requirements)
  * [Setup & Run](#setup---run)
    + [1 - Configure base Linux system](#1---configure-base-linux-system)
    + [2 - Clone & Configure](#2---clone---configure)
    + [3 - Docker Build & Launch](#3---docker-build---launch)
    + [4 - Access Container](#4---access-container)
  * [Subsystems](#subsystems)
    + [Robot (UR)](#robot--ur-)
    + [Camera (realsense)](#camera--realsense-)
    + [Gripper (Robotiq E-Pick)](#gripper--robotiq-e-pick-)
    + [Troubleshooting/Common Issues](#troubleshooting-common-issues)
      - [Docker socket/permissions](#docker-socket-permissions)
      - [reset build cache due to build issues](#reset-build-cache-due-to-build-issues)
      - [Wayland GUI access](#wayland-gui-access)
      - [Maintenance (SAFE)](#maintenance--safe-)
      - [Maintenance (DESTRUCTIVE)](#maintenance--destructive-)
  * [Design Notes](#design-notes)
    + [**Deployment vs. Development Containers**](#--deployment-vs-development-containers--)
    + [Package Overview](#package-overview)
      - [Gripper](#gripper)
        * [1. `grip_interface`](#1--grip-interface-)
        * [2. `grip_command_package`](#2--grip-command-package-)
        * [3. `serial`](#3--serial-)
  * [Appendix](#appendix)
    + [Gripper WoW](#gripper-wow)
    + [VNC setup](#vnc-setup)
    + [Convert between euler quaternions](#convert-between-euler-quaternions)
    + [Typical WoW](#typical-wow)
  * [License](#license)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

---

## Overview

**recycleBot** provides a portable ROS2 Jazzy workspace configured for simulation, vision, and robotics hardware operation, integrating:
- motion control of **UR robots (UR16e / UR10e)**,
- **serial-controlled gripper** interface, and
- **Intel Realsense** vision node for perception.  

It uses **Docker Compose**, so every setup is version-consistent, portable across systems, and easy to transition from development to deployment.

---

## System Requirements
- **Ubuntu 24.04 LTS** (or compatible)  
- **Docker Engine ≥ 24** and **docker-compose plugin**  
- **git-lfs** for large files  
- (optional) **Real-time kernel** for UR control

---

## Setup & Run 

### 1 - Configure base Linux system

- **Ubuntu 24.04 LTS**  
  ```bash
  hostnamectl
  sudo apt update && sudo apt full-upgrade
  sudo do-release-upgrade
  # finalize by rebooting:
  sudo reboot
  ```
-  **Real time kernel on Ubuntu (needed for UR control)**
  ```bash
  # attach account to canonical to enable pro ubuntu mode
  sudo pro attach
  # update your dependencies
  sudo apt update && sudo apt install ubuntu-advantage-tools
  # enable real time kernel mode 
  sudo pro enable realtime-kernel

  # reboot the system
  reboot
  ``` 
- **Docker**
   - install using the updated [docker installation page](https://docs.docker.com/engine/install/ubuntu/#prerequisites) WoW
   - ensure docker is given access priviledges:
      1. add the `docker` group (if it doesn't already exist):
        ```
         sudo groupadd docker
        ```
      2. add the connected user "$USER" to the `docker` group. (change username  if you do not want to use your current user):
    
        ```
         sudo gpasswd -a $USER docker
        ```
    
      3. run  `newgrp docker` or log out/in to activate the changes to groups.
    
      4.  use the following to check if you can run Docker without `sudo`.
    
        ```
         docker run hello-world
        ```
    
      5.  run this in your host system terminal to allow Docker to Use Wayland in your host system (in case your ubuntu uses wayland instead of X11 for GUI forwarding to container)
      ```
        xhost +si:localuser:$USER
      ```

---

### 2 - Clone & Configure

```bash
cd ~
git clone https://github.com/TU-robolab/recyclebot.git
cd recyclebot
git lfs pull

# configure your local environment variables using the export_env script
./export_env.sh

# allow Docker X11 display access
xhost +si:localuser:root
xhost +local:ur16e
xhost +local:root
```

---

### 3 - Docker Build & Launch

```bash
docker compose --env-file .env   -f docker-compose.base.yml -f docker-compose.dev.yml build
# Optional clean build
# docker compose ... build --no-cache

docker compose --env-file .env   -f docker-compose.base.yml -f docker-compose.dev.yml up -d
```

---

### 4 - Access Container

1. Open the dev Container
```bash
  docker exec -it recyclebot-dev-1 bash
	source /ros_entrypoint.sh
```
2. Source and build ROS workspace (inside the container):
```bash
colcon build --cmake-clean-first
source install/setup.bash
```   

---

## Subsystems 

### Robot (UR)

1.	Start External Control on pendant.
2.	Launch driver 
```bash
colcon build --cmake-clean-first
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur16e robot_ip:=192.168.1.102 kinematics_params_file:="/home/ur16e/ros2_ws/src/recycle_bot/my_robot_calibration.yaml" launch_rviz:=false
# launch smoke demo to move/test robot 
ros2 launch recycle_bot rec_bot_smoke.launch.py
```
3. 	Verify topic/service is correctly setup.

---

### Camera (realsense)

1.	Launch camera 
```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
2. 	Verify topic/service is correctly setup:
```bash
ros2 run rviz2 rviz2
# or use
realsense-viewer
```

---

### Gripper (Robotiq E-Pick)

1.	Launch the node:
```bash
ros2 launch grip_command_package master.launch.py debug:=true
```
2. Grip or release with:
```bash
    ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'grip'}"

    ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'release'}"
```
3. Echo topic /object_detection/status
```
    ros2 topic echo /object_detection/status
```

---

### Troubleshooting/Common Issues

#### Docker socket/permissions

* permission denied while trying to connect to the Docker daemon socket
  
```bash
sudo systemctl status docker
sudo systemctl start docker # if not started , if so , do sudo systemctl restart docker
sudo usermod -aG docker $USER && newgrp docker
docker run hello-world
ls -l /var/run/docker.sock   # should be group 'docker'
```
* check docker daemon socket:
```bash
ls -l /var/run/docker.sock

# output should be
$ srw-rw---- 1 root docker ...

``` 

---

#### reset build cache due to build issues

* for build problems in the container, resets the container to a clean state by deleting the `build_cache` volume:
```
docker volume rm build_cache
```

---

#### Wayland GUI access

```bash
xhost +si:localuser:$USER
xhost +local:root
```

---

#### Maintenance (SAFE)

```bash
# unused data
docker system prune -f
docker builder prune -f
```

---

#### Maintenance (DESTRUCTIVE)

```bash
# remove all images
docker system df
docker rmi $(docker images -q) --force
docker builder prune -a --force
docker rm -f $(docker ps -aq)
```

---

## Design Notes

* there are two main types of containers: **development containers** and **deployment containers**:
- We create an image in two steps:
  - **base** - **ROS & basic packages are installed** (ROS2 jazzy - supported until 2029).
  - **dev - packages used by recyclebot for vision, sim, and control **

You will find different kinds of files in this repository.  

- **`Dockerfile`**: instructions to build container, with any necessary tools and dependencies
- `docker-compose*.yml`  configuration to build, run and combine the images built using `Dockerfile`.
- `devcontainer.json`
  - builds services and provides the configuration vscode eventually uses (using the `*.yml files*`).
  - Mounts local source code  to the container's ROS workspace directory.
- `apt-**-packages` - contains list of packages installed in each phase of the docker image build

---

### **Deployment vs. Development Containers**

- **Deployment (prod) Containers** :
  - Everything (ROS, your configurations, workspace, etc.) is set up in the `Dockerfile`.
  - No bind mounts or local directories are used; the container runs in isolation.
- **Development (dev) Containers**:
  - Used for active prototyping
  - prioritizes  flexibility to edit files, build, and experiment without frequent image rebuilds via:
    - **build-cache** to the `.bashrc` or build artifacts in the container are stored in the **build_cache** volume, so you don’t lose progress when the container shuts down.
    - **git based development** - syncing code to container is simple by updating git repositories and using the bind mount, and can happen in your local computers/setup
      - if you want to **debug locally in container**, you can use the **VScode dev containers plugin directly**
        - this allows you to run/develop projects inside the container environment directly in VS Code

---

### Package Overview

#### Gripper 
*  this has 3 main components:

##### 1. `grip_interface`

A simple and clean ROS 2 service definition for triggering gripper actions.

* **Service Definition**

```
string action
---
bool success
string message
```

- `action`: Command string to be interpreted by the gripper (e.g. "grip", "release").
- `success`: Indicates whether the command was executed successfully.
- `message`: Human-readable status or error description.


##### 2. `grip_command_package`

This package is the core control logic for operating the gripper via a serial interface. It includes both a ROS 2 node and a launch file, integrating the hardware interface (remote_serial) and the UR robot driver for full system startup.

*  **Key Components**
Implements the grip_interface service under `/gripper_action`
Publishes serial commands via `/serial/com1/inject/output`
Subscribes to `/serial/com1/inspect/input` to receive inspection data
Publishes object detection results on `/object_detection/status`
Periodically sends sensor queries using a timer
Has a debug parameter (set via launch file) to control verbose logging
Initializes the gripper on startup via a custom serial command

* Launch File: `master_launch.py`	
	- UR Robot Driver
	Launches ur_robot_driver with tool communication enabled, using `/tmp/ttyUR` as the tool port.	
	- Serial Interface (remote_serial)
	Starts the `remote_serial_standalone` node with full configuration for baud rate, data bits, stop bits, flow control, etc.
	- Gripper Node
	Launches `gripper_node` from `grip_command_package`.

This launch setup ensures the entire pipeline — robot arm, serial access, and gripper control — comes online seamlessly.

##### 3. `serial`

This package integrates the powerful and flexible [`remote_serial`](https://github.com/openvmp/serial) implementation.

- Full ROS 2 integration for serial I/O
- Handles serial line **saturation gracefully**
- Prevents data loss and blocking behavior
- Exposes serial ports as ROS 2 interfaces for introspection and debugging

---

## Appendix

### Gripper WoW

* Start the robot:
* 	Start robot over teach pendant and initiialize it so its at least in robot idle mode

* Start the system:
```bash
ros2 launch grip_command_package master.launch.py
```
This will:
- Start the UR driver
- Connect to the gripper over serial
- Bring up the service interface
- Publishes the gripper status on `/object_detection/status`

* Send a Command
You can call the service like this:
```bash
ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'grip'}"
```
or
```bash
ros2 service call /gripper_action grip_interface/srv/GripCommand "{action: 'release'}"
```

* Echo the gripper detection status:
```bash
ros2 topic echo /object_detection/status
```

---

### VNC setup

* `vnc setup cv@cv-NUC8i3BEH:~/recyclebot$ x0vncserver -localhost no -passwordfile ~/.vnc/passwd -display $DISPLAY`
### Convert between euler quaternions
* https://www.andre-gaschler.com/rotationconverter/

---

### Typical WoW
1. typical build of container
   * you should see something like: 

     ![image-20250119211628872](resources/image-20250119211628872.png)
2. run `docker ps` to see final container:

   ![image-20250119211726811](resources/image-20250119211726811.png)

3. connect to container once built:

   ```bash
   docker exec -it recyclebot-dev-1 /bin/bash
   ```

   * you should be able to ls/ run basic ROS commands:

     ![image-20250119211846522](resources/image-20250119211846522.png)

     ![image-20250119211917016](resources/image-20250119211917016.png)

---

## License 

This repository is distributed under the MIT License unless otherwise stated.

---
Maintained by Triku Studio
© 2025 Triku Studio — All Rights Reserved.

