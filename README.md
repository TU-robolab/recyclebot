# recyclebot


For recyclebot, the  devenv workflow  uses Docker for ROS. This ensures that code runs in an isolated environment on the computer at hand, ensuring portability and version consistency between devices.

[TOC]



## Design of Container Structure

* there are two main types of containers: **development containers** and **deployment containers**:

------

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

------

### **Common issues**

**Resetting the Build Cache due to build issues**:

- for build problems in the container, resets the container to a clean state by deleting the `build_cache` volume:

  ```bash
  docker volume rm build_cache
  ```


**permission denied while trying to connect to the Docker daemon socket**

* ensure docker daemon is running:
  ```bash
  sudo systemctl status docker
  sudo systemctl start docker # if not started , if so , do sudo systemctl restart docker
  sudo systemctl enable docker # ensure it starts in system init
  ```

* Verify docker is installed correctly:

  ```bash
  docker --version
  docker run hello-world
  ```

* check docker daemon socket:

  ```bash
  ls -l /var/run/docker.sock
  
  # output should be
  $ srw-rw---- 1 root docker ...
  
  ```

  

_______

### Setting up the recyclebot environment

#### 1. **Prepare a Docker Image**

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

#### Prerequisites

*  Ubuntu linux (preferably 24.04.1 LTS) - use `hostnamectl` cmd to check the current version

  * ```bash
    # if not current version, upgrade with:
    sudo apt full-upgrade
    sudo apt install update-manager-core
    sudo do-release-upgrade
    
    # finalize by rebooting:
    sudo reboot
    ```

* Docker -  install using the updated [docker installation page](https://docs.docker.com/engine/install/ubuntu/#prerequisites) WoW

  * ensure docker is given access priviledges:

    - add the `docker` group (if it doesn't already exist):
    
      ```
       sudo groupadd docker
      ```
    
    - add the connected user "$USER" to the `docker` group. (change username  if you do not want to use your current user):
    
      ```
       sudo gpasswd -a $USER docker
      ```
    
    - run  `newgrp docker` or log out/in to activate the changes to groups.
    
    - use the following to check if you can run Docker without `sudo`.
    
      ```
       docker run hello-world
      ```
    
      

#### Run basic devcontainer setup with docker compose

1. clone the repository into your computer home: 

   ```bash
   cd ~
   git clone https://github.com/TU-robolab/recyclebot.git
   ```

2. configure your local environment variables using the export_env script

   ```bash
   ./export_env.sh
   ```

3. build your container
   ```bash
   docker compose -f docker-compose.base.yml -f docker-compose.dev.yml up -d
   ```

   * you should see something like: 

     ![image-20250119211628872](To Do/image-20250119211628872.png)

4. run `docker ps` to see final container:

   ![image-20250119211726811](To Do/image-20250119211726811.png)

5. connect to container once built:

   ```bash
   docker exec -it recyclebot-dev-1 /bin/bash
   ```

   * you should be able to ls/ run basic ROS commands:

     ![image-20250119211846522](resources/image-20250119211846522.png)

     ![image-20250119211917016](resources/image-20250119211917016.png)
