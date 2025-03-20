ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS ros2_base
ENV ROS_DISTRO=${ROS_DISTRO}

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# make default shell in base stage bash instead of sh
SHELL ["/bin/bash", "-c"]

# install basic packages and set up locales
RUN apt-get update -y && apt-get -y install --no-install-recommends locales gettext \
&& rm -rf /var/lib/apt/lists/*
RUN locale-gen en_GB.UTF-8; update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
ENV LANG en_GB.UTF-8

# install apt base packages
COPY apt-base-packages /tmp/apt-base-packages
RUN apt-get update && \
apt-get install -y $(cut -d# -f1 </tmp/apt-base-packages | envsubst) \
&& rm -rf /var/lib/apt/lists/* /tmp/apt-base-packages

## development stage
ARG ROS_DISTRO=jazzy
FROM ros2_base AS ros2_dev
ENV ROS_DISTRO=${ROS_DISTRO}

# disable terminal interaction for apt for dev stage
ENV DEBIAN_FRONTEND=noninteractive

# make default shell in dev stage bash instead of sh
SHELL ["/bin/bash", "-c"]

# add user with same UID and GID as your host system, replace if one exists with same UID
ARG USER_NAME=ros2
ARG USER_ID
ARG GROUP_ID
ARG GROUP_NAME

# process replacements if needed
RUN if getent group ${GROUP_ID} > /dev/null; then \
OLD_GROUP_NAME=$(getent group ${GROUP_ID} |  cut -d: -f1); \
groupmod --new-name ${USER_NAME} ${OLD_GROUP_NAME}; \
else \
groupadd --gid ${GROUP_ID} ${USER_NAME}; \
fi \
&& if getent passwd ${USER_ID} > /dev/null; then \
OLD_USER_NAME=$(getent passwd ${USER_ID} |  cut -d: -f1); \
usermod -l ${USER_NAME} ${OLD_USER_NAME}; \
usermod -d /home/${USER_NAME} -m ${USER_NAME}; \
else \
useradd -s /bin/bash --uid ${USER_ID} --gid ${GROUP_ID} -m ${USER_NAME}; \
fi \
&& apt-get update \
&& apt-get install -y sudo \
&& echo ${USER_NAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER_NAME} \
&& chmod 0440 /etc/sudoers.d/${USER_NAME}

# add user to video group to allow access to video sources
RUN usermod --append --groups video ${USER_NAME}

# setup user for rest of container
USER ${USER_NAME}

# create a custom ros2 overlay workspace for development
ENV ROS2_WS=/home/${USER_NAME}/ros2_ws
RUN mkdir -p ${ROS2_WS}/src && sudo chown -R ${USER_NAME}:${USER_NAME} ${ROS2_WS}


# install apt dev packages
COPY apt-dev-packages /tmp/apt-dev-packages
RUN sudo apt-get update \
&& sudo apt-get install -y $(cut -d# -f1 </tmp/apt-dev-packages | envsubst) \
&& sudo apt-get clean \
&& sudo rm -rf /var/lib/apt/lists/* /tmp/apt-dev-packages

# build the ROS2 workspace 
RUN cd ${ROS2_WS}/src \
&& sudo apt-get update \
&& . /opt/ros/${ROS_DISTRO}/setup.bash \ 
&& cd .. \
&& rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
&& sudo apt-get clean \
&& sudo rm -rf /var/lib/apt/lists/* \
&& colcon build --symlink-install 
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# create a virtual python environment inside Docker
COPY ./packages/recycle_bot/requirements.txt /tmp/requirements.txt
RUN python3 -m venv /venv
ENV PATH="/venv/bin:$PATH"

# install python dependencies inside the virtual environment
RUN cd ${ROS2_WS}/src \
&& python3 -m pip install -r /tmp/requirements.txt \
&& rm -rf /tmp/requirements.txt

# source ROS workspace automatically when new terminal is opened
RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" | sudo tee -a /home/${USER_NAME}/.bashrc \
    && echo ". ${ROS2_WS}/install/setup.bash" | sudo tee -a /home/${USER_NAME}/.bashrc \
    && echo "alias ros='ros2'" | sudo tee -a /home/${USER_NAME}/.bashrc

# set working directory for the container    
WORKDIR ${ROS2_WS}
    
# copy entrypoint script for ROS sourcing in the main terminal
COPY --chmod=755 ./ros_entrypoint.sh /ros_entrypoint.sh
# Source ROS in the main terminal, use exec to forward signals properly
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

