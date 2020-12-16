# Ensure the base image is Ubuntu 20.04
FROM nvidia/cudagl:11.0-devel-ubuntu20.04

# Set up time zone
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install necessary software
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get --no-install-recommends install -y \
    bash \
    build-essential \
    collectl \
    curl \
    git \
    gnupg2 \
    lsb-release \
    nano \
    python3 \
    python3-dev \
    python3-pip \
    sudo \
    unzip \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Create root dir for the benchmark
RUN mkdir -p /root/simbenchmark

# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Install pip packages incl. pybullet
COPY ./requirements.txt /root/simbenchmark/
RUN python3 -m pip install -r /root/simbenchmark/requirements.txt

# Install ROS noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get --no-install-recommends install -y \
    gazebo11 \
    libgazebo11-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-ros-core \
 && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN rosdep init; rosdep update

# Install WeBots dependencies from https://github.com/cyberbotics/webots/wiki/Docker#get-and-compile-webots
# and https://github.com/cyberbotics/webots/blob/master/scripts/install/linux_runtime_dependencies.sh
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get --no-install-recommends install -y \
    execstack \
    ffmpeg \
    g++ \
    libavcodec-extra \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    libnss3 \
    libusb-dev \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcomposite-dev \
    libxkbcommon-x11-dev \
    libxtst6 \
    lsb-release \
    make \
    mesa-utils \
    xorg-dev \
    xpra \
    xserver-xorg-video-dummy \
    xvfb \
 && rm -rf /var/lib/apt/lists/*

# Install WeBots
RUN wget --no-verbose https://github.com/cyberbotics/webots/releases/download/R2020b-rev1/webots_2020b-rev1_amd64.deb \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y ./webots_2020b-rev1_amd64.deb \
    && rm -rf /var/lib/apt/lists/* \
    && rm ./webots_2020b-rev1_amd64.deb

# Install MuJoCo
# Missing dependencies according to https://github.com/ethz-asl/reinmav-gym/issues/35
# and https://github.com/openai/mujoco-py/issues/147
# RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get --no-install-recommends install -y \
#     libosmesa6-dev \
#     patchelf \
#  && rm -rf /var/lib/apt/lists/*
# RUN mkdir -p /root/.mujoco
# RUN wget https://www.roboti.us/download/mujoco200_linux.zip \
#     && unzip ./mujoco200_linux.zip -d /root/.mujoco \
#     && mv /root/.mujoco/mujoco200_linux /root/.mujoco/mujoco200 \
#     && rm ./mujoco200_linux.zip
# RUN python3 -m pip install mujoco-py

# Copy ROS workspace into docker image
COPY ./environments/Gazebo/catkin_ws/ /root/simbenchmark/environments/Gazebo/catkin_ws/

# Set Gazebo catkin_ws path
ENV ROS_WS=/root/simbenchmark/environments/Gazebo/catkin_ws

# Install ros/catkin dependencies
RUN apt-get update \
    && rosdep install -y -r --from-paths $ROS_WS/src --ignore-src --rosdistro noetic || : \
    && rm -rf /var/lib/apt/lists/*

# Build the Gazebo catkin_ws
ENV ROS_BLACKLIST_PACKAGES="robotiq_2f_140_gripper_visualization robotiq_2f_85_gripper_visualization robotiq_2f_c2_gripper_visualization robotiq_2f_gripper_action_server robotiq_3f_gripper_articulated_gazebo_plugins"
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash; \
    catkin config --init \
                  --workspace $ROS_WS \
                  --extend /opt/ros/$ROS_DISTRO \
                  --blacklist $ROS_BLACKLIST_PACKAGES"]

# Source and build the ROS workspace
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash; \
    catkin build --workspace $ROS_WS --summarize --no-status --force-color"]

# Setup environment
RUN echo "export AUDIODEV=null" >> /root/.bashrc; \
    echo "export DISPLAY=:99" >> /root/.bashrc; \
    echo "export GAZEBO_MODEL_PATH=/root/simbenchmark/environments/Gazebo/environment/models" >> /root/.bashrc; \
    echo "export GAZEBO_RESOURCE_PATH=/root/simbenchmark/environments/Gazebo/environment" >> /root/.bashrc; \
    echo "export LIBGL_ALWAYS_SOFTWARE=true" >> /root/.bashrc; \
    echo "export PYTHONIOENCODING=UTF-8" >> /root/.bashrc \
    echo "export PYTHONPATH=$PYTHONPATH:/usr/local/webots/lib/controller/python38 #(or python37)" >> /root/.bashrc; \
    echo "export WEBOTS_HOME=/usr/local/webots" >> /root/.bashrc; \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/webots/lib/controller:/usr/local/webots/lib/webots:/root/.mujoco/mujoco200/bin" >> /root/.bashrc; \
    echo "source /root/simbenchmark/environments/Gazebo/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copy full workspace into docker
COPY . /root/simbenchmark

# Switch working directory
WORKDIR /root/simbenchmark

# Run the benchmark
ENTRYPOINT ["/bin/bash", "-c", "-i", "./run_all_benchmarks.sh"]
