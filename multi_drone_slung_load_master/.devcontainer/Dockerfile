FROM osrf/ros:humble-desktop

##############
### SYSTEM ###
##############
# Create working directory
WORKDIR /workspace_docker_build 

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages and install others
RUN sudo apt-get update && \
    sudo apt upgrade -y && \
    sudo apt install -y git && \
    sudo apt-get install -y nano tmux

# Ensure root user
USER root

# Set noninteractive mode for any prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    unzip \
    gcc-arm-none-eabi \
    graphviz \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# gcc-arm-none-eabi - for PX4 firmware build
# graphviz - for rqt_graph 
# iproute2 and iputils - for ip command and ping


##############
### GITHUB ###
##############

# Install gh for authentication
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | tee /usr/share/keyrings/githubcli-archive-keyring.gpg > /dev/null \
    && chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
    && apt update \
    && apt install -y gh

# Copy the token file into the container
# COPY .auth/token_gh.txt /root/token_gh.txt

# # Authenticate GitHub using the token from the file
# RUN cat /root/token_gh.txt | gh auth login --with-token && rm /root/token_gh.txt

# # Setup git credentials
# RUN git config --global user.name "" \
#     && git config --global user.email ""



##############
### QGC ##
##############

# Install necessary packages and remove modemmanager
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    && apt-get remove -y modemmanager \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies required to run AppImages (adjust as needed)
RUN apt-get update && apt-get install -y \
    libfuse2 \
    fuse \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Download the AppImage
#RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O QGroundControl.AppImage
RUN wget https://github.com/mavlink/qgroundcontrol/releases/download/v5.0.6/QGroundControl-x86_64.AppImage -O QGroundControl.AppImage

# Make it executable
RUN chmod +x QGroundControl.AppImage


##############
### ROS2 ###
##############

# Source the ROS setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /multi_drone_slung_load_master/ws_ros2/install/setup.bash" >> ~/.bashrc

RUN echo "export GZ_VERSION=garden" >> ~/.bashrc

# Install cycloneDDS
### Install and configure cyclonedds - install required for building gz bridge

RUN apt-get update && apt-get install -y curl gnupg lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    echo >> ~/.bashrc && \
    echo '# Path to library for cyclonedds' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo >> ~/.bashrc


##############
### PYTHON ###
##############
# Install Python dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
    #python3-venv \

# Set Python 3 as the default python
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1


##############
### MOVE EARLIER ###
##############
# Install Docker CLI
RUN apt-get update && \
    apt-get install -y ca-certificates curl gnupg lsb-release && \
    mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
    gpg --dearmor -o /etc/apt/keyrings/docker.gpg && \
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
      https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | \
      tee /etc/apt/sources.list.d/docker.list > /dev/null && \
    apt-get update && \
    apt-get install -y docker-ce-cli && \
    rm -rf /var/lib/apt/lists/*

# Gazebo transport msgs
# RUN apt-get update && \
#     apt-get install -y python3-gz-transport13 python3-gz-msgs10
#     && rm -rf /var/lib/apt/lists/*

##############
### ROS2 2 ###
##############

### ZENOH
RUN echo '# ROS2 RTPS network' >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc

RUN echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

# For physical testing: RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc, 0 for simulation


# Gazebo 
RUN echo 'export GZ_SIM_RESOURCE_PATH=/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc

##############
### ALIASES ##
##############

# Add aliases to .bashrc
RUN echo 'alias qgc="bash /multi_drone_slung_load_master/run_qgc.sh"' >> /root/.bashrc

RUN echo 'alias ws_build="colcon build --base-paths /multi_drone_slung_load_master/ws_ros2 --build-base /multi_drone_slung_load_master/ws_ros2/build --install-base /multi_drone_slung_load_master/ws_ros2/install"' >> ~/.bashrc

RUN echo 'alias mk_px4="cd /multi_drone_slung_load_master/repos/PX4-Autopilot && make px4_sitl_default gz_x500"' >> ~/.bashrc

RUN echo 'alias edit_bashrc="nano /root/.bashrc"' >> ~/.bashrc

# Rosdep update
#RUN rosdep update