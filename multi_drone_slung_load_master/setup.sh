#!/bin/bash

# Function to deliniate setup stages
print_stage() {
  echo -e "\n\033[1;34m=====================\033[0m"
  echo -e "\033[1;34m$1\033[0m"
  echo -e "\033[1;34m=====================\033[0m"
}

##############
### UPDATE PX4 MSGS TO MATCH ###
##############
# Copy of px4_msgs
# rm -f /multi_drone_slung_load_master/ws_ros2/src/px4_msgs/msg/*.msg
# rm -f /multi_drone_slung_load_master/ws_ros2/src/px4_msgs/srv/*.srv
# cp /multi_drone_slung_load_master/repos/PX4-Autopilot/msg/*.msg /multi_drone_slung_load_master/ws_ros2/src/px4_msgs/msg/
# cp /multi_drone_slung_load_master/repos/PX4-Autopilot/msg/versioned/*.msg /multi_drone_slung_load_master/ws_ros2/src/px4_msgs/msg/
# cp /multi_drone_slung_load_master/repos/PX4-Autopilot/srv/*.srv /multi_drone_slung_load_master/ws_ros2/src/px4_msgs/srv/


##############
### PX4 1 ###
##############
# print_stage "PX4 setup script"

# bash /multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/setup/ubuntu.sh


##############
### PYTHON ###
##############
print_stage "Python setup"

# Install Python dependencies (note this has to happen here rather than in the Dockerfile as the above setup add required python package sources)
pip install -r /multi_drone_slung_load_master/requirements.txt

# Fix xacro
pip3 uninstall xacro -y
apt update
apt install --reinstall ros-humble-xacro

##############
### Micro-XRCE-DDS-Agent ###
##############
# print_stage "Micro-XRCE-DDS-Agent setup"

# cd /multi_drone_slung_load_master/repos/Micro-XRCE-DDS-Agent
# mkdir build
# cd build
# cmake ..
# make
# make install #sudo
# ldconfig /usr/local/lib/ #sudo


##############
### PX4 2 ###
##############
# print_stage "PX4 build" 

# cd /multi_drone_slung_load_master/repos/PX4-Autopilot/
# print_stage "PX4 setup"

# make px4_sitl gz_x500 # simulation


# NOTE: Sometimes have to run this script twice to properly set up
