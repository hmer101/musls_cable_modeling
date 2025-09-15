# musls_cable_modeling
Code and data accompanying ICRA 2026 submission: Dynamics Modeling of a Multi-UAV Slung Load System Using a Discrete-Link Cable Approach

## Structure


## How to use

1. Build and open the development container
2. Inside the development container: run bash setup.sh to install all python requirements


This repository contains the custom files and settings required to recreate the results seen in the paper. It also contains all simulated and real world data files and the corresponding code to process them to reproduce the graphics seen in the paper. 

- It does not contain the in-depth formation control implementation required to fly in simulation and the real world as this is not the main subject of this paper. To implement this, simply send waypoints to each of the three drones in the MUSLS using these instructions: https://docs.px4.io/main/en/ros2/offboard_control.


### Data files
- The Dockerfile contains the whole environment you need to run and process the collected datafiles including all dependencies and python packages.

- Contains all of the custom logfiles generated from simulated and real world flights.
- Contains all of the rosbag files from real world flights, containing all TFs of the load and drones respectively. To use the rosbag files, first extract the .zip folders containing the files, then play the .db3 files with rosbag play.
- process_results.py is the main file to follow to process custom logfiles and produce the corresponding graphs. It contains metadata about all custom datafiles stored in this repository.


### Simulation setup
- This repo contains the world and model files required to simulate the MUSLS as in the paper. To recreate the simulation, follow the instructions here (for Gazebo): https://docs.px4.io/main/en/simulation/.

To use the MUSLS files:
- Install PX4 autopilot inside the Docker container
- Copy all files from multi_drone_slung_load_master/repos/PX4-Autopilot into their corresponding locations in the installed PX4 instance
- Run the simulation with the multi_rigid_link_2m.sdf world

- To change the properties of the cable, edit cable_params.yaml, then run update_all.py



