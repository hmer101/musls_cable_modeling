# musls_cable_modeling
Code and data accompanying ICRA 2026 submission: Dynamics Modeling of a Multi-UAV Slung Load System Using a Discrete-Link Cable Approach

## Structure


## How to use
This repository contains the custom files and settings required to recreate the results seen in the paper. It also contains all simulated and real world data files and the corresponding code to process them to reproduce the graphics seen in the paper. 

- It does not contain the in-depth formation control implementation required to fly in simulation and the real world as this is not the main subject of this paper. To implement this, simply send waypoints to each of the three drones in the MUSLS using these instructions: https://docs.px4.io/main/en/ros2/offboard_control.

### Simulation setup
- The Dockerfile contains the whole environment you need to run and process the collected datafiles including all dependencies and python packages.

- Build and open the development container
- Run bash setup.sh


- Install PX4 autopilot inside the Docker container
- Copy all files from multi_drone_slung_load_master/repos/PX4-Autopilot into their corresponding locations in the installed PX4 instance

- To change the properties of the cable, edit cable_params.yaml, then run update_all.py


### Data files
- Contains all of the rosbag files and the custom logfiles generated from simulated and real world flights.
- process_results.py is the main file to follow to process custom logfiles and produce the corresponding graphs.
