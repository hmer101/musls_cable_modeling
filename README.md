# musls_cable_modeling
Code and data accompanying **ICRA 2026 submission: _Dynamics Modeling of a Multi-UAV Slung Load System Using a Discrete-Link Cable Approach_**

![flights](https://github.com/hmer101/musls_cable_modeling/blob/main/gif_mocap_flights.gif)

If you use this code in a research setting, please cite:

```text
@inproceedings{merton2026dynamics,
  title={Dynamics Modeling of a Multi-UAV Slung Load System Using a Discrete-Link Cable Approach},
  author={Merton, Harvey and Hunter, Ian W},
  booktitle={2026 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={15757--15764},
  year={2026},
  organization={IEEE}
}
```

The repository that contains the full flight code to generate this data can be found [here](https://github.com/hmer101/multi_drone_slung_load_master/tree/main), while the ICRA 2026 accompanying overview video can be found [here](https://www.youtube.com/watch?v=VATTPYYDwYY).

---

## 📂 Repository Structure
```text
.
├── LICENSE
├── multi_drone_slung_load_master/     # Root folder for development container
│   ├── data_and_processing/
│   │   ├── data/                      # Custom logfiles + ROS2 bagfiles
│   │   └── src/                       # Code for processing data and generating plots
│   ├── docker-compose.yml             # Docker container orchestration
│   ├── repos/
│   │   └── PX4-Autopilot/             # Custom PX4 simulation files
│   ├── requirements.txt               # Python dependencies
│   ├── run_qgc.sh                     # Script to run QGroundControl (if installed)
│   └── setup.sh                       # Setup script (post-container creation)
└── README.md
```

---

## 🚀 Getting Started

1. **Build and start the development container**  
   - Open the repo in VS Code (or your preferred environment) with DevContainers enabled.  
   - cntrl + shift + p to open the command palette in VS code, then type 'rebuild and reopen in devcontainer'.

2. **Install dependencies inside the container**  
   ```bash
   bash setup.sh
   ```
   This installs all Python requirements from `requirements.txt`.

3. **Verify the environment**  
   - The Docker container includes all dependencies (ROS2, PX4 simulation setup, Python libraries) needed to reproduce results.  
   - Optional: run `./run_qgc.sh` if you have QGroundControl installed in the container.  

---

## 📊 Data Files

This repository includes **all simulated and real-world data files** used in the paper:

- **Custom logfiles** (from simulated and real flights).  
- **ROS2 bagfiles (.db3)** for real-world flights containing TFs of the load and drones.  
  - First, unzip the provided archives.  
  - Play a bagfile with:  
    ```bash
    ros2 bag play filename.db3
    ```

### Processing the data
- Use **`process_results.py`** (in `data_and_processing/src/`) as the main entry point.  
- This script:  
  1. Loads metadata about all datasets.  
  2. Processes logfiles.  
  3. Generates plots and figures corresponding to the paper.  

---

## 🛠️ Simulation Setup

This repo also provides the world and model files needed to simulate the Multi-UAV Slung Load System (MUSLS):

1. **Install PX4 Autopilot** inside the Docker container, see [PX4 Ubuntu Development Environment](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html).  
2. **Copy the custom PX4 files** from:  
   ```
   multi_drone_slung_load_master/repos/PX4-Autopilot
   ```  
   into the corresponding directories of your PX4 installation.  
3. **Run the simulation** using Gazebo with the provided world file:  
   ```
   multi_rigid_link_2m.sdf
   ```
4. **Modify cable properties** if needed:  
   - Edit `cable_params.yaml`  
   - Run `update_all.py` to apply changes.  

For more details on running PX4 with ROS2 offboard control, see the [PX4 Offboard Control Guide](https://docs.px4.io/main/en/ros/offboard_control).

---

## 📌 Notes

- This repository **does not** include the full formation control implementation required for real-world/simulated flights (not the main focus of this paper). That code can be found in [this open-source repository](https://github.com/hmer101/multi_drone_slung_load_master/tree/main).  
- To execute formation control, simply send position waypoints to each drone in the MUSLS using PX4’s standard offboard control API.  
- The provided container + scripts are enough to:  
  - Reproduce the figures and results in the paper.  
  - Replay real flight data.  
  - Reproduce MUSLS simulations (providing formation control is implemented as described) as all customizable cable parameters are provided.


