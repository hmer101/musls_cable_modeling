from Logfile import Logfile
from LogCabinet import LogCabinet
import math
import numpy as np
from utils_logs import COLOR_MAP, COLOR_MAP_2, COLOR_MAP_BLACK, COLOR_MAP_RED, COLOR_MAP_ORANGE, COLOR_MAP_YELLOW, COLOR_MAP_PINK, COLOR_MAP_GREEN, COLOR_MAP_BLUE, COLOR_MAP_DARK_PURPLE


# Helper functions
# Add start time to trajectory bounds
def tb_add_start_time(start_time, tb_orig):
    return [(start + start_time, end + start_time) for start, end in tb_orig]

# Construct logfiles from file data
def construct_logfiles(file_data_list, num_drones, unwrap_angles, common_time, use_trajectory_slicing):
    logfiles = []
    
    for file_data in file_data_list:
        filepath, metadata, selected_trajectories, trajectory_bounds = file_data
        
        # Extract start times from metadata
        circle_start_time = metadata.get("circle_start_time", 0.0)
        yaw_engage_start_time = metadata.get("yaw_engage_start_time", 152.82)
        shm_start_time = metadata.get("shm_start_time", 305.75)
        
        # Adjust the bounds using tb_add_start_time for each trajectory
        tb = {}
        
        for trajectory_type, bounds in trajectory_bounds.items():
            if trajectory_type == "CIRCLE":
                tb[trajectory_type] = tb_add_start_time(circle_start_time, bounds)
            elif trajectory_type == "YAW_ENGAGE":
                tb[trajectory_type] = tb_add_start_time(yaw_engage_start_time, bounds)
            elif trajectory_type == "LINEAR_SHM":
                tb[trajectory_type] = tb_add_start_time(shm_start_time, bounds)
        
        # Create the logfile with the adjusted bounds
        logfile = Logfile(
            num_drones=num_drones,
            unwrap_angles=unwrap_angles,
            common_time=common_time,
            use_trajectory_slicing=use_trajectory_slicing,
            filepath=filepath,
            metadata=metadata,
            selected_trajectories=selected_trajectories,
            trajectory_bounds=tb,  # Adjusted bounds with start times
        )
        
        logfiles.append(logfile)
    
    return logfiles


if __name__ == '__main__':
    # SIM
    directory_logs = '/multi_drone_slung_load_master/data_and_processing/data/'

    ## GENERAL SETTINGS ##
    num_drones = 3
    unwrap_angles = {"CIRCLE": False, "LINEAR_SHM": True, "YAW_ENGAGE": True}
    common_time = None  # One timebase per stage
    use_trajectory_slicing = True

    # Mission circle
    circle_start_time = 0.0
    circle_duration = 31.42
    circle_tb = [(i * circle_duration, (i + 1) * circle_duration) for i in range(3)]

    # Mission yaw engage
    yaw_engage_start_time = 152.82
    yaw_engage_duration = 22.69
    yaw_engage_gap = 24.9
    yaw_engage_tb = [(i * (yaw_engage_duration + yaw_engage_gap), (i + 1) * (yaw_engage_duration + yaw_engage_gap)) for i in range(3)]
    yaw_engage_tb_custom = [(0.0, 22.69), (47.59, 69.88), (94.79, 117.48)]

    yaw_engage_start_time_sim = 168.80 
    yaw_engage_tb_custom_sim = [(0.0, 22.528), (63.288, 85.6205), (127.092, 150.208)]

    yaw_engage_start_time_sim_2 = 176.708 
    yaw_engage_tb_custom_sim_2 = [(0.0, 22.344), (71.244, 93.912), (143.456, 166.58)]

    # Mission linear shm
    shm_start_time = 305.863 
    shm_duration = math.pi 
    shm_num_pairs = 9
    shm_tb = [(i * shm_duration, (i + 1) * shm_duration) for i in range(shm_num_pairs)]
    
    shm_start_time_sim = 365.99 
    shm_start_time_sim_2 = 400.984


    ### CONSTRUCT LOGFILES ###
    logfiles_real_list = [
        [
            f'{directory_logs}real/2025_07_30_01_19_58_logger1.txt',
            {"type": "real", "trial": 1, "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time, "shm_start_time": shm_start_time},
            {"CIRCLE": [1], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, # CIRCLE: 1(kept in): load yaw small wabbly 2: drone 2 orient spike ~8sec, drone 1 orient spike ~28 sec. SHM: 0 is out at start. 1 load yaw change less
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom, 
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}real/2025_07_30_14_36_05_logger1.txt',
            {"type": "real", "trial": 1, "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time, "shm_start_time": shm_start_time},
            {"CIRCLE": [2], "YAW_ENGAGE": [1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, # CIRCLE: 1 and 2(smaller, to left) drone1 large orient spike ~26s (kept in as localized error). YAW: 0 drone2 large orientation spike, SHM: 0 is out at start. 1 load yaw change less. PERHAPS MORE TO REMOVE HERE?
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom, 
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}real/2025_07_30_20_44_05_logger1.txt',
            {"type": "real", "trial": 1, "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time, "shm_start_time": shm_start_time},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, # CIRCLE: 1 SHM: 0 is out at start. 1 lag in y
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom, 
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}real/2025_07_30_21_36_24_logger1.txt',
            {"type": "real", "trial": 4, "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time, "shm_start_time": shm_start_time},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, # 0 is out at start. 1 has load z that doesn't vary as much as the others. 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}real/2025_07_30_22_14_22_logger1.txt',
            {"type": "real", "trial": 5, "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time, "shm_start_time": shm_start_time},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 4, 5, 6, 7, 8]}, # 0 is out at start. 1 - y is slightly behind in load and drone 1. 3 - drone 1 and load fall out of y for a bit.
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom, 
                "LINEAR_SHM": shm_tb 
            }
        ]
    ]

    # Sensitivity analysis
    logfiles_sim_list_sense_num_elements = [
        [
            f'{directory_logs}sense_num_elements/2025_08_25_22_17_24_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Data after 424.137 is taken from the 10 element logfile as this went unstable - cut last few reps",
             "num_elements": 5, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6]},  #Linear SHM: cut last two as copied from 10 elements after crash - too unstable at end of SHM
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_22_47_24_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Still gets unstable: oscillations grow lager and with lengthening period",
             "num_elements": 6, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6]},  # Linear SHM: cut last two as copied from 10 elements after crash - too unstable at end of SHM
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_23_11_06_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Drone 1's cable disengages - part of the reason that oscaillations also grow increasingly unstable. Would crash if left to keep going.",
             "num_elements": 7, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_23_32_48_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Still tiny cable disengagement at end of SHM",
             "num_elements": 8, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_23_52_01_logger8.txt',
            {"type": "sim", "trial": 1, "description": "No cable disengagement, very stable.",
             "num_elements": 9, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_21_55_59_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 10, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_00_10_30_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 11, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_13_59_01_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 12, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_13_39_51_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 13, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},   
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_13_21_18_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 14, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},   
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_21_36_55_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 15, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_13_02_09_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 16, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_11_56_31_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 17, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_14_18_37_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 18, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_14_46_40_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 19, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_12_14_57_logger8.txt',
            {"type": "sim", "trial": 2, "description": "See some left and right yaw movement with some trials - increased inconsistency",
             "num_elements": 20, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_15_32_11_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 21, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_16_26_43_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 22, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_16_49_57_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 23, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0002, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_17_41_27_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Was swinging at 0.0002 engage timestep so decreased to 0.0001",
             "num_elements": 24, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},   
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_25_20_36_50_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 25, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_18_07_44_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 26, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_18_34_33_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 27, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_26_19_03_33_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 28, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_num_elements/2025_08_21_00_54_25_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Select this damping factor!",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ]
    ]

    logfiles_sim_list_sense_damping = [ 
        # [
        #     f'{directory_logs}sense_damping/2025_08_19_23_17_01_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Load response looks too violent, too much cable vibration during normal lifting (visually wrong). During yaw reset, cable visually has nowhere near enough damping as it flutters.",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.00025, "max_step_size_lower": 0.0019, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.0019,
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_19_22_03_18_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Load response looks too violent, too much cable vibration during normal lifting (visually wrong).",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.0005, "max_step_size_lower": 0.0019, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.0019,
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_19_21_20_26_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Load response looks too violent.",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.001, "max_step_size_lower": 0.0019, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # Yaw engage: 0: out on load's z pos and drones poses 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        [
            f'{directory_logs}sense_damping/2025_08_20_23_30_12_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Period still too small",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.0015, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sense_damping/2025_08_21_00_54_25_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Select this damping factor!",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_21_02_01_31_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Damping too high - decay too fast",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.0025, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 1 is unstable
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_19_17_19_25_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Might have to redo due to instability",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.003, "step_size": 0.0005, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_19_16_52_22_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Might need a smaller step size redo due to instability",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.004, "step_size": 0.0019, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 0 is pretty off/unstable in y, x and yaw for load. 1 - load y pos has large oscillations. 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_20_00_09_23_logger8.txt',
        #     {"type": "sim", "trial": 2, "description": "Param sensitivity test. Decreased ",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.005, "max_step_size_lower": 0.0019, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.0019, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_18_20_21_25_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. .",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.0075, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        [
            f'{directory_logs}sense_damping/2025_08_14_21_02_28_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.01, 
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim, "shm_start_time": shm_start_time_sim},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim,  
                "LINEAR_SHM": shm_tb 
            }
        ] #,
        # [
        #     f'{directory_logs}sense_damping/2025_08_18_19_52_12_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.05, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_damping/2025_08_18_12_50_46_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. This extra damping = much more fluid motion. Looks too stiff/damped visually.",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.1, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim, "shm_start_time": shm_start_time_sim},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ], 
        # [
        #     f'{directory_logs}sense_damping/2025_08_18_13_05_28_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. This extra damping = much more fluid motion. Looks too stiff/damped visually.",
        #      "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 1, 
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim, "shm_start_time": shm_start_time_sim},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ]    
    ]

    logfiles_sim_list_sense_friction = [ 
        [
            f'{directory_logs}sense_friction/2025_08_21_00_54_25_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Select this damping factor!",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 0 is munched
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        # [
        #     f'{directory_logs}sense_friction/2025_08_21_16_16_48_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
        #      "num_elements": 29, "joint_friction": 0.00005, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 1 load off in yaw
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_friction/2025_08_21_15_20_50_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
        #      "num_elements": 29, "joint_friction": 0.0001, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        [
            f'{directory_logs}sense_friction/2025_08_21_17_03_41_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
             "num_elements": 29, "joint_friction": 0.00015, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        # [
        #     f'{directory_logs}sense_friction/2025_08_25_15_10_10_logger8.txt',
        #     {"type": "sim", "trial": 3, "description": "Param sensitivity test",
        #      "num_elements": 29, "joint_friction": 0.0002, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        [
            f'{directory_logs}sense_friction/2025_08_24_22_39_23_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
             "num_elements": 29, "joint_friction": 0.0003, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.00025, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ] #,
        # [
        #     f'{directory_logs}sense_friction/2025_08_24_23_36_47_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
        #      "num_elements": 29, "joint_friction": 0.0004, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.00025, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_friction/2025_08_21_12_54_21_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test. Lower timestep: 0.0001 - too slow/cut out, 0.0002 - too fast/bounce. Reduced range of slow lower to +/-0.05m",
        #      "num_elements": 29, "joint_friction": 0.0005, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0001, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004,
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ],
        # [
        #     f'{directory_logs}sense_friction/2025_08_21_13_46_59_logger8.txt',
        #     {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
        #      "num_elements": 29, "joint_friction": 0.001, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0001, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004,
        #      "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
        #     {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
        #     {
        #         "CIRCLE": circle_tb,  
        #         "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
        #         "LINEAR_SHM": shm_tb 
        #     }
        # ]  
    ]

    # Final test
    logfiles_sim_list_final_test = [
        [
            f'{directory_logs}sim_final/2025_08_25_21_36_55_logger8.txt',
            {"type": "sim", "trial": 1, "description": "",
             "num_elements": 15, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: [0,1]
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sim_final/2025_08_26_20_24_11_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Large y oscillations on engage",
             "num_elements": 15, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sim_final/2025_08_26_20_49_51_logger8.txt',
            {"type": "sim", "trial": 3, "description": "Large x oscillations on engage",
             "num_elements": 15, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sim_final/2025_08_26_21_15_20_logger8.txt',
            {"type": "sim", "trial": 4, "description": "Large y and x oscillations on engage",
             "num_elements": 15, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ],
        [
            f'{directory_logs}sim_final/2025_08_26_22_08_06_logger8.txt',
            {"type": "sim", "trial": 5, "description": "Large y oscillations on engage, one has large x oscillations",
             "num_elements": 15, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ]
    ]

    logfiles_sim_list = logfiles_sim_list_final_test

    ### Construct real logfiles
    logfiles_real = construct_logfiles(logfiles_real_list, num_drones, unwrap_angles, common_time, use_trajectory_slicing)


    #### PROCESS AND VIZUALIZE RESULTS ####
    # WHAT
    to_show = 'log_cabinet' #'log_single' 'log_cabinet' 'sense_elems' 'visual_compare_damping' 'visual_compare_friction' ('sense_damping' 'sense_friction')

    # STAGE
    #stage_to_plot = 'CIRCLE'
    stage_to_plot = 'YAW_ENGAGE'
    #stage_to_plot = 'LINEAR_SHM'

    # OPTIONS
    print_summary_table = True
    plot_results = True


    # Construct logfiles and process
    if to_show == 'log_single': 
        # Construct logfiles
        logfiles_sim = construct_logfiles(logfiles_sim_list, num_drones, unwrap_angles, common_time, use_trajectory_slicing)

        # LOG
        log = logfiles_sim[0]

        # Plot a specific stage and selected trajectory
        log.plot_stage(stage=stage_to_plot, show_qs=False, show_des=False, show_mean_ci_gt=False, show_mean_ci_qs=False, plot_raw_trajectories=True, confidence=0.95)
        #log.plot_stage(stage=stage_to_plot, show_qs=False, show_des=False, show_mean_ci_gt=True, show_mean_ci_qs=False, plot_raw_trajectories=False, confidence=0.95)

        # FULL
        #log.plot_stage(stage=stage_to_plot, trajectory="full", plot_3d = True, show_qs=False, show_des=True, show_mean_ci_gt=False, show_mean_ci_qs=False, plot_raw_trajectories=True, confidence=0.95)

        # Summary table
        if print_summary_table:
            log.print_summary_statistics(weight_pos=1.0, weight_ori=1.0)

    elif to_show == 'log_cabinet':
        # Construct logfiles
        logfiles_sim = construct_logfiles(logfiles_sim_list, num_drones, unwrap_angles, common_time, use_trajectory_slicing)

        # Sim vs real analysis
        cabinet = LogCabinet(
            logs_1=logfiles_real,
            logs_2=logfiles_sim,
        )

        if print_summary_table:
            cabinet.print_summary_statistics()

        # Log cabinet
        # cabinet.plot_group(
        #     group="real",
        #     stage=stage_to_plot,
        #     sliced_trajs=use_trajectory_slicing,
        #     show_qs=False,
        #     show_des=False,
        #     plot_raw_trajectories=True,
        #     plot_3d=False
        # )

        # Set limits
        x_lims = None
        y_lims = None
        x_ticks = None
        y_ticks = None
        
        if stage_to_plot == 'YAW_ENGAGE':
            # Limits
            x_lims = [[(0, 12),(0, 12)]]
            #y_lims = [[(0, 0.45),(0, 175)]]

            # Ticks
            x_ticks = [(list(range(0, 13, 2)), list(range(0, 13, 2)))]
            #y_ticks = [(np.arange(0, 0.6, 0.2), list(range(0, 181, 60)))]
        # elif stage_to_plot == 'LINEAR_SHM':
        #     pass

        # Plot
        figsize = [(3.5, 4) for _ in range(num_drones + 2)]
        #axis_labels = ['Time (s)', 'Z position (m)', 'Yaw angle (deg)']
        x_axis_top_fig_on = False

        cabinet.plot_stage(
                aggregated_df_1=cabinet.aggregated_df_1,
                aggregated_df_2=cabinet.aggregated_df_2, #cabinet.aggregated_df_real,  # List of logfiles to plot
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=False,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=True,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=None,
                show_cis=True,
                x_axis_top_fig_on=False,
                color_map=COLOR_MAP,
                figsize=figsize,
                show_legend=False,
                legend_special=True,
                legend_extra1='', #' - real'
                linestyle1="solid",
                legend_extra2='', #' - sim'
                linestyle2="dashed",
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=True
            )
        
    elif to_show == 'sense_elems':
        # Construct logfile dictionaries for sensitivity analysis
        # num_elements
        logfiles_sim_sense_num_elements = construct_logfiles(logfiles_sim_list_sense_num_elements, num_drones, unwrap_angles, common_time, use_trajectory_slicing)
        logfiles_sim_sense_num_elements_dict = {logfile.metadata["num_elements"]: [logfile] for logfile in logfiles_sim_sense_num_elements}

        logs_2_variations = logfiles_sim_sense_num_elements_dict
        parameter_name = "Number of links"
        baseline_ind = -1

        # Limits
        x_lims = [(4, 31),(4, 31)]
        y_lims = [(0, 401),(0, 26)]

        # Ticks
        x_ticks = [(list(range(5, 31, 5)), list(range(5, 31, 5)))] #[4] + 
        y_ticks = [(np.arange(0, 401, 50), list(range(0, 26, 5)))]

    # elif to_show == 'sense_damping':
    #     # Construct logfile dictionaries for sensitivity analysis
    #     # Damping
    #     logfiles_sim_sense_damping = construct_logfiles(logfiles_sim_list_sense_damping, num_drones, unwrap_angles, common_time, use_trajectory_slicing)
    #     logfiles_sim_sense_damping_dict = {logfile.metadata["joint_damping"]: [logfile] for logfile in logfiles_sim_sense_damping}

    #     logs_2_variations = logfiles_sim_sense_damping_dict
    #     parameter_name = "joint_damping (N/m/s)"
    #     baseline_ind = 4

    # elif to_show == 'sense_friction':
    #     # Construct logfile dictionaries for sensitivity analysis
    #     # Friction
    #     logfiles_sim_sense_friction = construct_logfiles(logfiles_sim_list_sense_friction, num_drones, unwrap_angles, common_time, use_trajectory_slicing)
    #     logfiles_sim_sense_friction_dict = {logfile.metadata["joint_friction"]: [logfile] for logfile in logfiles_sim_sense_friction}

    #     logs_2_variations = logfiles_sim_sense_friction_dict
    #     parameter_name = "joint_friction (N)"
    #     baseline_ind = 0

    elif to_show == 'visual_compare_damping':
        # Construct logfiles
        # Real
        cabinet_real = LogCabinet(
            logs_1=logfiles_real
        )

        # Get common time
        common_time_real = cabinet_real.common_time
        
        ## Damping
        # Logfile lists
        damping_00015_list = [[
            f'{directory_logs}sense_damping/2025_08_20_23_30_12_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Period still too small",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.0015, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]}, 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ]]

        damping_0002_list = [
            [
            f'{directory_logs}sense_damping/2025_08_20_21_39_47_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test. Load response looks too violent.",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 2 - huge y and z load instabilities and large pitch. 1 - has a large load x oscialltion 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
            ],
            [
            f'{directory_logs}sense_damping/2025_08_21_00_54_25_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Select this damping factor!",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 0 is munched
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ]]

        damping_00025_list = [[
            f'{directory_logs}sense_damping/2025_08_21_02_01_31_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test. Damping too high - decay too fast",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.0025, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ]]

        damping_001_list = [
            [
            f'{directory_logs}sense_damping/2025_08_14_21_02_28_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.01, 
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim, "shm_start_time": shm_start_time_sim},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim,  
                "LINEAR_SHM": shm_tb 
            }
        ]]

        # Logfiles
        logfiles_sim_damping_00015 = construct_logfiles(damping_00015_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_damping_0002 = construct_logfiles(damping_0002_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_damping_00025 = construct_logfiles(damping_00025_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_damping_001 = construct_logfiles(damping_001_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)

        # Log cabinets
        cabinet_damping_00015 = LogCabinet(
            logs_1=logfiles_sim_damping_00015
        )

        cabinet_damping_0002 = LogCabinet(
            logs_1=logfiles_sim_damping_0002
        )

        cabinet_damping_00025 = LogCabinet(
            logs_1=logfiles_sim_damping_00025
        )

        cabinet_damping_001 = LogCabinet(
            logs_1=logfiles_sim_damping_001
        )

        # Set limits
        x_lims = None
        y_lims = None
        
        if stage_to_plot == 'YAW_ENGAGE':
            # Limits
            x_lims = [[(0, 12),(0, 12)]]
            y_lims = [[(0, 0.45),(0, 175)]]

            # Ticks
            x_ticks = [(list(range(0, 13, 2)), list(range(0, 13, 2)))]
            y_ticks = [(np.arange(0, 0.6, 0.2), list(range(0, 181, 60)))] #[(np.arange(0, 0.6, 0.1), list(range(0, 176, 25)))]

        # Plot
        figsize = [(4, 4.5) for _ in range(num_drones + 2)]
        axis_labels = ['Time (s)', 'Z position (m)', 'Yaw angle (deg)']
        x_axis_top_fig_on = False

        axes = cabinet_real.plot_stage(
                aggregated_df_1=cabinet_real.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=None,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                figsize=figsize,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='real',
                linestyle1="solid",
                color_map=COLOR_MAP_BLACK,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=False
            )

        cabinet_damping_00015.plot_stage(
                aggregated_df_1=cabinet_damping_00015.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=axes,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='0.0015 Ns/rad',
                linestyle1="solid",
                color_map=COLOR_MAP_BLUE,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=False
            )
        
        cabinet_damping_0002.plot_stage(
                aggregated_df_1=cabinet_damping_0002.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=axes,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='0.002 Ns/rad',
                linestyle1="solid",
                color_map=COLOR_MAP_ORANGE,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=False
            )
        
        # Need to add an extra trajectory before adding back!
        # cabinet_damping_00025.plot_stage(
        #         aggregated_df_1=cabinet_damping_00025.aggregated_df_1,
        #         stage=stage_to_plot,  # Stage to plot
        #         show_qs=False,  # Show qs data
        #         show_des=True,  # Show desired positions and orientations
        #         source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
        #         show_mean_ci_gt=True,  # Show confidence intervals for ground truth
        #         show_mean_ci_qs=False,  # Show confidence intervals for quaternions
        #         show_cis=False,
        #         plot_raw_trajectories=False,  # Plot raw trajectories
        #         plot_3d=False,  # Plot in 3D
        #         confidence=0.95,  # Confidence interval for CI
        #         axes_list=axes,
        #         axis_labels=axis_labels,
        #         x_axis_top_fig_on=x_axis_top_fig_on,
        #         legend_shared=True,
        #         legend_supress_prefix=True,
        #         legend_extra1='0.0025 Ns/rad',
        #         linestyle1="solid",
        #         color_map=COLOR_MAP_BLUE,
        #         trans_plot=['z'],
        #         orient_plot=['yaw'],
        #         x_lims=x_lims,
        #         y_lims=y_lims,
        #         x_ticks=x_ticks,
        #         y_ticks=y_ticks,
        #         show_plot=False
        #     )
        
        cabinet_damping_001.plot_stage(
                aggregated_df_1=cabinet_damping_001.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=axes,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='0.01 Ns/rad',
                linestyle1='solid',
                color_map=COLOR_MAP_YELLOW,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=True
            )
    
    elif to_show == 'visual_compare_friction':
        # Construct logfiles
        # Real
        cabinet_real = LogCabinet(
            logs_1=logfiles_real
        )

        # Get common time
        common_time_real = cabinet_real.common_time
        
        ## Damping
        # Logfile lists
        friction_0_list = [
            [
            f'{directory_logs}sense_friction/2025_08_20_21_39_47_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test. Load response looks too violent.",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 2 - huge y and z load instabilities and large pitch. 1 - has a large load x oscialltion 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
            ],
            [
            f'{directory_logs}sense_friction/2025_08_21_00_54_25_logger8.txt',
            {"type": "sim", "trial": 2, "description": "Param sensitivity test. Select this damping factor!",
             "num_elements": 29, "joint_friction": 0.0, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0005, "max_step_size_yaw_engage": 0.0001, "max_step_size_default": 0.004,
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 0 is munched
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
            ]
        ]

        friction_00001_list = [
            [
            f'{directory_logs}sense_friction/2025_08_21_15_20_50_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
             "num_elements": 29, "joint_friction": 0.0001, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
            ]
        ]

        friction_00002_list = [
            [
            f'{directory_logs}sense_friction/2025_08_25_15_10_10_logger8.txt',
            {"type": "sim", "trial": 3, "description": "Param sensitivity test",
             "num_elements": 29, "joint_friction": 0.0002, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
            ]
        ]

        friction_000015_list = [
            [
                f'{directory_logs}sense_friction/2025_08_21_17_03_41_logger8.txt',
                {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
                "num_elements": 29, "joint_friction": 0.00015, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.0002, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
                "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
                {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
                {
                    "CIRCLE": circle_tb,  
                    "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                    "LINEAR_SHM": shm_tb 
                }
            ]
        ]

        friction_00003_list = [
            [
            f'{directory_logs}sense_friction/2025_08_24_22_39_23_logger8.txt',
            {"type": "sim", "trial": 1, "description": "Param sensitivity test.",
             "num_elements": 29, "joint_friction": 0.0003, "joint_stiffness": 0.0, "joint_damping": 0.002, "max_step_size_lower": 0.00025, "max_step_size_yaw_engage": 0.00015, "max_step_size_default": 0.004, #0.0001
             "circle_start_time": circle_start_time, "yaw_engage_start_time": yaw_engage_start_time_sim_2, "shm_start_time": shm_start_time_sim_2},
            {"CIRCLE": [1, 2], "YAW_ENGAGE": [0, 1, 2], "LINEAR_SHM": [2, 3, 4, 5, 6, 7, 8]},  # YAW engage: 
            {
                "CIRCLE": circle_tb,  
                "YAW_ENGAGE": yaw_engage_tb_custom_sim_2,  
                "LINEAR_SHM": shm_tb 
            }
        ]
        ]


        # Logfiles
        logfiles_sim_friction_0 = construct_logfiles(friction_0_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_friction_00001 = construct_logfiles(friction_00001_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_friction_000015 = construct_logfiles(friction_000015_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_friction_00002 = construct_logfiles(friction_00002_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)
        logfiles_sim_friction_00003 = construct_logfiles(friction_00003_list, num_drones, unwrap_angles, common_time_real, use_trajectory_slicing)

        # Log cabinets
        cabinet_friction_0 = LogCabinet(
            logs_1=logfiles_sim_friction_0
        )

        cabinet_friction_00001 = LogCabinet(
            logs_1=logfiles_sim_friction_00001
        )

        cabinet_friction_000015 = LogCabinet(
            logs_1=logfiles_sim_friction_000015
        )

        cabinet_friction_00002 = LogCabinet(
            logs_1=logfiles_sim_friction_00002
        )

        cabinet_friction_00003 = LogCabinet(
            logs_1=logfiles_sim_friction_00003
        )       

        # Set limits
        x_lims = None
        y_lims = None
        x_ticks = None
        y_ticks = None
        
        if stage_to_plot == 'YAW_ENGAGE':
            # Limits
            x_lims = [[(0, 12),(0, 12)]]
            y_lims = [[(0, 0.45),(0, 175)]]

            # Ticks
            x_ticks = [(list(range(0, 13, 2)), list(range(0, 13, 2)))]
            y_ticks = [(np.arange(0, 0.6, 0.2), list(range(0, 181, 60)))]
        # elif stage_to_plot == 'LINEAR_SHM':
        #     pass

        # Plot
        figsize = [(4, 4.5) for _ in range(num_drones + 2)]
        axis_labels = ['Time (s)', 'Z position (m)', 'Yaw angle (deg)']
        x_axis_top_fig_on = False

        axes = cabinet_real.plot_stage(
                aggregated_df_1=cabinet_real.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=None,
                figsize=figsize,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='real',
                linestyle1="solid",
                color_map=COLOR_MAP_BLACK,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=False
            )
        
        cabinet_friction_0.plot_stage(
                aggregated_df_1=cabinet_friction_0.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=axes,
                figsize=figsize,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='0 mN',
                linestyle1="solid",
                color_map=COLOR_MAP_BLUE,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=False
            )

        # cabinet_friction_00001.plot_stage(
        #         aggregated_df_1=cabinet_friction_00001.aggregated_df_1,
        #         stage=stage_to_plot,  # Stage to plot
        #         show_qs=False,  # Show qs data
        #         show_des=True,  # Show desired positions and orientations
        #         source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
        #         show_mean_ci_gt=True,  # Show confidence intervals for ground truth
        #         show_mean_ci_qs=False,  # Show confidence intervals for quaternions
        #         show_cis=False,
        #         plot_raw_trajectories=False,  # Plot raw trajectories
        #         plot_3d=False,  # Plot in 3D
        #         confidence=0.95,  # Confidence interval for CI
        #         axes_list=axes,
        #         figsize=figsize,
        #         axis_labels=axis_labels,
        #         x_axis_top_fig_on=x_axis_top_fig_on,
        #         legend_shared=True,
        #         legend_supress_prefix=True,
        #         legend_extra1='0.1 mN',
        #         linestyle1="solid",
        #         color_map=COLOR_MAP_GREEN,
        #         trans_plot=['z'],
        #         orient_plot=['yaw'],
        #         x_lims=x_lims,
        #         y_lims=y_lims,
        #         x_ticks=x_ticks,
        #         y_ticks=y_ticks,
        #         show_plot=False
        #     )

        cabinet_friction_000015.plot_stage(
                aggregated_df_1=cabinet_friction_000015.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=axes,
                figsize=figsize,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='0.15 mN',
                linestyle1="solid",
                color_map=COLOR_MAP_ORANGE,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=False
            )
        
        # cabinet_friction_00002.plot_stage(
        #         aggregated_df_1=cabinet_friction_00002.aggregated_df_1,
        #         stage=stage_to_plot,  # Stage to plot
        #         show_qs=False,  # Show qs data
        #         show_des=True,  # Show desired positions and orientations
        #         source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
        #         show_mean_ci_gt=True,  # Show confidence intervals for ground truth
        #         show_mean_ci_qs=False,  # Show confidence intervals for quaternions
        #         show_cis=False,
        #         plot_raw_trajectories=False,  # Plot raw trajectories
        #         plot_3d=False,  # Plot in 3D
        #         confidence=0.95,  # Confidence interval for CI
        #         axes_list=axes,
        #         figsize=figsize,
        #         axis_labels=axis_labels,
        #         x_axis_top_fig_on=x_axis_top_fig_on,
        #         legend_shared=True,
        #         legend_supress_prefix=True,
        #         legend_extra1='0.2 mN',
        #         linestyle1="solid",
        #         color_map=COLOR_MAP_BLUE,
        #         trans_plot=['z'],
        #         orient_plot=['yaw'],
        #         x_lims=x_lims,
        #         y_lims=y_lims,
        #         x_ticks=x_ticks,
        #         y_ticks=y_ticks,
        #         show_plot=False
        #     )
        
        cabinet_friction_00003.plot_stage(
                aggregated_df_1=cabinet_friction_00003.aggregated_df_1,
                stage=stage_to_plot,  # Stage to plot
                show_qs=False,  # Show qs data
                show_des=True,  # Show desired positions and orientations
                source_name=f'Log Cabinet {stage_to_plot}',  # Title to include in the plot
                show_mean_ci_gt=True,  # Show confidence intervals for ground truth
                show_mean_ci_qs=False,  # Show confidence intervals for quaternions
                show_cis=False,
                plot_raw_trajectories=False,  # Plot raw trajectories
                plot_3d=False,  # Plot in 3D
                confidence=0.95,  # Confidence interval for CI
                axes_list=axes,
                figsize=figsize,
                axis_labels=axis_labels,
                x_axis_top_fig_on=x_axis_top_fig_on,
                legend_shared=True,
                legend_supress_prefix=True,
                legend_extra1='0.3 mN',
                linestyle1="solid",
                color_map=COLOR_MAP_YELLOW,
                trans_plot=['z'],
                orient_plot=['yaw'],
                x_lims=x_lims,
                y_lims=y_lims,
                x_ticks=x_ticks,
                y_ticks=y_ticks,
                show_plot=True
            )
    
    # Sensitivity analysis
    if to_show.split("_", 1)[0] == "sense":
        # Construct log cabinet
        cabinet_sense = LogCabinet(
            logs_1=logfiles_real,
            num_drones=num_drones,
            logs_2_variations=logs_2_variations
        )

        # Plot parameter sensitivity
        cabinet_sense.plot_param_sensitivity(
            parameter_name=parameter_name, percent_baseline_ind=baseline_ind, show_titles=False, x_lims=x_lims, y_lims=y_lims, x_ticks=x_ticks, y_ticks=y_ticks, legend_shared=True)


    # Hold plots open until user closes them
    input("Press Enter to close all plots...")
