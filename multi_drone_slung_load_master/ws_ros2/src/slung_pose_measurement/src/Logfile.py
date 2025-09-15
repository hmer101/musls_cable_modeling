import os
import numpy as np
from scipy.stats import norm
import pandas as pd
import matplotlib.pyplot as plt
from typing import Dict, Optional, Any, List, Tuple, Union
from tabulate import tabulate
import utils_logs
from utils_logs import title_font_size, axes_label_font_size, legend_font_size, ticks_font_size, \
                       time_ticks, y_ticks_pos, y_ticks_ori, x_ticks_traj, y_ticks_traj, z_ticks_traj, \
                       COLOR_MAP, DRONE_COLORS

import autograd.numpy as anp
from pymanopt import Problem
from pymanopt.manifolds import SpecialOrthogonalGroup, Euclidean, Product
from pymanopt.optimizers import SteepestDescent
from pymanopt.function import autograd
from scipy.spatial.transform import Rotation as Rscipy


class Logfile:
    MISSION_PHASES = {
        0: "UNASSIGNED",
        1: "CIRCLE",
        2: "LINEAR_SHM",
        3: "TENSION_ENGAGE",
        4: "YAW_ENGAGE",
        9: "HOLD"
    }

    def __init__(
        self,
        filepath: str,
        unwrap_angles: Dict[str, Any] = {"CIRCLE": False, "LINEAR_SHM": True, "YAW_ENGAGE": True},
        num_drones: int = 3,
        cable_lengths: float = 2.421,
        metadata: Optional[Dict[str, Any]] = None,
        common_time: Optional[Dict[str, Optional[np.ndarray]]] = None,
        trajectory_bounds: Optional[Dict[str, List[Tuple[float, float]]]] = None,
        selected_trajectories: Optional[Dict[str, List[int]]] = None,
        use_trajectory_slicing: bool = False,
        calc_load_pose_rod: bool = False
    ):
        self.filepath = filepath
        self.filename = filepath.split("/")[-1]
        self.num_drones = num_drones
        self.cable_lengths = cable_lengths
        self.unwrap_angles = unwrap_angles
        self.metadata = metadata or {}
        self.common_time = common_time or {}  # One timebase per stage
        self.trajectory_bounds = trajectory_bounds or {}
        self.selected_trajectories = selected_trajectories or {}
        self.use_trajectory_slicing = use_trajectory_slicing

        self.stage_data = self._load_data_from_file(filepath)
        self.tables = self._generate_tables()
        self.error_dataframe = None

        self.compute_error_dataframe()

        if calc_load_pose_rod:
            self.compute_load_pose_rod()

        #self.rebuild()

    # Logfile processing
    def _load_data_from_file(self, filepath: str) -> Dict[str, np.ndarray]:
        segments = self._parse_log_by_stage(filepath)
        stage_data = {}
        for stage, lines in segments.items():
            parsed_array = self._parse_stage_data(lines, stage_name=stage)
            if parsed_array.size > 0:
                stage_data[stage] = parsed_array
        return stage_data

    def _generate_tables(self) -> Dict[str, Dict[str, Dict[str, pd.DataFrame]]]:
        tables = {}
        #tables_old = {}
        for stage, data in self.stage_data.items():
            stage_tables = {}
            #stage_tables_old = {}

            if self.use_trajectory_slicing and stage in self.trajectory_bounds:
                bounds = self.trajectory_bounds[stage]
                selected = self.selected_trajectories.get(stage, list(range(len(bounds))))
                aligned_slices = []

                for i in selected:
                    t_start, t_end = bounds[i]
                    mask = (data[:, 0] >= t_start) & (data[:, 0] <= t_end)
                    sliced_data = data[mask]
                    aligned_slices.append(sliced_data)

                    # relative_time = sliced_data[:, 0] - sliced_data[0, 0]
                    # stage_tables_old[f"traj_{i+1}"] = self._build_entity_tables(sliced_data, relative_time)

                # Automatically compute aligned timebase if not manually set
                if stage not in self.common_time or self.common_time[stage] is None:
                    self.common_time[stage] = self._get_common_timebase(aligned_slices)

                for i, sliced_data in zip(selected, aligned_slices):
                    aligned_data = self.interpolate_raw_data(sliced_data, self.common_time[stage])
                    stage_tables[f"traj_{i+1}"] = self._build_entity_tables(aligned_data, self.common_time[stage])

            elif not self.use_trajectory_slicing:
                time = data[:, 0]
                stage_tables["full"] = self._build_entity_tables(data, time)
                #stage_tables_old["full"] = self._build_entity_tables(data, time)

                # Still have to set the common time for comparison between logfiles
                if stage not in self.common_time or self.common_time[stage] is None:
                    self.common_time[stage] = time
            
            else:
                pass
                #print("Trajectory slicing is enabled but no bounds provided for stage: " + stage)

            tables[stage] = stage_tables
            #tables_old[stage] = stage_tables_old
        return tables

    def _build_entity_tables(self, data: np.ndarray, time: np.ndarray) -> Dict[str, pd.DataFrame]:
        entity_tables = {}

        load_rows = {
            'gt': [(data[i, 7:10], data[i, 10:13]) for i in range(len(time))],
            'des': [(data[i, 1:4], data[i, 4:7]) for i in range(len(time))],
            'qs': [(data[i, 21:24], data[i, 24:27]) for i in range(len(time))]
        }
        entity_tables['load'] = pd.DataFrame(load_rows, index=time).T

        for i in range(self.num_drones):
            base = 35 + 20 * i
            drone_rows = {
                'gt': [(data[j, base + 6:base + 9], data[j, base + 9:base + 12]) for j in range(len(time))],
                'des': [(data[j, base:base + 3], data[j, base + 3:base + 6]) for j in range(len(time))],
                'qs': [None for _ in range(len(time))]
            }
            entity_tables[f'drone_{i+1}'] = pd.DataFrame(drone_rows, index=time).T

        return entity_tables

    def _parse_log_by_stage(self, filename):
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f.readlines()]

        try:
            start_idx = next(i for i, line in enumerate(lines) if line.startswith("PHASE_MISSION")) + 1
            end_idx = next(i for i, line in enumerate(lines[start_idx:], start=start_idx) if line.startswith("PHASE_MISSION"))
            lines = lines[start_idx:end_idx]
        except StopIteration:
            lines = []

        segments = {}
        current_lines = []
        current_stage = None
        started = False

        for line in lines:
            if line.startswith("MISSION_STAGE"):
                started = True
                if current_stage is not None and current_lines:
                    if current_stage not in segments:
                        segments[current_stage] = []
                    segments[current_stage].extend(current_lines)
                    current_lines = []
                try:
                    stage_num = int(line.split()[1])
                    current_stage = self.MISSION_PHASES.get(stage_num, f"UNKNOWN_{stage_num}")
                except:
                    current_stage = "UNKNOWN"
            elif started and line:
                current_lines.append(line)

        if current_stage and current_lines:
            if current_stage not in segments:
                segments[current_stage] = []
            segments[current_stage].extend(current_lines)

        return segments

    def _parse_stage_data(self, lines, stage_name: str, start_time=None, end_time=None):
        parsed = []
        for line in lines:
            data = list(map(float, line.split()))
            if len(data) != 35 + 20 * self.num_drones:
                continue
            if start_time is not None and end_time is not None:
                if not (start_time <= data[0] <= end_time):
                    continue
            parsed.append(data)

        data_array = np.array(parsed)
        if data_array.size == 0:
            return data_array

        def convert_column(col_idx):
            if self.unwrap_angles.get(stage_name, True):
                data_array[:, col_idx] = self._unwrap_and_convert_to_degrees(data_array[:, col_idx])
            else:
                data_array[:, col_idx] = np.degrees(data_array[:, col_idx])

        for col in [4, 5, 6]:
            convert_column(col)
        for col in [10, 11, 12]:
            convert_column(col)
        for col in [23, 24, 25]:
            convert_column(col)
        for d in range(self.num_drones):
            base = 35 + 20 * d
            for offset in [3, 4, 5]:
                convert_column(base + offset)
            for offset in [9, 10, 11]:
                convert_column(base + offset)

        return data_array
    
    def rebuild(self):
        """
        Rebuilds the Logfile object by reloading the data from the file.
        This is useful if the file has been modified externally.
        """
        self.stage_data = self._load_data_from_file(self.filepath)
        self.tables = self._generate_tables()
        self.compute_error_dataframe()


    # Getters and setters
    def set_common_times(self, new_common_times: Dict[str, Optional[np.ndarray]]):
        """
        Sets the common time for all stages in the Logfile.
        :param new_common_times: A dictionary mapping stage names to their common time arrays.
        """
        if not isinstance(new_common_times, dict):
            raise ValueError("new_common_times must be a dictionary.")

        #print(f'setting common times for logfile {self.filepath}')

        for stage, timebase in new_common_times.items():
            if stage not in self.common_time:
                #print(f"Stage '{stage}' not found in common_time. Adding it.")
                pass
            else:
                pass
                #print(f"Stage '{stage}' already exists in common_time. Overwriting.")

            self.common_time[stage] = timebase

    
    def get_common_time(self) -> Optional[Dict[str, Optional[np.ndarray]]]:
        return self.common_time

    def set_metadata(self, key: str, value: Any):
        self.metadata[key] = value

    def get_stage_data(self, stage: str) -> Optional[np.ndarray]:
        return self.stage_data.get(stage)

    def get_table(self, stage: str, entity: str, trajectory: str = "full") -> pd.DataFrame:
        return self.tables[stage][trajectory][entity]

    # Helper methods
    def _get_common_timebase(self, data_slices: List[np.ndarray]) -> np.ndarray:
        """Finds the common timebase by intersecting all trajectory time arrays."""
        # Filter out empty arrays
        non_empty_slices = [d for d in data_slices if len(d) > 0]

        if len(non_empty_slices) == 0:
            raise ValueError("All data slices are empty.")

        # Find the minimum length of the non-empty arrays
        min_length = min(len(d) for d in non_empty_slices)

        # Truncate all data slices to the minimum length and compute the timebase
        truncated_times = [d[:min_length, 0] - d[0, 0] for d in non_empty_slices]
        avg_time = np.mean(np.stack(truncated_times), axis=0)

        return avg_time

    def _unwrap_and_convert_to_degrees(self, data_rad):
        raw_deg = np.degrees(data_rad)
        unwrapped = np.unwrap(data_rad)
        unwrapped_deg = np.degrees(unwrapped)
        num_neg = np.sum(raw_deg < 0)
        num_pos = np.sum(raw_deg >= 0)
        if num_neg > num_pos and np.mean(unwrapped_deg) > 180:
            unwrapped_deg -= 360
        elif num_pos > num_neg and np.mean(unwrapped_deg) < -180:
            unwrapped_deg += 360
        return unwrapped_deg

    def shift_time(self, stage: str, t_shift: float):
        data = self.get_stage_data(stage)
        if data is None:
            return
        self.stage_data[stage] = data.copy()
        self.stage_data[stage][:, 0] += t_shift
        self.tables = self._generate_tables()

    def interpolate_to(self, stage: str, target_timestamps: np.ndarray) -> np.ndarray:
        data = self.get_stage_data(stage)
        if data is None:
            raise ValueError(f"Stage '{stage}' not found.")

        source_time = data[:, 0]
        interpolated = np.empty((len(target_timestamps), data.shape[1]))
        for col in range(data.shape[1]):
            if col == 0:
                interpolated[:, 0] = target_timestamps
            else:
                interpolated[:, col] = np.interp(
                    target_timestamps,
                    source_time,
                    data[:, col]
                )
        return interpolated

    def interpolate_raw_data(self, sliced_data: np.ndarray, target_timestamps: np.ndarray) -> np.ndarray:
        source_time = sliced_data[:, 0] - sliced_data[0, 0]
        interpolated = np.empty((len(target_timestamps), sliced_data.shape[1]))
        for col in range(sliced_data.shape[1]):
            if col == 0:
                interpolated[:, 0] = target_timestamps  # Set time directly
            else:
                interpolated[:, col] = np.interp(
                    target_timestamps,
                    source_time,
                    sliced_data[:, col]
                )
        return interpolated

    # Main computation methods
    def compute_ci(
        self,
        stage: str,
        trajectories: List[str],
        quantity: str,
        source: str = 'gt',
        entity: str = 'load',
        confidence: float = 0.95
    ):
        data_arrays = []

        for traj in trajectories:
            if traj not in self.tables[stage] or entity not in self.tables[stage][traj]:
                continue

            table = self.tables[stage][traj][entity]
            time = table.columns.to_numpy()

            if quantity == 'pos':
                data = np.stack([table[t][source][0] for t in time])
            elif quantity == 'rpy':
                data = np.stack([table[t][source][1] for t in time])
            else:
                raise ValueError("Unsupported quantity type. Use 'pos' or 'rpy'.")
            
            data_arrays.append(data)

        if not data_arrays:
            raise ValueError(f"No valid data found for {entity}.{source}.{quantity} CI computation.")

        # Ensure shape compatibility
        shapes = [arr.shape for arr in data_arrays]
        if len(set(shapes)) != 1:
            raise ValueError(f"Shape mismatch in {entity} {quantity}: {shapes}")

        data_matrix = np.stack(data_arrays)  # Shape: (N_trajectories, T, D)
        mean = np.mean(data_matrix, axis=0)
        std = np.std(data_matrix, axis=0, ddof=1)
        sem = std / np.sqrt(data_matrix.shape[0])

        z = norm.ppf(0.5 + confidence / 2)
        ci_lower = mean - z * sem
        ci_upper = mean + z * sem

        return mean, ci_lower, ci_upper


    def compute_error_dataframe(self):
        error_dict = {}

        # Iterate over stages and trajectory data
        for stage, traj_data in self.tables.items():
            selected = self.selected_trajectories.get(stage, list(traj_data.keys()))

            stage_errors = {}
            load_pos = []
            load_rpy = []
            drone_pos = {f'drone_{i+1}': [] for i in range(self.num_drones)}  # Initialize for each drone
            drone_rpy = {f'drone_{i+1}': [] for i in range(self.num_drones)}  # Initialize for each drone
            valid_trajs = []

            # Loop through the selected trajectories for the current stage
            for traj in selected:
                traj_key = f"traj_{traj+1}" if self.use_trajectory_slicing else traj
                if traj_key not in traj_data:
                    continue

                tables = traj_data[traj_key]
                time = tables['load'].columns.to_numpy()
                pos = np.stack([tables['load'][t]['gt'][0] for t in time])
                rpy = np.stack([tables['load'][t]['gt'][1] for t in time])

                load_pos.append(pos)
                load_rpy.append(rpy)
                valid_trajs.append(traj_key)

                # Now compute drone errors (assuming 'drone_1', 'drone_2', ..., 'drone_n' exist)
                for drone_idx in range(1, self.num_drones + 1):
                    drone_key = f'drone_{drone_idx}'
                    if drone_key in tables:
                        drone_tables = tables[drone_key]
                        drone_pos_data = np.stack([drone_tables[t]['gt'][0] for t in time])
                        drone_rpy_data = np.stack([drone_tables[t]['gt'][1] for t in time])

                        drone_pos[drone_key].append(drone_pos_data)
                        drone_rpy[drone_key].append(drone_rpy_data)

            if not valid_trajs:
                continue  # No valid trajectories for this stage

            # Calculate mean load position and orientation
            mean_pos = np.mean(np.stack(load_pos), axis=0)
            mean_rpy = np.mean(np.stack(load_rpy), axis=0)

            # Store load errors for each valid trajectory
            for idx, traj_key in enumerate(valid_trajs):
                pos_diff = load_pos[idx] - mean_pos
                rpy_diff = load_rpy[idx] - mean_rpy
                translational_error = np.linalg.norm(pos_diff, axis=1)  # Euclidean distance for each time point
                geodesic_rpy_error = utils_logs.compute_geodesic_distance(load_rpy[idx], mean_rpy)  # Use raw rpy for geodesic distance

                # Flatten the arrays for each time step (to ensure 1D per column)
                df = pd.DataFrame({
                    'x': pos_diff[:, 0], 'y': pos_diff[:, 1], 'z': pos_diff[:, 2],
                    'roll': rpy_diff[:, 0], 'pitch': rpy_diff[:, 1], 'yaw': rpy_diff[:, 2],
                    'translational_error': translational_error,  # Flatten to ensure 1D array
                    'geodesic_rpy_error': geodesic_rpy_error  # Flatten to ensure 1D array
                })
                stage_errors[('load', traj_key)] = df

            # Now handle drone errors
            drone_stage_errors = {}
            for drone_key in drone_pos:
                if drone_pos[drone_key]:  # If there are any positions collected for this drone
                    # Calculate mean drone position and orientation
                    mean_drone_pos = np.mean(np.stack(drone_pos[drone_key]), axis=0)
                    mean_drone_rpy = np.mean(np.stack(drone_rpy[drone_key]), axis=0)

                    # Calculate and store the errors for each drone
                    for idx, pos_data in enumerate(drone_pos[drone_key]):
                        pos_diff = pos_data - mean_drone_pos
                        rpy_diff = drone_rpy[drone_key][idx] - mean_drone_rpy
                        translational_error = np.linalg.norm(pos_diff, axis=1)  # Euclidean distance for each time point
                        geodesic_rpy_error = utils_logs.compute_geodesic_distance(drone_rpy[drone_key][idx], mean_drone_rpy)  # Use raw rpy for geodesic distance

                        # Flatten the arrays for each time step (to ensure 1D per column)
                        df = pd.DataFrame({
                            'x': pos_diff[:, 0], 'y': pos_diff[:, 1], 'z': pos_diff[:, 2],
                            'roll': rpy_diff[:, 0], 'pitch': rpy_diff[:, 1], 'yaw': rpy_diff[:, 2],
                            'translational_error': translational_error,  # Flatten to ensure 1D array
                            'geodesic_rpy_error': geodesic_rpy_error  # Flatten to ensure 1D array
                        })
                        drone_stage_errors[(drone_key, valid_trajs[idx])] = df

            # Add both load and drone errors to the stage_errors
            stage_errors.update(drone_stage_errors)

            error_dict[stage] = stage_errors

        if error_dict:
            multi_stage_frames = {}

            for stage, trajs in error_dict.items():
                if not trajs:
                    print(f"[DEBUG] Skipping empty stage: {stage}")
                    continue

                try:
                    # Concatenate the errors by trajectory, including drones and loads
                    stage_df = pd.concat(trajs, keys=trajs.keys(), names=['vehicle', 'trajectory'])
                    multi_stage_frames[stage] = stage_df
                except Exception as e:
                    print(f"[ERROR] Failed to concat trajectories in stage '{stage}': {e}")
                    continue

            # Final concatenation with proper names (stage is already in the multi_stage_frames keys)
            try:
                self.error_dataframe = pd.concat(multi_stage_frames, keys=multi_stage_frames.keys(), names=['stage'])
            except Exception as e:
                print(f"[ERROR] Final concat failed: {e}")
                self.error_dataframe = None
                                
        else:
            self.error_dataframe = None
            print("Warning: No valid error data found to compute error_dataframe.")


    def compute_aggregate_errors(self, weight_pos=1.0, weight_ori=1.0):
        if self.error_dataframe is None:
            raise ValueError("Error dataframe has not been computed. Call compute_error_dataframe first.")

        summary = {}
        drone_summary = {}
        load_summary = {}

        for stage in self.error_dataframe.index.get_level_values('stage').unique():
            df_stage = self.error_dataframe.loc[stage]

            # Initialize dictionaries to store drone and load-level errors
            drone_summary[stage] = {}
            load_summary[stage] = {}

            # Calculate the mean translational distance (mean of 'translational_error')
            mean_translational_distance = df_stage['translational_error'].mean()

            # Calculate the mean geodesic distance (mean of 'geodesic_rpy_error')
            mean_geodesic_distance = df_stage['geodesic_rpy_error'].mean()

            # Compute weighted final score (unitless, balanced)
            final_score = weight_pos * mean_translational_distance + weight_ori * mean_geodesic_distance

            summary[stage] = {
                'Mean Translational Distance': mean_translational_distance,
                'Mean Geodesic Distance': mean_geodesic_distance,
                'Final Score': final_score
            }

            # Now compute the errors per drone (assuming 'vehicle' level contains drone_1, drone_2, etc.)
            for vehicle_label in df_stage.index.get_level_values('vehicle').unique():
                if vehicle_label == 'load':  # Skip load processing to handle separately
                    continue

                # Extract the data for the specific drone
                df_drone = df_stage.xs(vehicle_label, level='vehicle')

                # Calculate the mean translational distance for this drone
                mean_translational_distance_drone = df_drone['translational_error'].mean()

                # Calculate the mean geodesic distance for this drone
                mean_geodesic_distance_drone = df_drone['geodesic_rpy_error'].mean()

                # Compute the final score for the drone
                final_score_drone = weight_pos * mean_translational_distance_drone + weight_ori * mean_geodesic_distance_drone

                drone_summary[stage][vehicle_label] = {
                    'Mean Translational Distance': mean_translational_distance_drone,
                    'Mean Geodesic Distance': mean_geodesic_distance_drone,
                    'Final Score': final_score_drone
                }

            # Now compute the errors per load (assuming 'load' is in the index)
            df_load = df_stage.xs('load', level='vehicle')  # Only extract 'load' data

            # Calculate the mean translational distance for the load
            mean_translational_distance_load = df_load['translational_error'].mean()

            # Calculate the mean geodesic distance for the load
            mean_geodesic_distance_load = df_load['geodesic_rpy_error'].mean()

            # Compute the final score for the load
            final_score_load = weight_pos * mean_translational_distance_load + weight_ori * mean_geodesic_distance_load

            load_summary[stage]['load'] = {
                'Mean Translational Distance': mean_translational_distance_load,
                'Mean Geodesic Distance': mean_geodesic_distance_load,
                'Final Score': final_score_load
            }

        # Convert to DataFrame for summary
        df_out = pd.DataFrame(summary).T

        # Add final row: mean across all stages
        df_out.loc['ALL'] = df_out.mean()

        # Convert to DataFrame for drone and load-specific summaries
        drone_df = pd.concat({stage: pd.DataFrame(drone_summary[stage]).T for stage in drone_summary}, axis=0)
        load_df = pd.concat({stage: pd.DataFrame(load_summary[stage]).T for stage in load_summary}, axis=0)

        return df_out, drone_df, load_df

    
    # Note: this does not provide a good baseline as the problem is underconstrained in yaw. \
    # Warm-starting with ground truth load pose just gives the ground truth pose back, while respecting the 
    # cable lengths. 
    def compute_load_pose_rod(self):
        """
        Estimate load pose from rigid rod assumption using PyManOpt optimization.
        Adds a new row 'rod' under the 'load' table alongside gt/des/qs.
        Caches results in a text file for reuse.
        """
        # Directory and file setup
        save_dir = os.path.join(os.path.dirname(self.filepath), "rod_cable_assumption")
        os.makedirs(save_dir, exist_ok=True)
        base_name = os.path.splitext(self.filename)[0]
        save_path = os.path.join(save_dir, f"{base_name}_rod_cable_assumption.txt")


        # If file exists, load results instead of recomputing
        if os.path.exists(save_path):
            with open(save_path, "r") as f:
                lines = [line.strip() for line in f.readlines()]

            stage, traj_key = None, None
            rod_data = {}
            for line in lines:
                if line.startswith("STAGE"):
                    stage = line.split(maxsplit=1)[1]
                    rod_data[stage] = {}
                elif line.startswith("TRAJ_KEY"):
                    traj_key = line.split(maxsplit=1)[1]
                    rod_data[stage][traj_key] = {}
                else:
                    parts = line.split()
                    t = float(parts[0])
                    t_opt = anp.array([float(parts[1]), float(parts[2]), float(parts[3])])
                    rpy = anp.array([float(parts[4]), float(parts[5]), float(parts[6])])
                    rod_data[stage][traj_key][t] = (t_opt, rpy)

            # Populate self.tables with loaded results
            for stage, trajs in rod_data.items():
                for traj_key, series in trajs.items():
                    if "load" in self.tables.get(stage, {}).get(traj_key, {}):
                        load_table = self.tables[stage][traj_key]["load"]
                        load_table.loc["rod"] = pd.Series(series)
            return

        # Otherwise, run optimization and save results
        O = anp.array([
            [0.1, 0.0, 0.03],
            [-0.05, 0.0866, 0.03],
            [-0.05, -0.0866, 0.03]
        ])
        L = self.cable_lengths

        manifold = Product([SpecialOrthogonalGroup(3), Euclidean(3)])
        solver = SteepestDescent(max_iterations=50, verbosity=0)

        with open(save_path, "w") as f:
            for stage, trajs in self.tables.items():
                f.write(f"STAGE {stage}\n")
                for traj_key, entities in trajs.items():
                    if not all(f"drone_{i+1}" in entities for i in range(self.num_drones)):
                        continue

                    drone_tables = [entities[f"drone_{i+1}"] for i in range(self.num_drones)]
                    load_table = entities["load"]
                    time = load_table.columns.to_numpy()

                    rod_series = {}
                    #R_prev, t_prev = anp.eye(3), anp.zeros(3)

                    offset_drone_cable = anp.array([0.04, 0.0, -0.08])

                    f.write(f"TRAJ_KEY {traj_key}\n")
                    for t in time:
                        # Warm start
                        R_prev = Rscipy.from_euler("ZYX", load_table[t]["gt"][1][::-1], degrees=True).as_matrix()
                        t_prev = load_table[t]["gt"][0]

                        # Build W (anchors) from drone positions
                        #W = anp.array([drone_tables[i][t]["gt"][0] for i in range(self.num_drones)])
                        W = []
                        for i in range(self.num_drones):
                            pos = drone_tables[i][t]["gt"][0]   # COM in world frame
                            rpy = drone_tables[i][t]["gt"][1]   # roll, pitch, yaw (deg)
                            R_drone = Rscipy.from_euler("ZYX", rpy[::-1], degrees=True).as_matrix()
                            attach_world = pos + R_drone @ offset_drone_cable
                            W.append(attach_world)
                        W = anp.array(W)


                        # Cost function for this timestep
                        @autograd(manifold)
                        def cost(R, trans):
                            residuals = [anp.linalg.norm(W[i] - (R @ O[i] + trans)) - L for i in range(3)]
                            return 0.5 * anp.sum(anp.square(residuals))

                        problem = Problem(manifold=manifold, cost=cost)
                        result = solver.run(problem, initial_point=(R_prev, t_prev))
                        R_opt, t_opt = result.point

                        rpy = Rscipy.from_matrix(R_opt).as_euler("ZYX", degrees=True)[::-1]

                        rod_series[t] = (t_opt, rpy)

                        # Save line
                        f.write(f"{t} {t_opt[0]} {t_opt[1]} {t_opt[2]} {rpy[0]} {rpy[1]} {rpy[2]}\n")

                        # Warm start
                        #R_prev, t_prev = R_opt, t_opt

                    # Insert into load table
                    load_table.loc["rod"] = pd.Series(rod_series)

        return


    ## PRINTING ##
    def print_summary_statistics(self, weight_pos=1.0, weight_ori=1.0):
        """
        Print the summary statistics including the aggregated errors for stages, 
        drones, and loads. Translational distance is in mm, and geodesic distance is in degrees.
        """
        # Convert Translational Distance to Millimeters and Geodesic Distance to Degrees
        # df_out = self.error_dataframe['stage_level']
        # drone_df = self.error_dataframe['drone_level']
        # load_df = self.error_dataframe['load_level']

        df_out, drone_df, load_df = self.compute_aggregate_errors(weight_pos=weight_pos, weight_ori=weight_ori)

        # Convert Translational Distance to Millimeters and Geodesic Distance to Degrees
        df_out['Mean Translational Distance'] *= 1000  # Convert from meters to millimeters
        df_out['Mean Geodesic Distance'] = df_out['Mean Geodesic Distance'] 
        
        # Print Aggregated Error Summary for Stages
        print("\n### Aggregated Error Summary (Stage level) (Translational in mm, Geodesic in Degrees) ###")
        print(df_out.to_string())  # Use .to_string() for a cleaner format
        print("\n")

        # Convert Translational Distance to Millimeters and Geodesic Distance to Degrees for Drone-Specific Errors
        drone_df['Mean Translational Distance'] *= 1000  # Convert from meters to millimeters
        drone_df['Mean Geodesic Distance'] = drone_df['Mean Geodesic Distance']
        
        # Print Drone-Specific Errors
        print("### Drone-Specific Errors (Translational in mm, Geodesic in Degrees) ###")
        print(tabulate(drone_df, headers='keys', tablefmt='pretty'))  # Pretty-print using tabulate
        print("\n")

        # Reorder columns in load_df to match the desired order and convert translational distance to mm
        desired_order = ['Mean Translational Distance', 'Mean Geodesic Distance', 'Final Score']
        load_df['Mean Translational Distance'] *= 1000  # Convert from meters to millimeters
        load_df['Mean Geodesic Distance'] = load_df['Mean Geodesic Distance']
        
        # Reorder the columns to match the desired order
        load_df = load_df[desired_order]
        
        # Print Load-Specific Errors
        print("### Load-Specific Errors (Translational in mm, Geodesic in Degrees) ###")
        print(tabulate(load_df, headers='keys', tablefmt='pretty'))  # This will keep the multi-index labels
        print("\n")

    ## PLOTTING ##
    def plot_stage(self, stage: str, trajectory: Union[str, List[str]] = None, show_qs: bool = True, show_des: bool = True, show_load_rod: bool = False, source_name: str = "", show_mean_ci_gt: bool = False, show_mean_ci_qs: bool = False, plot_raw_trajectories: bool = True, plot_3d : bool = True, confidence: float = 0.95):
        if stage not in self.tables:
            print(f"Stage '{stage}' not found.")
            return
        
        # Select trajectories to plot
        if trajectory is None:
            selected_trajectories = self.selected_trajectories.get(stage, [])
            trajectory = [f"traj_{i+1}" for i in selected_trajectories] if selected_trajectories else "full"

        trajectories = [trajectory] if isinstance(trajectory, str) else trajectory

        suffix = f"{stage} - {source_name}" if source_name else stage

        fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
        fig1.canvas.manager.set_window_title(f"Load Data - {suffix}")

        drone_figs = []
        for i in range(self.num_drones):
            fig, axs = plt.subplots(2, 1, figsize=(10, 8))
            fig.canvas.manager.set_window_title(f"Drone {i+1} Pose - {suffix}")
            drone_figs.append((fig, axs))

        tables = self.tables[stage][trajectories[0]]
        time = tables['load'].columns.to_numpy()

        # --- Mean 3D Trajectories ---
        if plot_3d:
            # Set up 3D plot
            fig3 = plt.figure(figsize=(10, 8))
            ax3d = fig3.add_subplot(111, projection='3d')
            fig3.canvas.manager.set_window_title(f"3D Trajectory - {suffix}")

            # Load GT mean trajectory
            pos_mean_load, _, _ = self.compute_ci(stage, trajectories, quantity='pos', source='gt', confidence=confidence, entity='load')
            ax3d.plot(pos_mean_load[:, 0], pos_mean_load[:, 1], pos_mean_load[:, 2], label='Load GT (mean)', color='r')

            # Load ROD mean (NEW)
            if show_load_rod: #and all("rod" in self.tables[stage][traj]["load"].loc for traj in trajectories)
                pos_mean_rod, _, _ = self.compute_ci(stage, trajectories, quantity='pos', source='rod',
                                                    confidence=confidence, entity='load')
                ax3d.plot(pos_mean_rod[:, 0], pos_mean_rod[:, 1], pos_mean_rod[:, 2],
                        label='Load ROD (mean)', color='m', linestyle='dashdot')
            
            # Drone GT mean trajectories
            for i in range(self.num_drones):
                pos_mean_drone, _, _ = self.compute_ci(stage, trajectories, quantity='pos', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                ax3d.plot(pos_mean_drone[:, 0], pos_mean_drone[:, 1], pos_mean_drone[:, 2], label=f'Drone {i+1} GT (mean)', color=DRONE_COLORS[i])

        # --- Load Desired ---
        if show_des:
            pos_load_des = np.stack([tables['load'][t]['des'][0] for t in time])
            rpy_load_des = np.stack([tables['load'][t]['des'][1] for t in time])
            for i, label in enumerate(['x', 'y', 'z']):
                ax1[0].plot(time, pos_load_des[:, i], linestyle='dashed', label=f'des_{label}', color=COLOR_MAP[label])
            for i, label in enumerate(['roll', 'pitch', 'yaw']):
                ax1[1].plot(time, rpy_load_des[:, i], linestyle='dashed', label=f'des_{label}', color=COLOR_MAP[label])

        # --- Drone Desired ---
        if show_des:
            for i in range(self.num_drones):
                drone_table = tables[f'drone_{i+1}']
                pos_drone_des = np.stack([drone_table[t]['des'][0] for t in time])
                rpy_drone_des = np.stack([drone_table[t]['des'][1] for t in time])
                axs = drone_figs[i][1]
                for j, label in enumerate(['x', 'y', 'z']):
                    axs[0].plot(time, pos_drone_des[:, j], linestyle='dashed', label=f'des_{label}', color=COLOR_MAP[label])
                for j, label in enumerate(['roll', 'pitch', 'yaw']):
                    axs[1].plot(time, rpy_drone_des[:, j], linestyle='dashed', label=f'des_{label}', color=COLOR_MAP[label])

        # --- Raw Trajectories ---
        if plot_raw_trajectories:
            for traj in trajectories:
                if traj not in self.tables[stage]:
                    continue
                tables = self.tables[stage][traj]
                time = tables['load'].columns.to_numpy()
                pos_load_gt = np.stack([tables['load'][t]['gt'][0] for t in time])
                rpy_load_gt = np.stack([tables['load'][t]['gt'][1] for t in time])
                if show_qs:
                    pos_load_qs = np.stack([tables['load'][t]['qs'][0] for t in time])
                    rpy_load_qs = np.stack([tables['load'][t]['qs'][1] for t in time])
                if show_load_rod: #and "rod" in tables['load'].loc
                    pos_load_rod = np.stack([tables['load'][t]['rod'][0] for t in time])
                    rpy_load_rod = np.stack([tables['load'][t]['rod'][1] for t in time])
                
                #ax3d.plot(pos_load_gt[:, 0], pos_load_gt[:, 1], pos_load_gt[:, 2], color='r')
                # Load GT
                for i, label in enumerate(['x', 'y', 'z']):
                    ax1[0].plot(time, pos_load_gt[:, i], color=COLOR_MAP[label])
                    if show_qs:
                        ax1[0].plot(time, pos_load_qs[:, i], linestyle='dotted', color=COLOR_MAP[label])
                    if show_load_rod:
                        ax1[0].plot(time, pos_load_rod[:, i], linestyle='dashdot', label=f'rod_{label}', color=COLOR_MAP[label])
                for i, label in enumerate(['roll', 'pitch', 'yaw']):
                    ax1[1].plot(time, rpy_load_gt[:, i], color=COLOR_MAP[label])
                    if show_qs:
                        ax1[1].plot(time, rpy_load_qs[:, i], linestyle='dotted', color=COLOR_MAP[label])
                    if show_load_rod:
                        ax1[1].plot(time, rpy_load_rod[:, i], linestyle='dashdot', label=f'rod_{label}', color=COLOR_MAP[label])

                # Drone raw trajectories
                for i in range(self.num_drones):
                    drone_table = tables[f'drone_{i+1}']
                    pos_gt = np.stack([drone_table[t]['gt'][0] for t in time])
                    rpy_gt = np.stack([drone_table[t]['gt'][1] for t in time])
                    axs = drone_figs[i][1]
                    for j, label in enumerate(['x', 'y', 'z']):
                        axs[0].plot(time, pos_gt[:, j], color=COLOR_MAP[label])
                    for j, label in enumerate(['roll', 'pitch', 'yaw']):
                        axs[1].plot(time, rpy_gt[:, j], color=COLOR_MAP[label])

        # --- Load Mean CI ---
        if show_mean_ci_gt:
            pos_mean, pos_lower, pos_upper = self.compute_ci(stage, trajectories, quantity='pos', source='gt', confidence=confidence)
            rpy_mean, rpy_lower, rpy_upper = self.compute_ci(stage, trajectories, quantity='rpy', source='gt', confidence=confidence)
            for i, label in enumerate(['x', 'y', 'z']):
                ax1[0].plot(time, pos_mean[:, i], label=f'gt_{label}', color=COLOR_MAP[label])
                ax1[0].fill_between(time, pos_lower[:, i], pos_upper[:, i], color=COLOR_MAP[label], alpha=0.3)
            for i, label in enumerate(['roll', 'pitch', 'yaw']):
                ax1[1].plot(time, rpy_mean[:, i], label=f'gt_{label}', color=COLOR_MAP[label])
                ax1[1].fill_between(time, rpy_lower[:, i], rpy_upper[:, i], color=COLOR_MAP[label], alpha=0.3)

            # Load ROD mean CI (NEW)
            if show_load_rod:
                pos_mean, pos_lower, pos_upper = self.compute_ci(stage, trajectories, quantity='pos', source='rod', confidence=confidence)
                rpy_mean, rpy_lower, rpy_upper = self.compute_ci(stage, trajectories, quantity='rpy', source='rod', confidence=confidence)
                for i, label in enumerate(['x', 'y', 'z']):
                    ax1[0].plot(time, pos_mean[:, i], linestyle='dashdot', label=f'rod_{label}', color=COLOR_MAP[label])
                    ax1[0].fill_between(time, pos_lower[:, i], pos_upper[:, i], color=COLOR_MAP[label], alpha=0.2)
                for i, label in enumerate(['roll', 'pitch', 'yaw']):
                    ax1[1].plot(time, rpy_mean[:, i], linestyle='dashdot', label=f'rod_{label}', color=COLOR_MAP[label])
                    ax1[1].fill_between(time, rpy_lower[:, i], rpy_upper[:, i], color=COLOR_MAP[label], alpha=0.2)

        if show_qs and show_mean_ci_qs:
            pos_mean, pos_lower, pos_upper = self.compute_ci(stage, trajectories, quantity='pos', source='qs', confidence=confidence)
            rpy_mean, rpy_lower, rpy_upper = self.compute_ci(stage, trajectories, quantity='rpy', source='qs', confidence=confidence)
            for i, label in enumerate(['x', 'y', 'z']):
                ax1[0].plot(time, pos_mean[:, i], linestyle='dotted', label=f'qs_{label}', color=COLOR_MAP[label])
                ax1[0].fill_between(time, pos_lower[:, i], pos_upper[:, i], color=COLOR_MAP[label], alpha=0.2)
            for i, label in enumerate(['roll', 'pitch', 'yaw']):
                ax1[1].plot(time, rpy_mean[:, i], linestyle='dotted', label=f'qs_{label}', color=COLOR_MAP[label])
                ax1[1].fill_between(time, rpy_lower[:, i], rpy_upper[:, i], color=COLOR_MAP[label], alpha=0.2)

        # --- Drone Mean CI ---
        for i in range(self.num_drones):
            axs = drone_figs[i][1]
            if show_mean_ci_gt:
                pos_mean, pos_lower, pos_upper = self.compute_ci(stage, trajectories, quantity='pos', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                rpy_mean, rpy_lower, rpy_upper = self.compute_ci(stage, trajectories, quantity='rpy', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                for j, label in enumerate(['x', 'y', 'z']):
                    axs[0].plot(time, pos_mean[:, j], label=f'gt_{label}', color=COLOR_MAP[label])
                    axs[0].fill_between(time, pos_lower[:, j], pos_upper[:, j], color=COLOR_MAP[label], alpha=0.3)
                for j, label in enumerate(['roll', 'pitch', 'yaw']):
                    axs[1].plot(time, rpy_mean[:, j], label=f'gt_{label}', color=COLOR_MAP[label])
                    axs[1].fill_between(time, rpy_lower[:, j], rpy_upper[:, j], color=COLOR_MAP[label], alpha=0.3)
            if show_qs and show_mean_ci_qs:
                pos_mean, pos_lower, pos_upper = self.compute_ci(stage, trajectories, quantity='pos', source='qs', confidence=confidence, entity=f'drone_{i+1}')
                rpy_mean, rpy_lower, rpy_upper = self.compute_ci(stage, trajectories, quantity='rpy', source='qs', confidence=confidence, entity=f'drone_{i+1}')
                for j, label in enumerate(['x', 'y', 'z']):
                    axs[0].plot(time, pos_mean[:, j], linestyle='dotted', label=f'qs_{label}', color=COLOR_MAP[label])
                    axs[0].fill_between(time, pos_lower[:, j], pos_upper[:, j], color=COLOR_MAP[label], alpha=0.2)
                for j, label in enumerate(['roll', 'pitch', 'yaw']):
                    axs[1].plot(time, rpy_mean[:, j], linestyle='dotted', label=f'qs_{label}', color=COLOR_MAP[label])
                    axs[1].fill_between(time, rpy_lower[:, j], rpy_upper[:, j], color=COLOR_MAP[label], alpha=0.2)

        ax1[0].set_ylabel('Position (m)', fontsize=axes_label_font_size)
        ax1[0].legend(fontsize=legend_font_size)
        ax1[0].tick_params(labelsize=ticks_font_size)
        utils_logs.set_custom_ticks(ax1[0], 'x', time_ticks)
        utils_logs.set_custom_ticks(ax1[0], 'y', y_ticks_pos)

        ax1[1].set_ylabel('Orientation (deg)', fontsize=axes_label_font_size)
        ax1[1].set_xlabel('Time (s)', fontsize=axes_label_font_size)
        ax1[1].legend(fontsize=legend_font_size)
        ax1[1].tick_params(labelsize=ticks_font_size)
        utils_logs.set_custom_ticks(ax1[1], 'x', time_ticks)
        utils_logs.set_custom_ticks(ax1[1], 'y', y_ticks_ori)

        for _, axs in drone_figs:
            axs[0].set_ylabel('Position (m)', fontsize=axes_label_font_size)
            axs[0].legend(fontsize=legend_font_size)
            axs[0].tick_params(labelsize=ticks_font_size)
            utils_logs.set_custom_ticks(axs[0], 'x', time_ticks)
            utils_logs.set_custom_ticks(axs[0], 'y', y_ticks_pos)
            axs[1].set_ylabel('Orientation (deg)', fontsize=axes_label_font_size)
            axs[1].set_xlabel('Time (s)', fontsize=axes_label_font_size)
            axs[1].legend(fontsize=legend_font_size)
            axs[1].tick_params(labelsize=ticks_font_size)
            utils_logs.set_custom_ticks(axs[1], 'x', time_ticks)
            utils_logs.set_custom_ticks(axs[1], 'y', y_ticks_ori)

        if plot_3d:
            ax3d.set_xlabel('X')
            ax3d.set_ylabel('Y')
            ax3d.set_zlabel('Z')
            ax3d.legend(fontsize=legend_font_size)
            utils_logs.set_custom_ticks(ax3d, 'x', x_ticks_traj)
            utils_logs.set_custom_ticks(ax3d, 'y', y_ticks_traj)
            utils_logs.set_custom_ticks(ax3d, 'z', z_ticks_traj)

        plt.tight_layout()
        plt.show(block=False)
