# LogCabinet.py
from typing import List, Optional, Any, Union
from Logfile import Logfile
from typing import Dict
import pandas as pd
import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt
import utils_logs
from tabulate import tabulate
from utils_logs import title_font_size, axes_label_font_size, legend_font_size, ticks_font_size, line_width,\
                       time_ticks, y_ticks_pos, y_ticks_ori, x_ticks_traj, y_ticks_traj, z_ticks_traj, \
                       COLOR_MAP, DRONE_COLORS, LOAD_COLORS

class LogCabinet:
    def __init__(self, 
                logs_1: Optional[List[Logfile]] = None, 
                logs_2: Optional[List[Logfile]] = None, 
                num_drones: int = 3,
                logs_2_variations: Optional[Dict[Any, List[Any]]] = None, 
                common_time: Optional[Dict[str, Optional[np.ndarray]]] = None):

        self.logs_1: List[Logfile] = logs_1 or []
        self.logs_2: List[Logfile] = logs_2 or []
        self.num_drones = num_drones
        self.sim_variation_dfs = {}  # key: variation value -> value: (df_out, drone_df, load_df)

        # Set common time for all logs if not input
        if common_time is None:
            self.common_time = self.logs_1[0].get_common_time() # Take the common time from the first logfile
        else: 
            self.common_time = common_time

        self.align_timebases(logs=self.logs_1, common_time=self.common_time)
        self.align_timebases(logs=self.logs_2, common_time=self.common_time)

        # Create aggreated dataframes for real and sim logs
        if len(self.logs_1) == 0:
            print("logs_1 is empty")    
        else:
            self.aggregated_df_1 = self.aggregate_tables(logs=self.logs_1)

        if len(self.logs_2) == 0:
            print("logs_2 is empty")
        else:
            self.aggregated_df_2  = self.aggregate_tables(logs=self.logs_2)

        # If variations are provided
        if logs_2_variations is not None:
            for var_value, logfiles_list in logs_2_variations.items():
                self.align_timebases(logs=logfiles_list, common_time=self.common_time)
                aggregated_df_2_var = self.aggregate_tables(logs=logfiles_list)
                df_out, drone_df, load_df = self.compute_comparison_error_dataframe(
                    self.aggregated_df_1, aggregated_df_2_var, confidence=0.95, weight_pos=1.0, weight_ori=1.0
                )
                self.sim_variation_dfs[var_value] = (df_out, drone_df, load_df)

    ### DATA PREPARATION ###
    def align_timebases(self, logs: List[Logfile], common_time) -> None:
        """
        Aligns the timebases of all logs in the provided list.
        This modifies the logs in place to ensure they share a common timebase.
        
        Args:
            logs (List[Logfile]): List of Logfile instances to align.
        """
        # Use the common time from the first logfile if not provided
        if common_time is None:
            common_time = logs[0].get_common_time()  

        # Round the common time to avoid floating point issues
        for stage, time in common_time.items():
            common_time[stage] = time.round(2) #3

        # Set the common time for all logfiles in the list
        for log in logs:
            log.set_common_times(common_time)
            log.rebuild()  # Rebuild the log to apply the new common time

    def aggregate_tables(self, logs: List[Logfile]) -> pd.DataFrame:
        """
        Aggregates the selected trajectories from all Logfile instances, 
        appending the logfile number to the trajectory name and includes the entity level.
        
        Args:
            logs (List[Logfile]): List of Logfile instances to aggregate.

        Returns:
            pd.DataFrame: A DataFrame with an index consisting of [logfile, stage, trajectory, entity].
        """
        aggregated_data = []

        # Loop through each logfile
        for logfile_idx, lg in enumerate(logs, start=1):
            # Loop through each stage in the logfile
            for stage, stage_data in lg.tables.items():
                # Loop through each trajectory in the stage
                for trajectory, entity_data in stage_data.items():
                    # Loop through each entity in the trajectory (e.g., load, drone)
                    for entity, source_data in entity_data.items():
                        # Add a new level to the index with the logfile number
                        #data_copy = source_data.copy()
                        # data_copy['logfile'] = logfile_idx  # Add logfile index to the DataFrame

                        # Append the data to the list
                        # aggregated_data.append((logfile_idx, stage, trajectory, entity, data_copy))
                        # multi_index = pd.MultiIndex.from_tuples(
                        #     [(logfile_idx, stage, trajectory, entity)], 
                        #     names=['logfile', 'stage', 'trajectory', 'entity']
                        #    )
                        
                        # # Append the data to the list, using the multi_index as the index
                        # aggregated_data.append(pd.DataFrame(data, index=multi_index))
                        # Now loop through each 'source' level (gt, qs, des) for each entity
                        #for source, data in source_data.items():

                        times = source_data.columns

                        # Create a MultiIndex for each combination of logfile, stage, trajectory, entity, source, and time
                        multi_index = pd.MultiIndex.from_product(
                            [[logfile_idx], [stage], [trajectory], [entity], source_data.index, times],
                            names=['logfile', 'stage', 'trajectory', 'entity', 'source', 'time']
                        )

                        data = source_data.values.flatten()
                        aggregated_data.append(pd.DataFrame(data, index=multi_index))

                        # for source in ['gt', 'des', 'qs']:
                        #     data = source_data.loc[source]

                        #     # # Add a new level to the index with the logfile number, stage, trajectory, entity, and source
                        #     # multi_index = pd.MultiIndex.from_tuples(
                        #     #     [(logfile_idx, stage, trajectory, entity, source)], 
                        #     #     names=['logfile', 'stage', 'trajectory', 'entity', 'source']
                        #     # )
                            
                        #     # # Append the data to the list, using the multi_index as the index
                        #     # aggregated_data.append(pd.DataFrame(data, index=multi_index))

                        #     # Loop over the times (index of source_data)

                        #     for time in data.index:
                        #         # Create the MultiIndex with 'logfile', 'stage', 'trajectory', 'entity', 'source', and 'time'
                        #         multi_index = pd.MultiIndex.from_tuples(
                        #             [(logfile_idx, stage, trajectory, entity, source, time)], 
                        #             names=['logfile', 'stage', 'trajectory', 'entity', 'source', 'time']
                        #         )
                                
                        #         # Append the data at this specific time to the list, using the multi_index as the index
                        #         aggregated_data.append(pd.DataFrame([data.loc[time]], index=multi_index))

        # Concatenate all the DataFrames into one, using the multi-level index
        aggregated_df = pd.concat(aggregated_data)

        # Create the keys for the MultiIndex
        # keys = [
        #     (logfile_idx, stage, trajectory, entity) 
        #     for logfile_idx, stage, trajectory, entity, data in aggregated_data
        # ]

        # # Concatenate all the dataframes into one, adding the multi-level index
        # aggregated_df = pd.concat(
        #     [data_copy for _, _, _, _, data_copy in aggregated_data],
        #     keys=keys,
        #     names=['logfile', 'stage', 'trajectory', 'entity']  # Names for the MultiIndex
        # )

        return aggregated_df

    ### COMPUTATION ###
    def compute_ci(
        self,
        stage: str,
        aggregated_df: pd.DataFrame,
        quantity: str,
        source: str = 'gt',
        entity: str = 'load',
        confidence: float = 0.95
    ):
        """
        Compute the confidence interval for the specified quantity ('pos' or 'rpy') using the 
        aggregated dataframe. This function calculates the CI across all logfiles in the LogCabinet.

        Args:
            stage (str): The stage to compute the CI for.
            aggregated_df (pd.DataFrame): The aggregated dataframe containing data from multiple logfiles.
            quantity (str): The quantity to compute the CI for ('pos' or 'rpy').
            source (str): The source of data ('gt' or 'qs').
            entity (str): The entity for which CI is computed ('load', 'drone_x').
            confidence (float): The confidence level for the CI (default is 0.95).

        Returns:
            mean (np.ndarray): The mean of the data for each timepoint.
            ci_lower (np.ndarray): The lower bound of the confidence interval.
            ci_upper (np.ndarray): The upper bound of the confidence interval.
        """

        # Check if the stage, entity, quantity, and source exist in the dataframe
        if stage not in aggregated_df.index.get_level_values('stage').unique():
            raise ValueError(f"Stage '{stage}' not found in the aggregated dataframe.")
        
        if entity not in aggregated_df.index.get_level_values('entity').unique():
            raise ValueError(f"Entity '{entity}' not found in the aggregated dataframe.")
        
        if quantity not in ['pos', 'rpy']:
            raise ValueError(f"Unsupported quantity '{quantity}'. Valid values are 'pos' or 'rpy'.")
        
        if source not in ['gt', 'qs', 'des']:
            raise ValueError(f"Unsupported source '{source}'. Valid values are 'gt', 'qs', or 'des'.")

        # Directly filter the data by stage, entity, quantity, and source
        try:
            table = aggregated_df.xs(stage, level='stage').xs(entity, level='entity')
            table = table.xs(source, level='source') #.xs(quantity, level='quantity', axis=1)
        except KeyError:
            raise ValueError(f"Data not found for the given combination of stage '{stage}', entity '{entity}', quantity '{quantity}', and source '{source}'.")

        # The time points are not in columns but as part of the index, so we get them from the time level
        time_levels = table.index.get_level_values('time')

        # Initialize a list to collect data for each timepoint
        data_arrays = []

        for time in time_levels.unique():
            # Extract the data for each time point
            time_data = table.xs(time, level='time')
            
            # # If there is valid data for this timepoint, append it
            # if not time_data.isna().all().all():  # Skip if all data is NaN
            #     data_arrays.append(time_data.values.flatten())
            # else:
            #     # Handle the case where the data is all NaN at this time point
            #     data_arrays.append([np.nan] * len(time_data))

            if quantity == 'pos':
                # Select the first part of the tuple (position)
                selected_data = np.array([tup[0][0] for tup in time_data.values])
            elif quantity == 'rpy':
                # Select the second part of the tuple (orientation)
                selected_data = np.array([tup[0][1] for tup in time_data.values])
            else:
                raise ValueError(f"Unsupported quantity '{quantity}'. Valid values are 'pos' or 'rpy'.")

            # If there is valid data (not NaN), append it
            if not np.isnan(selected_data).all():
                data_arrays.append(selected_data)
            else:
                # Handle the case where the data is all NaN at this timepoint
                data_arrays.append([np.nan] * len(selected_data))


        # Stack the data into a matrix (rows = timepoints, columns = values)
        data_matrix = np.stack(data_arrays)

        # Compute the mean, standard deviation, and standard error of the mean across samples (axis=1)
        mean = np.nanmean(data_matrix, axis=1)
        std = np.nanstd(data_matrix, axis=1, ddof=1)
        sem = std / np.sqrt(data_matrix.shape[1])

        # Calculate the confidence intervals
        z = norm.ppf(0.5 + confidence / 2)
        ci_lower = mean - z * sem
        ci_upper = mean + z * sem

        return mean, ci_lower, ci_upper

    def compute_comparison_error_dataframe(self, real_aggregated_df, sim_aggregated_df, confidence=0.95, weight_pos=1.0, weight_ori=1.0):
        error_dict = {}

        # Initialize data for summary
        summary = {
            'Mean Translational Distance': [],
            'Mean Geodesic Distance': [],
            'Stage': [],
            'Entity': [],
            'Final Score': []
        }
        drone_summary = {}
        load_summary = {}

        # Get all unique stages in the real dataset
        stages = real_aggregated_df.index.get_level_values('stage').unique()

        # Loop through each stage and each entity
        for stage in stages:
            # Get all unique entities for this stage
            entities = real_aggregated_df.xs(stage, level='stage').index.get_level_values('entity').unique()

            # Create a dictionary to store the errors for each entity in this stage
            stage_errors = {}

            for entity in entities:
                try:
                    # Compute CI for real data
                    pos_mean_real, _, _ = self.compute_ci(stage, real_aggregated_df, quantity='pos', source='gt', confidence=confidence, entity=entity)
                    rpy_mean_real, _, _ = self.compute_ci(stage, real_aggregated_df, quantity='rpy', source='gt', confidence=confidence, entity=entity)

                    # Compute CI for simulated data
                    pos_mean_sim, _, _ = self.compute_ci(stage, sim_aggregated_df, quantity='pos', source='gt', confidence=confidence, entity=entity)
                    rpy_mean_sim, _, _ = self.compute_ci(stage, sim_aggregated_df, quantity='rpy', source='gt', confidence=confidence, entity=entity)

                    # Compute the differences between real and simulated data
                    pos_diff = pos_mean_real - pos_mean_sim
                    rpy_diff = rpy_mean_real - rpy_mean_sim

                    # Compute translational error (Euclidean distance)
                    translational_error = np.linalg.norm(pos_diff, axis=1)

                    # Compute geodesic RPY error
                    geodesic_rpy_error = utils_logs.compute_geodesic_distance(rpy_mean_real, rpy_mean_sim)

                    # Create a DataFrame to store the errors
                    error_df = pd.DataFrame({
                        'x': pos_diff[:, 0], 'y': pos_diff[:, 1], 'z': pos_diff[:, 2],
                        'roll': rpy_diff[:, 0], 'pitch': rpy_diff[:, 1], 'yaw': rpy_diff[:, 2],
                        'translational_error': translational_error,
                        'geodesic_rpy_error': geodesic_rpy_error
                    })

                    # Store the errors for this entity
                    stage_errors[entity] = error_df

                    # Calculate mean translational and geodesic distances
                    mean_translational_distance = translational_error.mean()
                    mean_geodesic_distance = geodesic_rpy_error.mean()

                    # Store the summary stats for later printing
                    summary['Mean Translational Distance'].append(mean_translational_distance)
                    summary['Mean Geodesic Distance'].append(mean_geodesic_distance)
                    summary['Stage'].append(stage)
                    summary['Entity'].append(entity)
                    summary['Final Score'].append(mean_translational_distance + mean_geodesic_distance)

                    # Store drone/load-specific summaries
                    if entity.startswith('drone'):
                        if stage not in drone_summary:
                            drone_summary[stage] = []
                        drone_summary[stage].append({
                            'Mean Translational Distance': mean_translational_distance,
                            'Mean Geodesic Distance': mean_geodesic_distance,
                            'Final Score': mean_translational_distance + mean_geodesic_distance
                        })
                    else:
                        if stage not in load_summary:
                            load_summary[stage] = []
                        load_summary[stage].append({
                            'Mean Translational Distance': mean_translational_distance,
                            'Mean Geodesic Distance': mean_geodesic_distance,
                            'Final Score': mean_translational_distance + mean_geodesic_distance
                        })

                except Exception as e:
                    print(f"[ERROR] Error computing data for stage '{stage}', entity '{entity}': {e}")
                    continue

            # Store the errors for this stage
            if stage_errors:
                error_dict[stage] = stage_errors

        # Convert the summary into a DataFrame
        df_out = pd.DataFrame(summary).T

        # Convert to DataFrame for drone and load-specific summaries
        drone_df = pd.concat({stage: pd.DataFrame(drone_summary[stage]) for stage in drone_summary}, axis=0)
        load_df = pd.concat({stage: pd.DataFrame(load_summary[stage]) for stage in load_summary}, axis=0)

        # Add final row: mean across all stages
        drone_df.loc['ALL'] = drone_df.mean()
        load_df.loc['ALL'] = load_df.mean()

        return df_out, drone_df, load_df

    ### PRINTING ###
    def print_summary_statistics(self, weight_pos=1.0, weight_ori=1.0):
        """
        Print the summary statistics including the aggregated errors for stages, 
        drones, and loads. Translational distance is in mm, and geodesic distance is in degrees.
        """
        df_out, drone_df, load_df = self.compute_comparison_error_dataframe(
            self.aggregated_df_1, self.aggregated_df_2, confidence=0.95, weight_pos=weight_pos, weight_ori=weight_ori
        )

        # Convert Translational Distance to Millimeters and Geodesic Distance to Degrees
        # Handle 'df_out' by first ensuring it's in a transposed form with stages as index
        df_out = df_out.T  # Transpose so stages are in the index (if not already)

        # Now we safely access the 'Mean Translational Distance' and 'Mean Geodesic Distance'
        # Check if the necessary columns exist and multiply them by 1000 for the conversion
        if 'Mean Translational Distance' in df_out.columns:
            df_out['Mean Translational Distance'] *= 1000  # Convert from meters to millimeters
        if 'Mean Geodesic Distance' in df_out.columns:
            df_out['Mean Geodesic Distance'] = df_out['Mean Geodesic Distance']  # Already in degrees

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


    ### PLOTTING ###
    def plot_group(
        self,
        group: str,
        stage: str,
        sliced_trajs: bool = False,
        show_qs: bool = False,
        show_des: bool = False,
        show_mean_ci_gt: bool = True,
        show_mean_ci_qs: bool = False,
        plot_raw_trajectories: bool = False,
        plot_3d: bool = True,
        confidence: float = 0.95
    ):
        """
        Call Logfile.plot_stage() for each log in the chosen group ('real' or 'sim').
        Each figure window will have the source_name set to include the log filename.
        """
        if group not in ("real", "sim"):
            raise ValueError("group must be 'real' or 'sim'")

        logs = self.logs_1 if group == "real" else self.logs_2
        if not logs:
            print(f"[LogCabinet] No logs in group '{group}'.")
            return

        for lg in logs:
            # Select the trajectories to plot
            selected_trajectories = lg.selected_trajectories.get(stage, [])
            selected_trajectories = [f"traj_{i+1}" for i in selected_trajectories] if sliced_trajs else "full"
            
            # Call the plot_stage method for each Logfile with the selected trajectories
            lg.plot_stage(
                stage=stage,
                trajectory=selected_trajectories,
                show_qs=show_qs,
                show_des=show_des,
                source_name=f"{group}: {lg.filename}",
                show_mean_ci_gt=show_mean_ci_gt,
                show_mean_ci_qs=show_mean_ci_qs,
                plot_raw_trajectories=plot_raw_trajectories,
                plot_3d=plot_3d,
                confidence=confidence,
            )


    def plot_stage(self, 
                    aggregated_df_1: pd.DataFrame,  # First aggregated dataframe (real data)
                    aggregated_df_2: pd.DataFrame = None,  # Second aggregated dataframe (simulated data, optional)
                    stage: str = "",
                    show_qs: bool = True,
                    show_des: bool = True,
                    source_name: str = "",
                    show_mean_ci_gt: bool = False,
                    show_mean_ci_qs: bool = False,
                    show_cis: bool = True,
                    plot_raw_trajectories: bool = True,
                    plot_3d: bool = True,
                    confidence: float = 0.95, 
                    show_all_desired_trajectories: bool = False,
                    axes_list = None,
                    axis_labels = None,
                    x_axis_top_fig_on = True,
                    figsize = None,
                    show_legend: bool = True,
                    legend_special: bool = False,
                    legend_shared: bool = False,
                    legend_supress_prefix: bool = False,
                    legend_extra1: str = "", 
                    linestyle1="solid", 
                    legend_extra2: str = "",
                    linestyle2="dotted",
                    color_map=COLOR_MAP,
                    trans_plot=['x', 'y', 'z'],
                    orient_plot=['roll', 'pitch', 'yaw'],
                    x_lims = None,
                    y_lims = None,
                    x_ticks = None,
                    y_ticks = None,
                    show_plot: bool = True):
        """
        Plot the stage data for both real and simulated logfiles in the LogCabinet.
        The function overlays the data for real and simulated datasets on the same plot if both are provided.
        
        Args:
            aggregated_df_1 (pd.DataFrame): The first aggregated dataframe (real data).
            aggregated_df_2 (pd.DataFrame): The second aggregated dataframe (simulated data, optional).
            stage (str): The stage name (e.g., "CIRCLE").
            show_qs (bool): Whether to show quaternion data.
            show_des (bool): Whether to show desired positions and orientations.
            source_name (str): The source name to include in the plot title.
            show_mean_ci_gt (bool): Whether to show mean confidence intervals for ground truth.
            show_mean_ci_qs (bool): Whether to show mean confidence intervals for quaternions.
            plot_raw_trajectories (bool): Whether to show the raw trajectories.
            plot_3d (bool): Whether to plot in 3D.
            confidence (float): The confidence level for the CI calculation.
        """
        # Check if the stage exists in the first aggregated dataframe
        if stage not in aggregated_df_1.index.get_level_values('stage').unique():
            print(f"Stage '{stage}' not found in the first dataset.")
            return
        
        # Extract relevant data from both real and simulated aggregated dataframes
        aggregated_df_1_stage = aggregated_df_1.xs(stage, level='stage')
        aggregated_df_2_stage = aggregated_df_2.xs(stage, level='stage') if aggregated_df_2 is not None else None

        if figsize is None:
            figsize = [(10, 8) for _ in range(self.num_drones + 2)]

        # Prepare the plots for load and drone data
        # Create the plots if they're not passed in, otherwise use the existing plots
        if axes_list is None:
            # Load
            fig1, ax1 = plt.subplots(2, 1, figsize=figsize[1], sharex=not x_axis_top_fig_on, constrained_layout=True)
            fig1.canvas.manager.set_window_title(f"Load Data - {source_name}")
            #fig1.tight_layout()
        
            # Drones
            drone_figs = []
            for i in range(self.num_drones):
                fig, axs = plt.subplots(2, 1, figsize=figsize[i + 2], sharex=not x_axis_top_fig_on, constrained_layout=True)
                fig.canvas.manager.set_window_title(f"Drone {i+1} Pose - {source_name}")
                #fig.tight_layout()
                drone_figs.append((fig, axs))

        else: 
            # Load
            ax1 = axes_list[0]

            # Drones
            drone_figs = []
            for i in range(self.num_drones):
                ax = axes_list[i + 1]
                drone_figs.append((None, ax))

        # Axis titles
        if axis_labels is None:
            axis_labels = ['Time (s)', 'Position (m)', 'Orientation (deg)']

        # --- 3D Trajectories ---
        if plot_3d:
            fig3 = plt.figure(figsize=figsize[0])
            ax3d = fig3.add_subplot(111, projection='3d')
            fig3.canvas.manager.set_window_title(f"3D Trajectory - {source_name}")

            # Compute CI for Load and Drones for 3D plot in first dataset
            pos_mean_1, _, _ = self.compute_ci(stage, aggregated_df_1, quantity='pos', source='gt', confidence=confidence, entity='load')
            ax3d.plot(pos_mean_1[:, 0], pos_mean_1[:, 1], pos_mean_1[:, 2], label = f"{'Load real' if not legend_supress_prefix else ''} {legend_extra1}", color=LOAD_COLORS[0], linestyle=f'{linestyle1}') #(mean)

            for i in range(self.num_drones):
                pos_mean_1_drone, _, _ = self.compute_ci(stage, aggregated_df_1, quantity='pos', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                ax3d.plot(pos_mean_1_drone[:, 0], pos_mean_1_drone[:, 1], pos_mean_1_drone[:, 2], label = f"{f'Drone {i+1}' if not legend_supress_prefix else ''}{legend_extra1}", color=DRONE_COLORS[i], linestyle=f'{linestyle1}')

            # If simulated data is provided, plot simulated data on the same graph
            if aggregated_df_2 is not None:
                label = 'Load sim' #None
                if not legend_special:
                    label = f"{'Load' if not legend_supress_prefix else ''} {legend_extra2}"
                
                pos_mean_2, _, _ = self.compute_ci(stage, aggregated_df_2, quantity='pos', source='gt', confidence=confidence, entity='load')
                ax3d.plot(pos_mean_2[:, 0], pos_mean_2[:, 1], pos_mean_2[:, 2], label = label, color=LOAD_COLORS[1], linestyle=f'{linestyle2}') #(mean)

                for i in range(self.num_drones):
                    label = None

                    if not legend_special:
                        label = f"{'Drone {i+1}' if not legend_supress_prefix else ''} {legend_extra2}"

                    pos_mean_2_drone, _, _ = self.compute_ci(stage, aggregated_df_2, quantity='pos', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                    ax3d.plot(pos_mean_2_drone[:, 0], pos_mean_2_drone[:, 1], pos_mean_2_drone[:, 2], label = label, color=DRONE_COLORS[i], linestyle=f'{linestyle2}')

            if legend_special:
                handles, labels = ax3d.get_legend_handles_labels()

                # define desired order (by index)
                order = [0, 4, 1, 2, 3]

                # # reorder both handles and labels
                handles = [handles[i] for i in order]
                labels = [labels[i] for i in order]

                # Change to increase labels across a row first
                ncol = 3  # how many columns you want
                # row_order = []
                # for row in range(len(labels) // ncol + 1):
                #     for col in range(ncol):
                #         idx = row * ncol + col
                #         if idx < len(labels):
                #             row_order.append(idx)

                # handles = [handles[i] for i in row_order]
                # labels  = [labels[i] for i in row_order]

                fig3.legend(handles, labels, loc='lower center', fontsize=legend_font_size, ncol=ncol, bbox_to_anchor=(0.5, -0.01), columnspacing=0.5, handletextpad=0.5, frameon=False)
                fig3.tight_layout(rect=[0, 0.05, 0.7, 1])  # reserve 5% at bottom
                fig3.subplots_adjust(bottom=0.2, right=0.85)

        # Time based plots
        time = aggregated_df_1_stage.index.get_level_values('time').unique()

        # Iterate through all available logfiles and trajectories if plot_raw_trajectories is True
        if plot_raw_trajectories:
            for logfile in aggregated_df_1_stage.index.get_level_values('logfile').unique():
                # Filter aggregated dataframe for this logfile
                real_logfile_data = aggregated_df_1_stage.xs(logfile, level='logfile')
                sim_logfile_data = aggregated_df_2_stage.xs(logfile, level='logfile') if aggregated_df_2 is not None else None

                # Iterate through all available trajectories
                for traj in real_logfile_data.index.get_level_values('trajectory').unique():
                    # Filter dataframe for this trajectory
                    real_traj_data = real_logfile_data.xs(traj, level='trajectory')
                    sim_traj_data = sim_logfile_data.xs(traj, level='trajectory') if sim_logfile_data is not None else None

                    if show_all_desired_trajectories or (logfile == aggregated_df_1_stage.index.get_level_values('logfile').unique()[0] and traj == aggregated_df_1_stage.index.get_level_values('trajectory').unique()[0]):
                        # --- Check if 'load' exists in real and simulated data before accessing ---
                        if 'load' in real_traj_data.index.get_level_values('entity') and ('load' in sim_traj_data.index.get_level_values('entity') if sim_traj_data is not None else True):
                            # --- Load Desired ---
                            if show_des:
                                pos_load_des_1 = np.array([real_traj_data.loc[('load', 'des', t)][0][0] for t in time])
                                rpy_load_des_1 = np.array([real_traj_data.loc[('load', 'des', t)][0][1] for t in time])
                                pos_load_des_2 = np.array([sim_traj_data.loc[('load', 'des', t)][0][0] for t in time]) if sim_traj_data is not None else None
                                rpy_load_des_2 = np.array([sim_traj_data.loc[('load', 'des', t)][0][1] for t in time]) if sim_traj_data is not None else None

                                for i, label in enumerate(['x', 'y', 'z']):
                                    if label in trans_plot:
                                        ax1[0].plot(time, pos_load_des_1[:, i], linestyle='dashed', label=f'Real des_{label}', color=color_map[label])
                                        if pos_load_des_2 is not None:
                                            ax1[0].plot(time, pos_load_des_2[:, i], linestyle='dotted', label=f'Sim des_{label}', color=color_map[label])
                                for i, label in enumerate(['roll', 'pitch', 'yaw']):
                                    if label in orient_plot:
                                        ax1[1].plot(time, rpy_load_des_1[:, i], linestyle='dashed', label=f'Real des_{label}', color=color_map[label])
                                        if rpy_load_des_2 is not None:
                                            ax1[1].plot(time, rpy_load_des_2[:, i], linestyle='dotted', label=f'Sim des_{label}', color=color_map[label])

                            # --- Drone Desired ---
                            if show_des:
                                for i in range(self.num_drones):
                                    # Drone desired positions for real and simulated data
                                    pos_drone_des_1 = np.array([real_traj_data.loc[('drone_' + str(i+1), 'des', t)][0][0] for t in time])
                                    rpy_drone_des_1 = np.array([real_traj_data.loc[('drone_' + str(i+1), 'des', t)][0][1] for t in time])
                                    pos_drone_des_2 = np.array([sim_traj_data.loc[('drone_' + str(i+1), 'des', t)][0][0] for t in time]) if sim_traj_data is not None else None
                                    rpy_drone_des_2 = np.array([sim_traj_data.loc[('drone_' + str(i+1), 'des', t)][0][1] for t in time]) if sim_traj_data is not None else None

                                    axs = drone_figs[i][1]
                                    for j, label in enumerate(['x', 'y', 'z']):
                                        if label in trans_plot:
                                            axs[0].plot(time, pos_drone_des_1[:, j], linestyle='dashed', label=f'Real des_{label}', color=color_map[label])
                                            if pos_drone_des_2 is not None:
                                                axs[0].plot(time, pos_drone_des_2[:, j], linestyle='dotted', label=f'Sim des_{label}', color=color_map[label])
                                    for j, label in enumerate(['roll', 'pitch', 'yaw']):
                                        if label in orient_plot:
                                            axs[1].plot(time, rpy_drone_des_1[:, j], linestyle='dashed', label=f'Real des_{label}', color=color_map[label])
                                            if rpy_drone_des_2 is not None:
                                                axs[1].plot(time, rpy_drone_des_2[:, j], linestyle='dotted', label=f'Sim des_{label}', color=color_map[label])

                        # --- Raw Trajectories ---
                        pos_load_gt_1 = np.array([real_traj_data.loc[('load', 'gt', t)][0][0] for t in time])
                        rpy_load_gt_1 = np.array([real_traj_data.loc[('load', 'gt', t)][0][1] for t in time])
                        for i, label in enumerate(['x', 'y', 'z']):
                            if label in trans_plot:
                                ax1[0].plot(time, pos_load_gt_1[:, i], color=color_map[label])
                        for i, label in enumerate(['roll', 'pitch', 'yaw']):
                            if label in orient_plot:
                                ax1[1].plot(time, rpy_load_gt_1[:, i], color=color_map[label])

                        for i in range(self.num_drones):
                            pos_drone_gt_1 = np.array([real_traj_data.loc[('drone_' + str(i+1), 'gt', t)][0][0] for t in time])
                            rpy_drone_gt_1 = np.array([real_traj_data.loc[('drone_' + str(i+1), 'gt', t)][0][1] for t in time])
                            axs = drone_figs[i][1]
                            for j, label in enumerate(['x', 'y', 'z']):
                                if label in trans_plot:
                                    axs[0].plot(time, pos_drone_gt_1[:, j], color=color_map[label])
                            for j, label in enumerate(['roll', 'pitch', 'yaw']):
                                if label in orient_plot:
                                    axs[1].plot(time, rpy_drone_gt_1[:, j], color=color_map[label])

        # --- Load Mean CI ---
        if show_mean_ci_gt:
            pos_mean_1, pos_lower_1, pos_upper_1 = self.compute_ci(stage, aggregated_df_1, quantity='pos', source='gt', confidence=confidence, entity='load')
            rpy_mean_1, rpy_lower_1, rpy_upper_1 = self.compute_ci(stage, aggregated_df_1, quantity='rpy', source='gt', confidence=confidence, entity='load')
            for i, label in enumerate(['x', 'y', 'z']):
                if label in trans_plot:
                    ax1[0].plot(time, pos_mean_1[:, i], label=f"{f'{label}' if not legend_supress_prefix else ''}{legend_extra1}", color=color_map[label], linestyle=f'{linestyle1}')
                    if show_cis:
                        ax1[0].fill_between(time, pos_lower_1[:, i], pos_upper_1[:, i], color=color_map[label], alpha=0.3)
            for i, label in enumerate(['roll', 'pitch', 'yaw']):
                if label in orient_plot:
                    ax1[1].plot(time, rpy_mean_1[:, i], label=f"{f'{label}' if not legend_supress_prefix else ''}{legend_extra1}", color=color_map[label], linestyle=f'{linestyle1}')
                    if show_cis:
                        ax1[1].fill_between(time, rpy_lower_1[:, i], rpy_upper_1[:, i], color=color_map[label], alpha=0.3)

            # If simulated data is available, plot simulated data CI
            if aggregated_df_2 is not None:
                pos_mean_2, pos_lower_2, pos_upper_2 = self.compute_ci(stage, aggregated_df_2, quantity='pos', source='gt', confidence=confidence, entity='load')
                rpy_mean_2, rpy_lower_2, rpy_upper_2 = self.compute_ci(stage, aggregated_df_2, quantity='rpy', source='gt', confidence=confidence, entity='load')
                for i, label in enumerate(['x', 'y', 'z']):
                    if label in trans_plot:
                        legend_label = None

                        if not legend_special:
                            legend_label = f"{f'{label}' if not legend_supress_prefix else ''}{legend_extra2}"

                        ax1[0].plot(time, pos_mean_2[:, i], label=legend_label, color=color_map[label], linestyle=f'{linestyle2}')
                        ax1[0].fill_between(time, pos_lower_2[:, i], pos_upper_2[:, i], color=color_map[label], alpha=0.3)
                for i, label in enumerate(['roll', 'pitch', 'yaw']):
                    if label in orient_plot:
                        legend_label = None

                        if not legend_special:
                            legend_label = f"{f'{label}' if not legend_supress_prefix else ''}{legend_extra2}"

                        ax1[1].plot(time, rpy_mean_2[:, i], label=legend_label, color=color_map[label], linestyle=f'{linestyle2}')
                        ax1[1].fill_between(time, rpy_lower_2[:, i], rpy_upper_2[:, i], color=color_map[label], alpha=0.3)

        # --- Drone Mean CI ---
        for i in range(self.num_drones):
            axs = drone_figs[i][1]
            if show_mean_ci_gt:
                pos_mean_1, pos_lower_1, pos_upper_1 = self.compute_ci(stage, aggregated_df_1, quantity='pos', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                rpy_mean_1, rpy_lower_1, rpy_upper_1 = self.compute_ci(stage, aggregated_df_1, quantity='rpy', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                for j, label in enumerate(['x', 'y', 'z']):
                    if label in trans_plot:
                        axs[0].plot(time, pos_mean_1[:, j], label=f"{f'{label}' if not legend_supress_prefix else ''}{legend_extra1}", color=color_map[label], linestyle=f'{linestyle1}')
                        if show_cis:
                            axs[0].fill_between(time, pos_lower_1[:, j], pos_upper_1[:, j], color=color_map[label], alpha=0.3)
                for j, label in enumerate(['roll', 'pitch', 'yaw']):
                    if label in orient_plot:
                        axs[1].plot(time, rpy_mean_1[:, j], label=f"{f'{label}' if not legend_supress_prefix else ''}{legend_extra1}", color=color_map[label], linestyle=f'{linestyle1}')
                        if show_cis:
                            axs[1].fill_between(time, rpy_lower_1[:, j], rpy_upper_1[:, j], color=color_map[label], alpha=0.3)

            # If simulated data is available, plot simulated drone CI
            if aggregated_df_2 is not None:
                pos_mean_2, pos_lower_2, pos_upper_2 = self.compute_ci(stage, aggregated_df_2, quantity='pos', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                rpy_mean_2, rpy_lower_2, rpy_upper_2 = self.compute_ci(stage, aggregated_df_2, quantity='rpy', source='gt', confidence=confidence, entity=f'drone_{i+1}')
                for j, label in enumerate(['x', 'y', 'z']):
                    if label in trans_plot:
                        legend_label = None

                        if not legend_special:
                            legend_label = f"{f'{label} - ' if not legend_supress_prefix else ''}{legend_extra2}"

                        axs[0].plot(time, pos_mean_2[:, j], linestyle=f'{linestyle2}', label=legend_label, color=color_map[label])
                        if show_cis:
                            axs[0].fill_between(time, pos_lower_2[:, j], pos_upper_2[:, j], color=color_map[label], alpha=0.2)
                for j, label in enumerate(['roll', 'pitch', 'yaw']):
                    if label in orient_plot:
                        legend_label = None

                        if not legend_special:
                            legend_label = f"{f'{label} - ' if not legend_supress_prefix else ''}{legend_extra2}"

                        axs[1].plot(time, rpy_mean_2[:, j], linestyle=f'{linestyle2}', label=legend_label, color=color_map[label])
                        if show_cis:
                            axs[1].fill_between(time, rpy_lower_2[:, j], rpy_upper_2[:, j], color=color_map[label], alpha=0.2)

        ax1[0].set_ylabel(axis_labels[1], fontsize=axes_label_font_size)
        ax1[0].tick_params(labelsize=ticks_font_size)
        if x_lims is not None:
            ax1[0].set_xlim(*x_lims[0][0])
        if y_lims is not None:
            ax1[0].set_ylim(*y_lims[0][0])

        # if not x_axis_top_fig_on:
        #     ax1[0].set_xticklabels([])
        #     ax1[0].set_xticks([])

        ax1[1].set_ylabel(axis_labels[2], fontsize=axes_label_font_size)
        ax1[1].set_xlabel(axis_labels[0], fontsize=axes_label_font_size)
        ax1[1].tick_params(labelsize=ticks_font_size)
        
        # Limits
        if x_lims is not None:
            ax1[1].set_xlim(*x_lims[0][1])
        if y_lims is not None:
            ax1[1].set_ylim(*y_lims[0][1])

        # Ticks
        if x_ticks is not None:
            ax1[0].set_xticks(x_ticks[0][0])
            ax1[1].set_xticks(x_ticks[0][1])
        if y_ticks is not None:
            ax1[0].set_yticks(y_ticks[0][0])
            ax1[1].set_yticks(y_ticks[0][1])

        if not legend_shared and show_legend:
            ax1[0].legend(fontsize=legend_font_size)
            ax1[1].legend(fontsize=legend_font_size)

        for i, drone_fig in enumerate(drone_figs):
            axs = drone_fig[1]
            axs[0].set_ylabel(axis_labels[1], fontsize=axes_label_font_size)
            axs[0].tick_params(labelsize=ticks_font_size)
            axs[1].set_ylabel(axis_labels[2], fontsize=axes_label_font_size)
            axs[1].set_xlabel(axis_labels[0], fontsize=axes_label_font_size)
            axs[1].tick_params(labelsize=ticks_font_size)

            # Limits
            if x_lims is not None and len(x_lims) > i+1:
                axs[0].set_xlim(*x_lims[i+1][0])
                axs[1].set_xlim(*x_lims[i+1][1])
            if y_lims is not None and len(y_lims) > i+1:
                axs[0].set_ylim(*y_lims[i+1][0])
                axs[1].set_ylim(*y_lims[i+1][1])

            # Ticks
            if x_ticks is not None and len(x_ticks) > i+1:
                axs[0].set_xticks(x_ticks[i+1][0])
                axs[1].set_xticks(x_ticks[i+1][1])
            if y_ticks is not None and len(y_ticks) > i+1:
                axs[0].set_yticks(y_ticks[i+1][0])
                axs[1].set_yticks(y_ticks[i+1][1])

            if not legend_shared and show_legend:
            #     for i in range(2):
            #         # get handles and labels from the axis
            #         handles, labels = axs[i].get_legend_handles_labels()

            #         # place the legend relative to axs[0] in figure coordinates
            #         bbox = axs[i].get_position()   # returns Bbox in figure coordinates

            #         fig1.legend(handles, labels,
            #                 loc="upper right",
            #                 bbox_to_anchor=(bbox.x1, bbox.y1),  # right/top of axs[0]
            #                 bbox_transform=fig1.transFigure,     # <-- fixes it to figure coords
            #                 fontsize=legend_font_size,
            #                 frameon=True)

                axs[0].legend(fontsize=legend_font_size, loc="upper right")
                axs[1].legend(fontsize=legend_font_size, loc="upper right")

        if plot_3d:
            ax3d.set_xlabel('X (m)')
            ax3d.set_ylabel('Y (m)')
            ax3d.set_zlabel('Z (m)')

            if not legend_special:
                ax3d.legend(fontsize=legend_font_size)

        if show_plot:
            if legend_shared and show_legend:
                # Make shared legend
                # Collect all handles + labels from all axes
                handles, labels = [], []
                #for ax in #[ax1[0], ax1[1], *[drone_figs[i][1][0] for i in range(self.num_drones)], *[drone_figs[i][1][1] for i in range(self.num_drones)]]:
                h, l = ax1[0].get_legend_handles_labels()
                handles.extend(h)
                labels.extend(l)

                # Put a single shared legend at the bottom
                fig = ax1[0].get_figure()
                fig.legend(handles, labels, loc='lower center', fontsize=legend_font_size, ncol=4, bbox_to_anchor=(0.5, -0.01), columnspacing=0.5, handletextpad=0.5, frameon=False)  # adjust ncol
                #fig.subplots_adjust(bottom=0.2)  # leave room for legend

            #fig.subplots_adjust(left=0.17, bottom=0.22, right=0.95, top=0.95, hspace=0.12)
            plt.rcParams["font.family"] = "Nimbus Roman"
            plt.tight_layout(rect=[0, 0.03, 1, 1])
            plt.show(block=False)

        return [ax1, *[drone_figs[i][1] for i in range(self.num_drones)]]


    def plot_param_sensitivity(self, 
                               parameter_name: str, 
                               percent_baseline_ind: int = -1, 
                               show_titles: bool = True, 
                               x_lims=None,
                               y_lims=None,
                               x_ticks=None,
                               y_ticks=None,
                               legend_shared=False):
        """
        Plot load errors (translational and geodesic) vs a simulation parameter.
        Also prints a table showing % difference from final value for each trajectory.
        
        Assumes `self.sim_variation_dfs` is populated with keys being the parameter values.
        """
        if not self.sim_variation_dfs:
            raise ValueError("No simulation variations available for plotting.")
        
        # Sort parameter values for x-axis
        param_values = sorted(self.sim_variation_dfs.keys())

        # Prepare storage for each trajectory
        trajectories = ['CIRCLE', 'YAW_ENGAGE', 'LINEAR_SHM']
        colors = ['#116FBF', '#DD5400', '#853AD1'] #['red', 'green', 'blue']
        
        translational_data = {traj: [] for traj in trajectories}
        geodesic_data = {traj: [] for traj in trajectories}

        for var_value in param_values:
            _, _, load_df = self.sim_variation_dfs[var_value]
            # Ensure translational is in mm
            load_df['Mean Translational Distance'] *= 1000

            # Convert load_df index to a proper MultiIndex
            if not isinstance(load_df.index, pd.MultiIndex):
                load_df.index = pd.MultiIndex.from_tuples([idx if isinstance(idx, tuple) else (idx, 0) for idx in load_df.index])
            
            for traj in trajectories:
                # Extract row like ('CIRCLE', 0)
                row_key = next((idx for idx in load_df.index if idx[0] == traj), None)
                if row_key is None:
                    translational_data[traj].append(float('nan'))
                    geodesic_data[traj].append(float('nan'))
                else:
                    translational_data[traj].append(load_df.loc[row_key, 'Mean Translational Distance'])
                    geodesic_data[traj].append(load_df.loc[row_key, 'Mean Geodesic Distance'])
        
        # ---- Compute % difference from final value ----
        pct_diff_translational = pd.DataFrame(index=param_values, columns=trajectories)
        pct_diff_geodesic = pd.DataFrame(index=param_values, columns=trajectories)
        
        for traj in trajectories:
            baseline_trans = translational_data[traj][percent_baseline_ind]
            baseline_geo = geodesic_data[traj][percent_baseline_ind]
            for i, val in enumerate(translational_data[traj]):
                pct_diff_translational.iloc[i][traj] = ((val - baseline_trans)/baseline_trans)*100
            for i, val in enumerate(geodesic_data[traj]):
                pct_diff_geodesic.iloc[i][traj] = ((val - baseline_geo)/baseline_geo)*100
        
        print("\n--- Percent Difference from Selected Value: Translational Distance (%) ---")
        print(pct_diff_translational.round(2))

        print("\n--- Percent Difference from Selected Value: Geodesic Distance (%) ---")
        print(pct_diff_geodesic.round(2))
        
        # ---- Plotting ----
        fig, axs = plt.subplots(2, 1, figsize=(3.5, 4), sharex=True) #figsize=(8, 10)
        
        # Top: Translational distance
        for traj, color in zip(trajectories, colors):
            traj_name = utils_logs.convert_traj_to_label(traj)
            axs[0].plot(param_values, translational_data[traj], marker='o', color=color, label=traj_name, linewidth=line_width)
        
        axs[0].set_ylabel("Translational\ndistance (mm)", fontsize=axes_label_font_size)
        #axs[0].grid(True)
        axs[0].tick_params(labelsize=ticks_font_size)

        if not legend_shared:
            axs[0].legend(fontsize=legend_font_size)

        # Plot axis limits and ticks
        if x_lims is not None:
            axs[0].set_xlim(*x_lims[0])
        if y_lims is not None:
            axs[0].set_ylim(*y_lims[0])

        if x_ticks is not None:
            axs[0].set_xticks(x_ticks[0][0])
        if y_ticks is not None:
            axs[0].set_yticks(y_ticks[0][0])

        if show_titles:
            axs[0].set_title(f"Load Translational Distance vs {parameter_name}", fontsize=axes_label_font_size)
        
        # Bottom: Geodesic distance
        for traj, color in zip(trajectories, colors):
            traj_name = utils_logs.convert_traj_to_label(traj)
            axs[1].plot(param_values, geodesic_data[traj], marker='o', color=color, label=traj_name, linewidth=line_width)

        axs[1].set_xlabel(parameter_name, fontsize=axes_label_font_size)
        axs[1].set_ylabel("Geodesic\ndistance (deg)", fontsize=axes_label_font_size)
        axs[1].tick_params(labelsize=ticks_font_size)
        #axs[1].grid(True)

        # Plot axis limits and ticks
        if x_lims is not None:
            axs[1].set_xlim(*x_lims[1])
        if y_lims is not None:
            axs[1].set_ylim(*y_lims[1])

        if x_ticks is not None:
            axs[1].set_xticks(x_ticks[0][1])
        if y_ticks is not None:
            axs[1].set_yticks(y_ticks[0][1])

        if show_titles:
            axs[1].set_title(f"Load Geodesic Distance vs {parameter_name}", fontsize=axes_label_font_size)

        if legend_shared:
            # Make shared legend
            # Collect all handles + labels from all axes
            handles, labels = [], []
            #for ax in #[ax1[0], ax1[1], *[drone_figs[i][1][0] for i in range(self.num_drones)], *[drone_figs[i][1][1] for i in range(self.num_drones)]]:
            h, l = axs[0].get_legend_handles_labels()
            handles.extend(h)
            labels.extend(l)

            # Put a single shared legend at the bottom
            fig = axs[0].get_figure()
            fig.legend(handles, labels, loc='lower center', fontsize=legend_font_size, ncol=3, bbox_to_anchor=(0.5, -0.01), columnspacing=0.5, handletextpad=0.5, frameon=False) 
            #fig.subplots_adjust(bottom=-0.8)  # leave room for legend
        
        plt.rcParams["font.family"] = "Times" #"Times New Roman"
        plt.tight_layout(rect=[0, 0.03, 1, 1])
        plt.show(block=False)



    


