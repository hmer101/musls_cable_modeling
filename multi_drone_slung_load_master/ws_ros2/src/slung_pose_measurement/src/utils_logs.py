import numpy as np
from scipy.spatial.transform import Rotation as R

# Font size settings
title_font_size = 14 #22
axes_label_font_size = 11 #14 #22
legend_font_size = 10 #12 #16
ticks_font_size = 10 #12 #16

# Linewidth settings
line_width = 2

# Tick settings (arrays for precise control)
time_ticks = None  # e.g., np.arange(0, 80.5, 0.5)
y_ticks_pos = None  # e.g., np.arange(-1, 1.1, 0.2)
y_ticks_ori = None  # e.g., np.arange(-180, 181, 10)
x_ticks_traj = None  # e.g., np.arange(0, 85, 5)
y_ticks_traj = None  # e.g., np.arange(-5, 5.5, 0.5)
z_ticks_traj = None  # e.g., np.arange(0, 5.5, 0.5)

# Consistent color mapping
# COLOR_MAP = {
#     'x': 'r', 'y': 'g', 'z': 'b',
#     'roll': 'r', 'pitch': 'g', 'yaw': 'b'
# }

# COLOR_MAP = {
#     'x': '#E69F00',     # orange (instead of red)
#     'y': '#009E73',     # bluish green (instead of green)
#     'z': '#0072B2',     # blue (close to ROS blue)
#     'roll': '#E69F00',
#     'pitch': '#009E73',
#     'yaw': '#0072B2'
# }

# COLOR_MAP = {
#     'x': '#E69F00',     # orange (instead of red)
#     'y': '#009E73',     # bluish green (instead of green)
#     'z': '#0072B2',     # blue (close to ROS blue)
#     'roll': '#E69F00',
#     'pitch': '#009E73',
#     'yaw': '#0072B2'
# }

COLOR_MAP = {
    'x': '#000000',     # orange (instead of red)
    'y': '#116FBF',     # bluish green (instead of green)
    'z': '#DD5400',     # blue (close to ROS blue)
    'roll': '#000000',
    'pitch': '#116FBF',
    'yaw': '#DD5400'
}

# COLOR_MAP_2 = {
#     'x': '#CC79A7',     # magenta (warm)
#     'y': '#999933',     # olive (medium)
#     'z': '#332288',     # dark purple (cool)
#     'roll': '#CC79A7',
#     'pitch': '#999933',
#     'yaw': '#332288'
# }

COLOR_MAP_2 = {
    'x': '#D10B8C',     # magenta (warm)
    'y': '#999933',     # olive (medium)
    'z': '#853AD1',     # dark purple (cool)
    'roll': '#D10B8C',
    'pitch': '#999933',
    'yaw': '#853AD1'
}

COLOR_MAP_BLACK = {
    'x': '#000000',     # crimson red
    'y': '#000000',
    'z': '#000000',
    'roll': '#000000',
    'pitch': '#000000',
    'yaw': '#000000'
}

COLOR_MAP_RED = {
    'x': '#B2182B',     # crimson red
    'y': '#B2182B',
    'z': '#B2182B',
    'roll': '#B2182B',
    'pitch': '#B2182B',
    'yaw': '#B2182B'
}

# COLOR_MAP_PINK = {
#     'x': '#CC79A7',    
#     'y': '#CC79A7',
#     'z': '#CC79A7',
#     'roll': '#CC79A7',
#     'pitch': '#CC79A7',
#     'yaw': '#CC79A7'
# }

COLOR_MAP_PINK = {
    'x': '#D10B8C',    
    'y': '#D10B8C',
    'z': '#D10B8C',
    'roll': '#D10B8C',
    'pitch': '#D10B8C',
    'yaw': '#D10B8C'
}

# COLOR_MAP_ORANGE = {
#     'x': '#E69F00',     
#     'y': '#E69F00',
#     'z': '#E69F00',
#     'roll': '#E69F00',
#     'pitch': '#E69F00',
#     'yaw': '#E69F00'
# }

COLOR_MAP_ORANGE = {
    'x': '#DD5400',     
    'y': '#DD5400',
    'z': '#DD5400',
    'roll': '#DD5400',
    'pitch': '#DD5400',
    'yaw': '#DD5400'
}

COLOR_MAP_YELLOW = {
    'x': '#EDB21F',     
    'y': '#EDB21F',
    'z': '#EDB21F',
    'roll': '#EDB21F',
    'pitch': '#EDB21F',
    'yaw': '#EDB21F'
}

# COLOR_MAP_GREEN = {
#     'x': '#009E73',     
#     'y': '#009E73',
#     'z': '#009E73',
#     'roll': '#009E73',
#     'pitch': '#009E73',
#     'yaw': '#009E73'
# }

COLOR_MAP_GREEN = {
    'x': '#3AAF32',     
    'y': '#3AAF32',
    'z': '#3AAF32',
    'roll': '#3AAF32',
    'pitch': '#3AAF32',
    'yaw': '#3AAF32'
}

# COLOR_MAP_BLUE = {
#     'x': '#0072B2',     
#     'y': '#0072B2',
#     'z': '#0072B2',
#     'roll': '#0072B2',
#     'pitch': '#0072B2',
#     'yaw': '#0072B2'
# }

COLOR_MAP_BLUE = {
    'x': '#116FBF',     
    'y': '#116FBF',
    'z': '#116FBF',
    'roll': '#116FBF',
    'pitch': '#116FBF',
    'yaw': '#116FBF'
}

COLOR_MAP_LIGHT_BLUE = {
    'x': '#2EBEF0',     
    'y': '#2EBEF0',
    'z': '#2EBEF0',
    'roll': '#2EBEF0',
    'pitch': '#2EBEF0',
    'yaw': '#2EBEF0'
}

# COLOR_MAP_DARK_PURPLE = {
#     'x': '#332288',     
#     'y': '#332288',
#     'z': '#332288',
#     'roll': '#332288',
#     'pitch': '#332288',
#     'yaw': '#332288'
# }

COLOR_MAP_DARK_PURPLE = {
    'x': '#853AD1',     
    'y': '#853AD1',
    'z': '#853AD1',
    'roll': '#853AD1',
    'pitch': '#853AD1',
    'yaw': '#853AD1'
}


#DRONE_COLORS = ['g', 'b', 'm', 'y']
DRONE_COLORS = ['#DD5400', '#116FBF', '#EDB21F', '#B2182B']
LOAD_COLORS = ['#000000', '#2EBEF0']

def set_custom_ticks(ax, axis, ticks):
    if ticks is not None:
        if axis == 'x':
            ax.set_xticks(ticks)
        elif axis == 'y':
            ax.set_yticks(ticks)
        elif axis == 'z' and hasattr(ax, 'set_zticks'):
            ax.set_zticks(ticks)

def compute_geodesic_distance(rpy, mean_rpy):
    """
    Compute the geodesic distance (angle) for rotation differences between predicted and reference rotations.
    The `rpy` input is the predicted roll, pitch, and yaw values for each time step.
    The `mean_rpy` is the reference rotation (mean RPY).
    """
    geodesic_distances = []
    
    # Loop through each time step and compute the geodesic distance
    for t in range(rpy.shape[0]):  # Iterate through each time step
        rpy_t = rpy[t]  # Predicted rpy at time step t
        mean_rpy_t = mean_rpy[t]  # Reference mean rpy at time step t

        # Convert mean RPY (reference) to a rotation matrix
        R_ref = R.from_euler('ZYX', mean_rpy_t, degrees=True).as_matrix()

        # Convert the predicted RPY at time step t to a rotation matrix
        R_pred = R.from_euler('ZYX', rpy_t, degrees=True).as_matrix()

        # Geodesic distance formula: arccos((tr(R^T * R_ref) - 1) / 2)
        trace = np.trace(R_pred.T @ R_ref)  # tr(R^T * R_ref)
        geodesic_distances.append(np.arccos((trace - 1) / 2))  # Calculate geodesic distance for this time step
    
    return np.degrees(np.array(geodesic_distances))


def convert_traj_to_label(traj):
    """
    Convert a trajectory name from its code representation to a human-readable label.
    """
    
    traj_name = None
    if traj == 'CIRCLE':
        traj_name = 'Circle'
    elif traj == 'YAW_ENGAGE':
        traj_name = 'Yaw engage'
    elif traj == 'LINEAR_SHM':
        traj_name = 'Linear SHM'
    
    return traj_name