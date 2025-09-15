import numpy as np
import xml.etree.ElementTree as ET
import argparse

# Also see: multi_drone_slung_load's generate_spawn_points.py (does yaws too)


def generate_points_circle(n_points, radius, offset_x=0.0):
    """
    Generate a list of 2D points that lie on a circle.
    
    Parameters:
    - n_points: Number of points to generate.
    - radius: Radius of the circle.
    
    Returns:
    - List of tuples representing (x, y) coordinates on the circle.
    """
    points = []
    
    for i in range(n_points):
        angle = i * 2 * np.pi / n_points
        x = radius*np.cos(angle) + np.sign(np.cos(angle))*offset_x
        y = radius*np.sin(angle)
        points.append((x, y))
    
    return points


def transform_points(points, translation, rotation_angle):
    """
    Transform a list of 2D points from one coordinate system to another.
    
    Parameters:
    - points: List of tuples representing (x, y) coordinates in the original coordinate system.
    - translation: Tuple representing (tx, ty) translation vector.
    - rotation_angle: Angle in degrees by which to rotate the coordinate system.
    
    Returns:
    - Transformed points in the new coordinate system as a list of tuples.
    """
    # Convert rotation angle from degrees to radians
    #theta = np.deg2rad(rotation_angle)
    theta = rotation_angle
    
    # Define the rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    
    transformed_points = []
    
    for x, y in points:
        # Rotate the point
        rotated_point = rotation_matrix @ np.array([x, y])
        
        # Translate the rotated point
        translated_point = rotated_point + np.array(translation)
        
        # Append the result
        transformed_points.append(tuple(translated_point))
    
    return transformed_points


def update_poses_in_sdf(drones, tethers, sdf_file_path):
    """
    Update the poses of drones and tethers in the SDF file with the given positions.
    
    Parameters:
    - drones: List of tuples representing (x, y) coordinates for drones.
    - tethers: List of tuples representing (x, y) coordinates for tethers.
    - sdf_file_path: Path to the SDF file to update.
    """
    # Parse the SDF file
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    
    # Update the drone poses
    include_elements = root.findall(".//model[@name='swarm']//include")
    
    # Lists for drone and tether uris
    drone_uris = ['x500_camera_down'] #['x500_1', 'x500_2', 'x500_3']
    tether_uris = ['tether_2m'] #['tether_1', 'tether_2', 'tether_3']
    
    # Process drone poses
    cnt_drones = 0

    for i, include in enumerate(include_elements):
        # Get the uri by looking for the <uri> tag inside the <include> element
        uri_elem = include.find('uri')
        if uri_elem is not None:
            uri = uri_elem.text
            #print(f"Found uri: {uri}")

            # Check if the uri matches one of the drone uris
            if uri in drone_uris:
                # Find the pose tag within this include element
                pose_elem = include.find('.//pose')
                if pose_elem is not None:
                    # Update the pose (modify only x and y, leave z and rotation unchanged)
                    pose_values = pose_elem.text.split()
                    x, y = drones[cnt_drones]
                    pose_values[0] = str(x)  # Update x value
                    pose_values[1] = str(y)  # Update y value
                    pose_elem.text = " ".join(pose_values)

                    # Update the drone counter
                    cnt_drones += 1

    # Process tether poses
    cnt_tethers = 0

    for i, include in enumerate(include_elements):
        # Get the uri by looking for the <uri> tag inside the <include> element
        uri_elem = include.find('uri')
        if uri_elem is not None:
            uri = uri_elem.text
            #print(f"Found uri: {uri}")

            # Check if the uri matches one of the tether uris
            if uri in tether_uris:
                # Find the pose tag within this include element
                pose_elem = include.find('.//pose')
                if pose_elem is not None:
                    # Update the pose (modify only x and y, leave z and rotation unchanged)
                    pose_values = pose_elem.text.split()
                    x, y = tethers[cnt_tethers]
                    pose_values[0] = str(x)  # Update x value
                    pose_values[1] = str(y)  # Update y value
                    pose_elem.text = " ".join(pose_values)

                    # Update the tether counter
                    cnt_tethers += 1

    # Save the updated SDF file
    tree.write(sdf_file_path)

    print("SDF file updated successfully!")


def main(dist_connection_points):
    ## DEFINE PARAMETERS
    # Define load pose
    translation = (0, 0)
    rotation_angle = 0

    # Drones rel load
    n_points = 3
    r_load_cable_connect = 0.1 # m (radius of the circle around the load's center where the cables are connected)
    
    r_drones = dist_connection_points + r_load_cable_connect #1.6475 #1.6475 (2.4210 m cable) 1.645 (2.4171 m cable) 1.635 (2.40147 m cable) 1.65 (2.425 m cable) 1.625 (2.3858 m cable) 1.621 (2.37957 m cable) 2.1 (3.13 m cable) #1.5 (2.19 m cable) #m

    # Tether placement
    d_tether_from_drone_com_x = 0.04

    ## GENERATE POINTS
    points_drones = generate_points_circle(n_points, r_drones, offset_x=d_tether_from_drone_com_x) # Generate points around the load pose
    transformed_points_drones = transform_points(points_drones, translation, rotation_angle) # Transform points to surround the desired load pose

    # Generate tether placement points
    points_tethers = generate_points_circle(n_points, r_load_cable_connect)
    transformed_points_tethers = transform_points(points_tethers, translation, rotation_angle) # Transform points to surround the desired load pose


    ## PRINT
    # print("DRONES: ")
    # print(transformed_points_drones)

    # print("TETHERS: ")
    # print(transformed_points_tethers)

    ## Update the SDF file with the new poses
    sdf_file_path = '/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/worlds/multi_rigid_link_2m.sdf'
    update_poses_in_sdf(transformed_points_drones, transformed_points_tethers, sdf_file_path)


if __name__ == '__main__':
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Update drone and tether poses in an SDF file.")
    parser.add_argument("dist_connection_points", type=float, help="Distance between connection points")
    
    # Parse the arguments
    args = parser.parse_args()
    
    # Call main function with dist_connection_points as argument
    #dist_connection_points = 1.5476
    main(args.dist_connection_points)

