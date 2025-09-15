import numpy as np
import xml.etree.ElementTree as ET
import re
import subprocess
import yaml
import os
import shutil

### READING FROM OTHER FILES ###
# Function to extract num_elements and elem_length_main from model.xacro
def extract_from_xacro(xacro_file_path):
    with open(xacro_file_path, 'r') as xacro_file:
        content = xacro_file.read()
        
    # Use regular expressions to find num_elements
    num_elements_match = re.search(r'<xacro:property name="num_elements" value="([^"]+)" />', content)
    num_elements = int(num_elements_match.group(1)) if num_elements_match else None
    
    return num_elements

# Function to extract elem_length_main and elem_length_ends from model.sdf
def extract_from_sdf(sdf_file_path):
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()

    # Extract elem_length_main from the length of the cylinder in the visual element for element_1
    elem_length_main = None
    elem_length_ends = None

    # Find the visual element for element_1 to get elem_length_main
    for visual in root.findall(".//link"):
        if visual.get('name') == "element_1":
            length_elem = visual.find(".//geometry/cylinder/length")
            if length_elem is not None:
                elem_length_main = float(length_elem.text)

        if visual.get('name') == "element_first":
            length_elem = visual.find(".//geometry/cylinder/length")
            if length_elem is not None:
                elem_length_ends = float(length_elem.text)

    return elem_length_main, elem_length_ends


### FILE PROCESSING ###
def copy_and_rename_template_files(src_dir, dest_dir, filenames):
    """
    Copies template files from the source directory to the destination directory, 
    renaming them by removing the 'template_' prefix. Existing files are overwritten.
    
    Parameters:
    - src_dir: The source directory where the template files are located.
    - dest_dir: The destination directory where the files should be copied.
    - filenames: A list of filenames to be copied and renamed (without 'template_' prefix).
    """
    for filename in filenames:
        # Construct full source and destination paths
        src_file_path = os.path.join(src_dir, f"template_{filename}")
        dest_file_path = os.path.join(dest_dir, filename)
        
        # Check if the source file exists
        if os.path.exists(src_file_path):
            # Copy and rename the file (overwrite if the file already exists)
            shutil.copy(src_file_path, dest_file_path)
            #print(f"Copied and renamed {src_file_path} to {dest_file_path}")
        else:
            print(f"Source file {src_file_path} not found.")

# Function to load configuration from the cable_params.yaml file
def load_config(config_file_path):
    with open(config_file_path, 'r') as config_file:
        config = yaml.safe_load(config_file)
    return config

# Function to inject config variables into the generate_sdf.sh file
def inject_into_bash_script(config, bash_script_path):
    # Prepare the bash variables from the config
    bash_variables = {
        "JOINT1_AXIS": config["joint_1_axis"],
        "JOINT1_FRICTION": config["joint_friction"],
        "JOINT1_DAMPING": config["joint_damping"],
        "JOINT1_SPRING_REFERENCE": config["joint_1_spring_reference"],
        "JOINT1_SPRING_STIFFNESS": config["joint_stiffness"],
        "JOINT2_AXIS": config["joint_2_axis"],
        "JOINT2_FRICTION": config["joint_friction"],
        "JOINT2_DAMPING": config["joint_damping"],
        "JOINT2_SPRING_REFERENCE": config["joint_2_spring_reference"],
        "JOINT2_SPRING_STIFFNESS": config["joint_stiffness"],
        "K": config["joint_interval_k"],
        "OFFSET": config["joint_offset"],
        "JOINT1_K_DAMPING": config["joint_k_damping"],
        "JOINT1_K_FRICTION": config["joint_k_friction"],
        "JOINT1_K_SPRING_REFERENCE": config["joint_1_spring_reference"],
        "JOINT1_K_SPRING_STIFFNESS": config["joint_k_stiffness"],
        "JOINT2_K_DAMPING": config["joint_k_damping"],
        "JOINT2_K_FRICTION": config["joint_k_friction"],
        "JOINT2_K_SPRING_REFERENCE": config["joint_2_spring_reference"],
        "JOINT2_K_SPRING_STIFFNESS": config["joint_k_stiffness"],
    }
    
    # Read the bash script
    with open(bash_script_path, 'r') as file:
        bash_script = file.read()
    
    # Replace placeholders with values from YAML
    for key, value in bash_variables.items():
        bash_script = bash_script.replace(f"#{key}", str(value))
    
    # Write the modified script back
    with open(bash_script_path, 'w') as file:
        file.write(bash_script)

# Function to inject config variables into the model.xacro file
def inject_into_xacro(config, xacro_file_path):
    # Prepare xacro variables from the config
    xacro_variables = {
        "cable_radius": config["cable_radius"],
        "cable_mass": config["cable_mass"],
        "cable_length": config["cable_length"],
        "num_elements": config["num_elements"],
        "elem_length_ends": config["elem_length_ends"],
        "elem_mass_ends": config["elem_mass_ends"],
        "joint_friction": config["joint_friction"],
        "joint_damping": config["joint_damping"]
    }

    # Read the xacro file
    with open(xacro_file_path, 'r') as file:
        xacro_content = file.read()

    # Replace placeholders with values from YAML
    for key, value in xacro_variables.items():
        xacro_content = xacro_content.replace(f"${{{key}}}", str(value))

    # Write the modified xacro content back
    with open(xacro_file_path, 'w') as file:
        file.write(xacro_content)


### SCRIPT RUNNING ###
# Function to run the bash script
def run_bash_script(script_path, *args):
    try:
        # Ensure the script has executable permissions
        if not os.access(script_path, os.X_OK):
            print(f"Adding execute permissions to {script_path}...")
            subprocess.run(f"chmod +x {script_path}", shell=True, check=True)

        # Construct the bash command to run the script
        bash_command = f"bash {script_path} " + " ".join(args)
        
        # Run the bash script
        #print(f"Running command: {bash_command}")
        result = subprocess.run(bash_command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Print the output of the script
        #print("Script output:", result.stdout.decode())  # Print the output of the script
        #print("Script error output:", result.stderr.decode())  # Print any error output from the script
        
    except subprocess.CalledProcessError as e:
        # If the script fails, print the error
        print(f"Error executing script: {e}")
        print(f"Error message: {e.stderr.decode()}")
        print(f"Output message: {e.stdout.decode()}")


# Function to run another python script
def run_python_script(script_path, dist_connection_points):    
    try:
        # Running the Python script
        result = subprocess.run(['python3', script_path, str(dist_connection_points)], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Print the output from the script
        #print("Script output:", result.stdout.decode())
        
    except subprocess.CalledProcessError as e:
        # If the script fails, print the error
        print(f"Error executing script: {e}")
        print(f"Error message: {e.stderr.decode()}")


### CALCULATIONS ###
# Function to calculate the distance between connection points
def calculate_distance_connection_points(elem_length_main, num_elements):
    return elem_length_main / (np.sin(np.pi / (2 * num_elements)))


# Main function to update the SDF with the correct poses
def main():
    ### Define file paths and parameters ###
    dir_cable = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models/tether_2m"
    dir_world = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/worlds"

    script_generate_tether = f'{dir_cable}/generate_sdf.sh'
    script_update_poses = f'{dir_world}/update_drone_poses.py'

    xacro_file_path = f'{dir_cable}/model.xacro'
    tether_sdf_path = f'{dir_cable}/model.sdf'

    config_file_path = f'{dir_cable}/cable_params.yaml'

    template_filenames = [
        'generate_sdf.sh',
        'model.xacro'
    ]

    # Change to the cable directory
    os.chdir(dir_cable)  # This changes the current working directory to dir_cable

    # Load config from YAML file
    config = load_config(config_file_path)
    
    ### 0. INJECT CONFIG VALUES INTO XACRO AND SDF SCRIPT ###
    print("0. Set up files")

    # Copy and rename template files
    copy_and_rename_template_files(dir_cable, dir_cable, template_filenames)

    # Inject configuration values into bash script and xacro file
    inject_into_bash_script(config, script_generate_tether)
    inject_into_xacro(config, xacro_file_path)
    
    ### 1. GENERATE NEW CABLE SDF ###
    print("1. Generate cable SDF")
    run_bash_script(script_generate_tether)

    ### 2. CALCULATE DISTANCE BETWEEN CONNECTION POINTS ###
    print("2. Calculate distance between connection points")

    # Extract values from model.xacro and model.sdf
    num_elements = extract_from_xacro(xacro_file_path)
    elem_length_main, elem_length_ends = extract_from_sdf(tether_sdf_path)

    if num_elements is None or elem_length_main is None or elem_length_ends is None:
        print("Failed to extract necessary data from the files.")
        return

    # Calculate distance between connection points
    distance_connection_points = calculate_distance_connection_points(elem_length_main, num_elements)
    print(f"Cable length: {elem_length_main* num_elements} m")
    print(f"Distance between connection points: {distance_connection_points} m")


    ### 3. UPDATE DRONE AND TETHER POSES IN WORLD SDF ###
    print("3. Update drone and tether poses in world SDF")
    run_python_script(script_update_poses, distance_connection_points)

    ### 4. END ###
    print("COMPLETE: All tasks finished successfully!")


if __name__ == '__main__':
    main()
