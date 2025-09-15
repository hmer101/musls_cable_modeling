import numpy as np
import xml.etree.ElementTree as ET
import subprocess
import yaml
import os
import shutil
import sys

# ---------- XML helpers ----------
def _strip_ns(tag):
    if '}' in tag:
        return tag.split('}', 1)[1]
    return tag

def _nsmap_for_xacro():
    # Common xacro namespace; your file declares: xmlns:xacro="http://ros.org/wiki/xacro"
    return {'xacro': 'http://ros.org/wiki/xacro'}

# ---------- READ OTHER FILES ----------
def extract_from_xacro(xacro_file_path):
    """
    Robustly read num_elements from <xacro:property name="num_elements" value="..."/>
    regardless of whitespace or attribute ordering.
    """
    try:
        tree = ET.parse(xacro_file_path)
    except ET.ParseError as e:
        print(f"[extract_from_xacro] XML parse error in {xacro_file_path}: {e}")
        return None

    root = tree.getroot()
    ns = _nsmap_for_xacro()

    # try with namespace prefix
    props = root.findall(".//xacro:property", ns)
    if not props:
        # fallback: any element named 'property' regardless of ns
        props = [el for el in root.iter() if _strip_ns(el.tag) == "property"]

    num_elements = None
    for p in props:
        if p.get('name') == 'num_elements':
            val = p.get('value')
            if val is None:
                continue
            # value should be a number after your injection step
            try:
                num_elements = int(float(val))
            except ValueError:
                # if it’s an expression, print for debug
                print(f"[extract_from_xacro] num_elements value not numeric yet: {val}")
            break

    if num_elements is None:
        print(f"[extract_from_xacro] Could not find numeric num_elements in {xacro_file_path}")
    return num_elements


def _find_first_cylinder_length(link_elem):
    """
    Search a link element for the first <geometry><cylinder><length> value
    under either <visual> or <collision>.
    """
    # search both visual and collision blocks
    for sub in link_elem.iter():
        if _strip_ns(sub.tag) == "geometry":
            cyl = next((c for c in list(sub) if _strip_ns(c.tag) == "cylinder"), None)
            if cyl is None:
                continue
            length_elem = next((le for le in list(cyl) if _strip_ns(le.tag) == "length"), None)
            if length_elem is not None and length_elem.text:
                try:
                    return float(length_elem.text)
                except ValueError:
                    pass
    return None


def extract_from_sdf(sdf_file_path):
    """
    Read elem_length_main from link 'element_1' and elem_length_ends from link 'element_first'
    in the generated model.sdf.
    """
    try:
        tree = ET.parse(sdf_file_path)
    except ET.ParseError as e:
        print(f"[extract_from_sdf] XML parse error in {sdf_file_path}: {e}")
        return None, None
    except FileNotFoundError:
        print(f"[extract_from_sdf] File not found: {sdf_file_path}")
        return None, None

    root = tree.getroot()

    elem_length_main = None
    elem_length_ends = None

    # Iterate all link elements and match by name
    for link in root.iter():
        if _strip_ns(link.tag) != "link":
            continue
        name = link.get('name', '')
        if name == "element_1" and elem_length_main is None:
            elem_length_main = _find_first_cylinder_length(link)
        elif name == "element_first" and elem_length_ends is None:
            elem_length_ends = _find_first_cylinder_length(link)

        if elem_length_main is not None and elem_length_ends is not None:
            break

    if elem_length_main is None:
        print("[extract_from_sdf] Could not find cylinder length for link 'element_1'.")
    if elem_length_ends is None:
        print("[extract_from_sdf] Could not find cylinder length for link 'element_first'.")

    return elem_length_main, elem_length_ends

# ---------- FILE PROCESSING ----------
def copy_and_rename_template_files(src_dir, dest_dir, filenames):
    for filename in filenames:
        src_file_path = os.path.join(src_dir, f"template_{filename}")
        dest_file_path = os.path.join(dest_dir, filename)
        if os.path.exists(src_file_path):
            shutil.copy(src_file_path, dest_file_path)
        else:
            print(f"[copy_and_rename] Source file {src_file_path} not found.")

def load_config(config_file_path):
    with open(config_file_path, 'r') as config_file:
        return yaml.safe_load(config_file)

def inject_into_bash_script(config, bash_script_path):
    bash_variables = {
        "JOINT1_AXIS": config["joint_1_axis"],
        "JOINT1_FRICTION": config["joint_friction"],
        "JOINT1_DAMPING": config["joint_damping"],
        "JOINT1_SPRING_REFERENCE": config["joint_spring_reference"],
        "JOINT1_SPRING_STIFFNESS": config["joint_stiffness"],
        "JOINT2_AXIS": config["joint_2_axis"],
        "JOINT2_FRICTION": config["joint_friction"],
        "JOINT2_DAMPING": config["joint_damping"],
        "JOINT2_SPRING_REFERENCE": config["joint_spring_reference"],
        "JOINT2_SPRING_STIFFNESS": config["joint_stiffness"]
    }
    
    with open(bash_script_path, 'r') as f:
        script = f.read()
    for key, value in bash_variables.items():
        script = script.replace(f"#{key}", str(value))
    with open(bash_script_path, 'w') as f:
        f.write(script)

def inject_into_xacro(config, xacro_file_path):
    # Replace ${...} placeholders in the xacro template
    xacro_variables = {
        "cable_radius":     config["cable_radius"],
        "cable_mass":       config["cable_mass"],
        "cable_length":     config["cable_length"],
        "num_elements":     config["num_elements"],
        "elem_length_ends": config["elem_length_ends"],
        "elem_mass_ends":   config["elem_mass_ends"],
        "joint_friction":   config["joint_friction"],
        "joint_damping":    config["joint_damping"],
    }
    with open(xacro_file_path, 'r') as f:
        content = f.read()
    for key, value in xacro_variables.items():
        content = content.replace(f"${{{key}}}", str(value))
    with open(xacro_file_path, 'w') as f:
        f.write(content)

# ---------- RUN SCRIPTS ----------
def run_bash_script(script_path, *args):
    try:
        if not os.access(script_path, os.X_OK):
            subprocess.run(["chmod", "+x", script_path], check=True)
        cmd = ["bash", script_path, *map(str, args)]
        result = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Uncomment for debug:
        # print(result.stdout.decode())
        # print(result.stderr.decode())
    except subprocess.CalledProcessError as e:
        print(f"[run_bash_script] Error executing {script_path}: {e}")
        print(f"[stderr]\n{e.stderr.decode()}")
        print(f"[stdout]\n{e.stdout.decode()}")

def run_python_script(script_path, dist_connection_points):
    try:
        subprocess.run(['python3', script_path, str(dist_connection_points)],
                       check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        print(f"[run_python_script] Error executing {script_path}: {e}")
        print(f"[stderr]\n{e.stderr.decode()}")

# ---------- CALCULATIONS ----------
def calculate_distance_connection_points(elem_length_main, num_elements):
    return elem_length_main / (np.sin(np.pi / (2.0 * num_elements)))

# ---------- MAIN ----------
def main():
    # Paths
    dir_cable = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models/tether_2m"
    dir_world = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/worlds"

    script_generate_tether = f'{dir_cable}/generate_sdf.sh'
    script_update_poses   = f'{dir_world}/update_drone_poses.py'

    xacro_file_path  = f'{dir_cable}/model.xacro'
    tether_sdf_path  = f'{dir_cable}/model.sdf'
    config_file_path = f'{dir_cable}/cable_params.yaml'

    template_filenames = ['generate_sdf.sh', 'model.xacro']

    os.chdir(dir_cable)

    # 0) Setup
    print("0. Set up files")
    config = load_config(config_file_path)
    copy_and_rename_template_files(dir_cable, dir_cable, template_filenames)
    inject_into_bash_script(config, script_generate_tether)
    inject_into_xacro(config, xacro_file_path)

    # 1) Generate SDF
    print("1. Generate cable SDF")
    run_bash_script(script_generate_tether)

    # 2) Extract values and compute distances
    print("2. Calculate distance between connection points")
    num_elements = extract_from_xacro(xacro_file_path)
    elem_length_main, elem_length_ends = extract_from_sdf(tether_sdf_path)

    if num_elements is None or elem_length_main is None or elem_length_ends is None:
        print("Failed to extract necessary data from the files.")
        print(f"  num_elements: {num_elements}")
        print(f"  elem_length_main: {elem_length_main}")
        print(f"  elem_length_ends: {elem_length_ends}")
        sys.exit(1)

    cable_length = elem_length_main * num_elements + 2 * elem_length_ends
    distance_connection_points = cable_length #calculate_distance_connection_points(elem_length_main, num_elements) #cable_length
    print(f"Cable length: {cable_length:.6f} m") #elem_length_main * num_elements
    print(f"Distance between connection points: {distance_connection_points:.6f} m")

    # 3) Update world
    print("3. Update drone and tether poses in world SDF")
    run_python_script(script_update_poses, distance_connection_points)

    # 4) Done
    print("COMPLETE: All tasks finished successfully!")

if __name__ == '__main__':
    main()
