import os
import re
import subprocess

# Function to parse the CSV filename
def parse_filename(filename):
    pattern = r"(\d+)a-(\d+)-([a-zA-Z]+)-([a-zA-Z]+)-(\d+\.\d+)v-(\d+)(?:_(\d+))?"
    match = re.match(pattern, filename)
    if match:
        agents, version, solver, test_path, velocity, round_num, additional_round = match.groups()
        round_num = int(round_num) if round_num else 1  # Default to round 1 if not specified
        if additional_round:
            round_num = int(additional_round) + 1
        return {
            "agents": int(agents),
            "version": version,
            "solver": solver,
            "test_path": test_path,
            "velocity": velocity,
            "round_num": round_num
        }
    return None

# Function to find the large round number (rx) in the folder name
def find_large_round_number(folder_name):
    match = re.search(r'r(\d+)', folder_name)
    if match:
        return int(match.group(1))
    return None

# Function to call parsing_x.py with parameters
def call_parsing_script(csv_file, large_round_num):
    parsed_info = parse_filename(os.path.basename(csv_file))
    if parsed_info:
        version_folder = parsed_info['version']
        agents = parsed_info['agents']
        test_path = parsed_info['test_path']
        solver = parsed_info['solver']
        velocity = parsed_info['velocity']
        
        # Adjust solver name if necessary
        if solver == "gspat":
            solver = "gs-pat"
        
        # Construct reference file path
        ref_file_path = f"./{version_folder}/{agents}/{test_path}/{solver}/{velocity}/{large_round_num}/sim_output_TargetPosition.csv"
        print(csv_file)
        print(ref_file_path)

        # Call the relevant parsing file with parameters
        try:
            subprocess.run(['python', 'parsing_v2.py', csv_file, ref_file_path, str(agents), ""])
        except:
            raise Exception("DODODO")
            # subprocess.run(['python', 'parsing_v2.py', csv_file, ref_file_path, str(agents)])

# Function to get all CSV files from first-level subdirectories starting with "p"
def get_csv_files():
    folder_exclusion_list = []
    folder_explore_list = ["p4_x_r1_240813_1"]
    folder_explore_only_mode = True
    csv_files = []
    current_dir = os.getcwd()
    for dir_name in os.listdir(current_dir):
        if ((not folder_explore_only_mode and os.path.isdir(dir_name) and dir_name.startswith('p') and dir_name not in folder_exclusion_list) or (folder_explore_only_mode and os.path.isdir(dir_name) and dir_name in folder_explore_list)):
            folder_path = os.path.join(current_dir, dir_name)
            for file in os.listdir(folder_path):
                if file.endswith('.csv') and "pre_process" not in file:
                    csv_files.append(os.path.join(folder_path, file))
    return csv_files

# Main function to execute the script
def main():
    csv_files = get_csv_files()
    
    for csv_file in csv_files:
        # Get the parent folder name of the CSV file (e.g., p3_r1)
        parent_folder = os.path.basename(os.path.dirname(csv_file))
        
        # Find the large round number (rx) from the parent folder name
        large_round_num = find_large_round_number(parent_folder)
        if large_round_num is not None:
            call_parsing_script(csv_file, large_round_num)
        else:
            print(f"Error: Large round number not found in folder {parent_folder}")

if __name__ == "__main__":
    main()
