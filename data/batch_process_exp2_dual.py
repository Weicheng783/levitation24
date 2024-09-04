import os
import re
import subprocess

# Function to check if the CSV filename is in the pure form "number_number.csv"
def is_pure_form_csv(filename):
    pattern = r"^\d+_\d+\.\d+\.csv$"
    return re.match(pattern, filename) is not None

# Function to find the large round number (rx) in the folder name
def find_large_round_number(folder_name):
    match = re.search(r'r(\d+)', folder_name)
    if match:
        return int(match.group(1))
    return None

# Function to call parsing_v2.py with parameters
def call_parsing_script(csv_file, large_round_num):
    # Extract the take number and velocity from the filename
    filename = os.path.basename(csv_file)
    take_number, velocity = os.path.splitext(filename)[0].split('_')
    
    # Construct reference file path by replacing ".csv" with "_target_pos.csv"
    ref_file_path = csv_file.replace(".csv", "_target_pos.csv")
    print(f"Processing CSV file: {csv_file}")
    print(f"Reference file path: {ref_file_path}")
    print(f"Take number: {take_number}, Velocity: {velocity}")

    # Call the relevant parsing file with parameters
    try:
        subprocess.run([
            'python', 'parsing_v2.py', 
            csv_file,                      # actual_file
            ref_file_path,                 # reference_file
            "4",                           # num_of_particles
            "",                            # integration_dir (assuming it's empty in this case)
            "--take_number", take_number,  # take_number argument
            "--velocity", velocity         # velocity argument
        ])
    except Exception as e:
        print(f"Error occurred: {str(e)}")

# Function to get all pure form CSV files from subdirectories
def get_csv_files(root_dir):
    csv_files = []
    for subdir, _, files in os.walk(root_dir):
        for file in files:
            if is_pure_form_csv(file):
                csv_files.append(os.path.join(subdir, file))
    return csv_files

# Main function to execute the script
def main():
    root_dir = "C:/Users/weicheng/Desktop/exp_videos/EXP3"
    csv_files = get_csv_files(root_dir)
    
    for csv_file in csv_files:
        # Get the parent folder name of the CSV file
        parent_folder = os.path.basename(os.path.dirname(csv_file))
        
        # Find the large round number (rx) from the parent folder name
        large_round_num = find_large_round_number(parent_folder)
        if large_round_num is None:
            large_round_num = 1  # Default to 1 if not found
        
        call_parsing_script(csv_file, large_round_num)

if __name__ == "__main__":
    main()
