import argparse
import os

def parse_amplitude_file(file_path):
    amplitude_data = {}
    with open(file_path, 'r') as file:
        # Read the string from the first line
        string_line = file.readline().strip()
        # Read the rest of the file for amplitude values
        for line in file:
            parts = line.split()
            if len(parts) == 2:
                index, amplitude = parts
                amplitude_data[int(index)] = float(amplitude)
    return amplitude_data, string_line

def parse_drop_file(file_path):
    drop_data = {}
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.split()
            if len(parts) == 2:
                index, drop = parts
                drop_data[int(index)] = drop == 'y'
    return drop_data

def find_pairs(folder_path):
    amplitude_files = set()
    drop_files = set()
    
    # Traverse the directory and collect file names
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith("amplitudeFile.txt"):
                base_name = file.rsplit('_', 1)[0]
                amplitude_files.add(base_name)
            elif file.endswith("c_pre_process_drop.txt"):
                base_name = file.rsplit('_', 4)[0]
                drop_files.add(base_name)
    
    # Determine pairs
    pairs = []
    for base_name in amplitude_files:
        if base_name in drop_files:
            pairs.append((base_name + "_amplitudeFile.txt", base_name + "_c_pre_process_drop.txt"))
    
    return pairs

def process_pair(folder_path, amplitude_file, drop_file):
    amplitude_path = os.path.join(folder_path, amplitude_file)
    drop_path = os.path.join(folder_path, drop_file)
    
    amplitude_data, amplitude_string = parse_amplitude_file(amplitude_path)
    drop_data = parse_drop_file(drop_path)
    
    # Ensure all particle numbers are included
    all_particles = sorted(set(amplitude_data.keys()).union(drop_data.keys()))
    
    results = []
    drop_status = []

    for i in all_particles:
        drop_status.append(drop_data.get(i, False)) # Default to False if not in drop data

    for i in all_particles:
        amplitude = amplitude_data.get(i, None)
        if amplitude is not None:
            # Create the formatted output string
            result = f"{i} {amplitude_string.split('_')[1]} {amplitude:.0f} {amplitude_string}"
            for drop in drop_status:
                result += " " + str(drop)
            results.append(result)
    
    return results

def remove_empty_lines(file_path):
    # Read the contents of the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Filter out empty lines
    non_empty_lines = [line for line in lines if line.strip()]

    # Write the non-empty lines back to the file, ensuring no extra newline at the end
    with open(file_path, 'w') as file:
        if non_empty_lines:
            file.write(non_empty_lines[0].rstrip())  # Write the first line without trailing newline
            for line in non_empty_lines[1:]:
                file.write('\n' + line.rstrip())  # Write each subsequent line with a newline

def main():
    parser = argparse.ArgumentParser(description="Bayesian Network Evidence Adder")
    parser.add_argument('--num_particles', type=int, required=True, help='The number of particles in the Bayesian Network.')
    args = parser.parse_args()
    
    num_particles = args.num_particles
    
    file_path = "C:/Users/weicheng/Desktop/ServerSampler/bin/bnTrainerFolders.txt"
    output_file = "C:/Users/weicheng/Desktop/formal_dataset/training_data_" + str(num_particles) + "_particles.txt"
    
    # Read folder paths from the file
    with open(file_path, 'r') as file:
        folder_paths = [line.strip() for line in file if line.strip()]
    
    all_results = []
    
    for folder_path in folder_paths:
        if os.path.isdir(folder_path):
            pairs = find_pairs(folder_path)
            for amplitude_file, drop_file in pairs:
                results = process_pair(folder_path, amplitude_file, drop_file)
                if results:
                    all_results.append((folder_path, results))
    
    # Output results
    with open(output_file, 'a') as file:  # Open file in append mode
        for folder_path, results in all_results:
            for result in results:
                file.write("\n" + result)  # Ensure each result starts on a new line

    remove_empty_lines(output_file)

    with open(file_path, 'w') as file:
        file.write("")

if __name__ == "__main__":
    main()
