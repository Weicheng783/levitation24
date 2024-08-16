import csv
import itertools
import re
import os
import argparse
import numpy as np
import find_start
import find_start_addon
import visualisation_universal
from testingv4_dependent import match_particles_to_agents
import concurrent.futures
import threading
import shutil

visualise_after_processing = False

# Function to process a range of combinations
def process_combinations(thread_index, start, end, input_file_path, num_of_particles, output_file_path, second_quick_fail_mode, reference_file_path, original_output_path):
    local_input_file_path = f"{input_file_path}_{thread_index}.csv"
    local_output_file_path = f"{output_file_path}_{thread_index}.csv"
    shutil.copy(input_file_path, local_input_file_path)
    
    for current_combination in range(start, end):
        try:
            print("Thread #", thread_index, ", Task #", current_combination-start+1, ", Total", end-start)
            data, num_coordinates, _, _ = parse_csv_v2(local_input_file_path, num_of_particles, True, current_combination-1)

            # Write the transformed data to a new CSV file
            write_csv(local_output_file_path, data, num_coordinates)
            
            # Verify the CSV file
            verify_csv(local_output_file_path)

            print(f"Step 1B: Transformed data has been written to {local_output_file_path}")
            
            # Correctly call the main function from the find_start module
            try:
                output_second_path = find_start.main(local_output_file_path)
            except:
                if second_quick_fail_mode:
                    print("RETAKE FLAG for:", local_output_file_path)
                    print("In this pass, No movement detected in the data or the Amount of Data is not enough or Lacking particles.")
                    print("Advice: For a consistent result, we decide to not proceed this csv file as lacking of data or data corruption which can not use computer software to recover the data.")
                    print("Please RETAKE.")
                    return None
            
            if output_second_path is None:
                output_third_path = None
            else:
                # Step 3: Match particles to agents
                output_third_path = match_particles_to_agents(get_path_file(reference_file_path), output_second_path, False, 0.02, 0.0015, 0.001, True, num_of_particles)
            
            if output_third_path is not None:
                # Rename the result file
                final_output_path = f"{os.path.splitext(original_output_path)[0]}_s3.csv"
                os.rename(output_third_path, final_output_path)
                os.remove(local_input_file_path)
                return final_output_path
        except:
            pass
    
    os.rename(local_output_file_path)
    os.remove(local_input_file_path)
    return None

def generate_combinations(num_coordinates, num_of_particles):
    """Generate combinations of marker indices."""
    return list(itertools.combinations(range(1, num_coordinates + 1), num_of_particles))

def parse_csv_v2(file_path, num_of_particles, all_visible_markers_mode, prev_combination=0):
    # Initialize an empty list to store the transformed data
    transformed_data = []
    
    # Define the header pattern regex
    header_pattern = re.compile(r"Frame,Time \(Seconds\)((,X,Y,Z)+)")
    
    # Open the CSV file for reading
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        
        # Flag to indicate if the header is found
        header_found = False
        num_coordinates = 0
        
        all_data = []
        header = []
        
        for row in csv_reader:
            # Check if the current row matches the header pattern
            if not header_found:
                header_match = header_pattern.match(",".join(row))
                if header_match:
                    header_found = True
                    header = row
                    num_coordinates = (len(row) - 2) // 3
                    continue
            
            # Process rows after the header is found
            if header_found:
                # Convert the row elements to float and handle empty strings
                row = [float(item) if item else None for item in row]
                all_data.append(row)
    
    if all_visible_markers_mode:
        combinations = generate_combinations(num_coordinates, num_of_particles)
        # print(combinations)
        # raise Exception("aaa")
        selected_combination = combinations[prev_combination + 1]
        print("Using combination #", prev_combination + 2, ",", selected_combination, ", total", len(combinations))
        current_combination = prev_combination + 1
        selected_indices = sorted(selected_combination)
        
        # Filter columns based on the selected indices
        selected_columns = []
        for i in selected_indices:
            selected_columns.extend([2 + (i - 1) * 3, 3 + (i - 1) * 3, 4 + (i - 1) * 3])
    elif num_coordinates > num_of_particles:
        # Create a numpy array to easily handle the data
        data_array = np.array(all_data, dtype=np.float64)
        
        # Calculate scores for each particle based on valid data count and variance
        column_scores = []
        
        for i in range(num_coordinates):
            x_col = 2 + i * 3
            y_col = 3 + i * 3
            z_col = 4 + i * 3
            
            cols = [x_col, y_col, z_col]
            valid_data_count = np.sum(~np.isnan(data_array[:, cols]))
            variance = np.nanvar(data_array[:, cols])
            
            # Calculate a score to rank the importance of each particle
            # Higher score means more data points and more variance
            score = valid_data_count + variance
            column_scores.append((score, i))
        
        # Sort by score and select the top num_of_particles
        column_scores.sort(reverse=True, key=lambda x: x[0])
        selected_particles = column_scores[:num_of_particles]
        selected_indices = sorted([i[1] for i in selected_particles])
        
        # Filter columns based on the selected indices
        selected_columns = []
        for i in selected_indices:
            selected_columns.extend([2 + i * 3, 3 + i * 3, 4 + i * 3])
    else:
        selected_columns = [i for i in range(2, 2 + num_coordinates * 3)]
    
    for row in all_data:
        frame = row[0]
        time = row[1]
        coordinates = [row[i] for i in selected_columns]
        
        # Apply the coordinate transformation
        transformed_coordinates = []
        for i in range(0, len(coordinates), 3):
            if coordinates[i] is not None:
                x = coordinates[i]
                y = coordinates[i + 1]
                z = coordinates[i + 2]
                
                # Apply transformations
                x_prime = -z
                y_prime = -x
                z_prime = y
                
                transformed_coordinates.extend([x_prime, y_prime, z_prime])
            else:
                transformed_coordinates.extend([None, None, None])
        
        # Append the transformed data to the list
        transformed_data.append([frame, time] + transformed_coordinates)
    
    num_coordinates = len(selected_columns) // 3
    if(all_visible_markers_mode):
        return transformed_data, num_coordinates, current_combination, len(combinations)
    return transformed_data, num_coordinates

def parse_csv(file_path, num_of_particles, all_visible_markers_mode):
    # Initialize an empty list to store the transformed data
    transformed_data = []
    
    # Define the header pattern regex
    header_pattern = re.compile(r"Frame,Time \(Seconds\)((,X,Y,Z)+)")
    
    # Open the CSV file for reading
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        
        # Flag to indicate if the header is found
        header_found = False
        num_coordinates = 0
        
        all_data = []
        header = []
        
        for row in csv_reader:
            # Check if the current row matches the header pattern
            if not header_found:
                header_match = header_pattern.match(",".join(row))
                if header_match:
                    header_found = True
                    header = row
                    num_coordinates = (len(row) - 2) // 3
                    continue
            
            # Process rows after the header is found
            if header_found:
                # Convert the row elements to float and handle empty strings
                row = [float(item) if item else None for item in row]
                all_data.append(row)
    
    if num_coordinates > num_of_particles and not all_visible_markers_mode:
        # Create a numpy array to easily handle the data
        data_array = np.array(all_data, dtype=np.float64)
        
        # Calculate scores for each particle based on valid data count and variance
        column_scores = []
        
        for i in range(num_coordinates):
            x_col = 2 + i * 3
            y_col = 3 + i * 3
            z_col = 4 + i * 3
            
            cols = [x_col, y_col, z_col]
            valid_data_count = np.sum(~np.isnan(data_array[:, cols]))
            variance = np.nanvar(data_array[:, cols])
            
            # Calculate a score to rank the importance of each particle
            # Higher score means more data points and more variance
            score = valid_data_count + variance
            column_scores.append((score, i))
        
        # Sort by score and select the top num_of_particles
        column_scores.sort(reverse=True, key=lambda x: x[0])
        selected_particles = column_scores[:num_of_particles]
        selected_indices = sorted([i[1] for i in selected_particles])
        
        # Filter columns based on the selected indices
        selected_columns = []
        for i in selected_indices:
            selected_columns.extend([2 + i * 3, 3 + i * 3, 4 + i * 3])
    else:
        selected_columns = [i for i in range(2, 2 + num_coordinates * 3)]
    
    for row in all_data:
        frame = row[0]
        time = row[1]
        coordinates = [row[i] for i in selected_columns]
        
        # Apply the coordinate transformation
        transformed_coordinates = []
        for i in range(0, len(coordinates), 3):
            if coordinates[i] is not None:
                x = coordinates[i]
                y = coordinates[i + 1]
                z = coordinates[i + 2]
                
                # Apply transformations
                x_prime = -z
                y_prime = -x
                z_prime = y
                
                transformed_coordinates.extend([x_prime, y_prime, z_prime])
            else:
                transformed_coordinates.extend([None, None, None])
        
        # Append the transformed data to the list
        transformed_data.append([frame, time] + transformed_coordinates)
    
    num_coordinates = len(selected_columns) // 3
    return transformed_data, num_coordinates

def write_csv(output_file_path, data, num_coordinates):
    with open(output_file_path, 'w', newline='') as file:
        csv_writer = csv.writer(file)
        
        # Write the header row
        header = ['time']
        for i in range(num_coordinates):
            header += [f'x{i}', f'y{i}', f'z{i}']
        csv_writer.writerow(header)
        
        # Write the data rows
        for row in data:
            csv_writer.writerow(row[1:])

def verify_csv(output_file_path):
    # Verify if the CSV file is well-formed
    with open(output_file_path, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        num_columns = len(header)
        for i, row in enumerate(csv_reader):
            if len(row) != num_columns:
                print(f"Row {i+1} has {len(row)} columns, expected {num_columns}")

def generate_output_path(input_file_path, suffix):
    dir_name, base_name = os.path.split(input_file_path)
    name, ext = os.path.splitext(base_name)
    output_file_name = f"{name}_{suffix}{ext}"
    return os.path.join(dir_name, output_file_name)

def get_path_file(original_path):
    # Get the directory one level up from the original path
    parent_dir = os.path.dirname(original_path)
    
    # Create the new path by appending 'pathFile.csv'
    new_path = os.path.join(parent_dir, "pathFile.csv")
    
    return new_path

def main():
    parser = argparse.ArgumentParser(description="Transform CSV coordinates and visualize data.")
    parser.add_argument('actual_file', type=str, help='Path to the actual data CSV file.')
    parser.add_argument('reference_file', type=str, help='Path to the reference data CSV file.')
    parser.add_argument('num_of_particles', type=int, help='Maximum number of particles to process.')

    args = parser.parse_args()

    input_file_path = args.actual_file
    num_of_particles = args.num_of_particles
    reference_file_path = args.reference_file
    output_file_path = generate_output_path(input_file_path, 'pre_process_s1')

    # Parse the CSV file
    data, num_coordinates = parse_csv(input_file_path, num_of_particles, False)

    # Write the transformed data to a new CSV file
    write_csv(output_file_path, data, num_coordinates)

    # Verify the CSV file
    verify_csv(output_file_path)

    print(f"Step 1: Transformed data has been written to {output_file_path}")

    second_quick_fail_mode = False
    output_second_path = None

    # Correctly call the main function from the find_start module
    try:
        output_second_path = find_start.main(output_file_path)
    except:
        if(second_quick_fail_mode):
            print("RETAKE FLAG for:", output_file_path)
            print("In this pass, No movement detected in the data or the Amount of Data is not enough or Lacking particles.")
            print("Advice: For a consistent result, we decide to not proceed this csv file as lacking of data or data corruption which can not use computer software to recover the data.")
            print("Please RETAKE.")
            exit()

    if(output_second_path == None):
        output_third_path = None
    else:
        # Step 3: Match particles to agents
        output_third_path = match_particles_to_agents(get_path_file(reference_file_path), output_second_path)
        success_flag = True

    if(output_third_path == None):
        current_combination = 0
        prev_combination = 0
        total_num_combination = 0
        combination_max_try = 0
        first_enter_flag = True
        success_flag = False
        # while(True):
        if((total_num_combination != 0 and current_combination < total_num_combination) or first_enter_flag):
            # Turn on the all_visible_markers_mode to allow all trackers in the first processing step
            try:
                data, num_coordinates, current_combination, total_num_combination = parse_csv_v2(input_file_path, num_of_particles, True, prev_combination)
            except:
                pass
            prev_combination = current_combination

            # Concurrent parallel implementation for faster search
            # Determine the range of combinations each thread will process
            num_threads = 70
            combinations_per_thread = total_num_combination // num_threads
            ranges = [(i * combinations_per_thread, (i + 1) * combinations_per_thread) for i in range(num_threads)]

            # Use a ThreadPoolExecutor to manage threads
            with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
                # def process_combinations(thread_index, start, end, input_file_path, num_of_particles, output_file_path, second_quick_fail_mode, reference_file_path):
                futures = {
                    executor.submit(process_combinations, index, start, end, input_file_path, num_of_particles, output_file_path, second_quick_fail_mode, reference_file_path, output_second_path): (start, end) 
                    for index, (start, end) in enumerate(ranges)
                }
                
                for future in concurrent.futures.as_completed(futures):
                    output_third_path = future.result()
                    if output_third_path is not None:
                        print(f"Success! Found valid output at combination range: {futures[future]}")
                        # return output_third_path

            first_enter_flag = False
            if(output_third_path != None):
                success_flag = True
            else:
                print("No valid output found within the given range of combinations.")
                print("Please RETAKE.")
        else:
            pass
    
    if(success_flag):
        # Ensure visualisation_universal is correctly called if it is a function
        print("Pre-processing steps 1-3 is successful for this take!")
        if(visualise_after_processing):
            visualisation_universal.main(output_third_path, args.reference_file)
    else:
        print("** Data Corruption: No particle is matched after trying all possible unique combinations, please recollect the data for this take. **")

if __name__ == "__main__":
    main()
