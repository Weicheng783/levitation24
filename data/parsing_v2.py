import csv
import itertools
import re
import os
import argparse
import numpy as np
import find_start
import pandas as pd
import find_start_addon
import visualisation_universal
from testingv4_dependent import match_particles_to_agents

visualise_after_processing = False
direct_mode = False

def generate_combinations(num_coordinates, num_of_particles):
    """Generate combinations of marker indices."""
    return list(itertools.combinations(range(1, num_coordinates + 1), num_of_particles))

def parse_csv_v2(file_path, num_of_particles, all_visible_markers_mode, prev_combination=0):
    # Initialize an empty list to store the transformed data
    transformed_data = []
    
    # Define the header pattern regex
    header_pattern = re.compile(r"Frame,Time \(Seconds\)((,X,Y,Z)+)")

    exceed_cap = False
    
    # Open the CSV file for reading
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        
        if(not direct_mode):
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
        else:
            num_coordinates = 0
            
            all_data = []
            first_flag = True
            for row in csv_reader:
                if(first_flag):
                    first_flag = False
                    continue
                # Check if the current row matches the header pattern
                # print(row)
                # temp = []
                num_coordinates = (len(row) - 1) // 3
                # print(num_coordinates)
                # Process rows
                # Convert the row elements to float and handle empty strings
                row = [float(item) if item else None for item in row]
                # for item in row:
                #     # print(item)
                #     if(item is not float and item != ""):
                #         # print("here")
                #         temp.append(item)
                #     else:
                #         temp.append(None)

                    # row[item] = float(item)
                # print(all_data)
                all_data.append(row)
    
    if all_visible_markers_mode:
        # if(num_coordinates > 12):
        #     print("Number of coordinates captured:", num_coordinates)
        #     num_coordinates = 12
        #     print("Number of coordinates is capped at 12, for fast finding and more accurate combinations.")
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
            if(direct_mode):
                selected_columns.extend([1 + (i - 1) * 3, 2 + (i - 1) * 3, 3 + (i - 1) * 3])
            else:
                selected_columns.extend([2 + (i - 1) * 3, 3 + (i - 1) * 3, 4 + (i - 1) * 3])
    elif num_coordinates > num_of_particles:
        # Create a numpy array to easily handle the data
        data_array = np.array(all_data, dtype=np.float64)
        
        # Calculate scores for each particle based on valid data count and variance
        column_scores = []
        
        for i in range(num_coordinates):
            if(direct_mode):
                x_col = 1 + i * 3
                y_col = 2 + i * 3
                z_col = 3 + i * 3
            else:
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
            if(direct_mode):
                selected_columns.extend([1 + i * 3, 2 + i * 3, 3 + i * 3])
            else:
                selected_columns.extend([2 + i * 3, 3 + i * 3, 4 + i * 3])
    else:
        if(direct_mode):
            selected_columns = [i for i in range(1, 1 + num_coordinates * 3)]
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
                if(direct_mode):
                    x_prime = x
                    y_prime = y
                    z_prime = z
                else:
                    x_prime = -z
                    y_prime = -x
                    z_prime = y
                
                transformed_coordinates.extend([x_prime, y_prime, z_prime])
            else:
                transformed_coordinates.extend([None, None, None])
        
        # Append the transformed data to the list
        if(direct_mode):
            transformed_data.append([time] + transformed_coordinates)
        else:
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
        
        if(not direct_mode):
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
        else:
            num_coordinates = 0
            all_data = []
            first_time_in_row = True
            for row in csv_reader:
                if(first_time_in_row):
                    first_time_in_row = False
                    continue
                # Check if the current row matches the header pattern
                # print(row)
                temp = []
                num_coordinates = (len(row) - 1) // 3
                # print(num_coordinates)
                # Process rows
                # Convert the row elements to float and handle empty strings
                row = [float(item) if item else None for item in row]
                # for item in row:
                #     # print(item)
                #     if(item is not float and item != ""):
                #         # print("here")
                #         temp.append(item)
                #     else:
                #         temp.append(None)

                    # row[item] = float(item)
                # print(all_data)
                all_data.append(row)
    
    if num_coordinates > num_of_particles and not all_visible_markers_mode:
        # Create a numpy array to easily handle the data
        data_array = np.array(all_data, dtype=np.float64)
        
        # Calculate scores for each particle based on valid data count and variance
        column_scores = []
        
        for i in range(num_coordinates):
            if(direct_mode):
                x_col = 1 + i * 3
                y_col = 2 + i * 3
                z_col = 3 + i * 3 
            else:
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
            if(direct_mode):
                selected_columns.extend([1 + i * 3, 2 + i * 3, 3 + i * 3])
            else:
                selected_columns.extend([2 + i * 3, 3 + i * 3, 4 + i * 3])
    else:
        if(direct_mode):
            selected_columns = [i for i in range(1, 1 + num_coordinates * 3)]
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
                if(direct_mode):
                    x_prime = x
                    y_prime = y
                    z_prime = z
                else:
                    x_prime = -z
                    y_prime = -x
                    z_prime = y
                
                transformed_coordinates.extend([x_prime, y_prime, z_prime])
            else:
                transformed_coordinates.extend([None, None, None])
        
        # Append the transformed data to the list
        if(direct_mode):
            transformed_data.append([time] + transformed_coordinates)
        else:
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
        data_counter = 0
        for row in data:
            if(direct_mode):
                time_gen = []
                time_gen.append(0.001 * data_counter)
                csv_writer.writerow(time_gen + row[1:])
                data_counter += 1
            else:
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
    parser.add_argument('integration_dir', type=str, help='[Optional] Motive Tracker Integration Directory.', default="")

    args = parser.parse_args()

    input_file_path = args.actual_file
    num_of_particles = args.num_of_particles
    reference_file_path = args.reference_file
    integration_dir = args.integration_dir
    output_file_path = generate_output_path(input_file_path, 'c_pre_process_s1')

    trim_step_only = False
    df1_step_only = False
    visualize_only = False

    if(not trim_step_only and not df1_step_only and not visualize_only):
        # Pre-requisite cut to a threshold
        # Load the CSV file
        # Specify the column number after which you want to remove columns
        # (e.g., if you want to keep up to the 5th column, set column_number to 5)
        column_number = 12 * 3 + 2 # 12 particles, each with xyz, 2 columns for time and frame
        # Slice the DataFrame to keep only the columns up to the specified number
        with open(input_file_path, 'r', newline='') as infile, open(input_file_path.replace(".csv", "_c.csv"), 'w', newline='') as outfile:
            reader = csv.reader(infile)
            writer = csv.writer(outfile)
            
            # Iterate through each row in the input file
            for row in reader:
                # Write the row with only the first 'column_number' columns to the output file
                writer.writerow(row[:column_number])

        input_file_path = input_file_path.replace(".csv", "_c.csv")

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
            try:
                if(integration_dir != ""):
                    output_third_path = match_particles_to_agents(integration_dir + "_path_file.csv", output_second_path)
                else:
                    output_third_path = match_particles_to_agents(get_path_file(reference_file_path), output_second_path)
            except:
                if(integration_dir != ""):
                    output_third_path = match_particles_to_agents(integration_dir + "_path_file_exp.csv", output_second_path)
                else:
                    parent_dir_exp = os.path.dirname(reference_file_path)
                    # Create the new path by appending 'pathFile.csv'
                    new_path_exp = os.path.join(parent_dir_exp, "pathFile_exp.csv")
                    output_third_path = match_particles_to_agents(new_path_exp, output_second_path)

            success_flag = True

        if(output_third_path == None):
            current_combination = 0
            prev_combination = 0
            total_num_combination = 0
            combination_max_try = 2000
            first_enter_flag = True
            success_flag = False
            while(True):
                if((total_num_combination != 0 and current_combination < total_num_combination) or first_enter_flag):
                    # Turn on the all_visible_markers_mode to allow all trackers in the first processing step
                    try:
                        data, num_coordinates, current_combination, total_num_combination = parse_csv_v2(input_file_path, num_of_particles, True, prev_combination)
                        if(combination_max_try != 0 and current_combination > combination_max_try):
                            print("Current trial number exceeds the setting threshold.")
                            break
                    except:
                        break
                    prev_combination = current_combination
                    # Write the transformed data to a new CSV file
                    write_csv(output_file_path, data, num_coordinates)
                    # Verify the CSV file
                    verify_csv(output_file_path)
                    print(f"Step 1B: Transformed data has been written to {output_file_path}")
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
                        try:
                            if(integration_dir != ""):
                                output_third_path = match_particles_to_agents(integration_dir + "_path_file.csv", output_second_path, False, 0.03, 0.0015, 0.001, True, num_of_particles)
                            else:
                                output_third_path = match_particles_to_agents(get_path_file(reference_file_path), output_second_path, False, 0.03, 0.0015, 0.001, True, num_of_particles)
                        except:
                            if(integration_dir != ""):
                                output_third_path = match_particles_to_agents(integration_dir + "_path_file_exp.csv", output_second_path, False, 0.03, 0.0015, 0.001, True, num_of_particles)
                            else:
                                parent_dir_exp = os.path.dirname(reference_file_path)
                                # Create the new path by appending 'pathFile.csv'
                                new_path_exp = os.path.join(parent_dir_exp, "pathFile_exp.csv")
                                output_third_path = match_particles_to_agents(new_path_exp, output_second_path, False, 0.03, 0.0015, 0.001, True, num_of_particles)

                    first_enter_flag = False
                    if(output_third_path != None):
                        success_flag = True
                        break
                else:
                    break
        
        if(success_flag):
            # Ensure visualisation_universal is correctly called if it is a function
            print("Pre-processing steps 1-3 is successful for this take!")
            trim_csv_to_match(output_third_path, reference_file_path)
            output_fourth_path = generate_output_path(input_file_path, 'pre_process_s1_s2_s3_s4')

            output_r1_path, output_r1_label_path = compare_particles_and_add_status(output_fourth_path, reference_file_path, 0.1)
            merge_csv_files([output_r1_path, reference_file_path.replace('sim_output_TargetPosition.csv', 'sim_output_pointPressure.csv'), reference_file_path.replace('sim_output_TargetPosition.csv', 'sim_output_GorkovPotential.csv')], output_r1_path.replace("df1.csv","df2.csv"))
            output_r2_path = output_r1_path.replace("df1.csv","df2.csv")

            # Now it is time to decide whether particles dropped exclusively
            # Step 1: Load the CSV file
            df = pd.read_csv(output_r1_path.replace("df1.csv","df_labels.csv"), delimiter=',')  # Adjust the delimiter if necessary
            # Step 2: Initialize a list to hold the results
            results = []
            # Step 3: Iterate over each column to check for '1's
            col_count = 0
            for col in df.columns:
                if df[col].eq(1).any():
                    results.append(f"{col_count} y")
                else:
                    results.append(f"{col_count} n")
                col_count += 1
            
            output_r3_path = output_r1_path.replace("df1.csv","drop.txt")
            # Step 4: Write the results to a text file
            with open(output_r3_path, 'w') as f:
                for result in results:
                    f.write(result + '\n')
            print("Check completed. Drop Results saved.")
            
            if(visualise_after_processing):
                visualisation_universal.main(output_fourth_path, args.reference_file)
        else:
            print("** Data Corruption: No particle is matched after trying all possible unique combinations, please recollect the data for this take. **")
    else:
        trim_csv_to_match(generate_output_path(input_file_path, 'pre_process_s1_s2_s3'), reference_file_path)

    if(df1_step_only):
        output_r1_path, output_r1_label_path = compare_particles_and_add_status(generate_output_path(input_file_path, 'pre_process_s1_s2_s3_s4'), reference_file_path, 0.05)
        merge_csv_files([output_r1_path, reference_file_path.replace('sim_output_TargetPosition.csv', 'sim_output_pointPressure.csv'), reference_file_path.replace('sim_output_TargetPosition.csv', 'sim_output_GorkovPotential.csv')], output_r1_path.replace("df1.csv","df2.csv"))
        output_r2_path = output_r1_path.replace("df1.csv","df2.csv")
    
    if(visualize_only):
        visualisation_universal.main(generate_output_path(input_file_path, 'pre_process_s1_s2_s3_s4'), args.reference_file)


def trim_csv_to_match(csv_path1, csv_path2):
    # Read the CSV files
    if(os.path.exists(csv_path1)):
        df1 = pd.read_csv(csv_path1)
        df2 = pd.read_csv(csv_path2)
        
        # Get the number of rows in each dataframe
        rows_df1 = len(df1)
        rows_df2 = len(df2)
        
        # Trim df1 if it has more rows than df2
        if rows_df1 > rows_df2:
            df1 = df1.iloc[:rows_df2]
        
        # Create the output path by replacing ".csv" with "_s4.csv"
        output_path = csv_path1.replace('.csv', '_s4.csv')
        
        # Write the trimmed dataframe to the new CSV file
        df1.to_csv(output_path, index=False)
        
        print(f"Trimmed file saved to: {output_path}")


def compare_particles_and_add_status(trimmed_csv_path, reference_csv_path, threshold=0.05):
    # Read the CSV files
    df_trimmed = pd.read_csv(trimmed_csv_path)
    df_reference = pd.read_csv(reference_csv_path)
    
    # Initialize a list to store the new rows with distances and dropping status
    new_data = []
    dropping_status = np.zeros((len(df_trimmed.columns) - 1) // 3, dtype=bool)
    label_data = []
    
    # Loop through each row (time step) in the dataframes
    for idx, row in df_trimmed.iterrows():
        new_row = list(row)
        time_step = row[0]
        label_row = []
        
        # Loop through each particle (each 3 columns represent xyz)
        for i in range(1, len(row), 3):
            x1, y1, z1 = row[i], row[i+1], row[i+2]
            x2, y2, z2 = df_reference.iloc[idx, i], df_reference.iloc[idx, i+1], df_reference.iloc[idx, i+2]
            
            if pd.isna(x1) or pd.isna(y1) or pd.isna(z1) or pd.isna(x2) or pd.isna(y2) or pd.isna(z2):
                # If any coordinate is missing, use the previous distance and status
                distance = np.nan
                if new_data:
                    distance = new_data[-1][len(new_row)]
                new_row.append(distance)
                if(dropping_status[(i-1)//3]):
                    label_row.append(1)
                else:
                    label_row.append(0)
            else:
                # Calculate the Euclidean distance
                distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
                new_row.append(distance)
                
                # Determine if the particle has dropped
                if distance > threshold or dropping_status[(i-1)//3]:
                    dropping_status[(i-1)//3] = True
                
                if(dropping_status[(i-1)//3]):
                    label_row.append(1)
                else:
                    label_row.append(0)
        
        new_data.append(new_row)
        label_data.append(label_row)
    
    # Create a new dataframe with distances and dropping status
    original_columns = list(df_trimmed.columns)
    print(original_columns)
    print(len(original_columns))
    cccount = 0
    for i in range(1, len(original_columns), 3):
        original_columns[i] = "x" + str(cccount)
        original_columns[i+1] = "y" + str(cccount)
        original_columns[i+2] = "z" + str(cccount)
        cccount += 1

    new_columns = []
    for i in range(1, len(original_columns), 3):
        particle_number = i // 3
        new_columns.append(f'distance_{particle_number}')
    
    columns = original_columns + new_columns
    new_df = pd.DataFrame(new_data, columns=columns)

    label_columns = []
    cccount = 0
    for i in range(1, len(original_columns), 3):
        label_columns += ["drop_" + str(cccount)]
        cccount += 1

    label_df = pd.DataFrame(label_data, columns=label_columns)

    # Add the reference dataframe columns to the new dataframe
    # ref_columns = [f'ref_{coord}{i//3}' for i in range(1, len(df_reference.columns)) for coord in ['x', 'y', 'z']]
    
    # Drop the last column of the reference dataframe
    df_reference = df_reference.iloc[:, :-1]

    ref_columns = list(df_reference.columns)
    print(ref_columns)
    print(len(ref_columns))
    cccount = 0
    for i in range(1, len(ref_columns), 3):
        ref_columns[i] = "ref_x" + str(cccount)
        ref_columns[i+1] = "ref_y" + str(cccount)
        ref_columns[i+2] = "ref_z" + str(cccount)
        cccount += 1
    
    df_reference.columns = ref_columns
    
    combined_df = pd.concat([new_df, df_reference.drop(columns=['time'])], axis=1)
    
    # Create the output path by replacing ".csv" with "_status.csv"
    output_path = trimmed_csv_path.replace('s1_s2_s3_s4.csv', 'df1.csv')
    label_path = trimmed_csv_path.replace('s1_s2_s3_s4.csv', 'df_labels.csv')
    
    # Write the new dataframe to the new CSV file
    combined_df.to_csv(output_path, index=False)
    label_df.to_csv(label_path, index=False)
    
    print(f"Status file saved to: {output_path}")
    print(f"Label file saved to: {label_path}")
    return output_path, label_path

def merge_csv_files(file_paths, output_file):
    # Check if there are enough CSV files to merge
    if len(file_paths) < 2:
        print("Need at least two CSV files to merge.")
        return
    
    # Read the first CSV file
    merged_df = pd.read_csv(file_paths[0])
    
    # Round the first column of the first file to two decimal places
    merged_df[merged_df.columns[0]] = merged_df[merged_df.columns[0]].round(2)
    
    # Iterate over remaining CSV files
    for file in file_paths[1:]:
        df = pd.read_csv(file)
        
        # Drop the last column
        df.drop(df.columns[-1], axis=1, inplace=True)
        
        # Round the first column to two decimal places
        df[df.columns[0]] = df[df.columns[0]].round(2)
        
        # Merge with the first dataframe on the first column
        merged_df = pd.merge(merged_df, df, on=merged_df.columns[0])
    
    # Write the merged data to a new CSV file
    merged_df.to_csv(output_file, index=False)
    print(f"Merged CSV file saved to {output_file}")

if __name__ == "__main__":
    main()
