import csv
import re
import os
import argparse
import find_start
import visualisation_universal

def parse_csv(file_path):
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
        
        for row in csv_reader:
            # Check if the current row matches the header pattern
            if not header_found:
                header_match = header_pattern.match(",".join(row))
                if header_match:
                    header_found = True
                    num_coordinates = (len(row) - 2) // 3
                    continue
            
            # Process rows after the header is found
            if header_found:
                # Convert the row elements to float and handle empty strings
                row = [float(item) if item else None for item in row]
                
                # Extract the relevant coordinates
                frame = row[0]
                time = row[1]
                coordinates = row[2:2 + num_coordinates * 3]
                
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

def generate_output_path(input_file_path, suffix):
    dir_name, base_name = os.path.split(input_file_path)
    name, ext = os.path.splitext(base_name)
    output_file_name = f"{name}_{suffix}{ext}"
    return os.path.join(dir_name, output_file_name)


def main():
    parser = argparse.ArgumentParser(description="Transform CSV coordinates and visualize data.")
    parser.add_argument('actual_file', type=str, help='Path to the actual data CSV file.')
    parser.add_argument('reference_file', type=str, help='Path to the reference data CSV file.')

    args = parser.parse_args()

    input_file_path = args.actual_file
    output_file_path = generate_output_path(input_file_path, 'pre_process_s1')

    # Parse the CSV file
    data, num_coordinates = parse_csv(input_file_path)

    # Write the transformed data to a new CSV file
    write_csv(output_file_path, data, num_coordinates)

    print(f"Step 1: Transformed data has been written to {output_file_path}")

    # Correctly call the main function from the find_start module
    output_second_path = find_start.main(output_file_path)

    # Ensure visualisation_universal is correctly called if it is a function
    visualisation_universal.main(output_second_path, args.reference_file)

if __name__ == "__main__":
    main()
