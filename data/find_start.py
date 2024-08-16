import pandas as pd
import numpy as np
import os

def main(preprocessed_file_name):
    # Load the CSV file into a pandas DataFrame
    file_path = preprocessed_file_name
    df = pd.read_csv(file_path)

    # Define a threshold for movement detection (you can adjust this based on your data)
    movement_threshold = 0.004  # Example threshold; adjust as needed

    # Identify the columns for particle positions dynamically
    position_columns = [col for col in df.columns if col.startswith('x') or col.startswith('y') or col.startswith('z')]

    # Initialize variables
    initial_positions = None
    start_index = None

    # Iterate through each row and check movement
    for index, row in df.iterrows():
        # Extract positions for each particle at the current time
        positions = row[position_columns].values  # Convert to NumPy array for easier manipulation
        
        # Check if any particle has moved beyond the threshold
        if initial_positions is None:
            initial_positions = positions
            continue
        
        movement_detected = np.linalg.norm(positions - initial_positions) > movement_threshold
        
        if movement_detected:
            start_index = index
            break

    if start_index is None:
        raise ValueError("No movement detected in the data or the Amount of Data is not enough or Lacking particles.")

    # Remove frames before the detected start time
    df_movement = df.iloc[start_index:].reset_index(drop=True)

    # Reset the time column to start from 0.00 and count forward by the original time difference
    time_diff = df['time'].iloc[1] - df['time'].iloc[0]
    df_movement['time'] = np.arange(0, len(df_movement) * time_diff, time_diff)

    # Output the modified DataFrame to a new CSV file
    output_file_path = generate_output_path(file_path, 's2')
    df_movement.to_csv(output_file_path, index=False)

    print(f"Step 2: Starting Frame Finder: Modified data saved to {output_file_path}")
    return output_file_path

def generate_output_path(input_file_path, suffix):
    dir_name, base_name = os.path.split(input_file_path)
    name, ext = os.path.splitext(base_name)
    output_file_name = f"{name}_{suffix}{ext}"
    return os.path.join(dir_name, output_file_name)