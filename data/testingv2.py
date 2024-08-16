import pandas as pd
import numpy as np

# Function to check if positions are within the threshold
def is_within_threshold(agent_pos, particle_pos, threshold):
    return all(abs(a - p) <= threshold for a, p in zip(agent_pos, particle_pos))

# Function to read and parse the agent CSV
def parse_agents_csv(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
    agents_data = []
    current_agent_id = None
    for line in lines:
        parts = line.strip().split(',')
        if parts[0] == 'Agent ID':
            current_agent_id = int(parts[1])
            print(f"Reading data for Agent ID: {current_agent_id}")
        elif parts[0].startswith('Point'):
            time = round(float(parts[1]), 2)  # Round to nearest 0.01
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            print(f"  Time: {time}, Position: ({x}, {y}, {z})")
            agents_data.append({
                'agent_id': current_agent_id,
                'time': time,
                'x': x,
                'y': y,
                'z': z
            })
    
    return pd.DataFrame(agents_data)

file_name2 = r"C:\Users\weicheng\Desktop\formal_dataset\p3_r1\3a-240702-gspat-random-0.08v-1_pre_process_s1_s2.csv"
# file_name2 = r"c:\Users\weicheng\Desktop\actual_pos\GS_PAT_random-forward-v-0.02-s-1-p6_round_1_001_pre_process_s1_s2.csv"
# Load the CSV files
agents_df = parse_agents_csv(r"C:\Users\weicheng\Desktop\formal_dataset\240702\3\random\gs-pat\0.08\1\pathFile.csv")
# agents_df = parse_agents_csv(r"C:\Users\weicheng\Desktop\AcousticPathPlanning\AcousticPathPlanning\weicheng\6\random-GS_PAT-forward-v-0.02-s-1-round_1-pathFile.csv")
particles_df = pd.read_csv(file_name2)

# Round the time values in particles_df to the nearest 0.01
particles_df['time'] = particles_df['time'].round(2)

# Default to only compare at time 0
compare_time_zero_only = False

# Threshold value
threshold_step = 0.001
threshold_hi = 0.02
threshold_lo = 0.0015
threshold = threshold_lo

# Create a dictionary to store which particle matches which agent
matches = {}

while(True):
    # Determine the time steps to process
    time_steps = [0] if compare_time_zero_only else agents_df['time'].unique()
    print(f"Time steps to process: {time_steps}")

    # Loop through each time step
    for time in time_steps:
        print(f"Processing time step: {time}")
        # Get agents' positions at the current time
        agents_at_time = agents_df[agents_df['time'] == time]
        print(f"Agents at time {time}:")
        print(agents_at_time)
        
        # Get particles' positions at the current time
        particles_at_time = particles_df[particles_df['time'] == time]
        if particles_at_time.empty:
            print(f"No particles found at time {time}, skipping.")
            continue  # Skip if there are no particles at this time step

        particle_columns = [col for col in particles_df.columns if col.startswith('x')]
        print(f"Particle columns: {particle_columns}")

        # Compare each agent with each particle
        for _, agent_row in agents_at_time.iterrows():
            agent_id = agent_row['agent_id']
            agent_pos = (agent_row['x'], agent_row['y'], agent_row['z'])
            print(f"Comparing Agent {agent_id} at position {agent_pos}")

            for i in range(len(particle_columns)):
                particle_pos = (
                    particles_at_time.iloc[0][f'x{i}'], 
                    particles_at_time.iloc[0][f'y{i}'], 
                    particles_at_time.iloc[0][f'z{i}']
                )
                print(f"  Particle {i} at position {particle_pos}")

                if is_within_threshold(agent_pos, particle_pos, threshold):
                    matches[i] = agent_id
                    print(f"  Match found: Particle {i} matches Agent {agent_id}")
                    break  # Move to the next agent once a match is found
    
    # Check if all particles are matched
    if len(matches) == len(particle_columns):
        break  # Exit loop if all particles are matched
    
    if threshold >= threshold_hi:
        print("Maximum threshold reached, no match found.")
        break
    else:
        threshold += threshold_step

# Output the matches
if len(matches) == len(particle_columns):
    print("Particle to Agent Matches:")

    agent_map = {}

    for particle_id, agent_id in matches.items():
        print(f"Particle {particle_id} matches Agent {agent_id}")
        agent_map[agent_id] = particle_id
    
    # Create a new DataFrame for the output CSV
    output_data = {'time': particles_df['time']}
    release_count = 0

    best_mapping = []

    for agent_id in range(len(matches)):
        best_mapping.append(agent_map[agent_id])

    # Remap columns in df1 based on best_mapping
    remapped_columns = ['time']  # Start with 'time' column
    for idx in best_mapping:
        remapped_columns.extend([f'x{int(idx)}', f'y{int(idx)}', f'z{int(idx)}'])

    remapped_df = particles_df[remapped_columns]  # Create a new DataFrame with remapped columns

    # for particle_id, agent_id in matches.items():
    # for particle_id in range(len(matches)):
    #     agent_id = matches[particle_id]
    #     particle_x = particles_df[f'x{particle_id}']
    #     particle_y = particles_df[f'y{particle_id}']
    #     particle_z = particles_df[f'z{particle_id}']
    #     agent_data = agents_df[agents_df['agent_id'] == agent_id].iloc[0]
    #     output_data[f'x{particle_id}'] = particle_x
    #     output_data[f'y{particle_id}'] = particle_y
    #     output_data[f'z{particle_id}'] = particle_z
    
    # output_df = pd.DataFrame(output_data)
    
    # Save to CSV
    output_file_path = file_name2.replace(".csv", "_s3.csv")
    remapped_df.to_csv(output_file_path, index=False)
    
    print(f"Output saved to {output_file_path}")
else:
    print("Not all particles are matched.")

print("Threshold:", threshold)
