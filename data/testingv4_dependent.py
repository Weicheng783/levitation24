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
            # print(f"Reading data for Agent ID: {current_agent_id}")
        elif parts[0].startswith('Point'):
            time = round(float(parts[1]), 2)  # Round to nearest 0.01
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            # print(f"  Time: {time}, Position: ({x}, {y}, {z})")
            agents_data.append({
                'agent_id': current_agent_id,
                'time': time,
                'x': x,
                'y': y,
                'z': z
            })
    
    return pd.DataFrame(agents_data)

# Main function to perform the particle matching
def match_particles_to_agents(agent_csv_path, particle_csv_path, compare_time_zero_only=False, threshold_hi=0.02, threshold_lo=0.0015, threshold_step=0.001, particle_v2_mode=False, num_particles=4):
    # Load the CSV files
    agents_df = parse_agents_csv(agent_csv_path)
    particles_df = pd.read_csv(particle_csv_path)

    # Round the time values in particles_df to the nearest 0.01
    particles_df['time'] = particles_df['time'].round(2)

    # Threshold value
    threshold = threshold_lo

    # Create a dictionary to store which particle matches which agent
    matches = {}

    while(True):
        # Determine the time steps to process
        time_steps = [0] if compare_time_zero_only else agents_df['time'].unique()
        # print(f"Time steps to process: {time_steps}")

        # Loop through each time step
        for time in time_steps:
            # print(f"Processing time step: {time}")
            # Get agents' positions at the current time
            agents_at_time = agents_df[agents_df['time'] == time]
            # print(f"Agents at time {time}:")
            # print(agents_at_time)
            
            # Get particles' positions at the current time
            particles_at_time = particles_df[particles_df['time'] == time]
            if particles_at_time.empty:
                # print(f"No particles found at time {time}, skipping.")
                continue  # Skip if there are no particles at this time step

            particle_columns = [col for col in particles_df.columns if col.startswith('x')]
            # print(f"Particle columns: {particle_columns}")

            # Compare each agent with each particle
            for _, agent_row in agents_at_time.iterrows():
                agent_id = agent_row['agent_id']
                agent_pos = (agent_row['x'], agent_row['y'], agent_row['z'])
                # print(f"Comparing Agent {agent_id} at position {agent_pos}")

                for i in range(len(particle_columns)):
                    particle_pos = (
                        particles_at_time.iloc[0][f'x{i}'], 
                        particles_at_time.iloc[0][f'y{i}'], 
                        particles_at_time.iloc[0][f'z{i}']
                    )
                    # print(f"  Particle {i} at position {particle_pos}")

                    if is_within_threshold(agent_pos, particle_pos, threshold):
                        matches[i] = agent_id
                        # print(f"  Match found: Particle {i} matches Agent {agent_id}")
                        break  # Move to the next agent once a match is found
        
        # Check if all particles are matched
        if len(matches) == len(particle_columns):
            break  # Exit loop if all particles are matched
        
        if threshold >= threshold_hi:
            # if(particle_v2_mode):
            print("Maximum threshold reached, no match found.")
            print(matches)
            break
        else:
            threshold += threshold_step

    # Output the matches
    if len(matches) == len(particle_columns):
        print("Particle to Agent Matches:")

        # agent_map = {}

        # for particle_id, agent_id in matches.items():
        #     print(f"Particle {particle_id} matches Agent {agent_id}")
        #     agent_map[agent_id] = particle_id
        
        # # Create a new DataFrame for the output CSV
        # output_data = {'time': particles_df['time']}
        # release_count = 0

        # best_mapping = []

        # for agent_id in range(len(matches)):
        #     best_mapping.append(agent_map[agent_id])

        agent_map = {}
        repeated_matches = {}  # Track repeated matches

        # Populate the agent_map, track repeated matches
        for particle_id, agent_id in matches.items():
            print(f"Particle {particle_id} matches Agent {agent_id}")
            if agent_id not in agent_map:
                agent_map[agent_id] = particle_id
            else:
                # If a match is repeated, store the repeated particle_id in repeated_matches
                if agent_id not in repeated_matches:
                    repeated_matches[agent_id] = []
                repeated_matches[agent_id].append(particle_id)

        # Create a new DataFrame for the output CSV
        output_data = {'time': particles_df['time']}
        release_count = 0

        best_mapping = []
        unmatched_agents = []  # List to track unmatched agent IDs

        # First pass: Attempt to map all agents
        for agent_id in range(len(matches)):
            if agent_id in agent_map:
                best_mapping.append(agent_map[agent_id])
            else:
                unmatched_agents.append(agent_id)  # Track the unmatched agent

        # Second pass: Try to resolve unmatched agents with repeated matches
        for agent_id in unmatched_agents:
            resolved = False
            # Try to find a repeated match that hasn't been used yet
            for matched_agent_id, particles in repeated_matches.items():
                if particles:
                    best_mapping.append(particles.pop(0))  # Assign the first available repeated particle
                    resolved = True
                    break
            if not resolved:
                best_mapping.append(None)  # No match found, append None or a default value


        # Remap columns in particles_df based on best_mapping
        remapped_columns = ['time']  # Start with 'time' column
        for idx in best_mapping:
            remapped_columns.extend([f'x{int(idx)}', f'y{int(idx)}', f'z{int(idx)}'])

        remapped_df = particles_df[remapped_columns]  # Create a new DataFrame with remapped columns

        # Save to CSV
        output_file_path = particle_csv_path.replace(".csv", "_s3.csv")
        remapped_df.to_csv(output_file_path, index=False)
        
        print(f"Output saved to {output_file_path}")
        print("Threshold:", threshold)

        return output_file_path
    else:
        print("Not all particles are matched.")
        print(matches)
        print("Threshold:", threshold)
        return None

# If this script is run as a standalone file
if __name__ == "__main__":
    # Example usage
    agent_csv_path = r"C:\Users\weicheng\Desktop\formal_dataset\240702\3\random\gs-pat\0.08\1\pathFile.csv"
    particle_csv_path = r"C:\Users\weicheng\Desktop\formal_dataset\p3_r1\3a-240702-gspat-random-0.08v-1_pre_process_s1_s2.csv"
    match_particles_to_agents(agent_csv_path, particle_csv_path)
