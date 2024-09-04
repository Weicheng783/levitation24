import glob

def read_csv_custom(filepath):
    with open(filepath, 'r') as file:
        lines = file.readlines()
    return [line.strip().split(',') for line in lines]

def merge_csv_files(file_paths):
    merged_data = []
    
    for file_path in file_paths:
        data = read_csv_custom(file_path)
        
        agent_indices = [i for i, row in enumerate(data) if row[0].startswith('Agent ID')]
        
        for idx, start in enumerate(agent_indices):
            end = agent_indices[idx + 1] if idx + 1 < len(agent_indices) else len(data)
            agent_data = data[start:end]
            
            if len(merged_data) <= idx:
                # This is the first file for this agent, so add it as is.
                merged_data.append(agent_data)
            else:
                prev_agent_data = merged_data[idx]
                prev_last_point = prev_agent_data[-1]
                time_offset = float(prev_last_point[1])
                
                # Skip 'Agent ID' and 'Number of waypoints'
                waypoint_start_idx = 2
                new_points = []
                
                # Add new points, skipping the first point (initial position)
                for i, row in enumerate(agent_data[waypoint_start_idx:]):
                    if row[0].startswith('Point-'):
                        # Update the point number
                        new_point_number = len(prev_agent_data) - waypoint_start_idx + i
                        row[0] = f"Point-{new_point_number}"
                        # Adjust time
                        row[1] = str(float(row[1]) + time_offset)
                        new_points.append(row)
                
                # Update number of waypoints
                new_waypoints_count = len(prev_agent_data) - waypoint_start_idx + len(new_points)
                prev_agent_data[1][1] = str(new_waypoints_count)
                
                # Merge without repeating headers
                merged_data[idx] = prev_agent_data + new_points
    
    # Write merged data to a file
    with open('merged_output.csv', 'w') as file:
        for agent_data in merged_data:
            for row in agent_data:
                file.write(','.join(row) + '\n')

# Assuming you have multiple files named 'file1.csv', 'file2.csv', etc.
file_paths = [r"C:\Users\weicheng\Desktop\exp_videos\EXP4\pathFiles_combination\1.csv", r"C:\Users\weicheng\Desktop\exp_videos\EXP4\pathFiles_combination\2.csv", r"C:\Users\weicheng\Desktop\exp_videos\EXP4\pathFiles_combination\3.csv"]
merge_csv_files(file_paths)
