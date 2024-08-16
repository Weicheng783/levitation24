import csv
import numpy as np
import os

def interpolate_waypoints(points, interval_time=0.01, max_waypoints=None):
    interpolated_points = []
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        t1, x1, y1, z1 = p1
        t2, x2, y2, z2 = p2
        
        if t1 == t2:
            # Skip interpolation if the timestamps are the same
            interpolated_points.append(p1)  # Append the first point as is
            continue
        
        # Calculate the number of steps between points
        time_interval = t2 - t1
        num_steps = int(np.round(time_interval / interval_time))
        
        # Interpolate points
        for step in range(num_steps + 1):
            t = t1 + step * interval_time
            if t > t2:
                t = t2
            alpha = (t - t1) / (t2 - t1)
            x = x1 + alpha * (x2 - x1)
            y = y1 + alpha * (y2 - y1)
            z = z1 + alpha * (z2 - z1)
            interpolated_points.append([t, x, y, z])
    
    # Remove duplicates
    interpolated_points = remove_duplicates(interpolated_points)
    
    # Cap the number of waypoints if a threshold is provided
    if max_waypoints is not None and len(interpolated_points) > max_waypoints:
        interpolated_points = interpolated_points[:max_waypoints]
    
    return interpolated_points

def remove_duplicates(points):
    seen = set()
    unique_points = []
    for point in points:
        t, x, y, z = point
        if (t, x, y, z) not in seen:
            unique_points.append(point)
            seen.add((t, x, y, z))
    return unique_points

def parse_csv_input(filename):
    agents = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        
        agent_id = None
        waypoints = []
        for row in reader:
            if len(row) < 1:  # Skip empty lines
                continue
            if row[0].startswith("Agent ID"):
                if agent_id is not None:  # Save the previous agent's data
                    agents.append({'id': agent_id, 'waypoints': waypoints})
                agent_id = int(row[1])
                waypoints = []
            elif row[0].startswith("Number of waypoints"):
                num_waypoints = int(row[1])
            elif row[0].startswith("Point-"):
                t = float(row[1])
                x = float(row[2])
                y = float(row[3])
                z = float(row[4])
                waypoints.append([t, x, y, z])
        if agent_id is not None:  # Save the last agent's data
            agents.append({'id': agent_id, 'waypoints': waypoints})
    
    return agents

def write_csv_output(agents, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for agent in agents:
            writer.writerow(["Agent ID", agent['id']])
            writer.writerow(["Number of waypoints", len(agent['waypoints'])])
            for i, point in enumerate(agent['waypoints']):
                writer.writerow([f"Point-{i}"] + [f"{x:.6f}" for x in point])

def main(input_filename, max_waypoints=None):
    # Parse the input CSV file
    agents = parse_csv_input(input_filename)
    
    # Process each agent's waypoints
    for agent in agents:
        original_waypoints = agent['waypoints']
        interpolated_waypoints = interpolate_waypoints(original_waypoints, max_waypoints=max_waypoints)
        agent['waypoints'] = interpolated_waypoints
    
    # Generate output filename
    base, ext = os.path.splitext(input_filename)
    output_filename = f"{base}_s1{ext}"
    
    # Write the interpolated waypoints to the new CSV file
    write_csv_output(agents, output_filename)
    print(f"Interpolated CSV written to {output_filename}")

if __name__ == "__main__":
    # Replace with the actual path to your input file
    input_file = r"C:\Users\weicheng\Desktop\formal_dataset\240702\4\crossing\gs-pat\1.00\1\pathFile.csv"
    max_waypoints = 10000  # Set the maximum number of waypoints you want to allow
    main(input_file, max_waypoints=max_waypoints)
