import csv
import numpy as np
import os

def apply_speed_reduction(waypoints, start_time, end_time, reduction_amount=0.05, percentage_indi=False, check_curr_speed_only=False):
    if( (len(waypoints) == 2 and start_time > 0.00) or check_curr_speed_only ):
        start_time = 0.00 # Ensure we do not encounter UnboundLocalError when the particle amount is 2

    modified_waypoints = []
    flag = 0
    last_time_step = 0.00
    updated_t1 = 0.00
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        t1, x1, y1, z1 = p1
        t2, x2, y2, z2 = p2
        
        if t1 >= start_time and t2 <= end_time:
            # Calculate distance and time
            distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            time_interval = t2 - t1
            current_speed = distance / time_interval
            if(not check_curr_speed_only):
                print("time interval now:", time_interval)
                if(time_interval < 0.009):
                    if(flag == 1):
                        modified_waypoints.append([updated_t1, x1, y1, z1])
                        last_time_step = updated_t1
                        flag = 0
                    else:
                        if(i != 0):
                            if(len(waypoints) == 3):
                                modified_waypoints.append([last_time_step + new_time_interval, x1, y1, z1])
                                last_time_step += new_time_interval
                            else:
                                modified_waypoints.append([last_time_step + (t2 - t1), x1, y1, z1])
                                last_time_step += (t2 - t1)
                        else:
                            modified_waypoints.append([0.00, x1, y1, z1])
                            last_time_step = 0.00
                    
                    continue
                # Adjust speed
                if(percentage_indi):
                    new_speed = max(0, current_speed - (current_speed * reduction_amount))
                else:
                    new_speed = max(0, current_speed - reduction_amount)

                if(new_speed <= 0):
                    new_speed = 0.01
                
                # Adjust time based on new speed
                new_time_interval = distance / new_speed if new_speed > 0 else time_interval
                if(updated_t1 == 0.00):
                    updated_t1 = new_time_interval
                    # + new_time_interval
                else:
                    updated_t1 += new_time_interval

                # if(i != 0):
                flag = 1

                print("For waypoints t=", t1, "Current Speed", current_speed, "New Speed", new_speed, "updated_t1=", updated_t1)
                print("New time needed", new_time_interval, ", Prev time needed", time_interval)
            else:
                print("For waypoints at t=", t1, "Current Speed", current_speed)

        if(t1 == start_time and not check_curr_speed_only):
            modified_waypoints.append([start_time, x1, y1, z1])
            last_time_step = start_time
            flag = 0
            continue

        if(flag == 1 and not check_curr_speed_only):
            modified_waypoints.append([updated_t1, x1, y1, z1])
            print("flag 1", [updated_t1, x1, y1, z1])
            last_time_step = updated_t1
            flag = 0
        else:
            if(not check_curr_speed_only):
                if(i != 0):
                    modified_waypoints.append([last_time_step + (t2 - t1), x1, y1, z1])
                    last_time_step += (t2 - t1)
                else:
                    modified_waypoints.append([0.00, x1, y1, z1])
                    last_time_step = 0.00
    
    if(not check_curr_speed_only):
        if(len(waypoints) == 2):
            # t1, _, _, _ = modified_waypoints[-1]
            # t2, x2, y2, z2 = modified_waypoints[0]
            # print(modified_waypoints)
            _, x1, y1, z1 = waypoints[-1]
            modified_waypoints.append([new_time_interval, x1, y1, z1])
        else:
            t1, _, _, _ = modified_waypoints[-1]
            t2, x2, y2, z2 = modified_waypoints[-2]
            _, x1, y1, z1 = waypoints[-1]
            modified_waypoints.append([last_time_step + (t1 - t2), x1, y1, z1])
        
        print(modified_waypoints)
    
    if(not check_curr_speed_only):
        return modified_waypoints
    else:
        return None

def parse_csv_input(filename):
    agents = []
    with open(filename, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        data = list(reader)
        
        index = 0
        while index < len(data):
            if data[index][0].startswith("Agent ID"):
                agent_id = int(data[index][1])
                index += 1
                num_waypoints = int(data[index][1])
                index += 1
                waypoints = []
                for _ in range(num_waypoints):
                    point_data = data[index]
                    t = float(point_data[1])
                    x = float(point_data[2])
                    y = float(point_data[3])
                    z = float(point_data[4])
                    waypoints.append([t, x, y, z])
                    index += 1
                agents.append({'id': agent_id, 'waypoints': waypoints})
            else:
                index += 1
    
    return agents

def write_csv_output(agents, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for agent in agents:
            writer.writerow(["Agent ID", agent['id']])
            writer.writerow(["Number of waypoints", len(agent['waypoints'])])
            for i, point in enumerate(agent['waypoints']):
                writer.writerow([f"Point-{i}"] + [f"{x:.6f}" for x in point])

def main(input_filename, output_filename, agent_id, start_time, end_time, speed_reduction, percentage_indi, check_curr_speed_only):
    # Parse the input CSV file
    agents = parse_csv_input(input_filename)
    
    # Find the specified agent
    for agent in agents:
        if agent['id'] == agent_id:
            # Apply speed reduction to the specified agent
            modified_waypoints = apply_speed_reduction(agent['waypoints'], start_time, end_time, speed_reduction, percentage_indi, check_curr_speed_only)
            agent['waypoints'] = modified_waypoints
            break
    
    # Write the modified waypoints to the new CSV file
    if(not check_curr_speed_only):
        write_csv_output(agents, output_filename)
        print(f"Updated CSV written to {output_filename}")

if __name__ == "__main__":
    # Set the parameters for the specific task
    # input_filename = r"C:\Users\weicheng\Desktop\formal_dataset\240702\4\crossing\gs-pat\1.00\1\pathFile.csv"
    input_filename = r"C:\Users\weicheng\Desktop\OpenMPD_Demo1\weicheng_results\test.csv"
    output_filename = r"C:\Users\weicheng\Desktop\formal_dataset\240702\4\crossing\gs-pat\1.00\1\pathFile_s1_s2.csv"
    
    start_from_agent = 0
    last_until_agent = 3

    percentage_indi = True
    check_curr_speed_only = True

    for i in range(start_from_agent, last_until_agent+1):
        agent_id = i  # Example agent ID
        start_time = 0.01  # Start time for speed reduction
        end_time = 10   # End time for speed reduction
        speed_reduction = -0.15 # When percentage_indi on, this represents reduction percentage (0.5 = 50% reduction)
        # Run the main function with the specified parameters
        print("For Agent #", agent_id)
        main(input_filename, output_filename, agent_id, start_time, end_time, speed_reduction, percentage_indi, check_curr_speed_only)
