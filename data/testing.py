import pandas as pd
from scipy.spatial.distance import euclidean
from fastdtw import dtw

# Function to compare two CSV files
def compare_csv_files(file1, file2):
    # Read CSV files into DataFrames
    df1 = pd.read_csv(file1)
    df2 = pd.read_csv(file2)
    vote_list = []
    votes = []
    
    # Print headers to inspect
    print("Header of file 1:", df1.columns.tolist())
    print("Header of file 2:", df2.columns.tolist())
    print()
    
    # Determine number of xyz batches (assuming 'time' is the first column)
    num_particles = (len(df1.columns) - 1) // 3

    smallest_length = min(len(df1), len(df2))  # Consider the smaller length of rows

    # Add your comparison or processing logic here
    for i in range(smallest_length):
        temp_vote = []
        not_processing_this_row = False

        # Dictionary to keep track of which target particles have been assigned
        assigned_targets = {}

        for j in range(num_particles):
            cmp_result = []
            for k in range(num_particles):
                try:
                    # dtw_distance, _ = dtw(df1[[f'x{j}', f'y{j}', f'z{j}']].iloc[i].values,
                                        #   df2[[f'x{k}', f'y{k}', f'z{k}']].iloc[i].values)
                    distance = euclidean(df1[[f'x{j}', f'y{j}', f'z{j}']].iloc[i].tolist(), df2[[f'x{k}', f'y{k}', f'z{k}']].iloc[i].tolist())
                    cmp_result.append(distance)
                    # cmp_result.append(dtw_distance)
                except ValueError as e:
                    not_processing_this_row = True
                    break
            if not not_processing_this_row:
                # Find the index of the minimum distance
                min_index = cmp_result.index(min(cmp_result))
                
                # Check if the target particle is already assigned
                # if min_index in assigned_targets.values():
                    # dtw_distance, _ = dtw(df1[[f'x{j}', f'y{j}', f'z{j}']].iloc[i].values,
                    #                         df2[[f'x{k}', f'y{k}', f'z{k}']].iloc[i].values)
                    # if dtw_distance < distance:
                    #     distance = dtw_distance
                    #     min_index = k

                    # Implement fallback strategy here (e.g., use second vote if collision)
                    # For simplicity, you can choose the second highest vote as a fallback
                    # second_best_vote = find_second_best_vote(vote_list, temp_vote)
                    
                    # Check if second best vote exists and use it
                    # if second_best_vote is not None:
                    #     min_index = second_best_vote[temp_vote.index(min_index)]
                    # else:
                        # If no second best vote, use the next available target
                    # min_index = find_next_available_target(assigned_targets, num_particles)
                
                # assigned_targets[j] = min_index

                temp_vote.append(min_index)
                print("Row", i, "PF1:", j, "MAPPED TO:", cmp_result.index(min(cmp_result)), "in PF2.")

        if temp_vote in vote_list:
            votes[vote_list.index(temp_vote)] += 1
        else:
            vote_list.append(temp_vote)
            votes.append(1)
    
    # Determine the best mapping based on votes
    best_mapping = vote_list[votes.index(max(votes))]
    print("Result of Mapping:", best_mapping, ", Votes:", max(votes))
    print(vote_list)
    print(votes)

    # Remap columns in df1 based on best_mapping
    remapped_columns = ['time']  # Start with 'time' column
    for idx in best_mapping:
        remapped_columns.extend([f'x{idx}', f'y{idx}', f'z{idx}'])

    remapped_df = df1[remapped_columns]  # Create a new DataFrame with remapped columns

    # Write remapped_df to a new CSV file
    remapped_file = file1.replace('.csv', '_s3.csv')
    remapped_df.to_csv(remapped_file, index=False)
    print(f"Remapped file saved as '{remapped_file}'.")

def find_next_available_target(assigned_targets, num_particles):
    # Find the next available target index that is not already assigned
    for idx in range(num_particles):
        if idx not in assigned_targets.values():
            return idx
    # If all targets are assigned, handle this case as per your requirement
    return None

def find_second_best_vote(vote_list, current_vote):
    # Find the second highest vote result in vote_list
    # Exclude the current_vote from consideration
    vote_list_copy = vote_list[:]
    if current_vote in vote_list_copy:
        vote_list_copy.remove(current_vote)
    
    if vote_list_copy:
        return vote_list_copy[vote_list_copy.index(max(vote_list_copy))]
    else:
        return None

# Example usage
if __name__ == "__main__":
    file1 = r"C:\Users\weicheng\Desktop\formal_dataset\p4\4a-240702-gspat-random-0.08v-2_001_pre_process_s1_s2.csv"
    file2 = r"C:/Users/weicheng/Desktop/formal_dataset/240702/4/random/gs-pat/0.08/1/sim_output_TargetPosition.csv"
    
    compare_csv_files(file1, file2)
