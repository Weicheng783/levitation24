import os
import pandas as pd
import matplotlib.pyplot as plt
from prompt_toolkit import prompt
from prompt_toolkit.completion import WordCompleter
from prompt_toolkit.shortcuts import checkboxlist_dialog
import re
import argparse

# Set up argument parser
parser = argparse.ArgumentParser(description='Select which distance graph to show.')
parser.add_argument('--directory', type=str, default=r"C:\Users\weicheng\Desktop\exp_videos\EXP3_p6",
                    help='Top directory to search for the CSV files.')

# Parse the arguments
args = parser.parse_args()
directory = args.directory

# Function to extract the number from the directory path
def extract_number_from_path(path):
    match = re.search(r'_(\d{3,9})', path)
    return match.group(1) if match else None

# Function to clean filenames and directory names
def clean_name(name):
    # Keep only numbers, underscores, and alphanumeric characters
    return re.sub(r'[^0-9_]', '', name)

# Find all CSV files with the suffix 'df2.csv' in all subdirectories
csv_files = []
for root, dirs, files in os.walk(directory):
    for file in files:
        if file.endswith("df2.csv"):
            csv_files.append(os.path.join(root, file))

# Extract relevant parts of filenames for shortened legends
def extract_legend(filename, directory):
    match = re.search(r'(\d{1,3})_(\d{3,9})', filename)
    number_from_path = extract_number_from_path(directory)
    if match:
        return [f"{match.group(1)}_{int(match.group(2))}_{number_from_path}", number_from_path]
    else:
        match = re.search(r'(\d{1,3})_(\d{3,5})', filename)
        if match:
            return [f"velocity = {match.group(1)}_{number_from_path}", number_from_path]
        else:
            aaa = extract_and_transform_numbers(f"{clean_name(filename)}_{number_from_path}")
            print(aaa)
            # return [f"velocity = {clean_name(filename)}_{number_from_path}", number_from_path]
            return [f"Velocity: {aaa[1]}, Amplitude: {aaa[3]}", number_from_path]
            # return [f"Amplitude: {aaa[3]}", number_from_path]

def extract_and_transform_numbers(s):
    # Extract all sequences of digits from the string
    numbers = re.findall(r'\d+', s)
    
    # Process each number
    result = []
    for num in numbers:
        if len(num) > 1 and num[0] == '0':  # Check for leading zeros
            # Convert to float and scale down
            formatted_num = float(num) / 10**len(num.lstrip('0'))
            result.append(formatted_num)
        else:
            # Convert to integer
            result.append(int(num))
    
    return result

# Example usage
# s = "10_0355000____2_14000"
# transformed_numbers = extract_and_transform_numbers(s)
# print(transformed_numbers)

def format_velocity_amplitude(filename, number_from_path):
    # Assuming number_from_path is of the form "10_0355000__2_14000"
    parts = number_from_path.split("__")
    velocity_part = parts[0]
    amplitude_part = parts[1]

    # Process the velocity part
    # We expect the velocity part to be in the format "10_xxxxxx"
    velocity_value = float(velocity_part.split("_")[1]) / 10000000  # 10 million to get "0.355000"
    
    # Process the amplitude part
    # We expect the amplitude part to be in the format "2_xxxx"
    amplitude_value = amplitude_part.split("_")[1]  # Assuming the number after "_"

    return [f"Velocity = {velocity_value:.6f}, Amplitude = {amplitude_value}", number_from_path]
    # return [f"Amplitude = {amplitude_value}", number_from_path]


# Read the CSV files and store them in a dictionary
data_dict = {}
short_names = {}
for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    name_pool = extract_legend(os.path.basename(csv_file), os.path.dirname(csv_file))
    short_name = name_pool[0]
    # short_name = clean_name(short_name)
    data_dict[short_name] = df
    short_names[short_name] = clean_name(os.path.basename(csv_file)) + "_amp_" + name_pool[1]

# List all graph options to the user
print("Available distance columns to plot:")
for i in range(6):
    print(f"{i}: distance_{i}")

# Ask the user which graphs to plot
graph_completer = WordCompleter([str(i) for i in range(6)], ignore_case=True)
graphs_to_show = prompt("Enter the indices of distance graphs to show (e.g., 0 1 2): ", completer=graph_completer)
graphs_to_show = list(map(int, graphs_to_show.split()))

# Now provide checkboxes for user to select which CSV files to plot
checkbox_values = []
for short_name in short_names:
    checkbox_values.append((short_name, short_names[short_name]))

selected_files_dialog = checkboxlist_dialog(
    title="Select CSV files to show",
    text="Please select which CSV files to show:",
    values=checkbox_values,
).run()

selected_files = {short_name: (short_name in selected_files_dialog) for short_name in short_names}

# Function to plot the graph
def plot_graph(column_to_plot, selected_files):
    fig, ax = plt.subplots(figsize=(10, 6))
    for short_name, show_line in selected_files.items():
        if show_line:
            distance = data_dict[short_name][column_to_plot]
            x_values = []
            y_values = []
            for j in range(1, len(distance)):
                if distance[j] != distance[j-1]:
                    x_values.append(j)
                    y_values.append(distance[j])
            ax.plot(x_values, y_values, label=short_name)
    ax.set_xlabel('Time Steps (frame)')
    ax.set_ylabel('Deviation Amount (m)')
    ax.set_title(f'Exp.3: Particle #{column_to_plot.replace("distance_","")} Deviation with Time Steps (all amplitudes = 16000)')
    ax.legend(loc='upper right')
    plt.show()

# Plot the selected graphs
for graph_index in graphs_to_show:
    plot_graph(f'distance_{graph_index}', selected_files)
