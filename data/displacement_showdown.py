import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
from prompt_toolkit import prompt
from prompt_toolkit.completion import WordCompleter
from prompt_toolkit.shortcuts import checkboxlist_dialog
import re
import argparse

# Set up argument parser
parser = argparse.ArgumentParser(description='Select which distance graph to show.')
parser.add_argument('--directory', type=str, default=r"C:\Users\weicheng\Desktop\formal_dataset\p6_x_r1_diff_amp_240802",
                    help='Directory containing the CSV files.')

# Parse the arguments
args = parser.parse_args()
directory = args.directory

# Find all CSV files with the suffix 'df2.csv'
csv_files = glob.glob(os.path.join(directory, "*df2.csv"))

# Extract relevant parts of filenames for shortened legends
def extract_legend(filename):
    match = re.search(r'(\d{4,5})_(\d{3})', filename)
    if match:
        return f"{match.group(1)}_{int(match.group(2))}"
    else:
        match = re.search(r'(\d{4,5})', filename)
        if match:
            return match.group(1)
        else:
            return filename

# Read the CSV files and store them in a dictionary
data_dict = {}
short_names = {}
for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    short_name = extract_legend(os.path.basename(csv_file))
    data_dict[short_name] = df
    short_names[short_name] = os.path.basename(csv_file)

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
    ax.set_xlabel('Time Steps')
    ax.set_ylabel(column_to_plot)
    ax.set_title(f'Particle #{column_to_plot} Displacement Graph')
    ax.legend(loc='upper right')
    plt.show()

# Plot the selected graphs
for graph_index in graphs_to_show:
    plot_graph(f'distance_{graph_index}', selected_files)
