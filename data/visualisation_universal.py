import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import os

# Function to find the starting frame where particles begin to move
def detect_movement_start_frame(df, threshold=0.001):
    num_particles = (len(df.columns) - 1) // 3  # Calculate number of particles

    for frame in range(len(df)):
        # Attempt to reshape the data; handle potential ValueError
        try:
            positions = df.iloc[frame, 1:].values.reshape(num_particles, 3)
        except ValueError:
            continue  # Skip frames with incompatible data shape

        # Check if any particle's position indicates movement
        if np.any(np.linalg.norm(positions, axis=1) > threshold):
            return frame

    return 0  # Default to frame 0 if no movement detected

# Function to create the visualization for two CSV files side by side
def create_visualization(file_path1, file_path2, threshold):
    # Step 1: Read and Parse CSV Data
    df1 = pd.read_csv(file_path1)
    df2 = pd.read_csv(file_path2)

    # Determine the starting frame where particles begin to move for both datasets
    start_frame1 = detect_movement_start_frame(df1, threshold)
    start_frame2 = detect_movement_start_frame(df2, threshold)

    # Extract file names from paths
    title1 = os.path.basename(file_path1)
    title2 = os.path.basename(file_path2)

    # Step 2: Set Up 3D Plotting Environment
    fig = plt.figure(figsize=(14, 6))  # Adjusted figure size for two plots side by side

    # Plot for the first file
    ax1 = fig.add_subplot(121, projection='3d')  # Left subplot
    ax1.set_xlabel('X axis')
    ax1.set_ylabel('Y axis')
    ax1.set_zlabel('Z axis')
    ax1.set_title(f'Visualization 1: {title1}')

    # Plot for the second file
    ax2 = fig.add_subplot(122, projection='3d')  # Right subplot
    ax2.set_xlabel('X axis')
    ax2.set_ylabel('Y axis')
    ax2.set_zlabel('Z axis')
    ax2.set_title(f'Visualization 2: {title2}')

    # Set axis limits for both plots
    ax1.set_xlim(-0.2, 0.2)
    ax1.set_ylim(-0.2, 0.2)
    ax1.set_zlim(0, 0.25)

    ax2.set_xlim(-0.2, 0.2)
    ax2.set_ylim(-0.2, 0.2)
    ax2.set_zlim(0, 0.25)

    # Determine the number of particles for each dataset
    num_particles1 = (len(df1.columns) - 1) // 3
    num_particles2 = (len(df2.columns) - 1) // 3

    # Define colors for each particle
    colors1 = plt.cm.jet(np.linspace(0, 1, num_particles1))
    # colors2 = plt.cm.jet(np.linspace(0, 1, num_particles2))
    colors2 = colors1

    # Initialize scatter plots for each particle with explicit colors for both plots
    particles1 = [ax1.scatter([], [], [], c=[colors1[i]], label=f'Particle {i+1}') for i in range(num_particles1)]
    ax1.legend()

    particles2 = [ax2.scatter([], [], [], c=[colors2[i]], label=f'Particle {i+1}') for i in range(num_particles2)]
    ax2.legend()

    # Step 3: Update Function for Animation for both plots
    def update(frame):
        # Update for the first plot
        for i in range(num_particles1):
            if (i * 3 + 3) < len(df1.columns):
                x = df1.iloc[frame, i * 3 + 1]
                y = df1.iloc[frame, i * 3 + 2]
                z = df1.iloc[frame, i * 3 + 3]
                particles1[i]._offsets3d = ([x], [y], [z])
            else:
                particles1[i]._offsets3d = ([], [], [])  # Handle missing data

        # Update for the second plot, adjusting for starting frame alignment
        adjusted_frame2 = frame + (start_frame1 - start_frame2)
        for i in range(num_particles2):
            if (i * 3 + 3) < len(df2.columns) and adjusted_frame2 < len(df2):
                x = df2.iloc[adjusted_frame2, i * 3 + 1]
                y = df2.iloc[adjusted_frame2, i * 3 + 2]
                z = df2.iloc[adjusted_frame2, i * 3 + 3]
                particles2[i]._offsets3d = ([x], [y], [z])
            else:
                particles2[i]._offsets3d = ([], [], [])  # Handle missing data

    # Step 4: Set up a single universal slider
    ax_slider = plt.axes([0.3, 0.02, 0.4, 0.04], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Frame', start_frame1, max(len(df1), len(df2)) - 1, valinit=start_frame1, valstep=1)

    # Step 5: Slider update function
    def on_slider_update(val):
        frame = int(slider.val)
        update(frame)
        fig.canvas.draw_idle()

    slider.on_changed(on_slider_update)

    # Initialize the first frame for both plots
    update(start_frame1)

    # Adjust subplot parameters manually to avoid the warning
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    plt.show()

# Main function call for visualization
def main(actual_file, reference_file):
    file_path1 = actual_file
    file_path2 = reference_file
    threshold = 0.001
    create_visualization(file_path1, file_path2, threshold)
