import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import os
import argparse

# Function to find the starting frame where particles begin to move
def detect_movement_start_frame(df, threshold=0.001):
    num_particles = (len(df.columns) - 1) // 3  # Calculate number of particles

    for frame in range(len(df)):
        try:
            positions = df.iloc[frame, 1:].values.reshape(num_particles, 3)
        except ValueError:
            continue  # Skip frames with incompatible data shape

        if np.any(np.linalg.norm(positions, axis=1) > threshold):
            return frame

    return 0  # Default to frame 0 if no movement detected

# Function to create the visualization for a single CSV file
def create_visualization(file_path, threshold):
    # Step 1: Read and Parse CSV Data
    df = pd.read_csv(file_path)

    # Determine the starting frame where particles begin to move
    start_frame = detect_movement_start_frame(df, threshold)

    # Extract file name from path
    title = os.path.basename(file_path)

    # Step 2: Set Up 3D Plotting Environment
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title(f'Visualization: {title}')

    # Set axis limits
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0, 0.25)

    # Determine the number of particles
    num_particles = (len(df.columns) - 1) // 3

    # Define colors for each particle
    colors = plt.cm.jet(np.linspace(0, 1, num_particles))

    # Initialize scatter plots for each particle with explicit colors
    particles = [ax.scatter([], [], [], c=[colors[i]], label=f'Particle {i+1}') for i in range(num_particles)]
    ax.legend()

    # Step 3: Update Function for Animation
    def update(frame):
        for i in range(num_particles):
            if (i * 3 + 3) < len(df.columns):
                x = df.iloc[frame, i * 3 + 1]
                y = df.iloc[frame, i * 3 + 2]
                z = df.iloc[frame, i * 3 + 3]
                particles[i]._offsets3d = ([x], [y], [z])
            else:
                particles[i]._offsets3d = ([], [], [])  # Handle missing data

    # Step 4: Set up a slider
    ax_slider = plt.axes([0.2, 0.02, 0.65, 0.04], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Frame', start_frame, len(df) - 1, valinit=start_frame, valstep=1)

    # Step 5: Slider update function
    def on_slider_update(val):
        frame = int(slider.val)
        update(frame)
        fig.canvas.draw_idle()

    slider.on_changed(on_slider_update)

    # Initialize the first frame
    update(start_frame)

    # Adjust subplot parameters manually to avoid the warning
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.15)

    plt.show()

# Main function to handle command-line arguments
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize particle movement from a CSV file.")
    parser.add_argument('file_path', type=str, help='Path to the CSV file.')
    parser.add_argument('--threshold', type=float, default=0.001, help='Threshold for detecting particle movement.')
    
    args = parser.parse_args()
    create_visualization(args.file_path, args.threshold)