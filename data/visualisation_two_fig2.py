import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

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

# Function to create and save the first visualization
def create_visualization1(file_path1, threshold, colors):
    # Step 1: Read and Parse CSV Data
    df1 = pd.read_csv(file_path1)

    # Determine the starting frame where particles begin to move
    start_frame1 = detect_movement_start_frame(df1, threshold)

    # Step 2: Set Up 3D Plotting Environment for the first file
    fig1 = plt.figure(figsize=(7, 6))  # Adjusted figure size for one plot

    ax1 = fig1.add_subplot(111, projection='3d')  # Single subplot
    ax1.set_xlabel('X axis')
    ax1.set_ylabel('Y axis')
    ax1.set_zlabel('Z axis')

    # Set axis limits for the plot
    ax1.set_xlim(-0.2, 0.2)
    ax1.set_ylim(-0.2, 0.2)
    ax1.set_zlim(0, 0.25)

    # Determine the number of particles for the dataset
    num_particles1 = (len(df1.columns) - 1) // 3

    # Initialize scatter plots for each particle with explicit colors
    particles1 = [ax1.scatter([], [], [], c=[colors[i]], label=f'Particle {i+1}') for i in range(num_particles1)]
    ax1.legend()

    # Print the colors used for visualization 1
    print(f"Colors used in visualization 1:")
    for i, color in enumerate(colors):
        rgb_color = tuple(int(255 * c) for c in color[:3])  # Convert normalized color to 0-255 range
        print(f"x: Particle {i+1} - RGB color: {rgb_color}")

    # Step 3: Update Function for Animation
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

    # Step 4: Set up a slider for the first plot
    ax_slider1 = plt.axes([0.3, 0.02, 0.4, 0.04], facecolor='lightgoldenrodyellow')
    slider1 = Slider(ax_slider1, 'Frame', start_frame1, len(df1) - 1, valinit=start_frame1, valstep=1)

    # Step 5: Slider update function
    def on_slider_update(val):
        frame = int(slider1.val)
        update(frame)
        fig1.canvas.draw_idle()

    slider1.on_changed(on_slider_update)

    # Initialize the first frame for the plot
    update(start_frame1)

    # Adjust subplot parameters manually to avoid the warning
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    # Save the figure
    plt.savefig('fig1.png')
    plt.show()

# Function to create and save the second visualization
def create_visualization2(file_path2, threshold, colors):
    # Step 1: Read and Parse CSV Data
    df2 = pd.read_csv(file_path2)

    # Determine the starting frame where particles begin to move
    start_frame2 = detect_movement_start_frame(df2, threshold)

    # Step 2: Set Up 3D Plotting Environment for the second file
    fig2 = plt.figure(figsize=(7, 6))  # Adjusted figure size for one plot

    ax2 = fig2.add_subplot(111, projection='3d')  # Single subplot
    ax2.set_xlabel('X axis')
    ax2.set_ylabel('Y axis')
    ax2.set_zlabel('Z axis')

    # Set axis limits for the plot
    ax2.set_xlim(-0.2, 0.2)
    ax2.set_ylim(-0.2, 0.2)
    ax2.set_zlim(0, 0.25)

    # Determine the number of particles for the dataset
    num_particles2 = (len(df2.columns) - 1) // 3

    # Initialize scatter plots for each particle with explicit colors
    particles2 = [ax2.scatter([], [], [], c=[colors[i]], label=f'Particle {i+1}') for i in range(num_particles2)]
    ax2.legend()

    # Print the colors used for visualization 2
    print(f"Colors used in visualization 2:")
    for i, color in enumerate(colors):
        rgb_color = tuple(int(255 * c) for c in color[:3])  # Convert normalized color to 0-255 range
        print(f"x: Particle {i+1} - RGB color: {rgb_color}")

    # Step 3: Update Function for Animation
    def update(frame):
        # Update for the second plot
        for i in range(num_particles2):
            if (i * 3 + 3) < len(df2.columns):
                x = df2.iloc[frame, i * 3 + 1]
                y = df2.iloc[frame, i * 3 + 2]
                z = df2.iloc[frame, i * 3 + 3]
                particles2[i]._offsets3d = ([x], [y], [z])
            else:
                particles2[i]._offsets3d = ([], [], [])  # Handle missing data

    # Step 4: Set up a slider for the second plot
    ax_slider2 = plt.axes([0.3, 0.02, 0.4, 0.04], facecolor='lightgoldenrodyellow')
    slider2 = Slider(ax_slider2, 'Frame', start_frame2, len(df2) - 1, valinit=start_frame2, valstep=1)

    # Step 5: Slider update function
    def on_slider_update(val):
        frame = int(slider2.val)
        update(frame)
        fig2.canvas.draw_idle()

    slider2.on_changed(on_slider_update)

    # Initialize the first frame for the plot
    update(start_frame2)

    # Adjust subplot parameters manually to avoid the warning
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    # Save the figure
    plt.savefig('fig2.png')
    plt.show()

# Main function call for visualization
if __name__ == "__main__":
    # Define file paths and threshold

    file_path1 = r"C:\Users\weicheng\Desktop\formal_dataset\p4\4a-240702-gspat-random-0.02v-2_pre_process_s1_s2_s3.csv"
    file_path2 = "C:/Users/weicheng/Desktop/formal_dataset/240702/4/random/gs-pat/0.02/1/sim_output_TargetPosition.csv"
    
    # file_path1 = r"C:\Users\weicheng\Desktop\actual_pos\GS_PAT_random-forward-v-0.02-s-1-p6_round_1_001_pre_process_s1.csv"
    # file_path2 = r"C:\Users\weicheng\Desktop\AcousticPathPlanning\AcousticPathPlanning\weicheng\6\GS_PAT_random-forward-v-0.02-s-1-round_1_TargetPosition.csv"
    threshold = 0.001

    # Generate colors for particles (assuming the same number of particles in both datasets)
    num_particles = 4  # Replace with actual number of particles
    colors = plt.cm.jet(np.linspace(0, 1, num_particles))

    # Create visualizations
    create_visualization1(file_path1, threshold, colors)
    create_visualization2(file_path2, threshold, colors)
