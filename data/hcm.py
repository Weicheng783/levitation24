import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Load the data from CSV
file_path = r"C:\Users\weicheng\Desktop\formal_dataset\p2_r1\2a-240702-gspat-random-0.01v-1_001_pre_process_df1.csv"  # Replace with your actual file path
data = pd.read_csv(file_path)

# Drop the 'time' column
data = data.drop(columns=['time'])

# Compute the correlation matrix
correlation_matrix = data.corr()

# Set up the matplotlib figure
plt.figure(figsize=(10, 8))

# Draw the heatmap
sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', center=0)

# Set the title
plt.title('Heatmap of Variable Correlations')

# Show the plot
plt.show()
