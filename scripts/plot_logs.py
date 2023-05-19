import os
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import filedialog
from tkinter import Tk
import numpy as np
import seaborn as sns

def select_folder():
    root = Tk()
    root.withdraw()  # Hides the small tkinter window
    folder_selected = filedialog.askdirectory()  # Open the file dialog to choose the folder
    root.update()  # To make the filedialog window close
    root.destroy()  # Destroy the root window
    return folder_selected

def plot_data(folder_selected):
    # Find all csv files in the selected folder
    files_in_folder = os.listdir(folder_selected)
    csv_files = [f for f in files_in_folder if f.endswith(".csv")]

    occluded_files = [f for f in csv_files if "occluded" in f]

    fig, axs = plt.subplots(1, 1, figsize=(10, 6))  # Create a figure with one subplot

    # Set color palette
    colors = sns.color_palette("cool", len(occluded_files))

    # Plot all occluded data
    occluded_zero_counts = []
    durations = []
    for idx, file in enumerate(occluded_files):
        df = pd.read_csv(os.path.join(folder_selected, file))
        df['timestamp'] -= df['timestamp'].min()  # Adjust timestamps to start from 0
        
        # Sort the DataFrame by timestamp
        df = df.sort_values(by='timestamp')

        # Calculate the difference in time between successive samples where 'occluded' is zero
        df['time_diff'] = df.loc[df['occluded'] == 0, 'timestamp'].diff()

        # Calculate the total time duration for which 'occluded' is zero
        occluded_zero_time = df['time_diff'].sum()
        occluded_zero_counts.append(occluded_zero_time)
        
        # Store the durations for histogram
        durations.extend(df['time_diff'].dropna())

    # Create the bar chart for zero counts in occluded data
    bars = axs.bar(range(len(occluded_zero_counts)), occluded_zero_counts, color='skyblue')
    avg_zero_counts = np.mean(occluded_zero_counts)
    avg_bar = axs.bar(len(occluded_zero_counts), avg_zero_counts, color='red', alpha=0)  # Remove the red average bar
    axs.set_title('Duration of Occlusion for Line-of-Sight Obstacle in 15 Second Intervals (Ground Truth)')
    axs.set_xlabel('Interation Number')
    axs.set_ylabel('Duration [s]')
    axs.set_xlim(-1, 51)

    # Add a dashed red horizontal line for the average
    axs.axhline(avg_zero_counts, color='red', linestyle='dashed', label=f'Average: {avg_zero_counts:.2f} seconds')
    
    # Add a legend that only displays the red line as the average and its numeric value
    axs.legend()

    # Set the style
    plt.style.use('seaborn-darkgrid')
    plt.rcParams['axes.facecolor'] = 'lightgrey'
    plt.rcParams['figure.facecolor'] = 'white'

    # Adjust spacing
    plt.tight_layout()

    # Show the plot
    plt.show()


if __name__ == '__main__':
    folder = select_folder()
    plot_data(folder)
