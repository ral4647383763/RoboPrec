import numpy as np
import matplotlib.pyplot as plt
import re
import os # Added for creating directory

def read_data_from_file(filepath):
    """
    Reads numerical data from a file.
    The file is expected to contain space-separated numbers,
    potentially in scientific notation.
    """
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            # Remove any non-breaking space characters and then split
            cleaned_line = line.replace('\u00A0', ' ').strip()
            if cleaned_line: # ensure the line is not empty after cleaning
                # Use regex to find all numbers (including scientific notation)
                numbers_in_line = re.findall(r'[+-]?\d*\.?\d+(?:[eE][+-]?\d+)?', cleaned_line)
                try:
                    data.extend([float(num) for num in numbers_in_line])
                except ValueError as e:
                    print(f"Skipping line due to error: {e} in line: '{line.strip()}'")
    return np.array(data)

def plot_error_distribution(data, num_bins=50, label='Error Distribution', color='cornflowerblue'):
    """
    Plots the distribution of the error data.
    The y-axis represents the actual number of occurrences in each bin.
    The x-axis will be on a log scale.
    """
    if data.size == 0:
        print(f"No data to plot for label: {label}")
        return None, None # Return None if no data

    positive_data = data[data > 0]
    if positive_data.size == 0:
        print(f"No positive data to plot on a log scale for label: {label}.")
        counts, bin_edges = np.histogram(data, bins=num_bins)
        plt.step(bin_edges[:-1], counts, where='post', linewidth=4, label=f'{label} (linear scale)', color=color)
        plt.xlabel("Absolute Error (linear scale)", fontsize=14) # Added fontsize
        return bin_edges, counts

    min_val = positive_data.min()
    max_val = positive_data.max()

    if np.isclose(min_val, max_val) or min_val == 0:
        if min_val > 0 :
            min_log = np.log10(min_val * 0.9) if min_val * 0.9 > 0 else np.log10(min_val) -1
            max_log = np.log10(max_val * 1.1) if max_val * 1.1 > 0 else np.log10(max_val) +1
            if min_log >= max_log:
                min_log = np.log10(min_val) - 0.5
                max_log = np.log10(min_val) + 0.5
        else:
             counts, bin_edges = np.histogram(data, bins=num_bins)
             plt.step(bin_edges[:-1], counts, where='post', linewidth=4, label=f'{label} (linear scale - fallback)', color=color)
             plt.xlabel("Absolute Error (linear scale - fallback)", fontsize=14) # Added fontsize
             return bin_edges, counts
    else:
        min_log = np.log10(min_val)
        max_log = np.log10(max_val)

    if min_log < max_log :
         log_bins = np.logspace(min_log, max_log, num_bins + 1)
         counts, bin_edges = np.histogram(positive_data, bins=log_bins)
         plt.step(bin_edges[:-1], counts, where='post', linewidth=3, label=label, color=color)
         plt.xscale('log')
         plt.xlabel("Absolute Error (log scale)", fontsize=14)
    else:
        counts, bin_edges = np.histogram(data, bins=num_bins)
        plt.step(bin_edges[:-1], counts, where='post', linewidth=3, label=f'{label} (linear scale)', color=color)
        plt.xlabel("Absolute Error (linear scale)", fontsize=14)

    return bin_edges, counts


# --- Main execution ---

# Using the file names from your script
float_path_1 = 'float.txt' # Changed to avoid conflict if you run original
fixed_path_2 = 'fixed32.txt' # As in your script

# Create dummy files for the script to run if they don't exist
# This is for testing purposes if the actual files are not present
if not os.path.exists(float_path_1):
    with open(float_path_1, 'w') as f:
        f.write("1e-5 2e-5 1.5e-5\n")
        f.write("3e-5 3.7e-5\n")

if not os.path.exists(fixed_path_2):
    with open(fixed_path_2, 'w') as f:
        f.write("1e-6 2e-6 1.5e-6\n")
        f.write("0.5e-6 1.2e-6\n")

float_values_1 = read_data_from_file(float_path_1)
fixed_values_2 = read_data_from_file(fixed_path_2)


plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(12, 7))

# Plotting for data_values_1 is commented out as in your script
if float_values_1 is not None and float_values_1.size > 0: # Check if None before checking size
    bins1, counts1 = plot_error_distribution(float_values_1, num_bins=30, label='Float32 (float)', color='#a6cee3')
    if counts1 is not None and float_values_1[float_values_1 > 0].size > 0:
        # worst_case_1 = float_values_1[float_values_1 > 0].max() # Original dynamic calculation
        worst_case_1 = 3.702252467017966e-05
        plt.axvline(worst_case_1, color='#a6cee3', linestyle='--', linewidth=2.5, label=f'worst_case (float): {worst_case_1:.2e}')

if fixed_values_2 is not None and fixed_values_2.size > 0:
    bins2, counts2 = plot_error_distribution(fixed_values_2, num_bins=30, label='Fixed32 (fixed32)', color='#fdbf6f')
    if counts2 is not None and fixed_values_2[fixed_values_2 > 0].size > 0: # Check if counts2 is not None
        # worst_case_2 = fixed_values_2[fixed_values_2 > 0].max() # Original dynamic calculation
        worst_case_2 = 1.5106592147191372e-06 # Hardcoded as in your provided script
        plt.axvline(worst_case_2, color='#fdbf6f', linestyle='--', linewidth=2.5, label=f'worst_case (fixed): {worst_case_2:.2e}')


# The key change is here:
plt.ylabel("Number of Occurrences", fontsize=15) # Changed from "Number of Occurrences (Frequency)"
plt.title("Absolute Error: Forward Kinematics (Float32 vs Fixed32)", fontsize=15)
plt.legend(loc='upper left', fontsize=14)
plt.grid(True, which="both", ls="-", alpha=0.7)

# change font size of ticks
plt.tick_params(axis='both', which='major', labelsize=14)


# Ensure 'plots' directory exists before saving
plot_dir = "plots"
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)
save_path = os.path.join(plot_dir, "error_distribution_fk.png")

try:
    plt.savefig(save_path, dpi=600, bbox_inches='tight')
    print(f"Plot saved to {save_path}")
    # If running in an environment that displays plots, uncomment next line
    # plt.show()
except Exception as e:
    print(f"Error saving plot: {e}")