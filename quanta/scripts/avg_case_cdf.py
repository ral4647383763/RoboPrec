import numpy as np
import matplotlib.pyplot as plt
import re
import os

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

def plot_error_distribution(data, label='Error CDF', color='cornflowerblue'): # Changed name in label, removed num_bins
    """
    Plots the Cumulative Distribution Function (CDF) of the error data.
    The y-axis represents the cumulative probability.
    The x-axis will be on a log scale if positive data is available and spread enough.
    """
    if data.size == 0:
        print(f"No data to plot for label: {label}")
        return None, None # Return None if no data

    # --- CDF Calculation for all data (used for linear scale fallback) ---
    sorted_data_all = np.sort(data)
    y_cdf_all = np.arange(1, len(sorted_data_all) + 1) / len(sorted_data_all)
    # --- End CDF Calculation ---

    positive_data = data[data > 0]
    if positive_data.size == 0:
        print(f"No positive data to plot on a log scale for label: {label}. Plotting CDF on linear scale.")
        plt.step(sorted_data_all, y_cdf_all, where='post', linewidth=3, label=f'{label} (linear scale)', color=color)
        plt.xlabel("Absolute Error (linear scale)", fontsize=14)
        return sorted_data_all, y_cdf_all

    # If we have positive data, prepare its CDF for potential log scale plot
    sorted_positive_data = np.sort(positive_data)
    y_cdf_positive = np.arange(1, len(sorted_positive_data) + 1) / len(sorted_positive_data)

    min_val_pos = sorted_positive_data[0]
    max_val_pos = sorted_positive_data[-1]
    
    can_use_log_scale = False
    # Check if log scale is appropriate for positive data
    if min_val_pos > 0: # Ensure minimum positive value is indeed positive
        if not np.isclose(min_val_pos, max_val_pos): # Distinct min and max positive values
            min_log = np.log10(min_val_pos)
            max_log = np.log10(max_val_pos)
            if min_log < max_log:
                can_use_log_scale = True
        else: # Single unique positive value, or all positive values are very close
            can_use_log_scale = True # Log scale can still be applied

    if can_use_log_scale:
        plt.step(sorted_positive_data, y_cdf_positive, where='post', linewidth=3, label=label, color=color)
        plt.fill_between(sorted_positive_data, y_cdf_positive, step='post', alpha=0.2, color=color)
        
        plt.xscale('log')
        plt.xlabel("Absolute Error (log scale)", fontsize=14)
        return sorted_positive_data, y_cdf_positive
    else:
        # Fallback to linear scale using all data if log scale on positive data isn't suitable
        print(f"Log scale conditions not met for positive data in '{label}'. Plotting CDF of all data on linear scale.")
        plt.step(sorted_data_all, y_cdf_all, where='post', linewidth=3, label=f'{label} (linear scale)', color=color)
        plt.fill_between(sorted_data_all, y_cdf_all, step='post', alpha=0.2, color=color)
        
        plt.xlabel("Absolute Error (linear scale)", fontsize=14)
        return sorted_data_all, y_cdf_all

RNEA_DERIV_FLOAT_WORST_CASE = 9192.55641989646
RNEA_DERIV_FIXED_WORST_CASE = 359.57335074519267
RNEA_FLOAT_WORST_CASE = 0.010387476068994506
RNEA_FIXED_WORST_CASE = 0.000396022289046285
FK_FLOAT_WORST_CASE = 3.702252467017966e-05
FK_FIXED_WORST_CASE = 1.5106592147191372e-06

# --- Main execution ---
def main(float_txt, fixed_txt, algo_name):

    float_values = read_data_from_file(float_txt)
    fixed_values = read_data_from_file(fixed_txt)

    plt.style.use('seaborn-v0_8-whitegrid')
    plt.figure(figsize=(8, 3.5))

    x_vals1, cdf_vals1 = plot_error_distribution(float_values, label='float', color='#a6cee3')
    if cdf_vals1 is not None and float_values[float_values > 0].size > 0:
        worst_case_1 = None
        if algo_name == "Forward Kinematics":
            worst_case_1 = FK_FLOAT_WORST_CASE
        elif algo_name == "RNEA":
            worst_case_1 = RNEA_FLOAT_WORST_CASE
        elif algo_name == "RNEA Derivatives":
            worst_case_1 = RNEA_DERIV_FLOAT_WORST_CASE
        plt.axvline(worst_case_1, color='#a6cee3', linestyle='--', linewidth=2.5, label=f'float worst case: {worst_case_1:.1e}')

    x_vals2, cdf_vals2 = plot_error_distribution(fixed_values, label='fixed32', color='#fdbf6f')
    if cdf_vals2 is not None and fixed_values[fixed_values > 0].size > 0:
        worst_case_2 = None
        if algo_name == "Forward Kinematics":
            worst_case_2 = FK_FIXED_WORST_CASE
        elif algo_name == "RNEA":
            worst_case_2 = RNEA_FIXED_WORST_CASE
        elif algo_name == "RNEA Derivatives":
            worst_case_2 = RNEA_DERIV_FIXED_WORST_CASE
        plt.axvline(worst_case_2, color='#fdbf6f', linestyle='--', linewidth=2.5, label=f'fixed32 worst case: {worst_case_2:.1e}')


    # The key change for CDF y-axis label:
    plt.ylabel("Cumulative Probability", fontsize=15) # Changed from "Number of Occurrences"
    plt.title(f"Absolute Error CDF: {algo_name} (float vs fixed32)", fontsize=15) # Updated title
    plt.legend(loc='best', fontsize=14) # Changed loc to 'best' for potentially better placement with CDFs
    plt.grid(True, which="both", ls="-", alpha=0.7)

    plt.tick_params(axis='both', which='major', labelsize=14)

    plot_dir = "plots"
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)
    save_path = None
    #os.path.join(plot_dir, "error_cdf_rneaderiv.png") # Changed filename
    if algo_name == "Forward Kinematics":
        save_path = os.path.join(plot_dir, "cdf_fk.png")
    elif algo_name == "RNEA":
        save_path = os.path.join(plot_dir, "cdf_rnea.png")
    elif algo_name == "RNEA Derivatives":
        save_path = os.path.join(plot_dir, "cdf_rneaderiv.png")


    try:
        plt.savefig(save_path, dpi=600, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
        # plt.show() 
    except Exception as e:
        print(f"Error saving plot: {e}")


if __name__ == "__main__":
    main("fk_float.txt", "fk_fixed32.txt", "Forward Kinematics")
    main("rnea_float.txt", "rnea_fixed32.txt", "RNEA")
    main("rneaderiv_float.txt", "rneaderiv_fixed32.txt", "RNEA Derivatives")