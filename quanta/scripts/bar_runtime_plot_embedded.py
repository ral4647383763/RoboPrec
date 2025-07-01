import matplotlib.pyplot as plt
import numpy as np
import pandas as pd # Need pandas for isna check and nanmean
from matplotlib.patches import Patch
import math # Import math for calculating limits
from matplotlib.font_manager import FontProperties # For bold font
import warnings # To potentially suppress warnings if needed later

FONT_SIZE = 18
# 2 but conversion cost is removed

# Define pastel colors for numeric types
dtype_colors = {
    'double': '#a6cee3',  # pastel blue
    'float': '#fdbf6f',   # pastel orange
    'fixed32': '#b2df8a', # pastel green
    'fixed32-uniform': '#cab2d6', # pastel purple
}

# Algorithms remain, but they now define the subgroups.
algorithms = ['Forward kinematics', 'RNEA', 'RNEA derivatives']
# Abbreviated names for table rows
algo_short_names = {'Forward kinematics': 'FK', 'RNEA': 'RNEA', 'RNEA derivatives': 'RNEA Deriv'}


# This will include the runtimes without conversion costs
groups_excluding_conversion = [
    {
      "label": "Raspberry Pi Pico 2 ARM",
      "num_types": ["double", "float", "fixed32", "fixed32-uniform"],
      "data": {
           'Forward kinematics': [143.46, 12.88, 17.75, 17.46], # 11.13x
           'RNEA': [248.5, 18.15, 31.74, 32.14], # 13.69x
           'RNEA derivatives': [1349.34, 127.69, 218.35, np.nan] # 10.56x
      }
    },
    {
      "label": "Raspberry Pi Pico",
      "num_types": ["double", "float", "fixed32", "fixed32-uniform"],
      "data": {
           'Forward kinematics': [785.027, 423.103, 73.3, 72.395], # 10.84x
           'RNEA': [1368.86, 721.77, 169.78, 160.046], # 8.55x
           'RNEA derivatives': [7445.674, 3925.721, 955.50, np.nan] # 7.79x
      }
    },
    {
      "label": "Raspberry Pi Pico 2 RISC-V",
      "num_types": ["double", "float", "fixed32", "fixed32-uniform"],
      "data": {
           'Forward kinematics': [657.029, 151.399, 21.023, 19.878], # 33.03x
           'RNEA': [1295.217, 336.52, 38.18, 41.315], # 33.92x
           'RNEA derivatives': [34396.136, 1896.107, 279.81, np.nan] # 122.92x
      }
    },
]

# Data for each device group (total costs)
groups = [
    {
      "label": "Raspberry Pi Pico 2 ARM",
      "num_types": ["double", "float", "fixed32", "fixed32-uniform"],
      "data": {
           'Forward kinematics': [143.46, 12.88, 17.75, 17.46],
           'RNEA': [248.5, 18.15, 31.74, 32.14],
           'RNEA derivatives': [1349.34, 127.69, 218.35, np.nan]
      }
    },
    {
      "label": "Raspberry Pi Pico",
      "num_types": ["double", "float", "fixed32", "fixed32-uniform"],
      "data": {
           'Forward kinematics': [785.027, 423.103, 73.3, 72.395],
           'RNEA': [1368.86, 721.77, 169.78, 160.046],
           'RNEA derivatives': [7445.674, 3925.721, 955.50, np.nan]
      }
    },
    {
      "label": "Raspberry Pi Pico 2 RISC-V",
      "num_types": ["double", "float", "fixed32", "fixed32-uniform"],
      "data": {
           'Forward kinematics': [657.029, 151.399, 21.023, 19.878],
           'RNEA': [1295.217, 336.52, 38.18, 41.315],
           'RNEA derivatives': [34396.136, 1896.107, 279.81, np.nan]
      }
    },
]



# Parameters for bar positioning
bar_width = 0.2
cluster_spacing = 0.1
group_spacing = 0.8
n_groups = len(groups)

fig, ax = plt.subplots(figsize=(20, 10))
current_x = 0
group_centers_data = [] # Store group center info in data coordinates
algo_cluster_info = []
# all_values is not explicitly used beyond the y-limit calculation which is self-contained below

# --- Pre-calculate y-limit related values based on 'groups' (total costs) ---
print("--- Calculating Y-axis limits and NaN marker position ---")
all_numeric_values_for_ylim = []
for group_data_ylim in groups:
    group_num_types_list_ylim = group_data_ylim["num_types"]
    for algo_name_ylim in algorithms:
        if algo_name_ylim in group_data_ylim["data"]:
            algo_values_ylim = group_data_ylim["data"][algo_name_ylim]
            if algo_values_ylim is not None:
                for val_idx_ylim, value_ylim in enumerate(algo_values_ylim):
                    if val_idx_ylim < len(group_num_types_list_ylim):
                        if not pd.isna(value_ylim): # Collect all non-NaN numbers
                            all_numeric_values_for_ylim.append(value_ylim)

positive_values_for_log = [v for v in all_numeric_values_for_ylim if v > 0]

if not positive_values_for_log:
    min_positive_val = 0.01 # Default if no positive data
    print("Warning: No positive data for bottom_limit calculation. Using default min_positive_val.")
else:
    min_positive_val = min(positive_values_for_log)

# User's formula for bottom_limit, ensuring min_positive_val is used correctly for log10
if min_positive_val <= 0: # Should be caught by previous block, but as a safeguard
    print(f"Warning: min_positive_val ({min_positive_val}) is not positive. Adjusting to 0.01 for log scale.")
    min_positive_val = 0.01
    
#bottom_limit = 10**(math.floor(math.log10(min_positive_val))) - 1
bottom_limit = 5
x_marker_pos = 6
top_limit = 34453 

print(f"  Bottom limit: {bottom_limit:.3e}, x_marker_pos: {x_marker_pos:.3e}, Top limit: {top_limit}")
ax.set_ylim(bottom_limit, top_limit)


# --- Define Table Styling Parameters ---
TABLE_FONT_SIZE = 7
TABLE_HEADER_COLOR = '#EAEAF2'
TABLE_ROW_COLOR_ODD = '#FFFFFF'
TABLE_ROW_COLOR_EVEN = '#F2F2F2'
TABLE_EDGE_COLOR = 'darkgrey'
TABLE_LINEWIDTH = 0.76
TABLE_WIDTH_FACTOR = 0.77
TABLE_Y_BOTTOM_AX = 0.77
TABLE_HEIGHT_AX = 0.13

# --- First pass: Plot bars, calculate data positions ---
print("--- Pass 1: Plotting Bars and Calculating Data Coordinates ---")
for group_idx, group_total_data in enumerate(groups):
    group_exclusive_data = groups_excluding_conversion[group_idx] # Corresponding exclusive data

    group_start_data = current_x
    group_num_types = group_total_data["num_types"]
    n_types = len(group_num_types)
    group_cluster_centers_data = []

    for i, algo in enumerate(algorithms):
        # Calculate cluster width based on n_types for the current group_total_data
        cluster_width_data = n_types * bar_width
        
        if algo in group_total_data["data"]:
            algo_data_total = group_total_data["data"][algo]
            algo_data_exclusive = group_exclusive_data["data"].get(algo) # Use .get for safety

            n_types_in_algo_total = len(algo_data_total) if algo_data_total is not None else 0
            n_types_in_algo_exclusive = len(algo_data_exclusive) if algo_data_exclusive is not None else 0
            
            cluster_start_data = current_x
            cluster_center_data = cluster_start_data + cluster_width_data / 2.0
            group_cluster_centers_data.append(cluster_center_data)
            algo_cluster_info.append((cluster_center_data, algo))

            for j in range(n_types): # Iterate over all defined numeric types for the group
                # x-coordinate for the bar, consistent with original script's centering
                bar_center_x = (cluster_start_data + (j + 0.5) * bar_width) - bar_width/2
                current_dtype = group_num_types[j]
                
                value_total = np.nan
                if algo_data_total is not None and j < n_types_in_algo_total:
                    value_total = algo_data_total[j]

                value_exclusive = np.nan
                if algo_data_exclusive is not None and j < n_types_in_algo_exclusive:
                    value_exclusive = algo_data_exclusive[j]

                if not pd.isna(value_total): # If total value is available
                    if pd.isna(value_exclusive):
                        # If exclusive is NaN but total is not, assume exclusive part is 0
                        # print(f"Info: Exclusive value is NaN for {group_total_data['label']}/{algo}/{current_dtype} while total is {value_total:.2f}. Assuming exclusive part is 0.")
                        value_exclusive = 0.0
                    
                    # Ensure value_exclusive is not greater than value_total and non-negative
                    value_exclusive = max(0, min(value_exclusive, value_total))

                    conversion_cost = value_total - value_exclusive
                    if conversion_cost < 0: # Should be rare after clamping value_exclusive
                        conversion_cost = 0.0
                    
                    base_color = dtype_colors.get(current_dtype, 'grey')

                    # Plot base (value_exclusive)
                    if value_exclusive > 1e-9: # Plot only if height is significant
                        ax.bar(bar_center_x, value_exclusive, width=bar_width, color=base_color, label='_nolegend_')

                    # Plot conversion_cost on top
                    if conversion_cost > 1e-9: # Plot only if height is significant
                        ax.bar(bar_center_x, conversion_cost, width=bar_width, bottom=value_exclusive,
                               color=base_color, hatch='///', edgecolor='darkgrey', alpha=0.7, label='_nolegend_')
                
                else: # value_total is NaN, plot 'x' marker
                   ax.plot(bar_center_x, x_marker_pos, marker='x', color=dtype_colors.get(current_dtype, 'grey'), markersize=FONT_SIZE, linestyle='None')
            
            current_x += cluster_width_data + cluster_spacing
        else: # Algorithm not present in this group's total_data (should advance current_x)
            # This case assumes if algo is not in total_data, it's also not in exclusive_data for plotting
            # We still need to advance current_x to maintain spacing for other groups/algos
            # User's original code: estimated_cluster_width_data = n_types * bar_width; current_x += estimated_cluster_width_data + cluster_spacing
            # This part correctly advances current_x if an algorithm is entirely missing for a group.
            # The x_marker_pos logic is tied to `value_total` being NaN from existing data.
            # If an entire algo is missing, the inner loops for `j` won't run for it.
            estimated_cluster_width_data = n_types * bar_width
            current_x += estimated_cluster_width_data + cluster_spacing
            # Optionally, one could plot 'x's for these fully missing algo slots if desired,
            # but current logic follows if data is present or explicitly NaN.

    group_end_data = current_x - cluster_spacing if current_x > group_start_data else current_x # Avoid negative if no clusters
    group_center_data = np.mean(group_cluster_centers_data) if group_cluster_centers_data else group_start_data + (group_end_data - group_start_data)/2
    group_centers_data.append({ "label": group_total_data["label"], "start_data": group_start_data, "end_data": group_end_data, "center_data": group_center_data, "num_types": group_num_types, "data": group_total_data["data"] })
    current_x += group_spacing


# --- Setup Axes ---
ax.set_xlim(-0.5, current_x - group_spacing if current_x > 0 else 1.0) # Handle if current_x remains 0
ax.set_xticks([])
ax.set_ylabel("Runtime (µs)", fontsize=FONT_SIZE)
ax.tick_params(axis='y', labelsize=FONT_SIZE)
ax.set_title("Runtimes Across Different Platforms (Grouped by Device and Algorithm)", pad=50, fontsize=16)
ax.set_yscale('log')


# --- Force a draw to update transforms ---
print("\n--- Forcing Figure Draw ---"); fig.canvas.draw(); print("--- Figure Draw Complete ---\n")


# Using annotation position from user's last code
annotation_y_pos_ax = 0.98 

ylim_bottom, ylim_top = ax.get_ylim()
# Calculation unchanged from user's last code, but ensure ylim_bottom is positive for log10
if ylim_bottom <= 0 or ylim_top <= 0 or ylim_bottom >= ylim_top: 
    print(f"Warning: Invalid y-limits ({ylim_bottom:.2e}, {ylim_top:.2e}) for log scale annotation calculations. Using a default relative position.")
    annotation_y_pos_data = ylim_top * 0.9 if ylim_top > 0 else 1 # Default placement if limits are problematic
else: 
    log_range = math.log10(ylim_top) - math.log10(ylim_bottom)
    annotation_y_pos_data = 10**(math.log10(ylim_bottom) + log_range * annotation_y_pos_ax)


# --- Add Vertical Lines ---
print("\n--- Adding Vertical Lines ---")
for group_idx in range(n_groups - 1):
    # Ensure group_centers_data has enough elements before accessing
    if group_idx < len(group_centers_data) -1 :
        group_end_data = group_centers_data[group_idx]["end_data"]
        next_group_start_data = group_centers_data[group_idx+1]["start_data"]
        # Ensure start and end data are sensible before calculating line position
        if group_end_data < next_group_start_data :
            line_pos_data = (group_end_data + next_group_start_data)/2.0
            print(f"  Adding line between group {group_idx} and {group_idx+1} at data x={line_pos_data:.2f}")
            ax.axvline(line_pos_data, color='grey', linestyle='--', linewidth=1, ymin=0.0, ymax=1.0) # ymax=1.0 to span full height if desired

# --- Add algorithm subgroup labels (below x-axis) ---
print("\n--- Adding Algorithm Labels ---")
algo_label_y_pos = -0.05
# To prevent overlapping labels if algo_cluster_info has very close x_pos_data values
# (e.g. if some clusters have no width due to no data types)
# We can use a set to track plotted positions or a more sophisticated overlap check.
# For simplicity, let's ensure algo_cluster_info doesn't contain centers of zero-width clusters implicitly.
# The current algo_cluster_info logic seems okay as it's based on actual cluster centers.
plotted_algo_labels_x = []
min_algo_label_spacing = bar_width * 2 # Heuristic: don't plot labels closer than this

for x_pos_data, algo in algo_cluster_info:
    is_too_close = any(abs(x_pos_data - plotted_x) < min_algo_label_spacing for plotted_x in plotted_algo_labels_x)
    x_pos_data = x_pos_data - 0.1
    if not is_too_close:
        short_name = algo_short_names.get(algo, algo)
        ax.text(x_pos_data, algo_label_y_pos, short_name, ha='center', va='top', transform=ax.get_xaxis_transform(), fontsize=FONT_SIZE, color='black', rotation=0)
        plotted_algo_labels_x.append(x_pos_data)


# --- Add device group labels (below algorithm labels) ---
print("\n--- Adding Device Labels ---")
device_label_y_pos = -0.22
for group_info in group_centers_data:
    group_center_x_data = group_info["center_data"]
    label = group_info["label"]
    ax.text(group_center_x_data, device_label_y_pos, label, ha='center', va='top', transform=ax.get_xaxis_transform(), fontsize=FONT_SIZE, fontweight='bold', color='black')


final_legend_item_labels = [
    'double', 'float', 'fixed32', 'fixed32-uniform', 'fixed16'
]
legend_handles = []
conversion_cost_patch_object = Patch(facecolor='lightgrey', hatch='///', edgecolor='darkgrey', alpha=0.7, label='Conversion Cost')
for item_label in final_legend_item_labels:
    if item_label == 'Conversion Cost':
        legend_handles.append(conversion_cost_patch_object)
    elif item_label in dtype_colors:
        legend_handles.append(Patch(color=dtype_colors[item_label], label=item_label))

# Place legend above the subplots
fig.legend(handles=legend_handles, loc='upper center', bbox_to_anchor=(0.5, 0.95), ncol=len(legend_handles), borderaxespad=0., fontsize=FONT_SIZE)


# --- Apply Layout ---
print("\n--- Applying Layout ---")
# Adjusted rect to give more space for bottom labels and top legend
try: 
    plt.tight_layout(rect=[0, 0.0, 1, 1.]) 
except ValueError as e: 
    print(f"Warning: tight_layout failed: {e}. Using plt.subplots_adjust as fallback.")
    plt.subplots_adjust(left=0.05, right=0.98, bottom=0.20, top=0.88)

# Improve aesthetics
ax.yaxis.grid(True, linestyle='--', alpha=0.7) # Add horizontal grid lines
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)


# --- Save and Show ---
print("\n--- Saving and Showing Plot ---")
# Ensure plots directory exists
import os
if not os.path.exists("plots"):
    os.makedirs("plots")
plt.savefig("plots/runtime_embedded.png", dpi=600)
plt.show()
print("--- Done ---")

