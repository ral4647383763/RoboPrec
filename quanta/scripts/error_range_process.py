#!/usr/bin/env python3
import os
import re
import numpy as np
import matplotlib.pyplot as plt

FONT_SIZE = 15

# Regex to capture numbers in both plain and scientific notation.
NUMBER_REGEX = r'([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)'

def parse_range_file(range_path):
    """
    Parse a range.txt file where each line is:
      {name}: [{min}, {max}]
    Returns a dictionary mapping variable name -> (min, max).
    """
    ranges = {}
    pattern = r'([^:]+):\s*\[\s*' + NUMBER_REGEX + r'\s*,\s*' + NUMBER_REGEX + r'\s*\]'
    with open(range_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = re.match(pattern, line)
            if m:
                var_name = m.group(1).strip()
                min_val = float(m.group(2))
                max_val = float(m.group(3))
                ranges[var_name] = (min_val, max_val)
            else:
                print(f"Warning: could not parse line in {range_path}: {line}")
    return ranges

def parse_error_file(error_path):
    """
    Parse an error.txt file where each line is:
      {name}: {error}
    Returns a dictionary mapping variable name -> error (as float).
    """
    errors = {}
    pattern = r'([^:]+):\s*' + NUMBER_REGEX
    with open(error_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = re.match(pattern, line)
            if m:
                var_name = m.group(1).strip()
                error_val = float(m.group(2))
                errors[var_name] = error_val
            else:
                print(f"Warning: could not parse line in {error_path}: {line}")
    return errors

def process_all_data(base_dir):
    """
    Traverse the data folder with structure:
      base_dir/range&error/{algorithm_name}/{robot_name}/{range_classification}/
    In each range_classification folder:
      - Parse range.txt to compute the worst range span (max - min across all variables).
      - For each data type subfolder (named, e.g., "double", "float", etc.),
        parse error.txt to determine the worst error.
    Returns a list of dictionaries, one per experiment, with keys:
      - "label": a string label like "algorithm/robot"
      - "range_span": the worst range span (max - min)
      - "by_num_type": a dict mapping number type -> {"worst_error": ...}
    """
    all_data = []
    # base_dir should be something like "data/range&error"
    # Iterate over algorithm directories.
    for algorithm_name in os.listdir(base_dir):
        algorithm_path = os.path.join(base_dir, algorithm_name)
        if not os.path.isdir(algorithm_path):
            continue
        # Iterate over robot directories.
        for robot_name in os.listdir(algorithm_path):
            robot_path = os.path.join(algorithm_path, robot_name)
            if not os.path.isdir(robot_path):
                continue
            
            current_experiment_path = robot_path 
            effective_data_path = current_experiment_path 

            range_txt_path = os.path.join(current_experiment_path, "range.txt")
            if not os.path.exists(range_txt_path):
                found_range_txt_in_subdir = False
                if os.path.isdir(current_experiment_path):
                    for range_class_candidate in os.listdir(current_experiment_path):
                        if "large" in range_class_candidate and "large_range" not in range_class_candidate :
                             continue
                        
                        path_in_subdir = os.path.join(current_experiment_path, range_class_candidate)
                        if os.path.isdir(path_in_subdir): 
                            candidate_range_file = os.path.join(path_in_subdir, "range.txt")
                            if os.path.exists(candidate_range_file):
                                range_txt_path = candidate_range_file
                                effective_data_path = path_in_subdir 
                                found_range_txt_in_subdir = True
                                break 
                if not found_range_txt_in_subdir:
                    print(f"Warning: range.txt not found directly in {current_experiment_path} or in expected subdirectories for {algorithm_name}/{robot_name}.")
            
            if not os.path.exists(range_txt_path): 
                 print(f"Final check: range.txt still not found for {algorithm_name}/{robot_name}. Path: {range_txt_path}. Skipping entry.")
                 continue

            ranges = parse_range_file(range_txt_path)
            worst_range_span = 0.0
            if ranges: 
                for var, (mn, mx) in ranges.items():
                    span = mx - mn
                    if span > worst_range_span:
                        worst_range_span = span
            else: 
                worst_range_span = np.nan 

            by_num_type = {}
            if os.path.isdir(effective_data_path):
                for data_type in os.listdir(effective_data_path):
                    data_type_path = os.path.join(effective_data_path, data_type)
                    if not os.path.isdir(data_type_path):
                        continue
                    error_txt_path = os.path.join(data_type_path, "error.txt")
                    if not os.path.exists(error_txt_path):
                        continue
                    errors = parse_error_file(error_txt_path)
                    worst_error = max(errors.values()) if errors else np.nan
                    by_num_type[data_type] = {"worst_error": worst_error}
            else:
                 print(f"Warning: Data types path {effective_data_path} is not a directory for {algorithm_name}/{robot_name}")

            label = f"{algorithm_name}/{robot_name}"
            entry = {
                "label": label,
                "range_span": worst_range_span,
                "by_num_type": by_num_type
            }
            if by_num_type: 
                all_data.append(entry)
            else:
                print(f"Warning: No numerical type data (empty by_num_type) for {label} in {effective_data_path}")
    return all_data

# process_algorithm and process_subdivision functions would be here if used by main.
# For this request, assuming main uses process_all_data.

def plot_error_data_multilevel(all_data):
    """
    Plots a grouped bar chart for worst error values by number type,
    grouped by Robot, then by Algorithm, with increased spacing between Robot groups.
    Multilevel x-tick labels show Robot (top) and Algorithm (bottom).
    Visible bracket lines are drawn for each Robot group.
    Dashed vertical lines separate Robot groups.
    """
    import numpy as np
    import matplotlib.pyplot as plt

    if not all_data:
        print("No data provided to plot_error_data_multilevel.")
        return

    desired_order = ["double", "float", "fixed32", "fixed32-uniform"]
    pastel_colors = {
         "double": "#a6cee3",
         "float":  "#fdbf6f",
         "fixed32":  "#b2df8a",
         "fixed32-uniform": "#cab2d6"
    }
    
    # Original labels from data entries ("algorithm/robot")
    original_labels = [entry["label"] for entry in all_data]
    
    # Apply renaming for display
    modified_display_labels = [label.replace("4dof", "RoArm-m2\n4DoF")
                                    .replace("5dof", "RoArm-m3\n5DoF")
                                    .replace("6dof", "Indy7\n6DoF")
                                    .replace("7dof", "Franka Panda\n7DoF")
                                    .replace("forward_kinematics", "FK")
                                    .replace("rnea_derivatives", "RNEA Deriv")
                                    .replace("rnea", "RNEA")
                                for label in original_labels]
    
    # Grouped labels for logic: (algorithm_display_name, robot_display_name)
    label_grouped_for_logic = [tuple(lbl.split("/")) for lbl in modified_display_labels]
    
    # --- Generate x-coordinates with increased gap between robot groups ---
    x_coords_list = []
    current_x_pos_val = 0.0
    # Adjust this to increase/decrease space between robot groups.
    # 0.0 = no extra space; 1.0 = one algorithm slot extra space.
    gap_increment_between_robot_groups = 1.0 
    
    final_max_x_for_figsize = 0.0

    if not label_grouped_for_logic:
        x = np.array([])
    else:
        # First algorithm position
        x_coords_list.append(current_x_pos_val)
        
        for i in range(1, len(label_grouped_for_logic)):
            # Current algorithm's robot name is at index 1 of the tuple
            current_robot_name = label_grouped_for_logic[i][1]
            previous_robot_name = label_grouped_for_logic[i-1][1]
            
            current_x_pos_val += 1.0 # Standard increment for the next algorithm
            
            if current_robot_name != previous_robot_name:
                # New robot group detected, add the extra gap
                current_x_pos_val += gap_increment_between_robot_groups
            
            x_coords_list.append(current_x_pos_val)
        x = np.array(x_coords_list)
        if x.size > 0:
            final_max_x_for_figsize = x[-1]

    # --- End of x-coordinate generation ---

    # Algorithm names for bottom tick labels (grp[0] is algorithm display name)
    bottom_tick_labels_text = [grp[0] for grp in label_grouped_for_logic] 

    # Width of individual bars for number types
    bar_width_per_type = 0.8 / len(desired_order) 
    
    # Adjust figsize based on the new x-coordinate range
    # Use final_max_x_for_figsize to estimate width needed
    # Multiply by a factor (e.g., 0.8-1.0) for visual density plus margin
    width_scale_factor = 0.9 
    figure_width = max(12, (final_max_x_for_figsize + 1) * width_scale_factor + 2) # +2 for margins
    fig, ax = plt.subplots(figsize=(figure_width, 7))
    ax.set_yscale("log")
    
    # Plot bars for each number type
    for i, num_type in enumerate(desired_order):
        errors = []
        for entry in all_data: # Iterate through original all_data to fetch errors
            if num_type in entry["by_num_type"]:
                errors.append(entry["by_num_type"][num_type]["worst_error"])
            else:
                errors.append(np.nan)
        
        print("errors: ", errors)
        
        # Calculate offset for this number type's bars
        offset = (i - len(desired_order) / 2) * bar_width_per_type + bar_width_per_type / 2
        ax.bar(x + offset, errors, bar_width_per_type, label=num_type, color=pastel_colors.get(num_type, "#cccccc"))

        # Plot 'x' markers for NaN values
        y_pos_for_nan_marker = 5e-15 
        ####current_ymin, _ = ax.get_ylim()
        ####if y_pos_for_nan_marker < current_ymin and current_ymin > 0 : 
        ####    y_pos_for_nan_marker = current_ymin * 1.1 
        ####elif current_ymin <=0 and y_pos_for_nan_marker < 1e-15: # Handle cases where ymin is 0 or negative before log
        ####     y_pos_for_nan_marker = 1e-15


        for k_idx, error_value in enumerate(errors):
            if np.isnan(error_value):
                nan_marker_x_coord = x[k_idx] + offset
                ax.plot(nan_marker_x_coord, y_pos_for_nan_marker,
                        marker='x',
                        color=pastel_colors.get(num_type, "#333333"),
                        markersize=10,
                        linestyle='None',
                        clip_on=False)

    ax.set_ylim(1e-15, 1e7) # Set y-limits for log scale

    # Set primary (bottom) tick labels as algorithm names
    ax.set_xticks(x) # Use the new x-coordinates for tick positions
    formatted_bottom_labels = [bl.replace(" ", "\n") for bl in bottom_tick_labels_text]
    #ax.set_xticklabels(formatted_bottom_labels, rotation=0, ha="right", rotation_mode='anchor', fontsize=FONT_SIZE)
    ax.set_xticklabels(formatted_bottom_labels, rotation=0, fontsize=FONT_SIZE)
    
    # Helper function to compute boundaries for robot groups
    def compute_boundaries(group_index_in_tuple, grouped_labels_list):
        boundaries = []
        if not grouped_labels_list:
            return boundaries
        start_item_idx = 0
        current_group_identifier = grouped_labels_list[0][group_index_in_tuple]
        for i_grp, grp_tuple_item in enumerate(grouped_labels_list):
            if grp_tuple_item[group_index_in_tuple] != current_group_identifier:
                boundaries.append((start_item_idx, i_grp - 1, current_group_identifier))
                start_item_idx = i_grp
                current_group_identifier = grp_tuple_item[group_index_in_tuple]
        boundaries.append((start_item_idx, len(grouped_labels_list) - 1, current_group_identifier))
        return boundaries
    
    # Robot name is at index 1 of the (algorithm, robot) tuple in label_grouped_for_logic
    robot_group_boundaries = compute_boundaries(1, label_grouped_for_logic) 
    
    # Y-positions for robot group labels and brackets (offsets from x-axis)
    group_label_y_offset = -0.20 
    group_bracket_y_offset = -0.15 
    delta_bracket_height = 0.05
    
    # Draw robot group brackets and labels
    for start_idx_in_x_array, end_idx_in_x_array, robot_name_text in robot_group_boundaries:
        # Use x-coordinates corresponding to these indices
        left_bracket_edge = x[start_idx_in_x_array] - 0.5 
        right_bracket_edge = x[end_idx_in_x_array] + 0.5

        ax.plot([left_bracket_edge, right_bracket_edge],
                [group_bracket_y_offset, group_bracket_y_offset],
                transform=ax.get_xaxis_transform(),
                color="black", clip_on=False, linewidth=1.0)
        ax.plot([left_bracket_edge, left_bracket_edge],
                [group_bracket_y_offset, group_bracket_y_offset + delta_bracket_height],
                transform=ax.get_xaxis_transform(),
                color="black", clip_on=False, linewidth=1.0)
        ax.plot([right_bracket_edge, right_bracket_edge],
                [group_bracket_y_offset, group_bracket_y_offset + delta_bracket_height],
                transform=ax.get_xaxis_transform(),
                color="black", clip_on=False, linewidth=1.0)
        
        # Position for the robot group text label
        text_label_x_pos = (x[start_idx_in_x_array] + x[end_idx_in_x_array]) / 2.0
        ax.text(text_label_x_pos, group_label_y_offset, robot_name_text, ha="center", va="top",
                transform=ax.get_xaxis_transform(), fontweight="bold", fontsize=FONT_SIZE, clip_on=False)
    
    # General plot settings
    ax.set_xlabel("Robot / Algorithm", labelpad=90, fontsize=FONT_SIZE) 
    ax.tick_params(axis='y', labelsize=FONT_SIZE)
    ax.set_ylabel("Worst Error", fontsize=FONT_SIZE)
    ax.set_title("Worst Error by Number Type (Grouped by Robot, then Algorithm)", fontsize=FONT_SIZE, pad=30)
    ax.legend(fontsize=FONT_SIZE, title_fontsize=12, loc='upper center', ncol=4, bbox_to_anchor=(0.5, 1.10))

    ax.yaxis.grid(True, linestyle='--', alpha=0.7, zorder=0) 

    # Add dashed lines between robot groups (now using new x-coordinates)
    ymin_plot_area, ymax_plot_area = ax.get_ylim() 
    num_robot_groups = len(robot_group_boundaries)
    if num_robot_groups > 1 and x.size > 1: # Ensure there are groups and x-coordinates
        for i in range(num_robot_groups - 1): 
            # Index of the last algorithm of the current robot group in the x array
            current_robot_last_alg_idx = robot_group_boundaries[i][1] 
            
            # Ensure we don't go out of bounds for x when accessing the next algorithm's start
            if current_robot_last_alg_idx + 1 < len(x):
                # Dashed line in the middle of the gap between this group and the next
                separator_x_val = (x[current_robot_last_alg_idx] + x[current_robot_last_alg_idx + 1]) / 2.0
                
                ax.vlines(separator_x_val, ymin_plot_area, ymax_plot_area,
                          colors='grey',        
                          linestyles='dashed',  
                          linewidth=1.0,        
                          alpha=0.7,
                          zorder=0.5) 

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 1.0]) 
    # Dynamically adjust bottom margin based on label offsets
    bottom_margin_value = 0.1 # Default minimum
    if robot_group_boundaries: # If there are group labels
        bottom_margin_value = max(0.25, abs(group_label_y_offset) + bottom_margin_value)
    fig.subplots_adjust(bottom=bottom_margin_value)

    # mkdir plots if it doesn't exist
    plots_dir = "plots"
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)
        print(f"Created directory: {plots_dir}")

    plt.savefig("plots/error_plot.png", dpi=600)
    print("Saved plot to plots/error_plot.png")

# plot_range_data_multilevel and main functions would be here.
# Ensure main calls process_all_data and then this plot_error_data_multilevel.

def main():
    # Make sure base_dir points to the correct location of your "data/range&error"
    # Example: base_dir = "data/range&error" or os.path.join("path", "to", "data", "range&error")
    # For testing, ensure this path is correct relative to where you run the script.
    # If "data" is in the same directory as the script:
    base_dir = os.path.join(os.path.dirname(__file__) if '__file__' in locals() else '.', "data", "range&error")
    
    all_data = process_all_data(base_dir)
    
    if not all_data:
        print("No data processed. Check base_dir and data structure. Exiting.")
        print(f"Attempted base_dir: {os.path.abspath(base_dir)}")
        # Create dummy data for testing plot function if no real data is found
        # print("Using dummy data for plot testing as no real data was loaded.")
        # all_data = [
        #     {'label': 'algoA/robotX', 'by_num_type': {'double': {'worst_error': 0.1}, 'float': {'worst_error': 0.2}}},
        #     {'label': 'algoB/robotX', 'by_num_type': {'double': {'worst_error': 0.15}, 'fixed32': {'worst_error': 0.3}}},
        #     {'label': 'algoA/robotY', 'by_num_type': {'double': {'worst_error': 0.05}, 'float': {'worst_error': 0.1}}},
        #     {'label': 'algoC/robotX', 'by_num_type': {'float': {'worst_error': 0.25}, 'fixed16': {'worst_error': 0.5}}},
        #     {'label': 'algoB/robotY', 'by_num_type': {'double': {'worst_error': 0.07}, 'fixed32': {'worst_error': 0.15}}},
        # ]

    if not all_data: # Still no data after potential dummy load (if dummy load is commented out)
        return


    from pprint import pprint
    print(f"Collected {len(all_data)} entries.")
    if all_data:
        print("Example of initial data entry (first one):")
        pprint(all_data[0])

    # Convert int types to fixed for consistency in keys
    for entry in all_data:
        if "int32-uniform" in entry["by_num_type"]:
            entry["by_num_type"]["fixed32-uniform"] = entry["by_num_type"].pop("int32-uniform")
        if "int32" in entry["by_num_type"]:
            entry["by_num_type"]["fixed32"] = entry["by_num_type"].pop("int32")
        if "int16" in entry["by_num_type"]:
            #entry["by_num_type"]["fixed16"] = entry["by_num_type"].pop("int16")
            entry["by_num_type"].pop("int16")
    
    pprint("all_data:")
    for entry in all_data:
        pprint(entry)

    # Sort data by Robot (label part 1), then Algorithm (label part 0)
    # This is crucial for the grouping logic in the plot function
    all_data = sorted(all_data, key=lambda x: (x["label"].split("/")[1], x["label"].split("/")[0]))
    
    print("\nData sorted by Robot then Algorithm. Example (first entry after sort):")
    for entry in all_data:
        pprint(entry)

    plot_error_data_multilevel(all_data)
    
    # If you have a plot_range_data_multilevel function and want to use it:
    #plot_range_data_multilevel(all_data) 

if __name__ == "__main__":
    main()