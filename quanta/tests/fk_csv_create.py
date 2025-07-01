import csv
import random
import math

# --- Configuration ---
NUM_ROWS = 1000
NU = 6
FILENAME = "inputs.csv"

# Define random ranges for inputs
Q_MIN = -math.pi
Q_MAX = math.pi
V_MIN = -0.5
V_MAX = 0.5
A_MIN = -1.0
A_MAX = 1.0
# --- End Configuration ---

# Generate the CSV file
try:
    with open(FILENAME, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')

        # Optional: Write a header comment (will be skipped by the C++ reader)
        header_comment = ['#q[0]', 'q[1]', 'q[2]', 'q[3]', 'q[4]',
                          'v[0]', 'v[1]', 'v[2]', 'v[3]', 'v[4]',
                          'a[0]', 'a[1]', 'a[2]', 'a[3]', 'a[4]']
        # Writing it as a simple string starting with #
        csvfile.write(','.join(header_comment) + '\n')

        print(f"Generating {NUM_ROWS} rows of data for {FILENAME}...")

        for _ in range(NUM_ROWS):
            row_data = []

            # Generate NU q values
            for _ in range(NU):
                row_data.append(random.uniform(Q_MIN, Q_MAX))

            # Generate NU v values
            for _ in range(NU):
                row_data.append(random.uniform(V_MIN, V_MAX))

            # Generate NU a values
            for _ in range(NU):
                row_data.append(random.uniform(A_MIN, A_MAX))

            # Write the row to the CSV file
            csvwriter.writerow(row_data)

    print(f"Successfully created '{FILENAME}' with {NUM_ROWS} rows.")

except IOError as e:
    print(f"Error writing to file {FILENAME}: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")