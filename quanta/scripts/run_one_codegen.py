import argparse
import sys
import os

def create_precision_list():
    """
    Generates the comprehensive list of precision choices programmatically
    based on the patterns observed in the user's request.
    """
    choices = []

    # Generate Fixed<W>-<I> types, where W and I range from 1 to 64.
    # This creates combinations like 'Fixed1-1', 'Fixed1-2', ... 'Fixed64-64'.
    for w in range(1, 65):
        for i in range(1, 65):
            choices.append(f"Fixed{w}-{i}")

    # Generate Fixed<W> types, where W ranges from 1 to 300.
    # This creates types like 'Fixed1', 'Fixed2', ... 'Fixed300'.
    for w in range(1, 65):
        choices.append(f"Fixed{w}")

    # Add the specified floating-point and other high-precision types.
    choices.extend(['Float16', 'Float32', 'Float64', 'Quad', 'QuadDouble'])

    # Using a set removes any potential duplicates, and sorting makes the
    # --help message clean and easy to read.
    return sorted(list(set(choices)))

def main():
    """
    This script parses command-line arguments for a file, a conversion type,
    and a specific fixed-point or floating-point precision.
    """
    # Initialize the argument parser with a description.
    parser = argparse.ArgumentParser(
        description="A script to process a file with specific conversion types and precision.",
        # Using RawTextHelpFormatter allows for better control over help message formatting.
        formatter_class=argparse.RawTextHelpFormatter
    )

    # Generate the list of choices for the 'precision' argument dynamically.
    precision_choices = create_precision_list()

    # --- Define Command-Line Arguments ---

    # Argument 2: type (choice)
    # This is a required positional argument with two possible values.
    parser.add_argument('type',
                        choices=['ap_fixed', 'C'],
                        help='The conversion type ("ap_fixed" or "C").')

    # Argument 3: precision (choice)
    # This is a required positional argument with a very large set of choices.
    parser.add_argument('precision',
                        choices=precision_choices,
                        help='The data precision type (e.g., Fixed16-8, Float32).')

    # If the script is run without any arguments, print the help message and exit.
    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    # Parse the arguments provided by the user from the command line.
    args = parser.parse_args()

    # --- Script Execution ---

    # Display the parsed arguments to confirm they were received correctly.
    # You can replace this section with your main script logic.
    print("--- Inputs Received ---")
    print(f"Type:      {args.type}")
    print(f"Precision: {args.precision}")
    print("-----------------------")
    print("\nScript is ready for you to add your processing logic.")

    # Generate Daisy DSL
    os.system("cd /quanta; cargo run > ex.scala")

    # Apply analysis and generate code
    if args.type == 'ap_fixed':
        os.system(
            "cd ../daisy; mkdir -p output; rm ranges.txt; rm errors.txt; rm -rf output; mkdir output;"
            f"./daisy --codegen --lang=C --apfixed --precision={args.precision} --rangeMethod=interval --errorMethod=interval ../quanta/ex.scala"
        )
    else:
        os.system(
            "cd ../daisy; mkdir -p output; rm ranges.txt; rm errors.txt; rm -rf output; mkdir output;"
            f"./daisy --codegen --lang=C --precision={args.precision} --rangeMethod=interval --errorMethod=interval ../quanta/ex.scala"
        )

    
    os.system("cd /daisy; mkdir /quanta/output/; cp output/* ../quanta/output/")
    os.system("cp /daisy/ranges.txt /quanta/output/ranges.txt")
    os.system("cp /daisy/errors.txt /quanta/output/errors.txt")


    os.system("rm -rf /daisy/output/")

    # remove the generated Scala file
    os.system("cd /quanta; rm ex.scala")

if __name__ == "__main__":
    # This standard Python construct ensures that the main() function is called
    # only when the script is executed directly.
    main()